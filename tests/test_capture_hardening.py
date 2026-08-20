
import json
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from camera_robot_calibration import serialize_calibration_result
from compute_wcs_pointcloud import WCSPointCloudProcessor
from config_loader import load_config
from pointcloud_capture import PointCloudCapture
from robot_udp_server import RobotUDPServer, parse_json_packet
from storage_utils import atomic_write_yaml, load_yaml
from zivid_camera_manager import ZividCameraManager


def _config(tmp_path: Path) -> dict:
    config = load_config()
    config["zivid"]["mode"] = "mock"
    config["_config_dir"] = str(tmp_path)
    config["robot_udp"]["settle_time_ms"] = 0
    config["robot_udp"]["max_pose_age_ms"] = 5000
    config["paths"]["captures"] = "captures"
    config["paths"]["calibration_result"] = "calibration_results.yaml"
    config["zivid"]["settings_file"] = str((Path(__file__).parents[1] / "settings" / "capture_settings.yml").resolve())
    config["zivid"]["mock_point_count_per_plane"] = 5
    config["capture"]["preview_point_target"] = 5
    return config


def _packet(seq: int, x: float = 0.0) -> str:
    matrix = np.eye(4)
    matrix[0, 3] = x
    return json.dumps({"type": "POSE", "session_id": "capture-test", "seq": seq, "T_base_flange": matrix.tolist()})


def _inject(robot: RobotUDPServer, seq: int, x: float = 0.0) -> None:
    robot.inject_pose(parse_json_packet(_packet(seq, x), None, received_monotonic=time.monotonic()))


def _write_mock_calibration(path: Path) -> None:
    atomic_write_yaml(
        path,
        serialize_calibration_result(
            np.eye(4),
            [{"translation": 0.0, "rotation": 0.0}],
            sample_count=1,
            status="test",
            mock=True,
            synthetic=True,
            camera={"serial_number": "mock-camera", "model": "synthetic"},
        ),
    )


def _start_feeder(robot: RobotUDPServer, *, start_seq: int = 2, x: float = 0.0):
    stop = threading.Event()

    def feed() -> None:
        seq = start_seq
        while not stop.is_set():
            _inject(robot, seq, x)
            seq += 1
            time.sleep(0.001)

    thread = threading.Thread(target=feed)
    thread.start()
    return stop, thread


def test_capture_manifest_commits_only_after_required_artifacts(tmp_path: Path) -> None:
    config = _config(tmp_path)
    robot = RobotUDPServer(config["robot_udp"], config["kuka_pose"])
    _inject(robot, 1)
    _write_mock_calibration(tmp_path / "calibration_results.yaml")
    camera = ZividCameraManager(config)
    camera.connect()
    stop, feeder = _start_feeder(robot)
    try:
        result = PointCloudCapture(config, robot, camera).capture("req-1")
    finally:
        stop.set()
        feeder.join(timeout=1.0)

    manifest = load_yaml(Path(result["files"]["manifest"]))
    assert manifest["state"] == "COMMITTED"
    assert manifest["capture_id"] == "capture_0001"
    assert manifest["artifacts"]["transformed_ply"].endswith("transformed_cloud.ply")
    assert manifest["point_counts"]["finite_transformed"] == result["point_count"]


def test_incomplete_capture_manifest_is_failed_on_service_startup(tmp_path: Path) -> None:
    config = _config(tmp_path)
    capture_dir = tmp_path / "captures" / "capture_0007"
    capture_dir.mkdir(parents=True)
    atomic_write_yaml(capture_dir / "capture_manifest.yaml", {"capture_id": "capture_0007", "state": "STARTED"})

    PointCloudCapture(config, RobotUDPServer(config["robot_udp"], config["kuka_pose"]), ZividCameraManager(config))

    manifest = load_yaml(capture_dir / "capture_manifest.yaml")
    assert manifest["state"] == "FAILED"
    assert manifest["recovery"]["previous_state"] == "STARTED"


def test_wcs_latest_ignores_uncommitted_capture(tmp_path: Path) -> None:
    config = _config(tmp_path)
    uncommitted = tmp_path / "captures" / "capture_0001"
    committed = tmp_path / "captures" / "capture_0002"
    uncommitted.mkdir(parents=True)
    committed.mkdir(parents=True)
    atomic_write_yaml(uncommitted / "capture_manifest.yaml", {"capture_id": "capture_0001", "state": "STARTED"})
    atomic_write_yaml(committed / "capture_manifest.yaml", {"capture_id": "capture_0002", "state": "COMMITTED"})
    (committed / "transformed_cloud.ply").write_text("ply\n", encoding="ascii")

    processor = WCSPointCloudProcessor(config)
    assert processor._capture_directory(None).name == "capture_0002"
    with pytest.raises(Exception):
        processor._capture_directory("capture_0001")


def test_50_synthetic_capture_directories_have_unique_ids(tmp_path: Path) -> None:
    from storage_utils import allocate_numbered_directory

    names = [allocate_numbered_directory(tmp_path, "capture").name for _ in range(50)]
    assert len(names) == len(set(names))
    assert names[0] == "capture_0001"
    assert names[-1] == "capture_0050"


def test_capture_bridge_error_includes_stage_and_capture_directory(tmp_path: Path) -> None:
    config = _config(tmp_path)
    robot = RobotUDPServer(config['robot_udp'], config['kuka_pose'])
    _inject(robot, 1)
    _write_mock_calibration(tmp_path / 'calibration_results.yaml')
    camera = ZividCameraManager(config)
    camera.connect()

    with pytest.raises(Exception) as error:
        PointCloudCapture(config, robot, camera).capture('blocked-gh-solution')

    assert error.value.error_code == 'ROBOT_POSE_COVERAGE_INSUFFICIENT'
    details = error.value.details
    assert details['stage'] == 'post_capture_robot_validation'
    assert details['capture_directory'].endswith('capture_0001')
    assert details['capture_id'] == 'capture_0001'
    assert details['stationary_during_capture']['coverage_sufficient'] is False

    manifest = load_yaml(tmp_path / 'captures' / 'capture_0001' / 'capture_manifest.yaml')
    assert manifest['state'] == 'FAILED'
    manifest_details = manifest['errors'][0]['details']
    assert manifest_details['stage'] == 'post_capture_robot_validation'
    assert manifest_details['capture_directory'].endswith('capture_0001')