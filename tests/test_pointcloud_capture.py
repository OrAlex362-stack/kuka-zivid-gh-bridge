import json
import threading
import time

import numpy as np

from camera_robot_calibration import serialize_calibration_result
from config_loader import load_config
from pointcloud_capture import PointCloudCapture
from robot_udp_server import RobotUDPServer, parse_json_packet
from storage_utils import atomic_write_yaml
from zivid_camera_manager import ZividCameraManager


def test_mock_capture_writes_disk_artifacts_without_raw_http_payload(tmp_path) -> None:
    config = load_config()
    config["_config_dir"] = str(tmp_path)
    config["paths"]["captures"] = "captures"
    config["paths"]["calibration_result"] = "calibration_results.yaml"
    config["zivid"]["settings_file"] = str(
        (__import__("pathlib").Path(__file__).parents[1] / "settings" / "capture_settings.yml").resolve()
    )
    config["capture"]["preview_point_target"] = 100
    config["zivid"]["mock_point_count_per_plane"] = 100

    robot = RobotUDPServer(config["robot_udp"], config["kuka_pose"])
    now = time.monotonic()
    packet1 = json.dumps({"type": "POSE", "seq": 1, "T_base_flange": np.eye(4).tolist()})
    packet2 = json.dumps({"type": "POSE", "seq": 2, "T_base_flange": np.eye(4).tolist()})
    robot.inject_pose(parse_json_packet(packet1, None, received_monotonic=now - 0.6))
    robot.inject_pose(parse_json_packet(packet2, None, received_monotonic=now))

    calibration_file = tmp_path / "calibration_results.yaml"
    atomic_write_yaml(
        calibration_file,
        serialize_calibration_result(
            np.eye(4),
            [{"translation": 0.0, "rotation": 0.0}],
            sample_count=1,
            status="test",
        ),
    )
    camera = ZividCameraManager(config)
    camera.connect()
    stop = threading.Event()

    def feed_pose_history() -> None:
        seq = 3
        while not stop.is_set():
            packet = json.dumps({"type": "POSE", "seq": seq, "T_base_flange": np.eye(4).tolist()})
            robot.inject_pose(parse_json_packet(packet, None, received_monotonic=time.monotonic()))
            seq += 1
            time.sleep(0.001)

    feeder = threading.Thread(target=feed_pose_history)
    feeder.start()
    try:
        result = PointCloudCapture(config, robot, camera).capture()
    finally:
        stop.set()
        feeder.join(timeout=1.0)
    assert result["ok"]
    assert result["capture_id"] == "capture_0001"
    assert result["point_count"] == 300
    assert result["preview_point_count"] == 100
    assert (tmp_path / "captures" / "capture_0001" / "transformed_cloud.ply").is_file()
    assert "points" not in result
