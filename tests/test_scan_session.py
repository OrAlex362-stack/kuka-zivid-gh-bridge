from pathlib import Path
import json
import threading
import time

from fastapi.testclient import TestClient
import numpy as np
import pytest

from camera_robot_calibration import serialize_calibration_result
from config_loader import load_config
from robot_udp_server import parse_json_packet
from service import create_app
from storage_utils import atomic_write_yaml, load_yaml


def _config(tmp_path: Path) -> dict:
    config = load_config()
    config["zivid"]["mode"] = "mock"
    config["_config_dir"] = str(tmp_path)
    config["robot_udp"]["port"] = 0
    config["robot_udp"]["settle_time_ms"] = 0
    config["robot_udp"]["max_pose_age_ms"] = 5000
    config["paths"]["calibration_active"] = "calibration/active"
    config["paths"]["calibration_archive"] = "calibration/archive"
    config["paths"]["calibration_result"] = "calibration/calibration_results.yaml"
    config["paths"]["captures"] = "captures"
    config["paths"]["scans"] = "scans"
    config["paths"]["service_log"] = "logs/service.log"
    config["zivid"]["settings_file"] = str((Path(__file__).parents[1] / "settings" / "capture_settings.yml").resolve())
    config["zivid"]["mock_point_count_per_plane"] = 5
    config["capture"]["preview_point_target"] = 5
    config["scan"]["preview"]["max_points"] = 50
    config["scan"]["merge"]["voxel_size_mm"] = 0.0
    config["scan"]["merge"]["remove_statistical_outlier"]["enabled"] = False
    return config


def _matrix(x: float = 0.0) -> np.ndarray:
    matrix = np.eye(4)
    matrix[0, 3] = x
    return matrix


def _inject(app, seq: int, x: float = 0.0, *, received_monotonic: float | None = None) -> None:
    packet = json.dumps({"type": "POSE", "session_id": "scan-test", "seq": seq, "T_base_flange": _matrix(x).tolist()})
    app.state.robot.inject_pose(parse_json_packet(packet, None, received_monotonic=received_monotonic))


def _start_feeder(app, start_seq: int, x: float):
    stop = threading.Event()

    def feed() -> None:
        seq = start_seq
        while not stop.is_set():
            _inject(app, seq, x)
            seq += 1
            time.sleep(0.001)

    thread = threading.Thread(target=feed)
    thread.start()
    return stop, thread


def _write_calibration(config: dict) -> Path:
    result = Path(config["_config_dir"]) / config["paths"]["calibration_result"]
    atomic_write_yaml(
        result,
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
    return result


@pytest.mark.skipif(pytest.importorskip("importlib").util.find_spec("open3d") is None, reason="Open3D is required for scan merge")
def test_scan_session_start_capture_capture_merge_finish(tmp_path: Path) -> None:
    config = _config(tmp_path)
    _write_calibration(config)
    app = create_app(config_override=config)
    original_capture = app.state.camera.capture_2d_3d

    def slow_mock_capture():
        time.sleep(0.02)
        return original_capture()

    app.state.camera.capture_2d_3d = slow_mock_capture
    with TestClient(app) as client:
        assert client.post("/scan/start").json()["scan_id"] == "scan_0001"

        stop, feeder = _start_feeder(app, 1, 0.0)
        try:
            first = client.post("/scan/capture")
        finally:
            stop.set()
            feeder.join(timeout=1.0)
        assert first.status_code == 200

        stop, feeder = _start_feeder(app, 10000, 50.0)
        try:
            # Let the externally reached pose become the latest sample
            # before acquisition starts; crossing a pose transition
            # during capture must remain rejected by the backend.
            time.sleep(0.02)
            second = client.post("/scan/capture")
        finally:
            stop.set()
            feeder.join(timeout=1.0)
        assert second.status_code == 200
        assert second.json()["capture_count"] == 2

        merged = client.post("/scan/merge")
        assert merged.status_code == 200
        files = merged.json()["files"]
        assert Path(files["merged_downsampled"]).is_file()
        assert Path(files["preview_xyzrgb"]).is_file()

        finished = client.post("/scan/finish")
        assert finished.status_code == 200
        manifest = tmp_path / "scans" / "scan_0001" / "scan_manifest.yaml"
        assert manifest.is_file()
        payload = load_yaml(manifest)
        assert payload["state"] == "completed"
        assert payload["capture_count"] == 2
        assert payload["captures"][0]["T_base_camera"] == payload["captures"][0]["T_base_flange"]

        assert client.post("/scan/start").status_code == 200
        reset = client.post("/scan/reset")
        assert reset.status_code == 200
        assert manifest.is_file()


def test_scan_capture_requires_calibration(tmp_path: Path) -> None:
    config = _config(tmp_path)
    app = create_app(config_override=config)
    with TestClient(app) as client:
        assert client.post("/scan/start").status_code == 200
        stop, feeder = _start_feeder(app, 1, 0.0)
        try:
            response = client.post("/scan/capture")
        finally:
            stop.set()
            feeder.join(timeout=1.0)
        assert response.status_code == 422
        assert response.json()["error_code"] == "CALIBRATION_REQUIRED"


def test_scan_capture_rejects_moving_robot(tmp_path: Path) -> None:
    config = _config(tmp_path)
    config["robot_udp"]["settle_time_ms"] = 500
    _write_calibration(config)
    app = create_app(config_override=config)
    with TestClient(app) as client:
        assert client.post("/scan/start").status_code == 200
        now = time.monotonic()
        _inject(app, 1, 0.0, received_monotonic=now - 0.6)
        _inject(app, 2, 10.0, received_monotonic=now)
        response = client.post("/scan/capture")
        assert response.status_code == 409
        assert response.json()["error_code"] == "ROBOT_NOT_STATIONARY"


def test_scan_capture_rejects_stale_pose(tmp_path: Path) -> None:
    config = _config(tmp_path)
    config["robot_udp"]["max_pose_age_ms"] = 5
    config["robot_udp"]["connection_timeout_ms"] = 20000
    _write_calibration(config)
    app = create_app(config_override=config)
    with TestClient(app) as client:
        assert client.post("/scan/start").status_code == 200
        _inject(app, 1, 0.0, received_monotonic=time.monotonic() - 0.1)
        response = client.post("/scan/capture")
        assert response.status_code == 409
        assert response.json()["error_code"] == "ROBOT_POSE_STALE"


def test_scan_merge_reports_open3d_dependency_missing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    from bridge_errors import BridgeError
    from scan_session import ScanSessionManager

    config = _config(tmp_path)
    app = create_app(config_override=config)

    def missing_open3d():
        raise BridgeError(
            'SCAN_DEPENDENCY_MISSING',
            'Open3D is required for multi-view scan merge.',
            status_code=503,
            details={'cause': "No module named 'open3d'"},
        )

    monkeypatch.setattr(ScanSessionManager, '_import_open3d', staticmethod(missing_open3d))
    with TestClient(app) as client:
        assert client.post('/scan/start').status_code == 200
        response = client.post('/scan/merge')

    assert response.status_code == 503
    assert response.json()['error_code'] == 'SCAN_DEPENDENCY_MISSING'
    assert 'open3d' in response.json()['details']['cause']
