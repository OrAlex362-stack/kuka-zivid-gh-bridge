from pathlib import Path
import json
import threading
import time

from fastapi.testclient import TestClient
import numpy as np

from config_loader import load_config
from service import create_app
from robot_udp_server import parse_json_packet


def _test_config(tmp_path: Path) -> dict:
    config = load_config()
    config["_config_dir"] = str(tmp_path)
    config["robot_udp"]["port"] = 0
    config["paths"]["calibration_active"] = "calibration/active"
    config["paths"]["calibration_archive"] = "calibration/archive"
    config["paths"]["calibration_result"] = "calibration/calibration_results.yaml"
    config["paths"]["captures"] = "captures"
    config["paths"]["service_log"] = "logs/service.log"
    config["zivid"]["settings_file"] = str(
        (Path(__file__).parents[1] / "settings" / "capture_settings.yml").resolve()
    )
    return config


def test_health_and_status_work_in_mock_mode_without_robot_or_point_cloud_http(tmp_path) -> None:
    app = create_app(config_override=_test_config(tmp_path))
    with TestClient(app) as client:
        health = client.get("/health")
        assert health.status_code == 200
        assert health.json()["service"] == "ready"

        status = client.get("/status")
        assert status.status_code == 200
        payload = status.json()
        assert payload["robot"]["connected"] is False
        assert payload["camera"]["mode"] == "mock"
        assert "point_cloud" not in payload

        pose = client.get("/robot/pose")
        assert pose.status_code == 404
        assert pose.json()["error_code"] == "NO_ROBOT_POSE"


def test_required_grasshopper_routes_exist(tmp_path) -> None:
    app = create_app(config_override=_test_config(tmp_path))
    routes = {(route.path, next(iter(route.methods))) for route in app.routes if route.methods}
    paths = {path for path, _method in routes}
    assert {
        "/health",
        "/status",
        "/robot/pose",
        "/calibration/sample",
        "/calibration/solve",
        "/calibration/reset",
        "/capture",
        "/wcs/compute",
    }.issubset(paths)


def _inject_pose(app, seq: int, x: float) -> None:
    matrix = np.eye(4)
    matrix[0, 3] = x
    packet = json.dumps({"type": "POSE", "seq": seq, "T_base_flange": matrix.tolist()})
    app.state.robot.inject_pose(
        parse_json_packet(packet, None, received_monotonic=time.monotonic())
    )


def _start_pose_feeder(app, start_seq: int, x: float = 50.0):
    stop = threading.Event()

    def feed() -> None:
        seq = start_seq
        while not stop.is_set():
            _inject_pose(app, seq, x)
            seq += 1
            time.sleep(0.001)

    thread = threading.Thread(target=feed)
    thread.start()
    return stop, thread


def test_mock_http_calibration_and_capture_pipeline_returns_metadata_only(tmp_path) -> None:
    config = _test_config(tmp_path)
    config["robot_udp"]["settle_time_ms"] = 0
    config["robot_udp"]["max_pose_age_ms"] = 5000
    config["zivid"]["mock_point_count_per_plane"] = 10
    config["capture"]["preview_point_target"] = 10
    app = create_app(config_override=config)
    with TestClient(app) as client:
        _inject_pose(app, 1, 0.0)
        assert client.post("/calibration/sample").status_code == 200
        _inject_pose(app, 2, 50.0)
        assert client.post("/calibration/sample").status_code == 200
        solved = client.post("/calibration/solve")
        assert solved.status_code == 200
        assert solved.json()["sample_count"] == 2

        stop, feeder = _start_pose_feeder(app, 3, 50.0)
        try:
            captured = client.post("/capture")
        finally:
            stop.set()
            feeder.join(timeout=1.0)
        assert captured.status_code == 200
        payload = captured.json()
        assert payload["capture_id"] == "capture_0001"
        assert payload["point_count"] == 30
        assert "points" not in payload
        assert Path(payload["files"]["ply"]).is_file()



def test_duplicate_capture_request_id_replays_first_result(tmp_path) -> None:
    config = _test_config(tmp_path)
    config["robot_udp"]["settle_time_ms"] = 0
    config["robot_udp"]["max_pose_age_ms"] = 5000
    config["zivid"]["mock_point_count_per_plane"] = 3
    config["capture"]["preview_point_target"] = 3
    app = create_app(config_override=config)
    with TestClient(app) as client:
        _inject_pose(app, 1, 0.0)
        assert client.post("/calibration/sample").status_code == 200
        _inject_pose(app, 2, 50.0)
        assert client.post("/calibration/sample").status_code == 200
        assert client.post("/calibration/solve").status_code == 200

        stop, feeder = _start_pose_feeder(app, 3, 50.0)
        try:
            first = client.post("/capture", json={"request_id": "gh-capture-1"})
        finally:
            stop.set()
            feeder.join(timeout=1.0)
        second = client.post("/capture", json={"request_id": "gh-capture-1"})

        assert first.status_code == 200
        assert second.status_code == 200
        assert first.json()["capture_id"] == second.json()["capture_id"]
        assert second.json()["idempotent_replay"] is True
        assert len(list((tmp_path / "captures").glob("capture_*"))) == 1
