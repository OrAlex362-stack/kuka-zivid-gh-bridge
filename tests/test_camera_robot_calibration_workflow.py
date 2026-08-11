import json
from pathlib import Path
import time

import numpy as np

from camera_robot_calibration import CameraRobotCalibration
from config_loader import load_config
from robot_udp_server import RobotUDPServer, parse_json_packet
from zivid_camera_manager import ZividCameraManager


def _inject_matrix_pose(robot: RobotUDPServer, seq: int, x: float) -> None:
    matrix = np.eye(4)
    matrix[0, 3] = x
    packet = json.dumps({"type": "POSE", "seq": seq, "T_base_flange": matrix.tolist()})
    robot.inject_pose(parse_json_packet(packet, None, received_monotonic=time.monotonic()))


def test_mock_dynamic_calibration_add_solve_and_archive(tmp_path) -> None:
    config = load_config()
    config["_config_dir"] = str(tmp_path)
    config["paths"]["calibration_active"] = "calibration/active"
    config["paths"]["calibration_archive"] = "calibration/archive"
    config["paths"]["calibration_result"] = "calibration/calibration_results.yaml"
    config["robot_udp"]["settle_time_ms"] = 0
    config["robot_udp"]["max_pose_age_ms"] = 5000
    config["zivid"]["mock_point_count_per_plane"] = 10
    config["zivid"]["settings_file"] = str(
        (Path(__file__).parents[1] / "settings" / "capture_settings.yml").resolve()
    )

    robot = RobotUDPServer(config["robot_udp"], config["kuka_pose"])
    camera = ZividCameraManager(config)
    camera.connect()
    calibration = CameraRobotCalibration(config, robot, camera)

    _inject_matrix_pose(robot, 1, 0.0)
    first = calibration.add_sample()
    _inject_matrix_pose(robot, 2, 50.0)
    second = calibration.add_sample()
    assert first["sample_index"] == 1
    assert second["sample_index"] == 2
    assert calibration.status()["sample_count"] == 2

    solved = calibration.solve()
    assert solved["ok"]
    assert solved["sample_count"] == 2
    assert solved["T_flange_camera"][2][3] == 100.0
    assert calibration.status()["solved"]

    reset = calibration.reset()
    assert reset["previous_sample_count"] == 2
    assert Path(reset["archive"]).is_dir()
    assert calibration.status()["sample_count"] == 0
