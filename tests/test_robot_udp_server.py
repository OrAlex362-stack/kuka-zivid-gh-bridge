import json
import socket
import time

import numpy as np

from robot_udp_server import (
    RobotUDPServer,
    parse_delimited_packet,
    parse_json_packet,
    poses_stationary,
)


def _matrix_packet(seq: int, x: float = 0.0) -> str:
    matrix = np.eye(4)
    matrix[0, 3] = x
    return json.dumps({"type": "POSE", "seq": seq, "T_base_flange": matrix.tolist()})


def test_json_abc_packet_remains_displayable_without_convention() -> None:
    pose = parse_json_packet(
        json.dumps(
            {
                "type": "POSE",
                "seq": 1523,
                "timestamp": 12345678,
                "x": 1250.32,
                "y": -422.15,
                "z": 875.62,
                "a": 179.82,
                "b": 0.31,
                "c": 89.95,
                "state": "READY",
            }
        ),
        None,
        received_monotonic=10.0,
    )
    assert pose.seq == 1523
    assert pose.x == 1250.32
    assert pose.T_base_flange is None


def test_delimited_packet_with_configured_convention() -> None:
    pose = parse_delimited_packet(
        "POSE;SEQ=1523;X=10;Y=20;Z=30;A=0;B=0;C=0;STATE=READY",
        "RZ_A_RY_B_RX_C",
    )
    assert pose.T_base_flange is not None
    assert np.allclose(pose.T_base_flange[:3, 3], [10, 20, 30])


def test_matrix_packet_bypasses_euler_convention() -> None:
    pose = parse_json_packet(_matrix_packet(8, 123.0), None)
    assert pose.T_base_flange is not None
    assert pose.T_base_flange[0, 3] == 123.0


def test_stationary_history_and_stale_detection() -> None:
    poses = [
        parse_json_packet(_matrix_packet(index), None, received_monotonic=stamp)
        for index, stamp in enumerate((10.0, 10.3, 10.6), start=1)
    ]
    status = poses_stationary(
        poses,
        now_monotonic=10.61,
        max_pose_age_ms=100,
        settle_time_ms=500,
        max_translation_motion_mm=0.5,
        max_rotation_motion_deg=0.2,
    )
    assert status.fresh and status.stationary

    stale = poses_stationary(
        poses,
        now_monotonic=11.0,
        max_pose_age_ms=100,
        settle_time_ms=500,
        max_translation_motion_mm=0.5,
        max_rotation_motion_deg=0.2,
    )
    assert not stale.fresh
    assert stale.reason == "ROBOT_POSE_STALE"


def test_motion_is_detected() -> None:
    poses = [
        parse_json_packet(_matrix_packet(1, 0.0), None, received_monotonic=10.0),
        parse_json_packet(_matrix_packet(2, 2.0), None, received_monotonic=10.6),
    ]
    status = poses_stationary(
        poses,
        now_monotonic=10.61,
        max_pose_age_ms=100,
        settle_time_ms=500,
        max_translation_motion_mm=0.5,
        max_rotation_motion_deg=0.2,
    )
    assert not status.stationary
    assert status.reason == "ROBOT_MOVING"


def test_background_udp_receiver_survives_and_stores_latest_matrix_pose() -> None:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    config = {
        "bind_ip": "127.0.0.1",
        "port": port,
        "parser": "auto",
        "history_size": 10,
        "socket_timeout_ms": 20,
        "shutdown_timeout_ms": 1000,
        "max_packet_bytes": 16384,
        "connection_timeout_ms": 1000,
        "max_pose_age_ms": 250,
        "settle_time_ms": 0,
        "max_translation_motion_mm": 0.5,
        "max_rotation_motion_deg": 0.2,
    }
    server = RobotUDPServer(config, {"convention": None})
    server.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        deadline = time.monotonic() + 1.0
        while server.status()["receiver_running"] is False and time.monotonic() < deadline:
            time.sleep(0.005)
        sender.sendto(_matrix_packet(99, 321.0).encode("utf-8"), ("127.0.0.1", port))
        while server.latest_pose() is None and time.monotonic() < deadline:
            time.sleep(0.005)
        pose = server.latest_pose()
        assert pose is not None
        assert pose.seq == 99
        assert pose.T_base_flange is not None
        assert pose.T_base_flange[0, 3] == 321.0
    finally:
        sender.close()
        server.stop()
