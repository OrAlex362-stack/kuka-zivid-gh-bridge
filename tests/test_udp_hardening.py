
import json
import time

import numpy as np

from robot_udp_server import RobotUDPServer, parse_json_packet, poses_stationary_for_interval
from transform_utils import invert_transform, validate_transform


def _config(**overrides):
    config = {
        "bind_ip": "127.0.0.1",
        "port": 0,
        "parser": "auto",
        "history_size": 20,
        "socket_timeout_ms": 20,
        "shutdown_timeout_ms": 1000,
        "max_packet_bytes": 16384,
        "connection_timeout_ms": 1000,
        "max_pose_age_ms": 1000,
        "settle_time_ms": 500,
        "max_translation_motion_mm": 0.5,
        "max_rotation_motion_deg": 0.2,
        "expected_sender_ip": None,
    }
    config.update(overrides)
    return config


def _packet(seq: int, x: float = 0.0, *, session_id: str | None = "s1", timestamp=123.0, semantics: bool = True) -> str:
    matrix = np.eye(4)
    matrix[0, 3] = x
    payload = {
        "type": "POSE",
        "session_id": session_id,
        "seq": seq,
        "timestamp": timestamp,
        "T_base_flange": matrix.tolist(),
        "state": "READY",
    }
    if semantics:
        payload.update(
            {
                "matrix_name": "T_base_flange",
                "source_frame": "flange",
                "target_frame": "base",
                "units": "mm",
            }
        )
    return json.dumps(payload)


def _pose(seq: int, x: float = 0.0, *, session_id: str | None = "s1", t: float = 10.0, timestamp=123.0):
    return parse_json_packet(_packet(seq, x, session_id=session_id, timestamp=timestamp), None, received_monotonic=t)


def test_duplicate_and_out_of_order_sequence_packets_do_not_replace_latest_pose() -> None:
    server = RobotUDPServer(_config(), {"convention": None})
    server.inject_pose(_pose(10, 10.0, t=10.0))
    server.inject_pose(_pose(10, 99.0, t=10.1))
    server.inject_pose(_pose(9, 88.0, t=10.2))

    latest = server.latest_pose()
    assert latest is not None
    assert latest.seq == 10
    assert latest.T_base_flange[0, 3] == 10.0
    status = server.status()
    assert status["duplicate_packets"] == 1
    assert status["out_of_order_packets"] == 1


def test_new_session_accepts_sequence_reset_and_clears_stationary_history() -> None:
    server = RobotUDPServer(_config(), {"convention": None})
    server.inject_pose(_pose(1, 0.0, session_id="old", t=10.0))
    server.inject_pose(_pose(2, 0.0, session_id="old", t=10.6))
    assert server.stationary_status(now_monotonic=10.61).stationary

    server.inject_pose(_pose(1, 0.0, session_id="new", t=10.62))
    status = server.stationary_status(now_monotonic=10.63)
    assert status.reason == "ROBOT_SETTLING"
    assert status.sample_count == 1
    assert server.status()["session_restart_count"] == 1


def test_legacy_sequence_reset_without_new_session_is_rejected_conservatively() -> None:
    server = RobotUDPServer(_config(), {"convention": None})
    server.inject_pose(_pose(10, 10.0, session_id=None, t=10.0))
    server.inject_pose(_pose(1, 1.0, session_id=None, t=10.1))
    assert server.latest_pose().seq == 10
    assert server.status()["out_of_order_packets"] == 1


def test_unexpected_udp_sender_is_rejected_before_parsing() -> None:
    server = RobotUDPServer(_config(expected_sender_ip="192.0.2.10"), {"convention": None})
    server._handle_packet(_packet(1).encode("utf-8"), ("127.0.0.1", 50000))
    assert server.latest_pose() is None
    status = server.status()
    assert status["rejected_sender_packets"] == 1
    assert status["last_error"] == "UNEXPECTED_UDP_SENDER"


def test_robot_timestamp_values_are_metadata_not_receive_freshness_clock() -> None:
    server = RobotUDPServer(_config(max_pose_age_ms=100), {"convention": None})
    for index, robot_timestamp in enumerate([9999999999, -1, 42, 42], start=1):
        server.inject_pose(_pose(index, 0.0, t=10.0 + index * 0.01, timestamp=robot_timestamp))
    assert server.stationary_status(now_monotonic=10.05).fresh
    assert not server.stationary_status(now_monotonic=10.5).fresh


def test_pose_semantics_are_reported_but_not_physical_commissioning_proof() -> None:
    pose = _pose(1)
    assert pose.pose_semantics["matrix_name"] == "T_base_flange"
    assert pose.pose_semantics["software_semantics_valid"] is True
    assert "PHYSICAL_COMMISSIONING_REQUIRED" in pose.pose_semantics["validation_status"]


def test_interval_validation_detects_stationary_motion_and_insufficient_coverage() -> None:
    stationary = [_pose(1, 0.0, t=9.4), _pose(2, 0.0, t=10.0), _pose(3, 0.0, t=10.05), _pose(4, 0.0, t=10.11)]
    ok = poses_stationary_for_interval(
        stationary,
        interval_start_monotonic=10.0,
        interval_end_monotonic=10.1,
        now_monotonic=10.11,
        max_pose_age_ms=100,
        settle_time_ms=500,
        max_translation_motion_mm=0.5,
        max_rotation_motion_deg=0.2,
    )
    assert ok.stationary and ok.coverage_sufficient

    moving = [_pose(1, 0.0, t=9.4), _pose(2, 0.0, t=10.0), _pose(3, 2.0, t=10.05), _pose(4, 2.0, t=10.11)]
    moved = poses_stationary_for_interval(
        moving,
        interval_start_monotonic=10.0,
        interval_end_monotonic=10.1,
        now_monotonic=10.11,
        max_pose_age_ms=100,
        settle_time_ms=500,
        max_translation_motion_mm=0.5,
        max_rotation_motion_deg=0.2,
    )
    assert moved.reason == "ROBOT_MOVED_DURING_CAPTURE"

    insufficient = poses_stationary_for_interval(
        stationary[:2],
        interval_start_monotonic=10.0,
        interval_end_monotonic=10.1,
        now_monotonic=10.11,
        max_pose_age_ms=200,
        settle_time_ms=500,
        max_translation_motion_mm=0.5,
        max_rotation_motion_deg=0.2,
    )
    assert insufficient.reason == "ROBOT_POSE_COVERAGE_INSUFFICIENT"


def test_inverse_transform_can_be_numerically_valid_but_semantically_wrong() -> None:
    transform = np.eye(4)
    transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    transform[:3, 3] = [100.0, 20.0, 30.0]
    inverse = invert_transform(transform)
    validate_transform(transform)
    validate_transform(inverse)
    point_camera = np.array([10.0, 1.0, 2.0, 1.0])
    assert not np.allclose(transform @ point_camera, inverse @ point_camera)
