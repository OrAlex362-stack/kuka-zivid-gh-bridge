from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).parents[1]


def _load_script(name: str):
    path = ROOT / 'grasshopper' / name
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _Vec:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.X = x
        self.Y = y
        self.Z = z


class _Plane:
    Origin = _Vec(100.0, 200.0, 300.0)
    XAxis = _Vec(1.0, 0.0, 0.0)
    YAxis = _Vec(0.0, 1.0, 0.0)
    ZAxis = _Vec(0.0, 0.0, 1.0)


class _Transform:
    M00 = 0.0
    M01 = -1.0
    M02 = 0.0
    M03 = 10.0
    M10 = 1.0
    M11 = 0.0
    M12 = 0.0
    M13 = 20.0
    M20 = 0.0
    M21 = 0.0
    M22 = 1.0
    M23 = 30.0
    M30 = 0.0
    M31 = 0.0
    M32 = 0.0
    M33 = 1.0


def test_udp_publisher_accepts_plane_as_T_base_flange() -> None:
    udp = _load_script('udp_publisher.py')

    matrix = np.asarray(udp.plane_or_transform_to_matrix(_Plane()), dtype=float)

    assert matrix.tolist() == [
        [1.0, 0.0, 0.0, 100.0],
        [0.0, 1.0, 0.0, 200.0],
        [0.0, 0.0, 1.0, 300.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    assert np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0])
    assert np.isclose(np.linalg.det(matrix[:3, :3]), 1.0)
    assert np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3))
    p_flange = np.array([1.0, 2.0, 3.0, 1.0])
    assert np.allclose(matrix @ p_flange, [101.0, 202.0, 303.0, 1.0])

    packet = udp.build_pose_packet(_Plane(), 301, 'kuka-session', 'READY')
    assert packet['matrix_name'] == 'T_base_flange'
    assert packet['source_frame'] == 'flange'
    assert packet['target_frame'] == 'base'
    assert packet['units'] == 'mm'


def test_udp_publisher_accepts_transform() -> None:
    udp = _load_script('udp_publisher.py')

    matrix = np.asarray(udp.plane_or_transform_to_matrix(_Transform()), dtype=float)

    assert matrix.tolist() == [
        [0.0, -1.0, 0.0, 10.0],
        [1.0, 0.0, 0.0, 20.0],
        [0.0, 0.0, 1.0, 30.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def test_async_clients_parse_backend_error_code_before_unknown() -> None:
    capture = _load_script('async_capture_client.py')
    scan = _load_script('async_scan_client.py')

    payload = {
        'error_code': 'ROBOT_POSE_STALE',
        'details': {'stationary_during_capture': {'reason': 'ROBOT_POSE_COVERAGE_INSUFFICIENT'}},
    }

    assert capture.parse_error(payload) == 'ROBOT_POSE_STALE'
    assert scan.parse_error(payload) == 'ROBOT_POSE_STALE'
    assert capture.parse_error({'details': {'stationary_during_capture': {'reason': 'ROBOT_POSE_COVERAGE_INSUFFICIENT'}}}) == 'ROBOT_POSE_COVERAGE_INSUFFICIENT'
    assert scan.parse_error({}) == 'UNKNOWN_ERROR'