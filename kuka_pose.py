"""Centralized, explicitly configured conversion of KUKA X/Y/Z/A/B/C poses."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

from bridge_errors import BridgeError
from transform_utils import validate_transform


UNVERIFIED_MESSAGE = "KUKA rotation convention has not been verified."


def _rotation_x(angle_rad: float) -> NDArray[np.float64]:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rotation_y(angle_rad: float) -> NDArray[np.float64]:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rotation_z(angle_rad: float) -> NDArray[np.float64]:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def kuka_abc_to_matrix(
    x: float,
    y: float,
    z: float,
    a: float,
    b: float,
    c: float,
    convention: str | None,
) -> NDArray[np.float64]:
    """Convert millimetres/degrees to T_base_flange using an explicit formula.

    Convention names deliberately state multiplication order. No name is treated
    as an implicit KUKA default.
    """
    if convention is None or not str(convention).strip():
        raise BridgeError("KUKA_CONVENTION_UNVERIFIED", UNVERIFIED_MESSAGE, status_code=422)

    a_rad, b_rad, c_rad = map(math.radians, (float(a), float(b), float(c)))
    rotations = {
        "RZ_A_RY_B_RX_C": _rotation_z(a_rad) @ _rotation_y(b_rad) @ _rotation_x(c_rad),
        "RX_C_RY_B_RZ_A": _rotation_x(c_rad) @ _rotation_y(b_rad) @ _rotation_z(a_rad),
    }
    normalized = str(convention).strip().upper()
    if normalized not in rotations:
        supported = ", ".join(sorted(rotations))
        raise BridgeError(
            "KUKA_CONVENTION_UNSUPPORTED",
            f"Unsupported KUKA rotation convention '{convention}'. Supported: {supported}.",
            status_code=422,
        )

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotations[normalized]
    transform[:3, 3] = [float(x), float(y), float(z)]
    return validate_transform(transform, name="T_base_flange")
