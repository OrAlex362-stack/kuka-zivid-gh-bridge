"""Rigid-transform helpers using the T_target_source naming convention."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray


class TransformValidationError(ValueError):
    """Raised when a matrix is not a finite rigid 4x4 transform."""


def validate_transform(
    matrix: ArrayLike,
    *,
    name: str = "transform",
    atol: float = 1e-6,
) -> NDArray[np.float64]:
    value = np.asarray(matrix, dtype=np.float64)
    if value.shape != (4, 4):
        raise TransformValidationError(f"{name} must have shape (4, 4), got {value.shape}.")
    if not np.all(np.isfinite(value)):
        raise TransformValidationError(f"{name} contains non-finite values.")
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=atol, rtol=0.0):
        raise TransformValidationError(f"{name} last row must be [0, 0, 0, 1].")

    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=atol, rtol=0.0):
        error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro"))
        raise TransformValidationError(
            f"{name} rotation is not orthonormal (Frobenius error {error:.3g})."
        )
    determinant = float(np.linalg.det(rotation))
    if not math.isclose(determinant, 1.0, abs_tol=atol, rel_tol=0.0):
        raise TransformValidationError(
            f"{name} rotation determinant must be +1, got {determinant:.9g}."
        )
    return value.copy()


def invert_transform(T_target_source: ArrayLike, *, name: str = "transform") -> NDArray[np.float64]:
    """Return T_source_target; inversion is explicit at every call site."""
    transform = validate_transform(T_target_source, name=name)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -(rotation.T @ translation)
    return inverse


def compose_transforms(
    T_target_intermediate: ArrayLike,
    T_intermediate_source: ArrayLike,
    *,
    target_intermediate_name: str = "T_target_intermediate",
    intermediate_source_name: str = "T_intermediate_source",
) -> NDArray[np.float64]:
    left = validate_transform(T_target_intermediate, name=target_intermediate_name)
    right = validate_transform(T_intermediate_source, name=intermediate_source_name)
    return validate_transform(left @ right, name="composed transform")


def rotation_distance_deg(first: ArrayLike, second: ArrayLike) -> float:
    first_value = validate_transform(first, name="first transform")
    second_value = validate_transform(second, name="second transform")
    relative = first_value[:3, :3].T @ second_value[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def translation_distance_mm(first: ArrayLike, second: ArrayLike) -> float:
    first_value = validate_transform(first, name="first transform")
    second_value = validate_transform(second, name="second transform")
    return float(np.linalg.norm(first_value[:3, 3] - second_value[:3, 3]))


def transform_points(T_target_source: ArrayLike, points_source: ArrayLike) -> NDArray[np.float64]:
    transform = validate_transform(T_target_source, name="T_target_source")
    points = np.asarray(points_source, dtype=np.float64)
    if points.ndim < 2 or points.shape[-1] != 3:
        raise ValueError("points_source must end with an XYZ dimension of size 3.")
    flat = points.reshape(-1, 3)
    transformed = flat @ transform[:3, :3].T + transform[:3, 3]
    return transformed.reshape(points.shape)


def matrix_to_list(matrix: ArrayLike) -> list[list[float]]:
    return validate_transform(matrix).tolist()


def finite_xyz(points: ArrayLike) -> NDArray[np.float64]:
    value = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return value[np.all(np.isfinite(value), axis=1)]


def yaml_safe(value: Any) -> Any:
    """Convert NumPy scalars/arrays recursively into YAML/JSON-safe values."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): yaml_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [yaml_safe(item) for item in value]
    return value
