import numpy as np
import pytest

from transform_utils import (
    TransformValidationError,
    compose_transforms,
    invert_transform,
    transform_points,
    validate_transform,
)


def test_validate_transform_accepts_identity() -> None:
    assert np.array_equal(validate_transform(np.eye(4)), np.eye(4))


@pytest.mark.parametrize(
    "matrix",
    [
        np.eye(3),
        np.full((4, 4), np.nan),
        np.diag([1.0, 1.0, 1.0, 2.0]),
        np.diag([1.0, 1.0, -1.0, 1.0]),
    ],
)
def test_validate_transform_rejects_invalid_matrices(matrix: np.ndarray) -> None:
    with pytest.raises(TransformValidationError):
        validate_transform(matrix)


def test_matrix_multiplication_direction_base_flange_camera() -> None:
    T_base_flange = np.eye(4)
    T_base_flange[:3, 3] = [100.0, 0.0, 0.0]
    T_flange_camera = np.eye(4)
    T_flange_camera[:3, 3] = [0.0, 20.0, 0.0]
    T_base_camera = compose_transforms(
        T_base_flange,
        T_flange_camera,
        target_intermediate_name="T_base_flange",
        intermediate_source_name="T_flange_camera",
    )
    assert np.allclose(T_base_camera[:3, 3], [100.0, 20.0, 0.0])
    assert np.allclose(transform_points(T_base_camera, [[1.0, 2.0, 3.0]]), [[101.0, 22.0, 3.0]])
    assert np.allclose(invert_transform(T_base_camera) @ T_base_camera, np.eye(4))
