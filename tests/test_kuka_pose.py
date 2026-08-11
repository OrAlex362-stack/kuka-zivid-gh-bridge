import numpy as np
import pytest

from bridge_errors import BridgeError
from kuka_pose import kuka_abc_to_matrix


def test_unconfigured_convention_is_rejected() -> None:
    with pytest.raises(BridgeError, match="KUKA rotation convention has not been verified"):
        kuka_abc_to_matrix(0, 0, 0, 0, 0, 0, None)


def test_known_zero_reference_pose() -> None:
    transform = kuka_abc_to_matrix(10, 20, 30, 0, 0, 0, "RZ_A_RY_B_RX_C")
    assert np.allclose(transform[:3, :3], np.eye(3))
    assert np.allclose(transform[:3, 3], [10, 20, 30])


def test_known_positive_90_degree_a_reference_pose() -> None:
    transform = kuka_abc_to_matrix(0, 0, 0, 90, 0, 0, "RZ_A_RY_B_RX_C")
    expected = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    assert np.allclose(transform[:3, :3], expected, atol=1e-12)
