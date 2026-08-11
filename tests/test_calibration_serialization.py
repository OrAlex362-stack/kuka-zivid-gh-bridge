import numpy as np
import pytest

from camera_robot_calibration import serialize_calibration_result


def test_calibration_file_serialization_uses_directional_name_and_summary() -> None:
    transform = np.eye(4)
    transform[:3, 3] = [1.0, 2.0, 3.0]
    payload = serialize_calibration_result(
        transform,
        [
            {"translation": 0.2, "rotation": 0.1},
            {"translation": 0.4, "rotation": 0.3},
        ],
        sample_count=2,
        status="ok",
        timestamp="2026-01-01T00:00:00+00:00",
    )
    assert payload["calibration_type"] == "eye_in_hand"
    assert payload["transform"]["name"] == "T_flange_camera"
    assert payload["transform"]["from"] == "camera"
    assert payload["transform"]["to"] == "flange"
    assert payload["summary"]["translation_mean"] == pytest.approx(0.3)
    assert payload["summary"]["rotation_max"] == 0.3
