
from pathlib import Path

import numpy as np
import pytest

from bridge_errors import BridgeError
from camera_robot_calibration import serialize_calibration_result
from pointcloud_capture import load_T_flange_camera
from storage_utils import atomic_write_yaml, load_yaml


def _production_payload(serial: str = "CAM-1") -> dict:
    return serialize_calibration_result(
        np.eye(4),
        [{"translation": 0.1, "rotation": 0.01}],
        sample_count=1,
        status="ok",
        camera={"serial_number": serial, "model": "zivid-test"},
        robot={"robot_id": "KR-test", "controller_id": "KRC-test"},
        calibration={"type": "eye_in_hand", "sample_count": 1, "target_description": "asymmetric board", "sdk_version": "2.x"},
        source={"software_version": "test", "git_commit": "abc123"},
    )


def test_calibration_provenance_fields_are_serialized(tmp_path: Path) -> None:
    result = tmp_path / "calibration_results.yaml"
    atomic_write_yaml(result, _production_payload())
    payload = load_yaml(result)
    assert payload["transform"]["name"] == "T_flange_camera"
    assert payload["transform"]["from"] == "camera"
    assert payload["transform"]["to"] == "flange"
    assert payload["units"] == "mm"
    assert payload["camera"]["serial_number"] == "CAM-1"
    assert payload["robot"]["robot_id"] == "KR-test"
    assert payload["calibration"]["type"] == "eye_in_hand"
    assert "git_commit" in payload["source"]


def test_camera_serial_mismatch_is_rejected_for_non_mock_camera(tmp_path: Path) -> None:
    result = tmp_path / "calibration_results.yaml"
    atomic_write_yaml(result, _production_payload(serial="CAM-1"))
    with pytest.raises(BridgeError) as error:
        load_T_flange_camera(result, camera_mode="hardware", camera_serial_number="CAM-2")
    assert error.value.error_code == "CALIBRATION_CAMERA_MISMATCH"


def test_missing_production_provenance_is_rejected_for_non_mock_camera(tmp_path: Path) -> None:
    result = tmp_path / "calibration_results.yaml"
    atomic_write_yaml(
        result,
        {
            "calibration_type": "eye_in_hand",
            "sample_count": 1,
            "transform": {"name": "T_flange_camera", "from": "camera", "to": "flange", "matrix": np.eye(4).tolist()},
        },
    )
    with pytest.raises(BridgeError) as error:
        load_T_flange_camera(result, camera_mode="hardware")
    assert error.value.error_code == "CALIBRATION_PROVENANCE_INCOMPLETE"
