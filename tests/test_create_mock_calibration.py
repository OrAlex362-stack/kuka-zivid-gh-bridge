from pathlib import Path

import numpy as np
import pytest
import yaml

from bridge_errors import BridgeError
from pointcloud_capture import load_T_flange_camera
from storage_utils import atomic_write_yaml, load_yaml
from tools.create_mock_calibration import create_mock_calibration


PROJECT_ROOT = Path(__file__).parents[1]


def _config_file(tmp_path: Path, camera_mode: str) -> Path:
    with (PROJECT_ROOT / "config.yaml").open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    config["zivid"]["mode"] = camera_mode
    config["paths"]["calibration_result"] = "output/calibration_results.yaml"
    config_path = tmp_path / "config.yaml"
    atomic_write_yaml(config_path, config)
    return config_path


def test_mock_calibration_can_be_created_and_loaded_in_mock_mode(tmp_path) -> None:
    result_path = create_mock_calibration(_config_file(tmp_path, "mock"))

    assert result_path == (tmp_path / "output" / "calibration_results.yaml").resolve()
    payload = load_yaml(result_path)
    assert payload["calibration_type"] == "eye_in_hand"
    assert payload["transform"]["name"] == "T_flange_camera"
    assert payload["transform"]["from"] == "camera"
    assert payload["transform"]["to"] == "flange"
    assert np.array_equal(payload["transform"]["matrix"], np.eye(4))
    assert payload["mock"] is True
    assert payload["synthetic"] is True
    assert "NOT FOR PHYSICAL" in payload["warning"]

    loaded = load_T_flange_camera(result_path, camera_mode="mock")
    assert np.array_equal(loaded, np.eye(4))


@pytest.mark.parametrize("camera_mode", ["hardware", "file_camera"])
def test_mock_calibration_creation_is_rejected_outside_mock_mode(
    tmp_path, camera_mode
) -> None:
    config_path = _config_file(tmp_path, camera_mode)

    with pytest.raises(RuntimeError, match="Refusing to create mock calibration"):
        create_mock_calibration(config_path)

    assert not (tmp_path / "output" / "calibration_results.yaml").exists()


@pytest.mark.parametrize("marker", ["mock", "synthetic"])
def test_physical_mode_rejects_mock_or_synthetic_calibration(
    tmp_path, marker
) -> None:
    result_path = create_mock_calibration(_config_file(tmp_path, "mock"))
    payload = load_yaml(result_path)
    payload["mock"] = False
    payload["synthetic"] = False
    payload[marker] = True
    atomic_write_yaml(result_path, payload)

    with pytest.raises(BridgeError) as error:
        load_T_flange_camera(result_path, camera_mode="hardware")

    assert error.value.error_code == "MOCK_CALIBRATION_REJECTED"
