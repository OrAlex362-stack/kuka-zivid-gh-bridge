from pathlib import Path

import pytest

pytest.importorskip("open3d")

from compute_wcs_pointcloud import WCSPointCloudProcessor
from config_loader import load_config
from pointcloud_io import write_ply
from zivid_camera_manager import ZividCameraManager


def test_open3d_three_plane_pipeline_writes_required_outputs(tmp_path: Path) -> None:
    config = load_config()
    config["_config_dir"] = str(tmp_path)
    config["paths"]["captures"] = "captures"
    config["zivid"]["mock_point_count_per_plane"] = 1500
    config["wcs"]["plane_detection"].update(
        {
            "voxel_size_mm": 2.0,
            "distance_threshold_mm": 1.0,
            "ransac_n": 3,
            "num_iterations": 500,
            "min_inlier_count": 100,
        }
    )
    camera = ZividCameraManager(config)
    camera.connect()
    frame = camera.capture_2d_3d()
    capture_dir = tmp_path / "captures" / "capture_0001"
    capture_dir.mkdir(parents=True)
    write_ply(capture_dir / "transformed_cloud.ply", frame.xyz, frame.rgba)

    result = WCSPointCloudProcessor(config).compute("capture_0001")
    assert result["ok"]
    assert Path(result["files"]["parameters"]).is_file()
    assert Path(result["files"]["cropped_ply"]).is_file()
    dimensions = result["workpiece"]["dimensions_mm"]
    assert 350 < dimensions["width"] < 450
    assert 80 < dimensions["height"] < 120
    assert 170 < dimensions["length"] < 230
