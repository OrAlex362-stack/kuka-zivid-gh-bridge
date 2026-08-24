from __future__ import annotations

from pathlib import Path
import math

import numpy as np
import pytest

o3d = pytest.importorskip("open3d")

from config_loader import load_config
from pointcloud_io import read_xyzrgb, write_ply
from pointcloud_registration import ICPConfig, register_point_to_plane
from scan_session import ScanSessionManager
from storage_utils import load_yaml
from transform_utils import transform_points


def _rotation_z(degrees: float) -> np.ndarray:
    radians = math.radians(degrees)
    c = math.cos(radians)
    s = math.sin(radians)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _transform(translation: tuple[float, float, float], rotation_deg: float = 0.0) -> np.ndarray:
    value = np.eye(4, dtype=np.float64)
    value[:3, :3] = _rotation_z(rotation_deg)
    value[:3, 3] = np.asarray(translation, dtype=np.float64)
    return value


def _fixture_points() -> np.ndarray:
    grid = np.linspace(-20.0, 20.0, 13)
    xy = np.array([[x, y, 0.08 * x + 0.03 * y] for x in grid for y in grid], dtype=np.float64)
    yz = np.array([[-18.0, y, z] for y in grid for z in grid], dtype=np.float64)
    xz = np.array([[x, 17.0, z] for x in grid for z in grid], dtype=np.float64)
    return np.vstack([xy, yz, xz])


def _cloud(points: np.ndarray, colors: np.ndarray | None = None):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64, copy=False))
    if colors is not None:
        cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)
    return cloud


def _icp_config(**overrides) -> ICPConfig:
    values = {
        "enabled": True,
        "voxel_size_mm": 0.0,
        "max_correspondence_distance_mm": 8.0,
        "max_iterations": 80,
        "normal_radius_mm": 12.0,
        "normal_max_nn": 40,
        "min_fitness": 0.10,
        "max_inlier_rmse_mm": 3.0,
        "max_translation_correction_mm": 5.0,
        "max_rotation_correction_deg": 1.0,
    }
    values.update(overrides)
    return ICPConfig.from_mapping(values)


def test_point_to_plane_icp_recovers_small_translation_and_rotation() -> None:
    target_points = _fixture_points()
    T_error = _transform((1.0, -0.5, 0.4), rotation_deg=0.2)
    source_points = transform_points(T_error, target_points)

    result = register_point_to_plane(
        o3d,
        _cloud(source_points),
        _cloud(target_points),
        _icp_config(),
    )

    assert result.accepted
    assert result.reason is None
    assert result.fitness >= 0.90
    assert result.translation_correction_mm == pytest.approx(1.19, abs=0.25)
    assert result.rotation_correction_deg == pytest.approx(0.2, abs=0.15)
    corrected = transform_points(result.delta_transform, source_points)
    assert np.mean(np.linalg.norm(corrected - target_points, axis=1)) < 0.20


def test_excessive_icp_translation_is_rejected() -> None:
    target_points = _fixture_points()
    source_points = transform_points(_transform((20.0, 0.0, 0.0)), target_points)

    result = register_point_to_plane(
        o3d,
        _cloud(source_points),
        _cloud(target_points),
        _icp_config(max_correspondence_distance_mm=40.0, max_inlier_rmse_mm=10.0),
    )

    assert not result.accepted
    assert result.reason == "translation_limit_exceeded"
    assert result.translation_correction_mm is not None
    assert result.translation_correction_mm > 5.0
    assert np.allclose(result.applied_transform, np.eye(4))


def _scan_config(tmp_path: Path, *, icp_enabled: bool) -> dict:
    config = load_config()
    config["_config_dir"] = str(tmp_path)
    config["paths"]["scans"] = "scans"
    config["scan"]["preview"]["max_points"] = 10000
    config["scan"]["merge"]["voxel_size_mm"] = 0.0
    config["scan"]["merge"]["remove_statistical_outlier"]["enabled"] = False
    config["scan"]["merge"]["icp"].update(
        {
            "enabled": icp_enabled,
            "voxel_size_mm": 0.0,
            "max_correspondence_distance_mm": 8.0,
            "max_iterations": 80,
            "normal_radius_mm": 12.0,
            "normal_max_nn": 40,
            "min_fitness": 0.10,
            "max_inlier_rmse_mm": 3.0,
            "max_translation_correction_mm": 5.0,
            "max_rotation_correction_deg": 1.0,
        }
    )
    return config


def _manager_with_captures(tmp_path: Path, *, icp_enabled: bool, captures: list[tuple[np.ndarray, np.ndarray | None]]):
    config = _scan_config(tmp_path, icp_enabled=icp_enabled)
    manager = ScanSessionManager(config, robot=None, camera=None, capture=None)  # type: ignore[arg-type]
    scan_dir = tmp_path / "scans" / "scan_0001"
    scan_dir.mkdir(parents=True, exist_ok=True)
    records = []
    for index, (points, colors) in enumerate(captures, start=1):
        capture_dir = tmp_path / f"source_capture_{index:04d}"
        ply_path = capture_dir / "transformed_cloud.ply"
        write_ply(ply_path, points, colors, binary=True)
        capture_id = f"capture_{index:04d}"
        records.append(
            {
                "capture_id": capture_id,
                "T_base_flange": np.eye(4).tolist(),
                "T_flange_camera": np.eye(4).tolist(),
                "T_base_camera": np.eye(4).tolist(),
                "valid_points": int(len(points)),
                "rgb_available": colors is not None,
                "artifacts": {"base_ply": str(ply_path)},
            }
        )
    manager._active = {  # type: ignore[attr-defined]
        "scan_id": "scan_0001",
        "scan_dir": str(scan_dir),
        "state": "active",
        "created_at": "2026-08-24T00:00:00Z",
        "captures": records,
        "merge": {"merged": False},
        "artifacts": {},
        "warnings": [],
    }
    return manager


def test_scan_merge_icp_disabled_keeps_original_merge_behavior(tmp_path: Path) -> None:
    points = _fixture_points()
    shifted = transform_points(_transform((10.0, 0.0, 0.0)), points)
    manager = _manager_with_captures(tmp_path, icp_enabled=False, captures=[(points, None), (shifted, None)])

    result = manager._merge_active_locked(o3d)  # type: ignore[attr-defined]

    assert result["merge"]["icp"] == {"enabled": False}
    assert result["merge"]["raw_point_count"] == len(points) * 2
    assert "merged_pre_icp" not in result["files"]
    assert Path(result["files"]["preview_xyz"]).is_file()


def test_scan_merge_icp_preserves_rgb_and_writes_diagnostics(tmp_path: Path) -> None:
    points = _fixture_points()
    source_points = transform_points(_transform((1.0, -0.5, 0.4), rotation_deg=0.2), points)
    red = np.tile(np.array([[255, 0, 0]], dtype=np.uint8), (len(points), 1))
    blue = np.tile(np.array([[0, 0, 255]], dtype=np.uint8), (len(points), 1))
    manager = _manager_with_captures(tmp_path, icp_enabled=True, captures=[(points, red), (source_points, blue)])

    result = manager._merge_active_locked(o3d)  # type: ignore[attr-defined]

    files = result["files"]
    assert Path(files["merged_pre_icp"]).is_file()
    assert Path(files["merged_raw"]).is_file()
    assert Path(files["preview_xyzrgb"]).is_file()
    preview_xyz, preview_rgb = read_xyzrgb(Path(files["preview_xyzrgb"]))
    assert len(preview_xyz) == len(points) * 2
    assert np.count_nonzero(np.all(preview_rgb == [255, 0, 0], axis=1)) == len(points)
    assert np.count_nonzero(np.all(preview_rgb == [0, 0, 255], axis=1)) == len(points)

    icp = result["merge"]["icp"]
    assert icp["enabled"] is True
    assert icp["method"] == "point_to_plane"
    assert icp["anchor_capture"] == "capture_0001"
    assert icp["accepted_count"] == 2
    assert icp["rejected_count"] == 0
    assert icp["captures"][1]["accepted"] is True
    assert "fitness" in icp["captures"][1]
    manifest = load_yaml(tmp_path / "scans" / "scan_0001" / "scan_manifest.yaml")
    assert manifest["merge"]["icp"]["captures"][1]["accepted"] is True
    assert manifest["artifacts"]["preview_xyzrgb"] == files["preview_xyzrgb"]


def test_scan_merge_icp_insufficient_points_falls_back_to_initial_alignment(tmp_path: Path) -> None:
    points = _fixture_points()
    tiny = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float64)
    manager = _manager_with_captures(tmp_path, icp_enabled=True, captures=[(points, None), (tiny, None)])

    result = manager._merge_active_locked(o3d)  # type: ignore[attr-defined]

    icp = result["merge"]["icp"]
    assert icp["rejected_count"] == 1
    assert icp["captures"][1]["accepted"] is False
    assert icp["captures"][1]["reason"] == "insufficient_points"
    assert result["merge"]["raw_point_count"] == len(points) + len(tiny)
    manifest = load_yaml(tmp_path / "scans" / "scan_0001" / "scan_manifest.yaml")
    assert "initial Base-frame alignment was used" in manifest["warnings"][0]
