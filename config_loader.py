"""Configuration loading with paths resolved relative to config.yaml."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


REQUIRED_SECTIONS = {
    "api",
    "robot_udp",
    "kuka_pose",
    "zivid",
    "calibration",
    "capture",
    "wcs",
    "paths",
}

DEFAULT_SCAN_CONFIG: dict[str, Any] = {
    "enabled": True,
    "merge": {
        "voxel_size_mm": 2.0,
        "remove_statistical_outlier": {
            "enabled": True,
            "nb_neighbors": 20,
            "std_ratio": 2.0,
        },
        "icp": {
            "enabled": False,
            "max_correspondence_distance_mm": 5.0,
            "max_iterations": 50,
        },
    },
    "preview": {
        "max_points": 100000,
    },
}

DEFAULT_PROJECTION_CONFIG: dict[str, Any] = {
    "enabled": True,
    "deviation": {
        "mode": "absolute",
        "good_tolerance_mm": 1.0,
        "warning_tolerance_mm": 3.0,
        "palette": "pure_rgb",
        "point_radius_px": 3,
        "max_points": 20000,
    },
    "require_robot_stationary": True,
    "require_fresh_pose": True,
}


def _deep_merge_defaults(value: dict[str, Any], defaults: dict[str, Any]) -> dict[str, Any]:
    merged = deepcopy(defaults)
    for key, item in value.items():
        if isinstance(item, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge_defaults(item, merged[key])
        else:
            merged[key] = item
    return merged


def _positive_float(value: Any, *, name: str, allow_zero: bool = False) -> float:
    number = float(value)
    if allow_zero:
        valid = number >= 0.0
    else:
        valid = number > 0.0
    if not valid:
        comparator = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be {comparator}.")
    return number


def _positive_int(value: Any, *, name: str) -> int:
    number = int(value)
    if number <= 0:
        raise ValueError(f"{name} must be positive.")
    return number


def _validate_scan_config(config: dict[str, Any]) -> None:
    scan = config["scan"]
    if not isinstance(scan.get("enabled"), bool):
        raise ValueError("scan.enabled must be a boolean.")
    merge = scan["merge"]
    _positive_float(merge["voxel_size_mm"], name="scan.merge.voxel_size_mm", allow_zero=True)
    outlier = merge["remove_statistical_outlier"]
    if not isinstance(outlier.get("enabled"), bool):
        raise ValueError("scan.merge.remove_statistical_outlier.enabled must be a boolean.")
    _positive_int(outlier["nb_neighbors"], name="scan.merge.remove_statistical_outlier.nb_neighbors")
    _positive_float(outlier["std_ratio"], name="scan.merge.remove_statistical_outlier.std_ratio")
    icp = merge["icp"]
    if not isinstance(icp.get("enabled"), bool):
        raise ValueError("scan.merge.icp.enabled must be a boolean.")
    _positive_float(icp["max_correspondence_distance_mm"], name="scan.merge.icp.max_correspondence_distance_mm")
    _positive_int(icp["max_iterations"], name="scan.merge.icp.max_iterations")
    _positive_int(scan["preview"]["max_points"], name="scan.preview.max_points")


def _validate_projection_config(config: dict[str, Any]) -> None:
    projection = config["projection"]
    if not isinstance(projection.get("enabled"), bool):
        raise ValueError("projection.enabled must be a boolean.")
    if not isinstance(projection.get("require_robot_stationary"), bool):
        raise ValueError("projection.require_robot_stationary must be a boolean.")
    if not isinstance(projection.get("require_fresh_pose"), bool):
        raise ValueError("projection.require_fresh_pose must be a boolean.")
    deviation = projection["deviation"]
    if str(deviation["mode"]).lower() not in {"absolute", "signed"}:
        raise ValueError("projection.deviation.mode must be absolute or signed.")
    _positive_float(deviation["good_tolerance_mm"], name="projection.deviation.good_tolerance_mm", allow_zero=True)
    _positive_float(deviation["warning_tolerance_mm"], name="projection.deviation.warning_tolerance_mm", allow_zero=True)
    if float(deviation["warning_tolerance_mm"]) < float(deviation["good_tolerance_mm"]):
        raise ValueError("projection.deviation.warning_tolerance_mm must be greater than or equal to good_tolerance_mm.")
    if str(deviation["palette"]).lower() not in {"pure_rgb", "semantic"}:
        raise ValueError("projection.deviation.palette must be pure_rgb or semantic.")
    _positive_int(deviation["point_radius_px"], name="projection.deviation.point_radius_px")
    _positive_int(deviation["max_points"], name="projection.deviation.max_points")


def load_config(config_path: str | Path = "config.yaml") -> dict[str, Any]:
    path = Path(config_path).expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream)
    if not isinstance(loaded, dict):
        raise ValueError(f"Configuration root must be a mapping: {path}")
    missing = sorted(REQUIRED_SECTIONS - loaded.keys())
    if missing:
        raise ValueError(f"Missing config sections: {', '.join(missing)}")

    config = deepcopy(loaded)
    config["scan"] = _deep_merge_defaults(config.get("scan", {}), DEFAULT_SCAN_CONFIG)
    config["projection"] = _deep_merge_defaults(config.get("projection", {}), DEFAULT_PROJECTION_CONFIG)
    config["paths"].setdefault("scans", "data/scans")
    config["paths"].setdefault("projections", "data/projections")
    config["_config_file"] = str(path)
    config["_config_dir"] = str(path.parent)
    _validate_scan_config(config)
    _validate_projection_config(config)
    return config


def resolve_path(config: dict[str, Any], value: str | Path) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = Path(config["_config_dir"]) / path
    return path.resolve()


def configured_path(config: dict[str, Any], key: str) -> Path:
    return resolve_path(config, config["paths"][key])
