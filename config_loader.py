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
    config["_config_file"] = str(path)
    config["_config_dir"] = str(path.parent)
    return config


def resolve_path(config: dict[str, Any], value: str | Path) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = Path(config["_config_dir"]) / path
    return path.resolve()


def configured_path(config: dict[str, Any], key: str) -> Path:
    return resolve_path(config, config["paths"][key])
