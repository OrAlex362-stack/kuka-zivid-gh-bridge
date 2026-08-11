"""Create an identity Eye-in-Hand calibration for synthetic mock capture only."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Sequence

import numpy as np


# Support both ``python -m tools.create_mock_calibration`` and direct execution.
if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from camera_robot_calibration import (  # noqa: E402
    MOCK_CALIBRATION_WARNING,
    serialize_calibration_result,
)
from config_loader import configured_path, load_config  # noqa: E402
from storage_utils import atomic_write_yaml  # noqa: E402


def create_mock_calibration(config_path: str | Path = "config.yaml") -> Path:
    """Write the configured calibration result, but only in explicit mock mode."""

    config = load_config(config_path)
    configured_mode = config["zivid"].get("mode")
    camera_mode = str(configured_mode).strip().lower()
    if configured_mode is None or camera_mode != "mock":
        raise RuntimeError(
            "Refusing to create mock calibration: zivid.mode must be explicitly set "
            f"to 'mock' (current value: {configured_mode!r})."
        )

    T_flange_camera = np.eye(4, dtype=np.float64)
    payload = serialize_calibration_result(
        T_flange_camera,
        [],
        sample_count=0,
        status="mock_synthetic_identity",
        mock=True,
        synthetic=True,
        warning=MOCK_CALIBRATION_WARNING,
    )
    result_path = configured_path(config, "calibration_result")
    atomic_write_yaml(result_path, payload)
    return result_path


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config.yaml"),
        help="Path to config.yaml (default: ./config.yaml).",
    )
    args = parser.parse_args(argv)
    try:
        result_path = create_mock_calibration(args.config)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(MOCK_CALIBRATION_WARNING)
    print(f"Created: {result_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
