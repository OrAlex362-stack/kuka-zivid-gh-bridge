"""Small, deterministic persistence helpers."""

from __future__ import annotations

from datetime import datetime, timezone
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Callable

import yaml

from transform_utils import yaml_safe


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def allocate_numbered_directory(root: Path, prefix: str, *, width: int = 4) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    pattern = re.compile(rf"^{re.escape(prefix)}_(\d+)$")
    largest = 0
    for child in root.iterdir():
        match = pattern.match(child.name)
        if child.is_dir() and match:
            largest = max(largest, int(match.group(1)))
    number = largest + 1
    while True:
        candidate = root / f"{prefix}_{number:0{width}d}"
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            number += 1


def atomic_write_yaml(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            yaml.safe_dump(yaml_safe(payload), stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        temporary_path.replace(path)
    except Exception:
        temporary_path.unlink(missing_ok=True)
        raise


def atomic_write_via_temp(path: Path, writer: Callable[[Path], None]) -> None:
    """Write a file through a same-directory temporary path, then rename it.

    The writer receives a real filesystem path and must close any handles it
    opens before returning. This keeps non-YAML artifacts such as PLY, XYZ, NPZ,
    and ZDF files from appearing at their final path while still being written.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.stem}.", suffix=path.suffix, dir=path.parent
    )
    os.close(descriptor)
    temporary_path = Path(temporary_name)
    try:
        writer(temporary_path)
        temporary_path.replace(path)
    except Exception:
        temporary_path.unlink(missing_ok=True)
        raise


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream)
    if not isinstance(payload, dict):
        raise ValueError(f"Expected a YAML mapping in {path}.")
    return payload
