"""Small, deterministic persistence helpers."""

from __future__ import annotations

from datetime import datetime, timezone
import os
from pathlib import Path
import re
import shutil
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


def _path_is_ascii(path: Path) -> bool:
    try:
        str(path).encode("ascii")
        return True
    except UnicodeEncodeError:
        return False


def atomic_write_via_ascii_staging(path: Path, writer: Callable[[Path], None]) -> None:
    """Write through an ASCII-only staging path, then commit to ``path``.

    Some native Windows SDKs can open ordinary Python ``Path`` objects in
    Unicode locations while still failing when their C/C++ file API receives a
    non-ASCII path. Zivid ZDF save/load is one such boundary worth isolating.

    ``writer`` receives an ASCII-only staging filename. After it closes the
    file, Python performs a binary copy into a same-directory temporary file
    and atomically replaces the requested destination.

    Set ``ZIVID_ASCII_TEMP`` to override the staging directory. The selected
    staging directory itself must be ASCII-only.
    """
    path.parent.mkdir(parents=True, exist_ok=True)

    configured_root = os.environ.get("ZIVID_ASCII_TEMP")
    staging_root = Path(configured_root) if configured_root else Path(tempfile.gettempdir())

    if not _path_is_ascii(staging_root):
        raise RuntimeError(
            "Zivid ASCII staging directory contains non-ASCII characters: "
            f"{staging_root}. Set ZIVID_ASCII_TEMP to an ASCII-only writable path."
        )

    staging_root.mkdir(parents=True, exist_ok=True)
    staging_dir = Path(tempfile.mkdtemp(prefix="zivid_bridge_", dir=staging_root))
    staging_path = staging_dir / f"artifact{path.suffix}"

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.stem}.", suffix=path.suffix, dir=path.parent
    )
    os.close(descriptor)
    destination_temp = Path(temporary_name)

    try:
        writer(staging_path)

        if not staging_path.is_file():
            raise RuntimeError(
                f"Native writer returned without producing the staging file: {staging_path}"
            )

        # Python's binary copy handles the Unicode destination path; the native
        # SDK only ever sees the ASCII staging path.
        shutil.copyfile(staging_path, destination_temp)
        destination_temp.replace(path)
    except Exception:
        destination_temp.unlink(missing_ok=True)
        raise
    finally:
        shutil.rmtree(staging_dir, ignore_errors=True)


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream)
    if not isinstance(payload, dict):
        raise ValueError(f"Expected a YAML mapping in {path}.")
    return payload