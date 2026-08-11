"""Logging configuration for console and rolling service files."""

from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any

from config_loader import configured_path


def configure_logging(config: dict[str, Any]) -> Path:
    log_path = configured_path(config, "service_log")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    level = getattr(logging, str(config["api"]["log_level"]).upper(), logging.INFO)
    formatter = logging.Formatter(
        "%(asctime)sZ %(levelname)s %(name)s [%(threadName)s] %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S",
    )
    formatter.converter = __import__("time").gmtime

    root = logging.getLogger()
    root.setLevel(level)
    if not any(getattr(handler, "baseFilename", None) == str(log_path) for handler in root.handlers):
        file_handler = RotatingFileHandler(
            log_path,
            maxBytes=int(config["api"]["log_max_bytes"]),
            backupCount=int(config["api"]["log_backup_count"]),
            encoding="utf-8",
        )
        file_handler.setFormatter(formatter)
        root.addHandler(file_handler)
    if not any(getattr(handler, "_bridge_console", False) for handler in root.handlers):
        console_handler = logging.StreamHandler()
        console_handler._bridge_console = True  # type: ignore[attr-defined]
        console_handler.setFormatter(formatter)
        root.addHandler(console_handler)
    return log_path
