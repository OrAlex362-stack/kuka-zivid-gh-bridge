"""Lifetime-managed Zivid 2.18 camera adapter with hardware-independent mock mode."""

from __future__ import annotations

from dataclasses import dataclass
import importlib.metadata
import logging
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from bridge_errors import BridgeError
from config_loader import resolve_path
from storage_utils import atomic_write_via_temp
from transform_utils import validate_transform


LOGGER = logging.getLogger(__name__)


def _metadata_value(obj: Any, *names: str) -> str | None:
    for name in names:
        candidate = getattr(obj, name, None)
        if candidate is None:
            continue
        try:
            value = candidate() if callable(candidate) else candidate
        except Exception:
            continue
        if value not in (None, ""):
            return str(value)
    return None


@dataclass(slots=True)
class MockDetectionResult:
    is_valid: bool = True
    description: str = "Synthetic calibration board detection"


@dataclass(slots=True)
class CapturedFrame:
    xyz: NDArray[np.float32]
    rgba: NDArray[np.uint8] | None
    native_frame: Any | None
    mock: bool
    source: str

    def save_original(self, requested_zdf_path: Path) -> Path:
        if self.native_frame is not None:
            def _write_zdf(target: Path) -> None:
                self.native_frame.save(str(target))

            atomic_write_via_temp(requested_zdf_path, _write_zdf)
            return requested_zdf_path

        mock_path = requested_zdf_path.with_suffix(".mock.npz")

        def _write_mock(target: Path) -> None:
            if self.rgba is None:
                np.savez_compressed(target, xyz=self.xyz)
            else:
                np.savez_compressed(target, xyz=self.xyz, rgba=self.rgba)

        atomic_write_via_temp(mock_path, _write_mock)
        return mock_path


class ZividCameraManager:
    """Own one zivid.Application and serialize camera access for its lifetime."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.root_config = config
        self.config = config["zivid"]
        self.mode = str(self.config["mode"]).lower()
        if self.mode not in {"mock", "hardware", "file_camera"}:
            raise ValueError("zivid.mode must be mock, hardware, or file_camera")
        self.settings_path = resolve_path(config, self.config["settings_file"])
        self._lock = threading.RLock()
        self._application: Any | None = None
        self._camera: Any | None = None
        self._file_camera_source_frame: Any | None = None
        self._settings: Any | None = None
        self._zivid: Any | None = None
        self._connected = False
        self._last_error: str | None = None
        self._capture_count = 0
        self._camera_serial_number: str | None = None
        self._camera_model: str | None = None
        self._busy = False
        self._sdk_python_version: str | None = None
        try:
            self._sdk_python_version = importlib.metadata.version("zivid")
        except importlib.metadata.PackageNotFoundError:
            pass

    def _import_zivid(self) -> Any:
        if self._zivid is None:
            try:
                import zivid  # type: ignore[import-not-found]
            except Exception as exc:
                raise BridgeError(
                    "ZIVID_SDK_UNAVAILABLE",
                    "The Zivid Python SDK is unavailable. Use zivid.mode=mock or install the matching SDK wrapper.",
                    status_code=503,
                    details={"cause": str(exc)},
                ) from exc
            self._zivid = zivid
        return self._zivid

    def _ensure_application(self) -> Any:
        if self._application is None:
            zivid = self._import_zivid()
            self._application = zivid.Application()
            LOGGER.info("Created lifetime Zivid Application (Python wrapper %s)", self._sdk_python_version)
        return self._application

    def connect(self) -> bool:
        with self._lock:
            if self.mode == "mock":
                self._connected = True
                self._camera_serial_number = "mock-camera"
                self._camera_model = "synthetic"
                self._last_error = None
                LOGGER.info("Zivid camera manager connected in synthetic MOCK mode")
                return True
            try:
                application = self._ensure_application()
                zivid = self._import_zivid()
                file_camera_settings = None
                if self.mode == "hardware":
                    self._camera = application.connect_camera()
                else:
                    configured = self.config.get("file_camera_path")
                    if not configured:
                        raise ValueError("zivid.file_camera_path is required in file_camera mode")
                    source_path = resolve_path(self.root_config, configured)
                    if not source_path.is_file():
                        raise FileNotFoundError(f"File camera source not found: {source_path}")
                    if source_path.suffix.lower() == ".zdf":
                        self._file_camera_source_frame = zivid.Frame(str(source_path))
                        self._camera = application.create_file_camera(self._file_camera_source_frame)
                        # Acquisition settings are locked for a diagnostics-ZDF
                        # FileCamera. SDK 2.18 requires the frame's own settings.
                        file_camera_settings = self._file_camera_source_frame.settings
                    else:
                        self._camera = application.create_file_camera(str(source_path))
                # Zivid Python 2.18 uses the load classmethod. It also requires
                # Application to exist first, hence this order is intentional.
                configured_settings = zivid.Settings.load(str(self.settings_path))
                self._settings = (
                    file_camera_settings
                    if file_camera_settings is not None
                    else configured_settings
                )
                self._connected = True
                self._camera_serial_number = _metadata_value(self._camera, "serial_number", "serialNumber")
                self._camera_model = _metadata_value(self._camera, "model_name", "model", "info")
                self._last_error = None
                LOGGER.info("Connected Zivid camera in %s mode", self.mode)
                return True
            except Exception as exc:
                self._connected = False
                self._camera = None
                self._last_error = str(exc)
                LOGGER.exception("Unable to connect Zivid camera")
                return False

    def close(self) -> None:
        with self._lock:
            if self._camera is not None and self.mode == "hardware":
                try:
                    self._camera.disconnect()
                except Exception:
                    LOGGER.exception("Error while disconnecting Zivid camera")
            self._camera = None
            self._camera_serial_number = None if self.mode != "mock" else self._camera_serial_number
            self._camera_model = None if self.mode != "mock" else self._camera_model
            self._file_camera_source_frame = None
            self._settings = None
            self._connected = False
            # Application remains alive until this manager itself is released.
            LOGGER.info("Zivid camera manager stopped")

    def _synthetic_frame(self) -> CapturedFrame:
        seed = int(self.config["mock_seed"]) + self._capture_count
        rng = np.random.default_rng(seed)
        count = int(self.config["mock_point_count_per_plane"])
        noise = 0.15

        top = np.column_stack(
            [rng.uniform(0, 400, count), rng.uniform(0, 200, count), rng.normal(100, noise, count)]
        )
        side = np.column_stack(
            [rng.uniform(0, 400, count), rng.normal(0, noise, count), rng.uniform(0, 100, count)]
        )
        end = np.column_stack(
            [rng.normal(0, noise, count), rng.uniform(0, 200, count), rng.uniform(0, 100, count)]
        )
        xyz = np.vstack([top, side, end]).astype(np.float32)
        rgb = np.vstack(
            [
                np.tile([70, 180, 90, 255], (count, 1)),
                np.tile([80, 110, 220, 255], (count, 1)),
                np.tile([220, 130, 70, 255], (count, 1)),
            ]
        ).astype(np.uint8)
        return CapturedFrame(xyz=xyz, rgba=rgb, native_frame=None, mock=True, source="synthetic")

    def capture_2d_3d(self) -> CapturedFrame:
        with self._lock:
            if self._busy:
                raise BridgeError("CAMERA_BUSY", "The camera is already acquiring a frame.")
            self._busy = True
            try:
                self._capture_count += 1
                if self.mode == "mock":
                    return self._synthetic_frame()
                if not self._connected or self._camera is None:
                    attempts = int(self.config["reconnect_attempts"])
                    for attempt in range(attempts):
                        if self.connect():
                            break
                        if attempt + 1 < attempts:
                            time.sleep(float(self.config["reconnect_delay_ms"]) / 1000.0)
                if not self._connected or self._camera is None or self._settings is None:
                    raise BridgeError(
                        "CAMERA_UNAVAILABLE",
                        "No physical or file Zivid camera is available.",
                        status_code=503,
                        details={"cause": self._last_error},
                    )
                frame = self._camera.capture_2d_3d(self._settings)
                point_cloud = frame.point_cloud()
                xyz = np.asarray(point_cloud.copy_data("xyz"), dtype=np.float32)
                try:
                    rgba = np.asarray(point_cloud.copy_data("rgba_srgb"), dtype=np.uint8)
                except Exception:
                    rgba = None
                return CapturedFrame(
                    xyz=xyz,
                    rgba=rgba,
                    native_frame=frame,
                    mock=False,
                    source=self.mode,
                )
            except BridgeError:
                raise
            except Exception as exc:
                self._connected = False
                self._last_error = str(exc)
                LOGGER.exception("Zivid capture failed")
                raise BridgeError(
                    "CAMERA_CAPTURE_FAILED",
                    "Zivid 2D+3D capture failed.",
                    status_code=503,
                    details={"cause": str(exc)},
                ) from exc
            finally:
                self._busy = False

    def detect_calibration_board(self, frame: CapturedFrame) -> Any:
        if frame.mock:
            return MockDetectionResult()
        zivid = self._import_zivid()
        return zivid.calibration.detect_calibration_board(frame.native_frame)

    @staticmethod
    def detection_valid(detection_result: Any) -> bool:
        if isinstance(detection_result, MockDetectionResult):
            return detection_result.is_valid
        return bool(detection_result.valid())

    @staticmethod
    def detection_status(detection_result: Any) -> str:
        if isinstance(detection_result, MockDetectionResult):
            return detection_result.description
        return str(detection_result.status_description())

    def make_hand_eye_input(self, T_base_flange: NDArray[np.float64], detection_result: Any) -> Any:
        transform = validate_transform(T_base_flange, name="T_base_flange")
        if isinstance(detection_result, MockDetectionResult):
            return {"T_base_flange": transform.copy(), "detection": detection_result}
        zivid = self._import_zivid()
        robot_pose = zivid.calibration.Pose(transform)
        return zivid.calibration.HandEyeInput(robot_pose, detection_result)

    def solve_eye_in_hand(self, inputs: list[Any]) -> Any:
        if self.mode == "mock":
            if len(inputs) < 2:
                raise BridgeError(
                    "CALIBRATION_INSUFFICIENT_SAMPLES",
                    "Mock calibration requires at least two distinct samples.",
                )
            transform = validate_transform(
                self.root_config["calibration"]["mock_T_flange_camera"],
                name="mock T_flange_camera",
            )
            return {
                "valid": True,
                "status": "mock_solution",
                "transform": transform,
                "residuals": [
                    {"translation": 0.0, "rotation": 0.0} for _ in inputs
                ],
            }
        zivid = self._import_zivid()
        return zivid.calibration.calibrate_eye_in_hand(inputs)

    def load_calibration_observation(self, zdf_path: Path, T_base_flange: NDArray[np.float64]) -> tuple[Any, Any]:
        if zdf_path.suffix.lower() == ".npz":
            detection = MockDetectionResult()
            return self.make_hand_eye_input(T_base_flange, detection), None
        zivid = self._import_zivid()
        self._ensure_application()
        frame = zivid.Frame(str(zdf_path))
        detection = zivid.calibration.detect_calibration_board(frame)
        if not detection.valid():
            raise ValueError(f"Calibration board no longer detectable in {zdf_path}")
        return self.make_hand_eye_input(T_base_flange, detection), frame

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "connected": self._connected,
                "mode": self.mode,
                "busy": self._busy,
                "sdk_python_version": self._sdk_python_version,
                "serial_number": self._camera_serial_number,
                "model": self._camera_model,
                "settings_file": str(self.settings_path),
                "capture_count": self._capture_count,
                "last_error": self._last_error,
            }
