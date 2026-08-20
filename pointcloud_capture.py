"""Calibrated production capture with point clouds persisted outside HTTP."""

from __future__ import annotations

import hashlib
import json
import platform
import sys
import time
import logging
from pathlib import Path
import threading
from typing import Any

import numpy as np

from bridge_errors import BridgeError
from config_loader import configured_path
from pointcloud_io import write_ply, write_preview_xyz, write_preview_xyzrgb
from robot_udp_server import RobotUDPServer
from storage_utils import allocate_numbered_directory, atomic_write_yaml, load_yaml, utc_now_iso
from transform_utils import compose_transforms, matrix_to_list, transform_points, validate_transform
from zivid_camera_manager import ZividCameraManager


LOGGER = logging.getLogger(__name__)


def load_T_flange_camera(
    result_file: Path, *, camera_mode: str | None = None, camera_serial_number: str | None = None
) -> np.ndarray:
    if not result_file.is_file():
        raise BridgeError(
            "CALIBRATION_REQUIRED",
            "A valid calibration_results.yaml is required before production capture.",
            status_code=422,
            details={"result_file": str(result_file)},
        )
    try:
        payload = load_yaml(result_file)
        if payload.get("calibration_type") != "eye_in_hand":
            raise ValueError("calibration_type is not eye_in_hand")
        for marker in ("mock", "synthetic"):
            if marker in payload and not isinstance(payload[marker], bool):
                raise ValueError(f"{marker} marker must be a boolean")
        is_mock_calibration = payload.get("mock", False) or payload.get(
            "synthetic", False
        )
        normalized_camera_mode = (
            None if camera_mode is None else str(camera_mode).strip().lower()
        )
        if is_mock_calibration and normalized_camera_mode != "mock":
            raise BridgeError(
                "MOCK_CALIBRATION_REJECTED",
                "Mock/synthetic calibration cannot be used with a physical or non-mock camera.",
                status_code=422,
                details={
                    "result_file": str(result_file),
                    "camera_mode": normalized_camera_mode,
                },
            )
        production_mode = normalized_camera_mode in {"hardware", "file_camera"}
        if production_mode:
            missing = [key for key in ("camera", "robot", "calibration", "source", "units") if key not in payload]
            if missing:
                raise BridgeError(
                    "CALIBRATION_PROVENANCE_INCOMPLETE",
                    "Production calibration is missing required provenance metadata.",
                    status_code=422,
                    details={"result_file": str(result_file), "missing": missing},
                )
            if payload.get("units") != "mm":
                raise ValueError("calibration units must be mm")
            calibration_camera = payload.get("camera") if isinstance(payload.get("camera"), dict) else {}
            calibrated_serial = calibration_camera.get("serial_number")
            if calibrated_serial and camera_serial_number and str(calibrated_serial) != str(camera_serial_number):
                raise BridgeError(
                    "CALIBRATION_CAMERA_MISMATCH",
                    "Calibration camera serial number does not match the connected camera.",
                    status_code=422,
                    details={
                        "calibrated_serial_number": calibrated_serial,
                        "camera_serial_number": camera_serial_number,
                    },
                )
        transform = payload["transform"]
        if transform.get("name") != "T_flange_camera":
            raise ValueError("transform name is not T_flange_camera")
        if transform.get("from") != "camera" or transform.get("to") != "flange":
            raise ValueError("transform from/to frames are inconsistent")
        return validate_transform(transform["matrix"], name="T_flange_camera")
    except BridgeError:
        raise
    except Exception as exc:
        raise BridgeError(
            "CALIBRATION_RESULT_INVALID",
            "calibration_results.yaml is invalid or uses incompatible frame names.",
            status_code=422,
            details={"result_file": str(result_file), "cause": str(exc)},
        ) from exc


def _sha256_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_payload(payload: Any) -> str:
    encoded = json.dumps(payload, sort_keys=True, default=str, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _relative_or_string(root: Path, path: Path | None) -> str | None:
    if path is None:
        return None
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path)


class PointCloudCapture:
    STARTED = "STARTED"
    CAPTURED = "CAPTURED"
    TRANSFORMED = "TRANSFORMED"
    COMMITTED = "COMMITTED"
    FAILED = "FAILED"

    def __init__(
        self,
        config: dict[str, Any],
        robot: RobotUDPServer,
        camera: ZividCameraManager,
    ) -> None:
        self.root_config = config
        self.config = config["capture"]
        self.robot = robot
        self.camera = camera
        self.result_file = configured_path(config, "calibration_result")
        self.captures_root = configured_path(config, "captures")
        self.captures_root.mkdir(parents=True, exist_ok=True)
        self._operation_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._busy = False
        self._last_capture_id: str | None = None
        self._last_error: str | None = None
        self._idempotency: dict[str, dict[str, Any]] = {}
        self._recovered_incomplete_captures = 0
        self._recover_incomplete_captures()

    def _manifest_path(self, capture_dir: Path) -> Path:
        return capture_dir / "capture_manifest.yaml"

    def _manifest_state(self, capture_dir: Path) -> str | None:
        manifest_path = self._manifest_path(capture_dir)
        if not manifest_path.is_file():
            return None
        try:
            payload = load_yaml(manifest_path)
        except Exception:
            LOGGER.exception("Ignoring unreadable capture manifest: %s", manifest_path)
            return None
        return None if payload.get("state") is None else str(payload["state"])

    def _recover_incomplete_captures(self) -> None:
        count = 0
        for capture_dir in sorted(self.captures_root.glob("capture_[0-9][0-9][0-9][0-9]*")):
            if not capture_dir.is_dir():
                continue
            state = self._manifest_state(capture_dir)
            if state in {self.STARTED, self.CAPTURED, self.TRANSFORMED}:
                payload = load_yaml(self._manifest_path(capture_dir))
                payload.update(
                    {
                        "state": self.FAILED,
                        "recovery": {
                            "timestamp": utc_now_iso(),
                            "reason": "SERVICE_STARTED_WITH_INCOMPLETE_CAPTURE",
                            "previous_state": state,
                        },
                    }
                )
                atomic_write_yaml(self._manifest_path(capture_dir), payload)
                count += 1
        self._recovered_incomplete_captures = count

    def _write_manifest(self, capture_dir: Path, payload: dict[str, Any]) -> Path:
        manifest_path = self._manifest_path(capture_dir)
        atomic_write_yaml(manifest_path, payload)
        return manifest_path

    def _base_manifest(
        self,
        *,
        capture_id: str,
        state: str,
        request_id: str | None,
        timestamps: dict[str, Any],
        warnings: list[str] | None = None,
        errors: list[dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        camera_status = self.camera.status()
        latest = self.robot.latest_pose()
        return {
            "capture_id": capture_id,
            "state": state,
            "request_id": request_id,
            "timestamp_utc": utc_now_iso(),
            "timestamps": timestamps,
            "robot_packet": None if latest is None else {
                "session_id": latest.session_id,
                "seq": latest.seq,
                "received_timestamp_utc": latest.received_timestamp_utc,
                "received_monotonic": latest.received_monotonic,
                "robot_timestamp": latest.robot_timestamp,
                "pose_semantics": latest.pose_semantics,
            },
            "stationary_thresholds": {
                "max_pose_age_ms": self.robot.config.get("max_pose_age_ms"),
                "settle_time_ms": self.robot.config.get("settle_time_ms"),
                "max_translation_motion_mm": self.robot.config.get("max_translation_motion_mm"),
                "max_rotation_motion_deg": self.robot.config.get("max_rotation_motion_deg"),
            },
            "camera": {
                "mode": camera_status.get("mode"),
                "serial_number": camera_status.get("serial_number"),
                "model": camera_status.get("model"),
                "sdk_python_version": camera_status.get("sdk_python_version"),
            },
            "software": {
                "python_version": sys.version.split()[0],
                "platform": platform.platform(),
            },
            "warnings": warnings or [],
            "errors": errors or [],
        }

    def _metadata_base(
        self,
        capture_id: str,
        pose_before: Any,
        pose_after: Any,
        original_path: Path,
        timestamps: dict[str, Any],
    ) -> dict[str, Any]:
        return {
            "capture_id": capture_id,
            "timestamp": utc_now_iso(),
            "timestamps": timestamps,
            "robot_pose_before": pose_before.to_dict(include_raw_message=True),
            "robot_pose_after": (
                None if pose_after is None else pose_after.to_dict(include_raw_message=True)
            ),
            "settings_file": str(self.camera.settings_path),
            "settings_sha256": _sha256_file(self.camera.settings_path),
            "original_zdf": str(original_path) if original_path.suffix.lower() == ".zdf" else None,
            "original_camera_file": str(original_path),
        }

    def _failure_manifest(
        self,
        *,
        capture_dir: Path,
        capture_id: str,
        request_id: str | None,
        timestamps: dict[str, Any],
        error_code: str,
        message: str,
        extra: dict[str, Any] | None = None,
    ) -> None:
        manifest = self._base_manifest(
            capture_id=capture_id,
            state=self.FAILED,
            request_id=request_id,
            timestamps=timestamps,
            errors=[{"error_code": error_code, "message": message, "details": extra or {}}],
        )
        self._write_manifest(capture_dir, manifest)

    @staticmethod
    def _diagnostic_details(
        *,
        details: dict[str, Any] | None,
        message: str,
        stage: str,
        capture_dir: Path | None,
        capture_id: str | None,
    ) -> dict[str, Any]:
        diagnostic = dict(details or {})
        diagnostic.setdefault('cause', message)
        diagnostic['stage'] = stage
        diagnostic['capture_directory'] = None if capture_dir is None else str(capture_dir)
        if capture_id is not None:
            diagnostic.setdefault('capture_id', capture_id)
        return diagnostic

    def _finish_idempotent(self, request_id: str | None, result: dict[str, Any]) -> None:
        if request_id:
            with self._state_lock:
                self._idempotency[request_id] = dict(result)
                if len(self._idempotency) > int(self.config.get("idempotency_cache_size", 100)):
                    first_key = next(iter(self._idempotency))
                    self._idempotency.pop(first_key, None)

    def capture(self, request_id: str | None = None) -> dict[str, Any]:
        if request_id:
            with self._state_lock:
                if request_id in self._idempotency:
                    cached = dict(self._idempotency[request_id])
                    cached["idempotent_replay"] = True
                    return cached
        if not self._operation_lock.acquire(blocking=False):
            raise BridgeError("CAPTURE_BUSY", "A production capture is already running.")
        with self._state_lock:
            self._busy = True
            self._last_error = None
        capture_dir: Path | None = None
        capture_id: str | None = None
        timestamps: dict[str, Any] = {"capture_request_monotonic": time.monotonic()}
        stage = "validate_calibration"
        try:
            camera_status = self.camera.status()
            T_flange_camera = load_T_flange_camera(
                self.result_file,
                camera_mode=self.camera.mode,
                camera_serial_number=camera_status.get("serial_number"),
            )
            calibration_hash = _sha256_file(self.result_file)
            stage = "require_stationary_pose"
            pose_before = self.robot.require_stationary_pose()
            if pose_before.T_base_flange is None:
                raise BridgeError("NO_VALID_ROBOT_POSE", "No valid T_base_flange is available.")

            stage = "allocate_capture_directory"
            capture_dir = allocate_numbered_directory(self.captures_root, "capture")
            capture_id = capture_dir.name
            stage = "write_started_manifest"
            self._write_manifest(
                capture_dir,
                self._base_manifest(
                    capture_id=capture_id,
                    state=self.STARTED,
                    request_id=request_id,
                    timestamps=timestamps,
                ),
            )
            LOGGER.info("Production capture %s started at robot seq=%s", capture_id, pose_before.seq)
            timestamps["camera_capture_start_monotonic"] = time.monotonic()
            stage = "camera_capture"
            frame = self.camera.capture_2d_3d()
            timestamps["camera_capture_end_monotonic"] = time.monotonic()
            stage = "save_original_zdf"
            original_path = frame.save_original(capture_dir / "original_pointcloud.zdf")
            stage = "write_captured_manifest"
            self._write_manifest(
                capture_dir,
                {
                    **self._base_manifest(capture_id=capture_id, state=self.CAPTURED, request_id=request_id, timestamps=timestamps),
                    "artifacts": {"original_camera_file": _relative_or_string(self.captures_root, original_path)},
                },
            )

            timestamps["post_capture_validation_monotonic"] = time.monotonic()
            stage = "post_capture_robot_validation"
            pose_after = self.robot.latest_pose()
            if pose_after is None:
                metadata = self._metadata_base(capture_id, pose_before, None, original_path, timestamps)
                metadata.update({"valid": False, "error_code": "NO_ROBOT_POSE_AFTER_CAPTURE"})
                metadata_file = capture_dir / "pose_capture_info.yaml"
                atomic_write_yaml(metadata_file, metadata)
                self._failure_manifest(
                    capture_dir=capture_dir,
                    capture_id=capture_id,
                    request_id=request_id,
                    timestamps=timestamps,
                    error_code="NO_ROBOT_POSE_AFTER_CAPTURE",
                    message="No robot pose was available after camera acquisition.",
                    extra={
                        'metadata': str(metadata_file),
                        'stage': stage,
                        'capture_directory': str(capture_dir),
                    },
                )
                raise BridgeError(
                    "NO_ROBOT_POSE_AFTER_CAPTURE",
                    "No robot pose was available after camera acquisition.",
                    details={"capture_id": capture_id, "metadata": str(metadata_file)},
                )

            interval_status = self.robot.stationary_status_for_interval(
                interval_start_monotonic=timestamps["camera_capture_start_monotonic"],
                interval_end_monotonic=timestamps["camera_capture_end_monotonic"],
                now_monotonic=timestamps["post_capture_validation_monotonic"],
            )
            after_status = self.robot.stationary_status(now_monotonic=timestamps["post_capture_validation_monotonic"])
            movement = self.robot.movement_between(pose_before, pose_after)
            if not interval_status.fresh or not interval_status.coverage_sufficient or not interval_status.stationary or movement["moved"]:
                reason = interval_status.reason or ("ROBOT_MOVED_DURING_CAPTURE" if movement["moved"] else "ROBOT_MOVING")
                metadata = self._metadata_base(capture_id, pose_before, pose_after, original_path, timestamps)
                metadata.update(
                    {
                        "valid": False,
                        "error_code": reason,
                        "movement": movement,
                        "stationary_after": after_status.to_dict(),
                        "stationary_during_capture": interval_status.to_dict(),
                        "T_base_flange": matrix_to_list(pose_before.T_base_flange),
                        "T_flange_camera": matrix_to_list(T_flange_camera),
                        "T_base_camera": None,
                        "transformed_ply": None,
                        "preview_xyz": None,
                    }
                )
                metadata_file = capture_dir / "pose_capture_info.yaml"
                atomic_write_yaml(metadata_file, metadata)
                self._failure_manifest(
                    capture_dir=capture_dir,
                    capture_id=capture_id,
                    request_id=request_id,
                    timestamps=timestamps,
                    error_code=reason,
                    message="Robot pose history was not stationary for the software acquisition interval.",
                    extra={
                        'movement': movement,
                        'stationary_during_capture': interval_status.to_dict(),
                        'stage': stage,
                        'capture_directory': str(capture_dir),
                    },
                )
                raise BridgeError(
                    reason,
                    "Robot pose history was not stationary for the software acquisition interval.",
                    details={
                        "capture_id": capture_id,
                        "movement": movement,
                        "stationary_during_capture": interval_status.to_dict(),
                        "files": {"camera": str(original_path), "metadata": str(metadata_file)},
                    },
                )

            # Direction is explicit: p_base = T_base_flange @ T_flange_camera @ p_camera.
            stage = "compose_camera_transform"
            T_base_camera = compose_transforms(
                pose_before.T_base_flange,
                T_flange_camera,
                target_intermediate_name="T_base_flange",
                intermediate_source_name="T_flange_camera",
            )
            stage = "transform_pointcloud"
            xyz_base = transform_points(T_base_camera, frame.xyz)
            ply_path = capture_dir / "transformed_cloud.ply"
            stage = "write_transformed_ply"
            finite_point_count = write_ply(
                ply_path,
                xyz_base,
                frame.rgba,
                binary=bool(self.config["ply_binary"]),
            )
            preview_path = capture_dir / "preview_cloud.xyz"
            stage = "write_preview_xyz"
            preview_count = write_preview_xyz(
                preview_path, xyz_base, int(self.config["preview_point_target"])
            )
            preview_xyzrgb_path = None
            preview_xyzrgb_count = None
            if frame.rgba is not None:
                preview_xyzrgb_path = capture_dir / "preview_cloud.xyzrgb"
                stage = "write_preview_xyzrgb"
                preview_xyzrgb_count = write_preview_xyzrgb(
                    preview_xyzrgb_path, xyz_base, frame.rgba, int(self.config["preview_point_target"])
                )
            metadata_file = capture_dir / "pose_capture_info.yaml"
            metadata = self._metadata_base(capture_id, pose_before, pose_after, original_path, timestamps)
            metadata.update(
                {
                    "valid": True,
                    "movement": movement,
                    "stationary_during_capture": interval_status.to_dict(),
                    "T_base_flange": matrix_to_list(pose_before.T_base_flange),
                    "T_flange_camera": matrix_to_list(T_flange_camera),
                    "T_base_camera": matrix_to_list(T_base_camera),
                    "transformed_ply": str(ply_path),
                    "preview_xyz": str(preview_path),
                    "preview_xyzrgb": None if preview_xyzrgb_path is None else str(preview_xyzrgb_path),
                    "rgb_available": frame.rgba is not None,
                    "source_point_count": int(np.asarray(frame.xyz).reshape(-1, 3).shape[0]),
                    "finite_transformed_point_count": finite_point_count,
                    "preview_point_count": preview_count,
                    "preview_xyzrgb_point_count": preview_xyzrgb_count,
                    "camera_mode": self.camera.mode,
                }
            )
            stage = "write_capture_metadata"
            atomic_write_yaml(metadata_file, metadata)
            artifacts = {
                "original_camera_file": _relative_or_string(self.captures_root, original_path),
                "transformed_ply": _relative_or_string(self.captures_root, ply_path),
                "preview_xyz": _relative_or_string(self.captures_root, preview_path),
                "preview_xyzrgb": _relative_or_string(self.captures_root, preview_xyzrgb_path),
                "metadata": _relative_or_string(self.captures_root, metadata_file),
            }
            manifest = self._base_manifest(
                capture_id=capture_id,
                state=self.TRANSFORMED,
                request_id=request_id,
                timestamps=timestamps,
            )
            manifest.update(
                {
                    "robot_pose": {
                        "before": pose_before.to_dict(include_raw_message=False),
                        "after": pose_after.to_dict(include_raw_message=False),
                    },
                    "transforms": {
                        "T_base_flange": matrix_to_list(pose_before.T_base_flange),
                        "T_flange_camera": matrix_to_list(T_flange_camera),
                        "T_base_camera": matrix_to_list(T_base_camera),
                    },
                    "calibration": {
                        "file": _relative_or_string(Path(self.root_config["_config_dir"]), self.result_file),
                        "sha256": calibration_hash,
                    },
                    "settings": {
                        "file": _relative_or_string(Path(self.root_config["_config_dir"]), self.camera.settings_path),
                        "sha256": _sha256_file(self.camera.settings_path),
                    },
                    "config_sha256": _sha256_payload({key: value for key, value in self.root_config.items() if not str(key).startswith("_")}),
                    "point_counts": {
                        "source": int(np.asarray(frame.xyz).reshape(-1, 3).shape[0]),
                        "finite_transformed": finite_point_count,
                        "preview": preview_count,
                        "preview_xyzrgb": preview_xyzrgb_count,
                    },
                    "artifacts": artifacts,
                    "stationary_during_capture": interval_status.to_dict(),
                }
            )
            stage = "write_transformed_manifest"
            self._write_manifest(capture_dir, manifest)
            manifest["state"] = self.COMMITTED
            stage = "commit_manifest"
            manifest_path = self._write_manifest(capture_dir, manifest)
            with self._state_lock:
                self._last_capture_id = capture_id
            result = {
                "ok": True,
                "message": "Capture complete",
                "capture_id": capture_id,
                "point_count": finite_point_count,
                "preview_point_count": preview_count,
                "T_base_camera": matrix_to_list(T_base_camera),
                "files": {
                    "zdf": str(original_path) if original_path.suffix.lower() == ".zdf" else None,
                    "camera": str(original_path),
                    "ply": str(ply_path),
                    "preview_xyz": str(preview_path),
                    "preview_xyzrgb": None if preview_xyzrgb_path is None else str(preview_xyzrgb_path),
                    "metadata": str(metadata_file),
                    "manifest": str(manifest_path),
                },
            }
            self._finish_idempotent(request_id, result)
            LOGGER.info("Capture %s committed: %d finite points, preview %d", capture_id, finite_point_count, preview_count)
            return result
        except BridgeError as exc:
            with self._state_lock:
                self._last_error = exc.message
            diagnostic_details = self._diagnostic_details(
                details=exc.details,
                message=exc.message,
                stage=stage,
                capture_dir=capture_dir,
                capture_id=capture_id,
            )
            exc.details = diagnostic_details
            if capture_dir is not None and capture_id is not None and self._manifest_state(capture_dir) != self.FAILED:
                self._failure_manifest(
                    capture_dir=capture_dir,
                    capture_id=capture_id,
                    request_id=request_id,
                    timestamps=timestamps,
                    error_code=exc.error_code,
                    message=exc.message,
                    extra=diagnostic_details,
                )
            raise
        except Exception as exc:
            with self._state_lock:
                self._last_error = str(exc)
            if capture_dir is not None and capture_id is not None:
                self._failure_manifest(
                    capture_dir=capture_dir,
                    capture_id=capture_id,
                    request_id=request_id,
                    timestamps=timestamps,
                    error_code="CAPTURE_FAILED",
                    message="Production point-cloud capture failed.",
                    extra={"cause": str(exc), "stage": stage},
                )
            LOGGER.exception("Production point-cloud capture failed at stage=%s", stage)
            raise BridgeError(
                "CAPTURE_FAILED",
                "Production point-cloud capture failed.",
                status_code=500,
                details={
                    "cause": str(exc),
                    "stage": stage,
                    "capture_directory": None if capture_dir is None else str(capture_dir),
                },
            ) from exc
        finally:
            with self._state_lock:
                self._busy = False
            self._operation_lock.release()

    def status(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "busy": self._busy,
                "last_capture_id": self._last_capture_id,
                "last_error": self._last_error,
                "recovered_incomplete_captures": self._recovered_incomplete_captures,
                "idempotency_cache_size": len(self._idempotency),
            }
