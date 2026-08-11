"""Calibrated production capture with point clouds persisted outside HTTP."""

from __future__ import annotations

import logging
from pathlib import Path
import threading
from typing import Any

import numpy as np

from bridge_errors import BridgeError
from config_loader import configured_path
from pointcloud_io import write_ply, write_preview_xyz
from robot_udp_server import RobotUDPServer
from storage_utils import allocate_numbered_directory, atomic_write_yaml, load_yaml, utc_now_iso
from transform_utils import compose_transforms, matrix_to_list, transform_points, validate_transform
from zivid_camera_manager import ZividCameraManager


LOGGER = logging.getLogger(__name__)


def load_T_flange_camera(
    result_file: Path, *, camera_mode: str | None = None
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


class PointCloudCapture:
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

    def _metadata_base(
        self,
        capture_id: str,
        pose_before: Any,
        pose_after: Any,
        original_path: Path,
    ) -> dict[str, Any]:
        return {
            "capture_id": capture_id,
            "timestamp": utc_now_iso(),
            "robot_pose_before": pose_before.to_dict(include_raw_message=True),
            "robot_pose_after": (
                None if pose_after is None else pose_after.to_dict(include_raw_message=True)
            ),
            "settings_file": str(self.camera.settings_path),
            "original_zdf": str(original_path) if original_path.suffix.lower() == ".zdf" else None,
            "original_camera_file": str(original_path),
        }

    def capture(self) -> dict[str, Any]:
        if not self._operation_lock.acquire(blocking=False):
            raise BridgeError("CAPTURE_BUSY", "A production capture is already running.")
        with self._state_lock:
            self._busy = True
            self._last_error = None
        capture_dir: Path | None = None
        try:
            T_flange_camera = load_T_flange_camera(
                self.result_file, camera_mode=self.camera.mode
            )
            pose_before = self.robot.require_stationary_pose()
            if pose_before.T_base_flange is None:
                raise BridgeError("NO_VALID_ROBOT_POSE", "No valid T_base_flange is available.")

            capture_dir = allocate_numbered_directory(self.captures_root, "capture")
            capture_id = capture_dir.name
            LOGGER.info("Production capture %s started at robot seq=%s", capture_id, pose_before.seq)
            frame = self.camera.capture_2d_3d()
            original_path = frame.save_original(capture_dir / "original_pointcloud.zdf")
            pose_after = self.robot.latest_pose()
            if pose_after is None:
                metadata = self._metadata_base(
                    capture_id, pose_before, None, original_path
                )
                metadata.update({"valid": False, "error_code": "NO_ROBOT_POSE_AFTER_CAPTURE"})
                metadata_file = capture_dir / "pose_capture_info.yaml"
                atomic_write_yaml(metadata_file, metadata)
                raise BridgeError(
                    "NO_ROBOT_POSE_AFTER_CAPTURE",
                    "No robot pose was available after camera acquisition.",
                    details={"capture_id": capture_id, "metadata": str(metadata_file)},
                )

            after_status = self.robot.stationary_status()
            movement = self.robot.movement_between(pose_before, pose_after)
            if not after_status.fresh or movement["moved"]:
                metadata = self._metadata_base(
                    capture_id, pose_before, pose_after, original_path
                )
                metadata.update(
                    {
                        "valid": False,
                        "error_code": "ROBOT_MOVING",
                        "movement": movement,
                        "stationary_after": after_status.to_dict(),
                        "T_base_flange": matrix_to_list(pose_before.T_base_flange),
                        "T_flange_camera": matrix_to_list(T_flange_camera),
                        "T_base_camera": None,
                        "transformed_ply": None,
                        "preview_xyz": None,
                    }
                )
                metadata_file = capture_dir / "pose_capture_info.yaml"
                atomic_write_yaml(metadata_file, metadata)
                LOGGER.warning("Capture %s invalid: robot moved", capture_id)
                raise BridgeError(
                    "ROBOT_MOVING",
                    "Robot moved during camera acquisition.",
                    details={
                        "capture_id": capture_id,
                        "movement": movement,
                        "files": {
                            "camera": str(original_path),
                            "metadata": str(metadata_file),
                        },
                    },
                )

            # Direction is explicit: p_base = T_base_flange @ T_flange_camera @ p_camera.
            T_base_camera = compose_transforms(
                pose_before.T_base_flange,
                T_flange_camera,
                target_intermediate_name="T_base_flange",
                intermediate_source_name="T_flange_camera",
            )
            xyz_base = transform_points(T_base_camera, frame.xyz)
            ply_path = capture_dir / "transformed_cloud.ply"
            finite_point_count = write_ply(
                ply_path,
                xyz_base,
                frame.rgba,
                binary=bool(self.config["ply_binary"]),
            )
            preview_path = capture_dir / "preview_cloud.xyz"
            preview_count = write_preview_xyz(
                preview_path, xyz_base, int(self.config["preview_point_target"])
            )
            metadata_file = capture_dir / "pose_capture_info.yaml"
            metadata = self._metadata_base(
                capture_id, pose_before, pose_after, original_path
            )
            metadata.update(
                {
                    "valid": True,
                    "movement": movement,
                    "T_base_flange": matrix_to_list(pose_before.T_base_flange),
                    "T_flange_camera": matrix_to_list(T_flange_camera),
                    "T_base_camera": matrix_to_list(T_base_camera),
                    "transformed_ply": str(ply_path),
                    "preview_xyz": str(preview_path),
                    "source_point_count": int(np.asarray(frame.xyz).reshape(-1, 3).shape[0]),
                    "finite_transformed_point_count": finite_point_count,
                    "preview_point_count": preview_count,
                    "camera_mode": self.camera.mode,
                }
            )
            atomic_write_yaml(metadata_file, metadata)
            with self._state_lock:
                self._last_capture_id = capture_id
            LOGGER.info(
                "Capture %s complete: %d finite points, preview %d; outputs in %s",
                capture_id,
                finite_point_count,
                preview_count,
                capture_dir,
            )
            return {
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
                    "metadata": str(metadata_file),
                },
            }
        except BridgeError as exc:
            with self._state_lock:
                self._last_error = exc.message
            raise
        except Exception as exc:
            with self._state_lock:
                self._last_error = str(exc)
            LOGGER.exception("Production point-cloud capture failed")
            raise BridgeError(
                "CAPTURE_FAILED",
                "Production point-cloud capture failed.",
                status_code=500,
                details={
                    "cause": str(exc),
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
            }
