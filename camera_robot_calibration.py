"""Dynamic Eye-in-Hand calibration dataset and Zivid solve orchestration."""

from __future__ import annotations

import logging
from pathlib import Path
import shutil
import threading
from typing import Any

import numpy as np

from bridge_errors import BridgeError
from config_loader import configured_path
from robot_udp_server import RobotPose, RobotUDPServer
from storage_utils import allocate_numbered_directory, atomic_write_yaml, load_yaml, utc_now_iso
from transform_utils import (
    matrix_to_list,
    rotation_distance_deg,
    translation_distance_mm,
    validate_transform,
)
from zivid_camera_manager import ZividCameraManager


LOGGER = logging.getLogger(__name__)

MOCK_CALIBRATION_WARNING = "MOCK CALIBRATION - NOT FOR PHYSICAL ROBOT/CAMERA USE"


def _call_or_value(value: Any) -> Any:
    return value() if callable(value) else value


def serialize_calibration_result(
    T_flange_camera: Any,
    residuals: list[dict[str, float]],
    *,
    sample_count: int,
    status: str,
    timestamp: str | None = None,
    mock: bool = False,
    synthetic: bool = False,
    warning: str | None = None,
) -> dict[str, Any]:
    transform = validate_transform(T_flange_camera, name="T_flange_camera")
    translations = [float(item["translation"]) for item in residuals]
    rotations = [float(item["rotation"]) for item in residuals]
    payload = {
        "calibration_type": "eye_in_hand",
        "timestamp": timestamp or utc_now_iso(),
        "sample_count": int(sample_count),
        "status": str(status),
        "transform": {
            "name": "T_flange_camera",
            "from": "camera",
            "to": "flange",
            "matrix": matrix_to_list(transform),
        },
        "residuals": [
            {
                "sample": index,
                "translation": float(item["translation"]),
                "rotation": float(item["rotation"]),
            }
            for index, item in enumerate(residuals, start=1)
        ],
        "summary": {
            "translation_mean": float(np.mean(translations)) if translations else None,
            "translation_max": float(np.max(translations)) if translations else None,
            "rotation_mean": float(np.mean(rotations)) if rotations else None,
            "rotation_max": float(np.max(rotations)) if rotations else None,
        },
    }
    if mock:
        payload["mock"] = True
    if synthetic:
        payload["synthetic"] = True
    if warning is not None:
        payload["warning"] = str(warning)
    return payload


class CameraRobotCalibration:
    def __init__(
        self,
        config: dict[str, Any],
        robot: RobotUDPServer,
        camera: ZividCameraManager,
    ) -> None:
        self.root_config = config
        self.config = config["calibration"]
        self.robot = robot
        self.camera = camera
        self.active_dir = configured_path(config, "calibration_active")
        self.archive_dir = configured_path(config, "calibration_archive")
        self.result_file = configured_path(config, "calibration_result")
        self.active_dir.mkdir(parents=True, exist_ok=True)
        self.archive_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._operation_lock = threading.Lock()
        self._records: list[dict[str, Any]] = []
        self._inputs: list[Any | None] = []
        self._frames: list[Any] = []
        self._load_metadata()

    def _load_metadata(self) -> None:
        records: list[dict[str, Any]] = []
        for sample_dir in sorted(self.active_dir.glob("sample_[0-9][0-9][0-9][0-9]")):
            metadata_file = sample_dir / "sample.yaml"
            if not metadata_file.is_file():
                LOGGER.warning("Ignoring calibration directory without sample.yaml: %s", sample_dir)
                continue
            try:
                record = load_yaml(metadata_file)
                matrix = validate_transform(
                    record["T_base_flange"], name=f"T_base_flange in {metadata_file}"
                )
                record["T_base_flange"] = matrix_to_list(matrix)
                record["sample_dir"] = str(sample_dir)
                records.append(record)
            except Exception:
                LOGGER.exception("Ignoring invalid persisted calibration sample %s", metadata_file)
        with self._lock:
            self._records = records
            self._inputs = [None] * len(records)
        if records:
            LOGGER.info("Recovered %d calibration sample metadata records", len(records))

    def _record_matrix(self, record: dict[str, Any]) -> np.ndarray:
        return validate_transform(record["T_base_flange"], name="persisted T_base_flange")

    def _separation(self, transform: np.ndarray) -> tuple[float | None, float | None, int | None]:
        with self._lock:
            if not self._records:
                return None, None, None
            distances = [
                (
                    translation_distance_mm(transform, self._record_matrix(record)),
                    rotation_distance_deg(transform, self._record_matrix(record)),
                    index,
                )
                for index, record in enumerate(self._records, start=1)
            ]
        translation_threshold = max(
            float(self.config["min_translation_separation_mm"]), 1e-12
        )
        rotation_threshold = max(float(self.config["min_rotation_separation_deg"]), 1e-12)
        # Nearest is evaluated in normalized translation/rotation threshold
        # space so a duplicate is not hidden by a different pose that is close
        # in translation but far away in rotation (or vice versa).
        nearest = min(
            distances,
            key=lambda item: max(
                item[0] / translation_threshold, item[1] / rotation_threshold
            ),
        )
        return nearest

    def _check_distinct_pose(self, transform: np.ndarray) -> list[str]:
        translation, rotation, sample = self._separation(transform)
        if translation is None or rotation is None:
            return []
        thresholds = (
            float(self.config["min_translation_separation_mm"]),
            float(self.config["min_rotation_separation_deg"]),
        )
        if translation < thresholds[0] and rotation < thresholds[1]:
            message = (
                f"Pose is nearly identical to sample {sample}: {translation:.3f} mm and "
                f"{rotation:.3f} deg separation."
            )
            if str(self.config["duplicate_policy"]).lower() == "reject":
                raise BridgeError(
                    "CALIBRATION_POSE_TOO_SIMILAR",
                    message,
                    details={
                        "nearest_sample": sample,
                        "translation_separation_mm": translation,
                        "rotation_separation_deg": rotation,
                    },
                )
            return [message]
        return []

    def _save_rejected_attempt(
        self,
        *,
        reason: str,
        pose_before: RobotPose,
        pose_after: RobotPose | None,
        frame: Any | None,
        details: dict[str, Any] | None = None,
    ) -> Path:
        attempts_root = self.active_dir / "rejected_attempts"
        attempt_dir = allocate_numbered_directory(attempts_root, "attempt")
        original_path: Path | None = None
        if frame is not None:
            original_path = frame.save_original(attempt_dir / "calibration_frame.zdf")
        payload = {
            "accepted": False,
            "timestamp": utc_now_iso(),
            "reason": reason,
            "details": details or {},
            "robot_pose_before": pose_before.to_dict(include_raw_message=True),
            "robot_pose_after": (
                None if pose_after is None else pose_after.to_dict(include_raw_message=True)
            ),
            "camera_file": None if original_path is None else str(original_path),
        }
        atomic_write_yaml(attempt_dir / "rejected_sample.yaml", payload)
        return attempt_dir

    def add_sample(self) -> dict[str, Any]:
        if not self._operation_lock.acquire(blocking=False):
            raise BridgeError("CALIBRATION_BUSY", "A calibration operation is already running.")
        frame = None
        pose_before: RobotPose | None = None
        try:
            pose_before = self.robot.require_stationary_pose()
            if pose_before.T_base_flange is None:
                raise BridgeError("NO_VALID_ROBOT_POSE", "No valid T_base_flange is available.")
            warnings = self._check_distinct_pose(pose_before.T_base_flange)
            LOGGER.info("Calibration sample acquisition started at robot seq=%s", pose_before.seq)
            frame = self.camera.capture_2d_3d()
            detection = self.camera.detect_calibration_board(frame)
            pose_after = self.robot.latest_pose()
            if pose_after is None:
                rejected = self._save_rejected_attempt(
                    reason="NO_ROBOT_POSE_AFTER_CAPTURE",
                    pose_before=pose_before,
                    pose_after=None,
                    frame=frame,
                )
                raise BridgeError(
                    "NO_ROBOT_POSE_AFTER_CAPTURE",
                    "No robot pose was available after camera acquisition.",
                    details={"saved_attempt": str(rejected)},
                )

            after_status = self.robot.stationary_status()
            movement = self.robot.movement_between(pose_before, pose_after)
            if not after_status.fresh or movement["moved"]:
                details = {"movement": movement, "stationary_after": after_status.to_dict()}
                rejected = self._save_rejected_attempt(
                    reason="ROBOT_MOVING",
                    pose_before=pose_before,
                    pose_after=pose_after,
                    frame=frame,
                    details=details,
                )
                LOGGER.warning("Calibration sample rejected: robot moved; saved %s", rejected)
                raise BridgeError(
                    "ROBOT_MOVING",
                    "Robot moved during camera acquisition.",
                    details={**details, "saved_attempt": str(rejected)},
                )

            if not self.camera.detection_valid(detection):
                detection_status = self.camera.detection_status(detection)
                rejected = self._save_rejected_attempt(
                    reason="CALIBRATION_BOARD_NOT_DETECTED",
                    pose_before=pose_before,
                    pose_after=pose_after,
                    frame=frame,
                    details={"detection_status": detection_status},
                )
                LOGGER.warning("Calibration sample rejected: %s", detection_status)
                raise BridgeError(
                    "CALIBRATION_BOARD_NOT_DETECTED",
                    "Zivid calibration board detection was invalid.",
                    status_code=422,
                    details={"detection_status": detection_status, "saved_attempt": str(rejected)},
                )

            hand_eye_input = self.camera.make_hand_eye_input(pose_before.T_base_flange, detection)
            sample_dir = allocate_numbered_directory(self.active_dir, "sample")
            sample_index = int(sample_dir.name.rsplit("_", 1)[1])
            original_path = frame.save_original(sample_dir / "calibration_frame.zdf")
            metadata_path = sample_dir / "sample.yaml"
            record = {
                "accepted": True,
                "sample": sample_index,
                "timestamp": utc_now_iso(),
                "robot_pose_before": pose_before.to_dict(include_raw_message=True),
                "robot_pose_after": pose_after.to_dict(include_raw_message=True),
                "T_base_flange": matrix_to_list(pose_before.T_base_flange),
                "movement": movement,
                "detection_status": self.camera.detection_status(detection),
                "camera_file": str(original_path),
                "settings_file": str(self.camera.settings_path),
                "warnings": warnings,
                "sample_dir": str(sample_dir),
            }
            atomic_write_yaml(metadata_path, record)
            with self._lock:
                self._records.append(record)
                self._inputs.append(hand_eye_input)
                if frame.native_frame is not None:
                    self._frames.append(frame.native_frame)
                count = len(self._records)
                self._write_dataset_index_locked()
            LOGGER.info("Calibration sample %d accepted (%d active samples)", sample_index, count)
            return {
                "ok": True,
                "message": "Calibration sample accepted",
                "sample_index": sample_index,
                "sample_count": count,
                "target_sample_count": int(self.config["target_sample_count"]),
                "warnings": warnings,
                "files": {"camera": str(original_path), "metadata": str(metadata_path)},
            }
        except BridgeError:
            raise
        except Exception as exc:
            LOGGER.exception("Unexpected calibration sample failure")
            raise BridgeError(
                "CALIBRATION_SAMPLE_FAILED",
                "Calibration sample acquisition failed.",
                status_code=500,
                details={"cause": str(exc)},
            ) from exc
        finally:
            self._operation_lock.release()

    def _write_dataset_index_locked(self) -> None:
        atomic_write_yaml(
            self.active_dir / "dataset.yaml",
            {
                "calibration_type": "eye_in_hand",
                "updated": utc_now_iso(),
                "sample_count": len(self._records),
                "samples": [
                    {
                        "sample": record["sample"],
                        "metadata": str(Path(record["sample_dir"]) / "sample.yaml"),
                        "camera_file": record["camera_file"],
                    }
                    for record in self._records
                ],
            },
        )

    def _ensure_inputs(self) -> list[Any]:
        with self._lock:
            records = list(self._records)
            inputs = list(self._inputs)
        for index, (record, existing) in enumerate(zip(records, inputs)):
            if existing is not None:
                continue
            camera_path = Path(record["camera_file"])
            hand_eye_input, frame = self.camera.load_calibration_observation(
                camera_path, self._record_matrix(record)
            )
            inputs[index] = hand_eye_input
            if frame is not None:
                self._frames.append(frame)
        with self._lock:
            self._inputs = inputs
        return [item for item in inputs if item is not None]

    def solve(self) -> dict[str, Any]:
        if not self._operation_lock.acquire(blocking=False):
            raise BridgeError("CALIBRATION_BUSY", "A calibration operation is already running.")
        try:
            inputs = self._ensure_inputs()
            if not inputs:
                raise BridgeError(
                    "CALIBRATION_NO_SAMPLES", "No active calibration samples are available."
                )
            LOGGER.info("Solving Eye-in-Hand calibration with %d samples", len(inputs))
            output = self.camera.solve_eye_in_hand(inputs)
            if isinstance(output, dict):
                valid = bool(output["valid"])
                status = str(output["status"])
                transform = output["transform"]
                residuals = output["residuals"]
            else:
                valid = bool(_call_or_value(output.valid))
                status = str(_call_or_value(output.status))
                if not valid:
                    raise BridgeError(
                        "CALIBRATION_SOLVE_INVALID",
                        "Zivid Eye-in-Hand calibration returned an invalid result.",
                        details={"status": status},
                    )
                transform = np.asarray(_call_or_value(output.transform), dtype=np.float64)
                residuals = [
                    {
                        "translation": float(_call_or_value(item.translation)),
                        "rotation": float(_call_or_value(item.rotation)),
                    }
                    for item in _call_or_value(output.residuals)
                ]
            if not valid:
                raise BridgeError(
                    "CALIBRATION_SOLVE_INVALID",
                    "Eye-in-Hand calibration returned an invalid result.",
                    details={"status": status},
                )
            payload = serialize_calibration_result(
                transform,
                residuals,
                sample_count=len(inputs),
                status=status,
                mock=self.camera.mode == "mock",
                synthetic=self.camera.mode == "mock",
                warning=(
                    MOCK_CALIBRATION_WARNING if self.camera.mode == "mock" else None
                ),
            )
            atomic_write_yaml(self.result_file, payload)
            LOGGER.info("Calibration solved; T_flange_camera saved to %s", self.result_file)
            return {
                "ok": True,
                "message": "Eye-in-Hand calibration solved",
                "sample_count": len(inputs),
                "status": status,
                "T_flange_camera": payload["transform"]["matrix"],
                "summary": payload["summary"],
                "residuals": payload["residuals"],
                "result_file": str(self.result_file),
            }
        except BridgeError:
            raise
        except Exception as exc:
            LOGGER.exception("Eye-in-Hand calibration solve failed")
            raise BridgeError(
                "CALIBRATION_SOLVE_FAILED",
                "Zivid Eye-in-Hand calibration solve failed.",
                status_code=422,
                details={"cause": str(exc)},
            ) from exc
        finally:
            self._operation_lock.release()

    def reset(self) -> dict[str, Any]:
        if not self._operation_lock.acquire(blocking=False):
            raise BridgeError("CALIBRATION_BUSY", "A calibration operation is already running.")
        try:
            timestamp = utc_now_iso().replace(":", "-").replace("+", "_")
            archive_target = self.archive_dir / f"dataset_{timestamp}"
            archive_target.mkdir(parents=True, exist_ok=False)
            archived_dataset: str | None = None
            if self.active_dir.exists() and any(self.active_dir.iterdir()):
                data_target = archive_target / "active"
                shutil.move(str(self.active_dir), str(data_target))
                archived_dataset = str(data_target)
            else:
                self.active_dir.mkdir(parents=True, exist_ok=True)
            if self.result_file.exists():
                shutil.move(str(self.result_file), str(archive_target / self.result_file.name))
            self.active_dir.mkdir(parents=True, exist_ok=True)
            with self._lock:
                previous_count = len(self._records)
                self._records.clear()
                self._inputs.clear()
                self._frames.clear()
            LOGGER.info("Calibration dataset reset and archived at %s", archive_target)
            return {
                "ok": True,
                "message": "Active calibration dataset reset and archived",
                "previous_sample_count": previous_count,
                "archive": str(archive_target),
                "archived_dataset": archived_dataset,
            }
        finally:
            self._operation_lock.release()

    def status(self) -> dict[str, Any]:
        with self._lock:
            sample_count = len(self._records)
        solved_payload: dict[str, Any] | None = None
        if self.result_file.is_file():
            try:
                solved_payload = load_yaml(self.result_file)
            except Exception:
                LOGGER.exception("Calibration result file exists but is invalid: %s", self.result_file)
        solved = (
            solved_payload is not None
            and int(solved_payload.get("sample_count", -1)) == sample_count
        )
        return {
            "sample_count": sample_count,
            "target_sample_count": int(self.config["target_sample_count"]),
            "solved": solved,
            "result_stale": solved_payload is not None and not solved,
            "result_file": str(self.result_file) if solved_payload is not None else None,
        }
