"""Multi-view scan sessions built from committed Base-frame captures."""

from __future__ import annotations

import logging
from pathlib import Path
import threading
from typing import Any

import numpy as np

from bridge_errors import BridgeError
from config_loader import configured_path
from pointcloud_capture import PointCloudCapture
from pointcloud_io import write_preview_xyz, write_preview_xyzrgb
from robot_udp_server import RobotUDPServer
from storage_utils import allocate_numbered_directory, atomic_write_yaml, load_yaml, utc_now_iso
from transform_utils import matrix_to_list, validate_transform
from zivid_camera_manager import ZividCameraManager


LOGGER = logging.getLogger(__name__)


class ScanSessionManager:
    """Own one in-memory active scan while keeping completed scans immutable on disk."""

    def __init__(
        self,
        config: dict[str, Any],
        robot: RobotUDPServer,
        camera: ZividCameraManager,
        capture: PointCloudCapture,
    ) -> None:
        self.root_config = config
        self.config = config["scan"]
        self.robot = robot
        self.camera = camera
        self.capture = capture
        self.scans_root = configured_path(config, "scans")
        self.scans_root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._operation_lock = threading.Lock()
        self._active: dict[str, Any] | None = None
        self._last_scan_id: str | None = None
        self._last_error: str | None = None

    @staticmethod
    def _import_open3d() -> Any:
        try:
            import open3d as o3d  # type: ignore[import-not-found]
        except Exception as exc:
            raise BridgeError(
                "SCAN_DEPENDENCY_MISSING",
                "Open3D is required for multi-view scan merge.",
                status_code=503,
                details={"cause": str(exc)},
            ) from exc
        return o3d

    def _ensure_enabled(self) -> None:
        if not bool(self.config.get("enabled", True)):
            raise BridgeError("SCAN_DISABLED", "Multi-view scan is disabled in configuration.", status_code=403)

    def _manifest_path(self, scan_dir: Path) -> Path:
        return scan_dir / "scan_manifest.yaml"

    def _write_manifest(self, active: dict[str, Any]) -> None:
        scan_dir = Path(active["scan_dir"])
        payload = {
            "schema_version": 1,
            "scan_id": active["scan_id"],
            "state": active["state"],
            "created_at": active["created_at"],
            "finished_at": active.get("finished_at"),
            "coordinate_frame": "base",
            "units": "mm",
            "calibration": {
                "type": "eye_in_hand",
                "transform_name": "T_flange_camera",
                "source_frame": "camera",
                "target_frame": "flange",
                "file": active.get("calibration_file"),
                "sha256": active.get("calibration_sha256"),
            },
            "capture_count": len(active["captures"]),
            "captures": active["captures"],
            "merge": active.get("merge", {"merged": False}),
            "artifacts": active.get("artifacts", {}),
            "warnings": active.get("warnings", []),
        }
        atomic_write_yaml(self._manifest_path(scan_dir), payload)

    def _active_summary_locked(self) -> dict[str, Any]:
        if self._active is None:
            return {
                "active": False,
                "scan_id": self._last_scan_id,
                "capture_count": 0,
                "captures": [],
                "merged": False,
                "last_error": self._last_error,
            }
        return {
            "active": True,
            "scan_id": self._active["scan_id"],
            "state": self._active["state"],
            "capture_count": len(self._active["captures"]),
            "captures": [item["capture_id"] for item in self._active["captures"]],
            "merged": bool(self._active.get("merge", {}).get("merged", False)),
            "last_error": self._last_error,
        }

    def status(self) -> dict[str, Any]:
        with self._lock:
            return self._active_summary_locked()

    def start(self) -> dict[str, Any]:
        self._ensure_enabled()
        with self._lock:
            if self._active is not None:
                raise BridgeError(
                    "SCAN_ALREADY_ACTIVE",
                    "A scan session is already active. Finish or reset it before starting another scan.",
                    details={"scan_id": self._active["scan_id"]},
                )
            scan_dir = allocate_numbered_directory(self.scans_root, "scan")
            (scan_dir / "captures").mkdir()
            self._active = {
                "scan_id": scan_dir.name,
                "scan_dir": str(scan_dir),
                "state": "active",
                "created_at": utc_now_iso(),
                "captures": [],
                "merge": {"merged": False},
                "artifacts": {},
                "warnings": [],
            }
            self._last_scan_id = scan_dir.name
            self._last_error = None
            self._write_manifest(self._active)
            LOGGER.info("SCAN START scan_id=%s", scan_dir.name)
            return {"ok": True, "scan_id": scan_dir.name, "state": "active", "capture_count": 0}

    def _precheck_capture(self) -> None:
        robot_status = self.robot.status()
        if not robot_status.get("connected"):
            raise BridgeError("ROBOT_NOT_CONNECTED", "Robot UDP stream is not connected.", status_code=503)
        stationary = self.robot.stationary_status()
        if not stationary.exists:
            raise BridgeError("NO_ROBOT_POSE", "No robot pose has been received.", status_code=503)
        if not stationary.fresh:
            raise BridgeError("ROBOT_POSE_STALE", "The latest robot pose is stale.", details=stationary.to_dict())
        if not stationary.stationary:
            raise BridgeError("ROBOT_NOT_STATIONARY", "Robot must be stationary before scan capture.", details=stationary.to_dict())
        camera_status = self.camera.status()
        if not camera_status.get("connected") and self.camera.mode != "mock":
            raise BridgeError("CAMERA_UNAVAILABLE", "Camera is not connected.", status_code=503, details=camera_status)

    def capture_once(self) -> dict[str, Any]:
        self._ensure_enabled()
        if not self._operation_lock.acquire(blocking=False):
            raise BridgeError("SCAN_BUSY", "A scan operation is already running.")
        try:
            with self._lock:
                if self._active is None:
                    raise BridgeError("SCAN_NOT_ACTIVE", "Start a scan before adding captures.", status_code=409)
                scan_id = self._active["scan_id"]
            self._precheck_capture()
            try:
                result = self.capture.capture()
            except BridgeError as exc:
                LOGGER.warning("SCAN CAPTURE REJECTED scan_id=%s error=%s", scan_id, exc.error_code)
                raise

            source_manifest = load_yaml(Path(result["files"]["manifest"]))
            source_metadata = load_yaml(Path(result["files"]["metadata"]))
            transforms = source_manifest.get("transforms", {})
            T_base_flange = validate_transform(transforms["T_base_flange"], name="T_base_flange")
            T_flange_camera = validate_transform(transforms["T_flange_camera"], name="T_flange_camera")
            T_base_camera = validate_transform(transforms["T_base_camera"], name="T_base_camera")

            with self._lock:
                if self._active is None:
                    raise BridgeError("SCAN_NOT_ACTIVE", "The active scan was reset during capture.")
                scan_dir = Path(self._active["scan_dir"])
                scan_capture_dir = allocate_numbered_directory(scan_dir / "captures", "capture")
                capture_id = scan_capture_dir.name
                stationary_info = source_manifest.get("stationary_during_capture", {})
                record = {
                    "capture_id": capture_id,
                    "source_capture_id": result["capture_id"],
                    "timestamp": source_metadata.get("timestamp"),
                    "robot_pose_timestamp": source_metadata.get("robot_pose_before", {}).get("robot_timestamp"),
                    "pose_age_ms": stationary_info.get("pose_age_ms"),
                    "stationary": bool(stationary_info.get("stationary", False)),
                    "T_base_flange": matrix_to_list(T_base_flange),
                    "T_flange_camera": matrix_to_list(T_flange_camera),
                    "T_base_camera": matrix_to_list(T_base_camera),
                    "valid_points": int(result["point_count"]),
                    "rgb_available": source_metadata.get("rgb_available", result["files"].get("preview_xyzrgb") is not None),
                    "source_frame": "camera",
                    "target_frame": "base",
                    "artifacts": {
                        "source_capture_dir": str(Path(result["files"]["manifest"]).parent),
                        "base_ply": result["files"]["ply"],
                        "preview_xyz": result["files"].get("preview_xyz"),
                        "preview_xyzrgb": result["files"].get("preview_xyzrgb"),
                        "metadata": result["files"]["metadata"],
                        "manifest": result["files"]["manifest"],
                    },
                }
                atomic_write_yaml(scan_capture_dir / "scan_capture.yaml", record)
                if self._active.get("calibration_file") is None:
                    calibration = source_manifest.get("calibration", {})
                    self._active["calibration_file"] = calibration.get("file")
                    self._active["calibration_sha256"] = calibration.get("sha256")
                self._active["captures"].append(record)
                self._active["merge"] = {"merged": False}
                self._write_manifest(self._active)
                LOGGER.info(
                    "SCAN CAPTURE OK scan_id=%s capture_id=%s valid_points=%d pose_age=%s stationary=%s",
                    self._active["scan_id"],
                    capture_id,
                    int(result["point_count"]),
                    record["pose_age_ms"],
                    record["stationary"],
                )
                return {
                    "ok": True,
                    "scan_id": self._active["scan_id"],
                    "capture_id": capture_id,
                    "source_capture_id": result["capture_id"],
                    "capture_count": len(self._active["captures"]),
                    "point_count": int(result["point_count"]),
                    "files": record["artifacts"],
                }
        except BridgeError as exc:
            with self._lock:
                self._last_error = exc.message
            raise
        finally:
            self._operation_lock.release()

    @staticmethod
    def _cloud_arrays(o3d: Any, path: Path) -> tuple[np.ndarray, np.ndarray | None]:
        cloud = o3d.io.read_point_cloud(str(path), remove_nan_points=False, remove_infinite_points=False)
        points = np.asarray(cloud.points, dtype=np.float64).reshape(-1, 3)
        colors = np.asarray(cloud.colors, dtype=np.float64)
        if colors.size == 0:
            return points, None
        colors = colors.reshape(-1, 3)
        if colors.shape[0] != points.shape[0]:
            raise BridgeError(
                "COLOR_DATA_INVALID",
                "PLY color count does not match point count.",
                status_code=422,
                details={"path": str(path), "points": len(points), "colors": len(colors)},
            )
        return points, np.clip(np.rint(colors * 255.0), 0, 255).astype(np.uint8)

    def _merge_active_locked(self, o3d: Any) -> dict[str, Any]:
        if self._active is None:
            raise BridgeError("SCAN_NOT_ACTIVE", "Start a scan before merging.", status_code=409)
        captures = list(self._active["captures"])
        if not captures:
            raise BridgeError("SCAN_EMPTY", "The active scan has no captures to merge.", status_code=422)
        scan_dir = Path(self._active["scan_dir"])
        merge_config = self.config["merge"]
        if bool(merge_config.get("icp", {}).get("enabled", False)):
            self._active.setdefault("warnings", []).append("ICP requested but not implemented; robot/hand-eye alignment was used.")

        xyz_parts: list[np.ndarray] = []
        rgb_parts: list[np.ndarray] = []
        any_color = False
        for record in captures:
            points, colors = self._cloud_arrays(o3d, Path(record["artifacts"]["base_ply"]))
            xyz_parts.append(points)
            if colors is not None:
                any_color = True
                rgb_parts.append(colors)
            elif any_color:
                raise BridgeError("COLOR_DATA_INVALID", "Cannot merge mixed colored and uncolored captures.", status_code=422)
        merged_xyz = np.concatenate(xyz_parts, axis=0)
        merged_rgb = np.concatenate(rgb_parts, axis=0) if any_color else None
        finite_mask = np.all(np.isfinite(merged_xyz), axis=1)
        merged_xyz = merged_xyz[finite_mask]
        if merged_rgb is not None:
            merged_rgb = merged_rgb[finite_mask]

        raw_cloud = o3d.geometry.PointCloud()
        raw_cloud.points = o3d.utility.Vector3dVector(merged_xyz.astype(np.float64, copy=False))
        if merged_rgb is not None:
            raw_cloud.colors = o3d.utility.Vector3dVector(merged_rgb.astype(np.float64) / 255.0)
        raw_path = scan_dir / "merged_cloud_raw.ply"
        if not o3d.io.write_point_cloud(str(raw_path), raw_cloud, write_ascii=False):
            raise IOError(f"Open3D failed to write {raw_path}")

        voxel_size = float(merge_config.get("voxel_size_mm", 0.0))
        working = raw_cloud.voxel_down_sample(voxel_size) if voxel_size > 0 else raw_cloud
        outlier_config = merge_config.get("remove_statistical_outlier", {})
        if bool(outlier_config.get("enabled", True)) and len(working.points) > int(outlier_config.get("nb_neighbors", 20)):
            working, _indices = working.remove_statistical_outlier(
                nb_neighbors=int(outlier_config.get("nb_neighbors", 20)),
                std_ratio=float(outlier_config.get("std_ratio", 2.0)),
            )
        downsampled_path = scan_dir / "merged_cloud_downsampled.ply"
        if not o3d.io.write_point_cloud(str(downsampled_path), working, write_ascii=False):
            raise IOError(f"Open3D failed to write {downsampled_path}")

        final_xyz = np.asarray(working.points, dtype=np.float64)
        final_colors = np.asarray(working.colors, dtype=np.float64)
        preview_limit = int(self.config.get("preview", {}).get("max_points", 100000))
        preview_xyz_path = scan_dir / "merged_preview.xyz"
        preview_xyz_count = write_preview_xyz(preview_xyz_path, final_xyz, preview_limit)
        preview_xyzrgb_path = None
        preview_xyzrgb_count = None
        if final_colors.size:
            final_rgb = np.clip(np.rint(final_colors.reshape(-1, 3) * 255.0), 0, 255).astype(np.uint8)
            preview_xyzrgb_path = scan_dir / "merged_preview.xyzrgb"
            preview_xyzrgb_count = write_preview_xyzrgb(preview_xyzrgb_path, final_xyz, final_rgb, preview_limit)

        merge_payload = {
            "merged": True,
            "timestamp": utc_now_iso(),
            "raw_point_count": int(len(merged_xyz)),
            "final_point_count": int(len(final_xyz)),
            "voxel_size_mm": voxel_size,
            "statistical_outlier": {
                "enabled": bool(outlier_config.get("enabled", True)),
                "nb_neighbors": int(outlier_config.get("nb_neighbors", 20)),
                "std_ratio": float(outlier_config.get("std_ratio", 2.0)),
            },
            "icp": {"enabled": bool(merge_config.get("icp", {}).get("enabled", False))},
            "preview_point_count": preview_xyz_count,
            "preview_xyzrgb_point_count": preview_xyzrgb_count,
        }
        artifacts = {
            "merged_raw": str(raw_path),
            "merged_downsampled": str(downsampled_path),
            "preview_xyz": str(preview_xyz_path),
            "preview_xyzrgb": None if preview_xyzrgb_path is None else str(preview_xyzrgb_path),
        }
        self._active["merge"] = merge_payload
        self._active["artifacts"] = artifacts
        self._write_manifest(self._active)
        LOGGER.info(
            "SCAN MERGE OK scan_id=%s capture_count=%d raw_points=%d final_points=%d voxel_size=%s output=%s",
            self._active["scan_id"],
            len(captures),
            int(len(merged_xyz)),
            int(len(final_xyz)),
            voxel_size,
            downsampled_path,
        )
        return {
            "ok": True,
            "scan_id": self._active["scan_id"],
            "capture_count": len(captures),
            "merge": merge_payload,
            "files": artifacts,
        }

    def merge(self) -> dict[str, Any]:
        self._ensure_enabled()
        if not self._operation_lock.acquire(blocking=False):
            raise BridgeError("SCAN_BUSY", "A scan operation is already running.")
        try:
            o3d = self._import_open3d()
            with self._lock:
                if self._active is None:
                    raise BridgeError("SCAN_NOT_ACTIVE", "Start a scan before merging.", status_code=409)
                LOGGER.info("SCAN MERGE START scan_id=%s capture_count=%d", self._active["scan_id"], len(self._active["captures"]))
                self._last_error = None
                return self._merge_active_locked(o3d)
        except BridgeError as exc:
            with self._lock:
                self._last_error = exc.message
            raise
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
            LOGGER.exception("SCAN MERGE FAILED")
            raise BridgeError("SCAN_MERGE_FAILED", "Multi-view scan merge failed.", status_code=500, details={"cause": str(exc)}) from exc
        finally:
            self._operation_lock.release()

    def finish(self) -> dict[str, Any]:
        self._ensure_enabled()
        if not self._operation_lock.acquire(blocking=False):
            raise BridgeError("SCAN_BUSY", "A scan operation is already running.")
        try:
            o3d = self._import_open3d()
            with self._lock:
                if self._active is None:
                    raise BridgeError("SCAN_NOT_ACTIVE", "Start a scan before finishing.", status_code=409)
                if not self._active.get("merge", {}).get("merged", False):
                    self._merge_active_locked(o3d)
                self._active["state"] = "completed"
                self._active["finished_at"] = utc_now_iso()
                self._write_manifest(self._active)
                scan_id = self._active["scan_id"]
                capture_count = len(self._active["captures"])
                files = dict(self._active.get("artifacts", {}))
                files["manifest"] = str(self._manifest_path(Path(self._active["scan_dir"])))
                self._last_scan_id = scan_id
                self._active = None
                LOGGER.info("SCAN FINISH scan_id=%s capture_count=%d", scan_id, capture_count)
                return {"ok": True, "scan_id": scan_id, "state": "completed", "capture_count": capture_count, "files": files}
        except BridgeError as exc:
            with self._lock:
                self._last_error = exc.message
            raise
        finally:
            self._operation_lock.release()

    def reset(self) -> dict[str, Any]:
        self._ensure_enabled()
        with self._lock:
            if self._active is None:
                return {"ok": True, "active": False, "message": "No active scan to reset."}
            scan_id = self._active["scan_id"]
            self._active["state"] = "reset"
            self._active["finished_at"] = utc_now_iso()
            self._write_manifest(self._active)
            self._active = None
            self._last_scan_id = scan_id
            LOGGER.info("SCAN RESET scan_id=%s", scan_id)
            return {"ok": True, "active": False, "scan_id": scan_id, "message": "Active scan reset; completed scans were not deleted."}
