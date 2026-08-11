"""Three-plane WCS extraction from a point cloud already in robot Base coordinates."""

from __future__ import annotations

import itertools
import logging
from pathlib import Path
import re
import threading
from typing import Any

import numpy as np

from bridge_errors import BridgeError
from config_loader import configured_path
from storage_utils import atomic_write_yaml, utc_now_iso
from transform_utils import (
    invert_transform,
    matrix_to_list,
    transform_points,
    validate_transform,
)


LOGGER = logging.getLogger(__name__)
PLANE_ROLES = ("top", "side", "end")
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def _unit(vector: Any, *, name: str) -> np.ndarray:
    value = np.asarray(vector, dtype=np.float64)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must be a finite XYZ vector.")
    length = float(np.linalg.norm(value))
    if length <= 1e-12:
        raise ValueError(f"{name} must be non-zero.")
    return value / length


def _normalized_plane(equation: Any) -> np.ndarray:
    value = np.asarray(equation, dtype=np.float64)
    if value.shape != (4,) or not np.all(np.isfinite(value)):
        raise ValueError("Plane equation must contain finite [a, b, c, d].")
    length = float(np.linalg.norm(value[:3]))
    if length <= 1e-12:
        raise ValueError("Plane normal must be non-zero.")
    return value / length


def classify_planes(
    planes: list[dict[str, Any]], expected_directions_base: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    """Assign three detected planes to roles using global direction agreement."""
    if len(planes) != 3:
        raise ValueError(f"Exactly three planes are required for classification, got {len(planes)}.")
    expected = {
        role: _unit(expected_directions_base[role], name=f"expected {role} direction")
        for role in PLANE_ROLES
    }
    equations = [_normalized_plane(plane["equation"]) for plane in planes]
    best_permutation: tuple[int, ...] | None = None
    best_score = -np.inf
    for permutation in itertools.permutations(range(3)):
        score = sum(
            abs(float(np.dot(equations[plane_index][:3], expected[role])))
            for role, plane_index in zip(PLANE_ROLES, permutation)
        )
        if score > best_score:
            best_score = score
            best_permutation = permutation
    assert best_permutation is not None

    classified: dict[str, dict[str, Any]] = {}
    for role, plane_index in zip(PLANE_ROLES, best_permutation):
        equation = equations[plane_index].copy()
        if float(np.dot(equation[:3], expected[role])) < 0:
            equation *= -1.0
        record = dict(planes[plane_index])
        record["equation"] = equation
        record["normal"] = equation[:3]
        record["expected_direction_alignment"] = float(
            np.dot(equation[:3], expected[role])
        )
        record["detection_index"] = plane_index + 1
        classified[role] = record
    return classified


def construct_wcs_from_planes(
    classified: dict[str, dict[str, Any]]
) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    """Return T_base_wcs/T_wcs_base using nearest-rotation SVD orthogonalization."""
    equations = {role: _normalized_plane(classified[role]["equation"]) for role in PLANE_ROLES}
    # WCS x/y/z nominally follow end/side/top plane normals respectively.
    measured_axes = np.column_stack(
        [equations["end"][:3], equations["side"][:3], equations["top"][:3]]
    )
    u, _, vt = np.linalg.svd(measured_axes)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1.0
        rotation = u @ vt

    coefficient_matrix = np.vstack(
        [equations["end"][:3], equations["side"][:3], equations["top"][:3]]
    )
    offsets = -np.array(
        [equations["end"][3], equations["side"][3], equations["top"][3]],
        dtype=np.float64,
    )
    condition_number = float(np.linalg.cond(coefficient_matrix))
    try:
        origin_base = np.linalg.solve(coefficient_matrix, offsets)
    except np.linalg.LinAlgError as exc:
        raise BridgeError(
            "WCS_PLANES_DEGENERATE",
            "The three classified planes do not have a stable intersection.",
            status_code=422,
            details={"condition_number": condition_number},
        ) from exc

    T_base_wcs = np.eye(4, dtype=np.float64)
    T_base_wcs[:3, :3] = rotation
    T_base_wcs[:3, 3] = origin_base
    T_base_wcs = validate_transform(T_base_wcs, name="T_base_wcs")
    T_wcs_base = invert_transform(T_base_wcs, name="T_base_wcs")

    normal_list = [equations[role][:3] for role in PLANE_ROLES]
    errors = []
    for first, second in itertools.combinations(normal_list, 2):
        angle = np.degrees(np.arccos(np.clip(abs(float(np.dot(first, second))), -1.0, 1.0)))
        errors.append(abs(90.0 - float(angle)))
    quality = {
        "orthogonality_error_deg": max(errors, default=0.0),
        "plane_intersection_condition_number": condition_number,
    }
    return T_base_wcs, T_wcs_base, quality


class WCSPointCloudProcessor:
    def __init__(self, config: dict[str, Any]) -> None:
        self.root_config = config
        self.config = config["wcs"]
        self.captures_root = configured_path(config, "captures")
        self._operation_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._busy = False
        self._last_capture_id: str | None = None
        self._last_error: str | None = None

    def _capture_directory(self, capture_id: str | None) -> Path:
        if capture_id is not None:
            if re.fullmatch(r"capture_\d{4,}", capture_id) is None:
                raise BridgeError(
                    "CAPTURE_ID_INVALID",
                    "capture_id must use the form capture_0001.",
                    status_code=422,
                )
            directory = self.captures_root / capture_id
            if not directory.is_dir():
                raise BridgeError(
                    "CAPTURE_NOT_FOUND", f"Capture directory not found: {capture_id}.", status_code=404
                )
            return directory
        candidates = [
            child
            for child in self.captures_root.glob("capture_[0-9][0-9][0-9][0-9]*")
            if child.is_dir()
        ]
        if not candidates:
            raise BridgeError("CAPTURE_NOT_FOUND", "No production capture is available.", status_code=404)
        return max(candidates, key=lambda path: int(path.name.rsplit("_", 1)[1]))

    @staticmethod
    def _import_open3d() -> Any:
        try:
            import open3d as o3d  # type: ignore[import-not-found]
        except Exception as exc:
            raise BridgeError(
                "WCS_DEPENDENCY_MISSING",
                "Open3D is required for WCS plane processing. Use the documented Python 3.12 environment.",
                status_code=503,
                details={"cause": str(exc)},
            ) from exc
        return o3d

    def compute(self, capture_id: str | None = None) -> dict[str, Any]:
        if not self._operation_lock.acquire(blocking=False):
            raise BridgeError("WCS_BUSY", "WCS processing is already running.")
        with self._state_lock:
            self._busy = True
            self._last_error = None
        try:
            o3d = self._import_open3d()
            capture_dir = self._capture_directory(capture_id)
            source_path = capture_dir / "transformed_cloud.ply"
            if not source_path.is_file():
                raise BridgeError(
                    "TRANSFORMED_CLOUD_NOT_FOUND",
                    "The selected capture has no transformed_cloud.ply.",
                    status_code=404,
                    details={"capture_id": capture_dir.name},
                )
            LOGGER.info("WCS computation started for %s", capture_dir.name)
            cloud = o3d.io.read_point_cloud(str(source_path), remove_nan_points=True, remove_infinite_points=True)
            cloud = cloud.remove_non_finite_points()
            if len(cloud.points) == 0:
                raise BridgeError("WCS_EMPTY_CLOUD", "The transformed point cloud contains no finite points.")

            crop_config = self.config["crop"]
            if bool(crop_config["enabled"]):
                crop_box = o3d.geometry.AxisAlignedBoundingBox(
                    min_bound=np.asarray(crop_config["min_base_mm"], dtype=np.float64),
                    max_bound=np.asarray(crop_config["max_base_mm"], dtype=np.float64),
                )
                cropped = cloud.crop(crop_box)
            else:
                cropped = cloud
            if len(cropped.points) == 0:
                raise BridgeError("WCS_EMPTY_CROP", "The configured crop contains no finite points.")

            cropped_path = capture_dir / "cropped_cloud.ply"
            if not o3d.io.write_point_cloud(str(cropped_path), cropped, write_ascii=False):
                raise IOError(f"Open3D failed to write {cropped_path}")

            plane_config = self.config["plane_detection"]
            voxel_size = float(plane_config["voxel_size_mm"])
            working = cropped.voxel_down_sample(voxel_size) if voxel_size > 0 else cropped
            detected: list[dict[str, Any]] = []
            for detection_index in range(1, 4):
                minimum = max(int(plane_config["ransac_n"]), int(plane_config["min_inlier_count"]))
                if len(working.points) < minimum:
                    raise BridgeError(
                        "WCS_INSUFFICIENT_PLANE_POINTS",
                        f"Insufficient points remain for major plane {detection_index}.",
                        status_code=422,
                        details={"remaining_point_count": len(working.points)},
                    )
                model, inliers = working.segment_plane(
                    distance_threshold=float(plane_config["distance_threshold_mm"]),
                    ransac_n=int(plane_config["ransac_n"]),
                    num_iterations=int(plane_config["num_iterations"]),
                )
                if len(inliers) < int(plane_config["min_inlier_count"]):
                    raise BridgeError(
                        "WCS_PLANE_TOO_SMALL",
                        f"Detected major plane {detection_index} has too few inliers.",
                        status_code=422,
                        details={"inlier_count": len(inliers)},
                    )
                equation = _normalized_plane(model)
                inlier_points = np.asarray(working.points)[np.asarray(inliers, dtype=np.int64)]
                distances = np.abs(inlier_points @ equation[:3] + equation[3])
                detected.append(
                    {
                        "equation": equation,
                        "normal": equation[:3],
                        "inlier_count": int(len(inliers)),
                        "rmse_mm": float(np.sqrt(np.mean(np.square(distances)))),
                    }
                )
                working = working.select_by_index(inliers, invert=True)

            classified = classify_planes(detected, self.config["expected_directions_base"])
            T_base_wcs, T_wcs_base, quality = construct_wcs_from_planes(classified)
            points_base = np.asarray(cropped.points, dtype=np.float64)
            points_wcs = transform_points(T_wcs_base, points_base)
            extents = np.ptp(points_wcs, axis=0)
            dimensions = {
                dimension: float(extents[AXIS_INDEX[str(axis).lower()]])
                for dimension, axis in self.config["dimensions_axis"].items()
            }
            output_file = capture_dir / "output_parameters.yaml"
            payload = {
                "capture_id": capture_dir.name,
                "timestamp": utc_now_iso(),
                "source_cloud": str(source_path),
                "cropped_cloud": str(cropped_path),
                "workpiece": {
                    "origin_base_mm": T_base_wcs[:3, 3].tolist(),
                    "axes_base": {
                        "x": T_base_wcs[:3, 0].tolist(),
                        "y": T_base_wcs[:3, 1].tolist(),
                        "z": T_base_wcs[:3, 2].tolist(),
                    },
                    "dimensions_mm": dimensions,
                },
                "transform": {
                    "T_base_wcs": matrix_to_list(T_base_wcs),
                    "T_wcs_base": matrix_to_list(T_wcs_base),
                },
                "planes": {
                    role: {
                        "equation": np.asarray(classified[role]["equation"]).tolist(),
                        "normal": np.asarray(classified[role]["normal"]).tolist(),
                        "inlier_count": int(classified[role]["inlier_count"]),
                        "rmse_mm": float(classified[role]["rmse_mm"]),
                        "detection_index": int(classified[role]["detection_index"]),
                        "expected_direction_alignment": float(
                            classified[role]["expected_direction_alignment"]
                        ),
                    }
                    for role in PLANE_ROLES
                },
                "quality": {**quality, "warnings": []},
                "point_counts": {
                    "cropped": int(len(cropped.points)),
                    "plane_detection_input": int(len(cropped.voxel_down_sample(voxel_size).points))
                    if voxel_size > 0
                    else int(len(cropped.points)),
                },
            }
            atomic_write_yaml(output_file, payload)
            with self._state_lock:
                self._last_capture_id = capture_dir.name
            LOGGER.info("WCS result for %s saved to %s", capture_dir.name, output_file)
            return {
                "ok": True,
                "message": "WCS computation complete",
                "capture_id": capture_dir.name,
                "workpiece": payload["workpiece"],
                "transform": payload["transform"],
                "quality": payload["quality"],
                "planes": payload["planes"],
                "files": {
                    "parameters": str(output_file),
                    "cropped_ply": str(cropped_path),
                    "source_ply": str(source_path),
                },
            }
        except BridgeError as exc:
            with self._state_lock:
                self._last_error = exc.message
            raise
        except Exception as exc:
            with self._state_lock:
                self._last_error = str(exc)
            LOGGER.exception("WCS computation failed")
            raise BridgeError(
                "WCS_COMPUTE_FAILED",
                "WCS point-cloud processing failed.",
                status_code=500,
                details={"cause": str(exc)},
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
