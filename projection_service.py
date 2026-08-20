"""Deviation-map projection service using current robot pose and Zivid projector."""

from __future__ import annotations

from dataclasses import dataclass
import logging
from pathlib import Path
import struct
import threading
from typing import Any
import zlib

import numpy as np
from numpy.typing import ArrayLike, NDArray

from bridge_errors import BridgeError
from config_loader import configured_path
from pointcloud_capture import _sha256_file, load_T_flange_camera
from robot_udp_server import RobotUDPServer
from storage_utils import allocate_numbered_directory, atomic_write_via_temp, atomic_write_yaml, utc_now_iso
from transform_utils import compose_transforms, invert_transform, matrix_to_list, transform_points, validate_transform
from zivid_camera_manager import ZividCameraManager


LOGGER = logging.getLogger(__name__)


LABEL_GOOD = "GOOD"
LABEL_WARNING = "WARNING"
LABEL_BAD = "BAD"
LABEL_NEGATIVE = "NEGATIVE"
LABEL_ZERO = "ZERO"
LABEL_POSITIVE = "POSITIVE"

# Projector images are BGRA, not RGB/RGBA.
BGRA_BLACK = np.array([0, 0, 0, 255], dtype=np.uint8)
BGRA_GREEN = np.array([0, 255, 0, 255], dtype=np.uint8)
BGRA_BLUE = np.array([255, 0, 0, 255], dtype=np.uint8)
BGRA_RED = np.array([0, 0, 255, 255], dtype=np.uint8)
BGRA_YELLOW = np.array([0, 255, 255, 255], dtype=np.uint8)


@dataclass(slots=True)
class ProjectionRenderResult:
    image_bgra: NDArray[np.uint8]
    input_points: int
    finite_points: int
    sampled_points: int
    valid_camera_points: int
    visible_projector_points: int
    T_base_camera: NDArray[np.float64]
    T_flange_camera: NDArray[np.float64]
    T_base_flange: NDArray[np.float64]
    stationary: dict[str, Any]
    labels: list[str]
    projector_width: int
    projector_height: int


def filter_finite_points_and_deviations(
    points_base: ArrayLike,
    deviations_mm: ArrayLike,
) -> tuple[NDArray[np.float64], NDArray[np.float64], int]:
    points = np.asarray(points_base, dtype=np.float64)
    deviations = np.asarray(deviations_mm, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise BridgeError(
            "PROJECTION_POINTS_INVALID",
            "points_base must be an array of [X, Y, Z] rows in Robot Base coordinates.",
            status_code=422,
            details={"shape": list(points.shape)},
        )
    if deviations.ndim != 1:
        deviations = deviations.reshape(-1)
    if len(points) != len(deviations):
        raise BridgeError(
            "PROJECTION_INPUT_LENGTH_MISMATCH",
            "points_base and deviations_mm must have the same length.",
            status_code=422,
            details={"points": len(points), "deviations": len(deviations)},
        )
    if len(points) == 0:
        raise BridgeError("PROJECTION_EMPTY", "At least one deviation point is required.", status_code=422)
    finite_mask = np.all(np.isfinite(points), axis=1) & np.isfinite(deviations)
    filtered_points = points[finite_mask]
    filtered_deviations = deviations[finite_mask]
    if len(filtered_points) == 0:
        raise BridgeError("PROJECTION_NO_VALID_POINTS", "No finite XYZ/deviation rows remain after filtering.", status_code=422)
    return filtered_points, filtered_deviations, int(len(points))


def classify_deviations(
    deviations_mm: ArrayLike,
    *,
    mode: str,
    good_tolerance_mm: float,
    warning_tolerance_mm: float,
) -> list[str]:
    deviations = np.asarray(deviations_mm, dtype=np.float64).reshape(-1)
    if good_tolerance_mm < 0.0 or warning_tolerance_mm < 0.0:
        raise BridgeError("PROJECTION_TOLERANCE_INVALID", "Deviation tolerances must be non-negative.", status_code=422)
    if warning_tolerance_mm < good_tolerance_mm:
        raise BridgeError(
            "PROJECTION_TOLERANCE_INVALID",
            "warning_tolerance_mm must be greater than or equal to good_tolerance_mm.",
            status_code=422,
        )
    normalized = str(mode).lower()
    if normalized == "absolute":
        absolute = np.abs(deviations)
        labels = np.full(len(deviations), LABEL_BAD, dtype=object)
        labels[absolute <= warning_tolerance_mm] = LABEL_WARNING
        labels[absolute <= good_tolerance_mm] = LABEL_GOOD
        return [str(item) for item in labels]
    if normalized == "signed":
        labels = np.full(len(deviations), LABEL_ZERO, dtype=object)
        labels[deviations < -good_tolerance_mm] = LABEL_NEGATIVE
        labels[deviations > good_tolerance_mm] = LABEL_POSITIVE
        return [str(item) for item in labels]
    raise BridgeError(
        "PROJECTION_MODE_INVALID",
        "Projection mode must be absolute or signed.",
        status_code=422,
        details={"mode": mode},
    )


def colors_bgra_for_labels(labels: list[str], *, palette: str, opacity: int) -> NDArray[np.uint8]:
    alpha = int(opacity)
    if alpha < 0 or alpha > 255:
        raise BridgeError("PROJECTION_OPACITY_INVALID", "opacity must be in 0..255.", status_code=422)
    normalized = str(palette).lower()
    colors = np.empty((len(labels), 4), dtype=np.uint8)
    if normalized == "pure_rgb":
        mapping = {
            LABEL_GOOD: BGRA_GREEN,
            LABEL_WARNING: BGRA_BLUE,
            LABEL_BAD: BGRA_RED,
            LABEL_NEGATIVE: BGRA_BLUE,
            LABEL_ZERO: BGRA_GREEN,
            LABEL_POSITIVE: BGRA_RED,
        }
    elif normalized == "semantic":
        mapping = {
            LABEL_GOOD: BGRA_GREEN,
            LABEL_WARNING: BGRA_YELLOW,
            LABEL_BAD: BGRA_RED,
            LABEL_NEGATIVE: BGRA_BLUE,
            LABEL_ZERO: BGRA_GREEN,
            LABEL_POSITIVE: BGRA_RED,
        }
    else:
        raise BridgeError(
            "PROJECTION_PALETTE_INVALID",
            "Projection palette must be pure_rgb or semantic.",
            status_code=422,
            details={"palette": palette},
        )
    for index, label in enumerate(labels):
        colors[index] = mapping[label]
        colors[index, 3] = alpha
    return colors


def points_base_to_camera(points_base: ArrayLike, T_base_camera: ArrayLike) -> NDArray[np.float64]:
    T_camera_base = invert_transform(T_base_camera, name="T_base_camera")
    return transform_points(T_camera_base, points_base)


def _deterministic_take(indices: NDArray[np.int64], count: int) -> NDArray[np.int64]:
    if count <= 0 or len(indices) == 0:
        return np.empty((0,), dtype=np.int64)
    if len(indices) <= count:
        return indices
    selected = np.linspace(0, len(indices) - 1, num=count, dtype=np.int64)
    return indices[selected]


def sample_deviation_points(
    points: NDArray[np.float64],
    deviations: NDArray[np.float64],
    labels: list[str],
    max_points: int,
) -> tuple[NDArray[np.float64], NDArray[np.float64], list[str]]:
    if max_points <= 0:
        raise BridgeError("PROJECTION_MAX_POINTS_INVALID", "projection.deviation.max_points must be positive.", status_code=422)
    if len(points) <= max_points:
        return points, deviations, labels
    label_array = np.asarray(labels, dtype=object)
    selected_parts: list[NDArray[np.int64]] = []
    remaining = max_points
    for wanted in ([LABEL_BAD, LABEL_POSITIVE, LABEL_NEGATIVE], [LABEL_WARNING], [LABEL_GOOD, LABEL_ZERO]):
        mask = np.isin(label_array, wanted)
        candidates = np.flatnonzero(mask).astype(np.int64)
        take = _deterministic_take(candidates, remaining)
        selected_parts.append(take)
        remaining -= len(take)
        if remaining <= 0:
            break
    selected = np.concatenate(selected_parts) if selected_parts else np.empty((0,), dtype=np.int64)
    selected = np.sort(selected[:max_points])
    return points[selected], deviations[selected], [labels[int(index)] for index in selected]


def _resolution_to_width_height(resolution: Any) -> tuple[int, int]:
    width = getattr(resolution, "width", None)
    height = getattr(resolution, "height", None)
    if width is not None and height is not None:
        return int(width), int(height)
    if isinstance(resolution, dict):
        return int(resolution["width"]), int(resolution["height"])
    if isinstance(resolution, (tuple, list)) and len(resolution) == 2:
        return int(resolution[0]), int(resolution[1])
    array = np.asarray(resolution).reshape(-1)
    if len(array) == 2:
        return int(array[0]), int(array[1])
    raise BridgeError("PROJECTOR_RESOLUTION_INVALID", "Could not determine projector width and height.", status_code=503)


def _pixels_to_array(projector_pixels: Any) -> NDArray[np.float64]:
    array = np.asarray(projector_pixels)
    if array.dtype.names:
        names = array.dtype.names
        first = "u" if "u" in names else ("x" if "x" in names else names[0])
        second = "v" if "v" in names else ("y" if "y" in names else names[1])
        return np.column_stack([array[first], array[second]]).astype(np.float64)
    array = array.astype(np.float64)
    if array.ndim != 2 or array.shape[1] < 2:
        raise BridgeError(
            "PROJECTOR_PIXELS_INVALID",
            "Zivid projector pixel mapping must return an Nx2-like array.",
            status_code=503,
            details={"shape": list(array.shape)},
        )
    return array[:, :2]


def filter_projector_visible(
    projector_pixels: ArrayLike,
    camera_z: ArrayLike,
    width: int,
    height: int,
) -> tuple[NDArray[np.int64], NDArray[np.int64], NDArray[np.float64], NDArray[np.bool_]]:
    pixels = np.asarray(projector_pixels, dtype=np.float64).reshape(-1, 2)
    z_values = np.asarray(camera_z, dtype=np.float64).reshape(-1)
    if len(pixels) != len(z_values):
        raise ValueError("projector pixel and camera Z counts do not match")
    finite = np.all(np.isfinite(pixels), axis=1) & np.isfinite(z_values)
    inside_float = finite & (pixels[:, 0] >= 0.0) & (pixels[:, 0] < width) & (pixels[:, 1] >= 0.0) & (pixels[:, 1] < height)
    rounded = np.rint(pixels).astype(np.int64)
    inside_pixel = inside_float & (rounded[:, 0] >= 0) & (rounded[:, 0] < width) & (rounded[:, 1] >= 0) & (rounded[:, 1] < height)
    return rounded[inside_pixel, 0], rounded[inside_pixel, 1], z_values[inside_pixel], inside_pixel


def rasterize_bgra_zbuffer(
    width: int,
    height: int,
    u: ArrayLike,
    v: ArrayLike,
    camera_z: ArrayLike,
    colors_bgra: ArrayLike,
    point_radius_px: int,
    opacity: int,
) -> NDArray[np.uint8]:
    radius = int(point_radius_px)
    if radius < 0:
        raise BridgeError("PROJECTION_RADIUS_INVALID", "point_radius_px must be non-negative.", status_code=422)
    image = np.empty((height, width, 4), dtype=np.uint8)
    image[:, :] = BGRA_BLACK
    # Background stays opaque black; opacity is applied to deviation point colors.
    depth = np.full((height, width), np.inf, dtype=np.float64)
    centers_u = np.asarray(u, dtype=np.int64).reshape(-1)
    centers_v = np.asarray(v, dtype=np.int64).reshape(-1)
    z_values = np.asarray(camera_z, dtype=np.float64).reshape(-1)
    colors = np.asarray(colors_bgra, dtype=np.uint8).reshape(-1, 4)
    offsets = [(0, 0)]
    if radius > 0:
        offsets = [
            (dx, dy)
            for dy in range(-radius, radius + 1)
            for dx in range(-radius, radius + 1)
            if dx * dx + dy * dy <= radius * radius
        ]
    for center_u, center_v, z_value, color in zip(centers_u, centers_v, z_values, colors):
        for dx, dy in offsets:
            x = int(center_u + dx)
            y = int(center_v + dy)
            if x < 0 or x >= width or y < 0 or y >= height:
                continue
            if z_value < depth[y, x]:
                depth[y, x] = z_value
                image[y, x] = color
    return image


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)


def write_png_from_bgra(path: Path, image_bgra: NDArray[np.uint8]) -> None:
    image = np.asarray(image_bgra, dtype=np.uint8)
    if image.ndim != 3 or image.shape[2] != 4:
        raise ValueError("image_bgra must have shape H x W x 4")
    height, width, _channels = image.shape
    rgba = image[:, :, [2, 1, 0, 3]]
    raw = b"".join(b"\x00" + rgba[row].tobytes() for row in range(height))
    payload = b"\x89PNG\r\n\x1a\n"
    payload += _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
    payload += _png_chunk(b"IDAT", zlib.compress(raw))
    payload += _png_chunk(b"IEND", b"")

    def _write(target: Path) -> None:
        target.write_bytes(payload)

    atomic_write_via_temp(path, _write)


class ProjectionService:
    """Own deviation-map projection state and the Zivid projection handle."""

    def __init__(self, config: dict[str, Any], robot: RobotUDPServer, camera: ZividCameraManager) -> None:
        self.root_config = config
        self.config = config["projection"]
        self.robot = robot
        self.camera = camera
        self.result_file = configured_path(config, "calibration_result")
        self.projections_root = configured_path(config, "projections")
        self.projections_root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._projection_handle: Any | None = None
        self._active = False
        self._projection_id: str | None = None
        self._projection_type: str | None = None
        self._image_path: str | None = None
        self._projected_points = 0
        self._last_error: str | None = None
        self._last_stop_reason: str | None = None

    def _ensure_enabled(self) -> None:
        if not bool(self.config.get("enabled", True)):
            raise BridgeError("PROJECTION_DISABLED", "Projection is disabled in configuration.", status_code=403)

    def _camera_projection_resources(self) -> tuple[Any, Any]:
        if hasattr(self.camera, "projection_resources"):
            return self.camera.projection_resources()  # type: ignore[no-any-return]
        raise BridgeError("CAMERA_PROJECTION_UNAVAILABLE", "Camera manager does not expose projection resources.", status_code=503)

    def _load_calibration(self) -> NDArray[np.float64]:
        camera_status = self.camera.status()
        T_flange_camera = load_T_flange_camera(
            self.result_file,
            camera_mode=self.camera.mode,
            camera_serial_number=camera_status.get("serial_number"),
        )
        if np.allclose(T_flange_camera, np.eye(4), atol=1e-9, rtol=0.0):
            raise BridgeError(
                "IDENTITY_CALIBRATION_REJECTED",
                "Projection requires an explicit non-identity T_flange_camera calibration.",
                status_code=422,
                details={"result_file": str(self.result_file)},
            )
        return T_flange_camera

    def _current_pose_and_transform(self) -> tuple[Any, NDArray[np.float64], NDArray[np.float64]]:
        if bool(self.config.get("require_fresh_pose", True)) or bool(self.config.get("require_robot_stationary", True)):
            pose = self.robot.require_stationary_pose()
        else:
            pose = self.robot.latest_pose()
        if pose is None or pose.T_base_flange is None:
            raise BridgeError("NO_VALID_ROBOT_POSE", "No valid current T_base_flange is available.", status_code=503)
        robot_status = self.robot.status()
        if not robot_status.get("connected"):
            raise BridgeError("ROBOT_NOT_CONNECTED", "Robot UDP stream is not connected.", status_code=503)
        if bool(self.config.get("require_fresh_pose", True)) and not robot_status.get("pose_valid"):
            raise BridgeError("ROBOT_POSE_STALE", "The latest robot pose is not fresh enough for projection.", details=robot_status)
        if bool(self.config.get("require_robot_stationary", True)) and not robot_status.get("stationary"):
            raise BridgeError("ROBOT_NOT_STATIONARY", "Robot must be stationary before projection starts.", details=robot_status)
        T_flange_camera = self._load_calibration()
        T_base_camera = compose_transforms(
            pose.T_base_flange,
            T_flange_camera,
            target_intermediate_name="T_base_flange_current",
            intermediate_source_name="T_flange_camera",
        )
        return pose, T_flange_camera, T_base_camera

    def _request_value(self, request: dict[str, Any], key: str, default: Any) -> Any:
        value = request.get(key, None)
        return default if value is None else value

    def _render_deviation(self, request: dict[str, Any], *, projection_api: Any, camera_object: Any) -> ProjectionRenderResult:
        deviation_config = self.config["deviation"]
        mode = str(self._request_value(request, "mode", deviation_config["mode"])).lower()
        good = float(self._request_value(request, "good_tolerance_mm", deviation_config["good_tolerance_mm"]))
        warning = float(self._request_value(request, "warning_tolerance_mm", deviation_config["warning_tolerance_mm"]))
        palette = str(self._request_value(request, "palette", deviation_config["palette"])).lower()
        radius = int(self._request_value(request, "point_radius_px", deviation_config["point_radius_px"]))
        opacity = int(self._request_value(request, "opacity", 255))
        max_points = int(deviation_config["max_points"])

        points, deviations, input_count = filter_finite_points_and_deviations(request["points_base"], request["deviations_mm"])
        finite_count = len(points)
        labels = classify_deviations(deviations, mode=mode, good_tolerance_mm=good, warning_tolerance_mm=warning)
        points, deviations, labels = sample_deviation_points(points, deviations, labels, max_points)
        sampled_count = len(points)
        colors = colors_bgra_for_labels(labels, palette=palette, opacity=opacity)
        pose, T_flange_camera, T_base_camera = self._current_pose_and_transform()
        points_camera = points_base_to_camera(points, T_base_camera)
        in_front_mask = np.isfinite(points_camera[:, 2]) & (points_camera[:, 2] > 0.0) & np.all(np.isfinite(points_camera), axis=1)
        points_camera = points_camera[in_front_mask]
        colors = colors[in_front_mask]
        labels = [label for label, keep in zip(labels, in_front_mask) if bool(keep)]

        resolution = projection_api.projector_resolution(camera_object)
        width, height = _resolution_to_width_height(resolution)
        if width <= 0 or height <= 0:
            raise BridgeError("PROJECTOR_RESOLUTION_INVALID", "Projector resolution must be positive.", status_code=503)
        if len(points_camera) == 0:
            image = rasterize_bgra_zbuffer(width, height, [], [], [], np.empty((0, 4), dtype=np.uint8), radius, opacity)
            return ProjectionRenderResult(
                image_bgra=image,
                input_points=input_count,
                finite_points=finite_count,
                sampled_points=sampled_count,
                valid_camera_points=0,
                visible_projector_points=0,
                T_base_camera=T_base_camera,
                T_flange_camera=T_flange_camera,
                T_base_flange=validate_transform(pose.T_base_flange, name="T_base_flange_current"),
                stationary=self.robot.stationary_status().to_dict(),
                labels=labels,
                projector_width=width,
                projector_height=height,
            )

        projector_pixels = _pixels_to_array(projection_api.pixels_from_3d_points(camera_object, points_camera))
        u, v, z, visible_mask = filter_projector_visible(projector_pixels, points_camera[:, 2], width, height)
        visible_colors = colors[visible_mask]
        visible_labels = [label for label, keep in zip(labels, visible_mask) if bool(keep)]
        image = rasterize_bgra_zbuffer(width, height, u, v, z, visible_colors, radius, opacity)
        return ProjectionRenderResult(
            image_bgra=image,
            input_points=input_count,
            finite_points=finite_count,
            sampled_points=sampled_count,
            valid_camera_points=len(points_camera),
            visible_projector_points=len(u),
            T_base_camera=T_base_camera,
            T_flange_camera=T_flange_camera,
            T_base_flange=validate_transform(pose.T_base_flange, name="T_base_flange_current"),
            stationary=self.robot.stationary_status().to_dict(),
            labels=visible_labels,
            projector_width=width,
            projector_height=height,
        )

    def _manifest_payload(
        self,
        *,
        projection_id: str,
        request: dict[str, Any],
        render: ProjectionRenderResult,
        image_path: Path,
    ) -> dict[str, Any]:
        deviation_config = self.config["deviation"]
        return {
            "schema_version": 1,
            "projection_id": projection_id,
            "type": "deviation_map",
            "created_at": utc_now_iso(),
            "coordinate_frame_input": "base",
            "units": "mm",
            "robot": {
                "stationary": bool(render.stationary.get("stationary", False)),
                "pose_age_ms": render.stationary.get("pose_age_ms"),
                "T_base_flange": matrix_to_list(render.T_base_flange),
            },
            "calibration": {
                "transform_name": "T_flange_camera",
                "file": str(self.result_file),
                "sha256": _sha256_file(self.result_file),
                "T_flange_camera": matrix_to_list(render.T_flange_camera),
            },
            "T_base_camera": matrix_to_list(render.T_base_camera),
            "deviation": {
                "mode": str(self._request_value(request, "mode", deviation_config["mode"])).lower(),
                "good_tolerance_mm": float(self._request_value(request, "good_tolerance_mm", deviation_config["good_tolerance_mm"])),
                "warning_tolerance_mm": float(self._request_value(request, "warning_tolerance_mm", deviation_config["warning_tolerance_mm"])),
                "palette": str(self._request_value(request, "palette", deviation_config["palette"])).lower(),
            },
            "points": {
                "input": render.input_points,
                "finite": render.finite_points,
                "sampled": render.sampled_points,
                "valid_camera": render.valid_camera_points,
                "projector_visible": render.visible_projector_points,
            },
            "projector": {
                "width": render.projector_width,
                "height": render.projector_height,
                "point_radius_px": int(self._request_value(request, "point_radius_px", deviation_config["point_radius_px"])),
            },
            "artifacts": {
                "image": image_path.name,
            },
        }

    def project_deviation(self, request: dict[str, Any]) -> dict[str, Any]:
        self._ensure_enabled()
        with self._lock:
            self.stop(reason="replace_projection")
            try:
                projection_api, camera_object = self._camera_projection_resources()
                render = self._render_deviation(request, projection_api=projection_api, camera_object=camera_object)
                projection_dir = allocate_numbered_directory(self.projections_root, "projection")
                projection_id = projection_dir.name
                image_path = projection_dir / "projector_image.png"
                write_png_from_bgra(image_path, render.image_bgra)
                atomic_write_yaml(
                    projection_dir / "projection_manifest.yaml",
                    self._manifest_payload(
                        projection_id=projection_id,
                        request=request,
                        render=render,
                        image_path=image_path,
                    ),
                )
                self._projection_handle = projection_api.show_image_bgra(camera_object, render.image_bgra)
                self._active = True
                self._projection_id = projection_id
                self._projection_type = "deviation_map"
                self._image_path = str(image_path)
                self._projected_points = render.visible_projector_points
                self._last_error = None
                self._last_stop_reason = None
                LOGGER.info("Projection %s started with %d visible points", projection_id, render.visible_projector_points)
                return {
                    "ok": True,
                    "projection_id": projection_id,
                    "active": True,
                    "input_points": render.input_points,
                    "valid_camera_points": render.valid_camera_points,
                    "visible_projector_points": render.visible_projector_points,
                    "image_path": str(image_path),
                    "T_base_camera": matrix_to_list(render.T_base_camera),
                }
            except BridgeError as exc:
                self._last_error = exc.message
                raise
            except Exception as exc:
                self._last_error = str(exc)
                LOGGER.exception("Deviation projection failed")
                raise BridgeError(
                    "PROJECTION_FAILED",
                    "Deviation projection failed.",
                    status_code=500,
                    details={"cause": str(exc)},
                ) from exc

    def stop(self, *, reason: str = "manual") -> dict[str, Any]:
        with self._lock:
            handle = self._projection_handle
            if handle is not None:
                try:
                    stop = getattr(handle, "stop", None)
                    if callable(stop):
                        stop()
                except Exception as exc:
                    self._last_error = str(exc)
                    LOGGER.exception("Projection handle stop failed")
                    raise BridgeError("PROJECTION_STOP_FAILED", "Projection stop failed.", status_code=500, details={"cause": str(exc)}) from exc
            was_active = self._active
            self._projection_handle = None
            self._active = False
            self._last_stop_reason = reason
            if was_active:
                LOGGER.info("Projection stopped: reason=%s projection_id=%s", reason, self._projection_id)
            return {"ok": True, "active": False, "projection_id": self._projection_id, "stopped": was_active, "reason": reason}

    def _stop_if_robot_not_stationary(self) -> None:
        with self._lock:
            if not self._active:
                return
        robot_status = self.robot.status()
        if not robot_status.get("connected") or not robot_status.get("pose_valid") or not robot_status.get("stationary"):
            self.stop(reason="robot_not_stationary_or_stale")

    def status(self) -> dict[str, Any]:
        self._stop_if_robot_not_stationary()
        with self._lock:
            return {
                "active": self._active,
                "projection_id": self._projection_id,
                "type": self._projection_type,
                "image_path": self._image_path,
                "projected_points": self._projected_points,
                "last_error": self._last_error,
                "last_stop_reason": self._last_stop_reason,
            }
