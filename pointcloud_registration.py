"""Bounded ICP refinement for already Base-frame aligned scan captures."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
from typing import Any

import numpy as np

from transform_utils import validate_transform


LOGGER = logging.getLogger(__name__)

IDENTITY_TRANSFORM = np.eye(4, dtype=np.float64)


@dataclass(frozen=True)
class ICPConfig:
    enabled: bool = False
    method: str = "point_to_plane"
    voxel_size_mm: float = 2.0
    max_correspondence_distance_mm: float = 5.0
    max_iterations: int = 50
    normal_radius_mm: float = 8.0
    normal_max_nn: int = 30
    min_fitness: float = 0.30
    max_inlier_rmse_mm: float = 3.0
    max_translation_correction_mm: float = 5.0
    max_rotation_correction_deg: float = 1.0
    failure_policy: str = "use_initial_alignment"
    min_points: int = 3

    @classmethod
    def from_mapping(cls, value: dict[str, Any] | None) -> "ICPConfig":
        data = {} if value is None else dict(value)
        return cls(
            enabled=bool(data.get("enabled", cls.enabled)),
            method=str(data.get("method", cls.method)).strip().lower(),
            voxel_size_mm=float(data.get("voxel_size_mm", cls.voxel_size_mm)),
            max_correspondence_distance_mm=float(
                data.get("max_correspondence_distance_mm", cls.max_correspondence_distance_mm)
            ),
            max_iterations=int(data.get("max_iterations", cls.max_iterations)),
            normal_radius_mm=float(data.get("normal_radius_mm", cls.normal_radius_mm)),
            normal_max_nn=int(data.get("normal_max_nn", cls.normal_max_nn)),
            min_fitness=float(data.get("min_fitness", cls.min_fitness)),
            max_inlier_rmse_mm=float(data.get("max_inlier_rmse_mm", cls.max_inlier_rmse_mm)),
            max_translation_correction_mm=float(
                data.get("max_translation_correction_mm", cls.max_translation_correction_mm)
            ),
            max_rotation_correction_deg=float(
                data.get("max_rotation_correction_deg", cls.max_rotation_correction_deg)
            ),
            failure_policy=str(data.get("failure_policy", cls.failure_policy)).strip().lower(),
            min_points=int(data.get("min_points", cls.min_points)),
        )


@dataclass(frozen=True)
class RegistrationResult:
    accepted: bool
    reason: str | None
    delta_transform: np.ndarray
    fitness: float | None = None
    inlier_rmse_mm: float | None = None
    translation_correction_mm: float | None = None
    rotation_correction_deg: float | None = None
    source_point_count: int = 0
    target_point_count: int = 0
    source_registration_point_count: int = 0
    target_registration_point_count: int = 0

    @property
    def applied_transform(self) -> np.ndarray:
        return self.delta_transform if self.accepted else IDENTITY_TRANSFORM.copy()

    def to_manifest(self, *, capture_id: str, role: str = "registered") -> dict[str, Any]:
        payload: dict[str, Any] = {
            "capture_id": capture_id,
            "role": role,
            "accepted": bool(self.accepted),
            "reason": self.reason,
            "source_point_count": int(self.source_point_count),
            "target_point_count": int(self.target_point_count),
            "source_registration_point_count": int(self.source_registration_point_count),
            "target_registration_point_count": int(self.target_registration_point_count),
            "delta_transform": self.applied_transform.tolist(),
        }
        if self.fitness is not None:
            payload["fitness"] = float(self.fitness)
        if self.inlier_rmse_mm is not None:
            payload["inlier_rmse_mm"] = float(self.inlier_rmse_mm)
        if self.translation_correction_mm is not None:
            payload["translation_correction_mm"] = float(self.translation_correction_mm)
        if self.rotation_correction_deg is not None:
            payload["rotation_correction_deg"] = float(self.rotation_correction_deg)
        if not self.accepted and self.delta_transform.shape == (4, 4) and np.all(np.isfinite(self.delta_transform)):
            payload["estimated_delta_transform"] = self.delta_transform.tolist()
        return payload


def rotation_angle_deg(rotation: np.ndarray) -> float:
    value = np.asarray(rotation, dtype=np.float64)
    if value.shape != (3, 3) or not np.all(np.isfinite(value)):
        return math.inf
    cosine = float((np.trace(value) - 1.0) / 2.0)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def transform_metrics(transform: np.ndarray) -> tuple[float, float]:
    value = np.asarray(transform, dtype=np.float64)
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        return math.inf, math.inf
    translation_mm = float(np.linalg.norm(value[:3, 3]))
    rotation_deg = rotation_angle_deg(value[:3, :3])
    return translation_mm, rotation_deg


def prepare_registration_cloud(o3d: Any, cloud: Any, config: ICPConfig) -> Any:
    if config.voxel_size_mm > 0.0:
        prepared = cloud.voxel_down_sample(config.voxel_size_mm)
    else:
        prepared = o3d.geometry.PointCloud(cloud)
    if len(prepared.points) >= config.min_points:
        prepared.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=config.normal_radius_mm,
                max_nn=config.normal_max_nn,
            )
        )
        prepared.normalize_normals()
    return prepared


def _rejection(reason: str, *, delta_transform: np.ndarray | None = None, **kwargs: Any) -> RegistrationResult:
    return RegistrationResult(
        accepted=False,
        reason=reason,
        delta_transform=IDENTITY_TRANSFORM.copy() if delta_transform is None else np.asarray(delta_transform, dtype=np.float64),
        **kwargs,
    )


def validate_registration(
    *,
    delta_transform: np.ndarray,
    fitness: float,
    inlier_rmse_mm: float,
    config: ICPConfig,
    source_point_count: int,
    target_point_count: int,
    source_registration_point_count: int,
    target_registration_point_count: int,
) -> RegistrationResult:
    try:
        validated = validate_transform(delta_transform, name="Delta T ICP", atol=1e-5)
    except Exception:
        return _rejection(
            "non_finite_or_invalid_transform",
            delta_transform=np.asarray(delta_transform, dtype=np.float64),
            fitness=fitness,
            inlier_rmse_mm=inlier_rmse_mm,
            source_point_count=source_point_count,
            target_point_count=target_point_count,
            source_registration_point_count=source_registration_point_count,
            target_registration_point_count=target_registration_point_count,
        )
    translation_mm, rotation_deg = transform_metrics(validated)
    common = {
        "fitness": fitness,
        "inlier_rmse_mm": inlier_rmse_mm,
        "translation_correction_mm": translation_mm,
        "rotation_correction_deg": rotation_deg,
        "source_point_count": source_point_count,
        "target_point_count": target_point_count,
        "source_registration_point_count": source_registration_point_count,
        "target_registration_point_count": target_registration_point_count,
    }
    if fitness < config.min_fitness:
        return _rejection("fitness_below_minimum", delta_transform=validated, **common)
    if inlier_rmse_mm > config.max_inlier_rmse_mm:
        return _rejection("rmse_limit_exceeded", delta_transform=validated, **common)
    if translation_mm > config.max_translation_correction_mm:
        return _rejection("translation_limit_exceeded", delta_transform=validated, **common)
    if rotation_deg > config.max_rotation_correction_deg:
        return _rejection("rotation_limit_exceeded", delta_transform=validated, **common)
    return RegistrationResult(accepted=True, reason=None, delta_transform=validated, **common)


def register_point_to_plane(o3d: Any, source_cloud: Any, target_cloud: Any, config: ICPConfig) -> RegistrationResult:
    source_count = len(source_cloud.points)
    target_count = len(target_cloud.points)
    if config.method != "point_to_plane":
        return _rejection(
            "unsupported_method",
            source_point_count=source_count,
            target_point_count=target_count,
        )
    if source_count < config.min_points or target_count < config.min_points:
        return _rejection(
            "insufficient_points",
            source_point_count=source_count,
            target_point_count=target_count,
        )
    try:
        source_registration = prepare_registration_cloud(o3d, source_cloud, config)
        target_registration = prepare_registration_cloud(o3d, target_cloud, config)
        source_registration_count = len(source_registration.points)
        target_registration_count = len(target_registration.points)
        if source_registration_count < config.min_points or target_registration_count < config.min_points:
            return _rejection(
                "insufficient_points",
                source_point_count=source_count,
                target_point_count=target_count,
                source_registration_point_count=source_registration_count,
                target_registration_point_count=target_registration_count,
            )
        result = o3d.pipelines.registration.registration_icp(
            source_registration,
            target_registration,
            config.max_correspondence_distance_mm,
            IDENTITY_TRANSFORM.copy(),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=config.max_iterations),
        )
    except Exception as exc:
        LOGGER.warning("ICP registration exception: %s", exc)
        return _rejection(
            "registration_exception",
            source_point_count=source_count,
            target_point_count=target_count,
        )
    return validate_registration(
        delta_transform=np.asarray(result.transformation, dtype=np.float64),
        fitness=float(result.fitness),
        inlier_rmse_mm=float(result.inlier_rmse),
        config=config,
        source_point_count=source_count,
        target_point_count=target_count,
        source_registration_point_count=source_registration_count,
        target_registration_point_count=target_registration_count,
    )
