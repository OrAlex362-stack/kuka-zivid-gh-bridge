from pathlib import Path
import json
import time

import numpy as np

from camera_robot_calibration import serialize_calibration_result
from config_loader import load_config
from projection_service import (
    BGRA_RED,
    ProjectionService,
    classify_deviations,
    colors_bgra_for_labels,
    filter_finite_points_and_deviations,
    filter_projector_visible,
    points_base_to_camera,
    rasterize_bgra_zbuffer,
)
from robot_udp_server import RobotUDPServer, parse_json_packet
from storage_utils import atomic_write_yaml


class FakeProjectionHandle:
    def __init__(self) -> None:
        self.stop_count = 0

    def stop(self) -> None:
        self.stop_count += 1


class FakeProjectionApi:
    def __init__(self) -> None:
        self.last_image = None
        self.handle = FakeProjectionHandle()

    def projector_resolution(self, _camera):
        return (16, 12)

    def pixels_from_3d_points(self, _camera, points_camera):
        points = np.asarray(points_camera, dtype=np.float64)
        return np.column_stack([points[:, 0] + 8.0, points[:, 1] + 6.0])

    def show_image_bgra(self, _camera, image):
        self.last_image = np.asarray(image, dtype=np.uint8).copy()
        self.handle = FakeProjectionHandle()
        return self.handle


class FakeCameraManager:
    mode = "mock"
    settings_path = Path("settings/capture_settings.yml")

    def __init__(self) -> None:
        self.api = FakeProjectionApi()

    def status(self):
        return {
            "connected": True,
            "mode": self.mode,
            "busy": False,
            "serial_number": "mock-camera",
            "model": "synthetic",
            "sdk_python_version": "test",
        }

    def projection_resources(self):
        return self.api, object()


def _config(tmp_path: Path) -> dict:
    config = load_config()
    config["zivid"]["mode"] = "mock"
    config["_config_dir"] = str(tmp_path)
    config["robot_udp"]["port"] = 0
    config["robot_udp"]["settle_time_ms"] = 0
    config["robot_udp"]["max_pose_age_ms"] = 5000
    config["paths"]["calibration_result"] = "calibration/calibration_results.yaml"
    config["paths"]["projections"] = "projections"
    config["projection"]["deviation"]["max_points"] = 100
    return config


def _write_calibration(config: dict) -> None:
    T_flange_camera = np.eye(4)
    T_flange_camera[2, 3] = 10.0
    path = Path(config["_config_dir"]) / config["paths"]["calibration_result"]
    atomic_write_yaml(
        path,
        serialize_calibration_result(
            T_flange_camera,
            [{"translation": 0.0, "rotation": 0.0}],
            sample_count=1,
            status="test",
            mock=True,
            synthetic=True,
            camera={"serial_number": "mock-camera", "model": "synthetic"},
        ),
    )


def _inject_pose(robot: RobotUDPServer, seq: int = 1) -> None:
    T_base_flange = np.eye(4)
    T_base_flange[0, 3] = 100.0
    packet = json.dumps({"type": "POSE", "session_id": "projection-test", "seq": seq, "T_base_flange": T_base_flange.tolist()})
    robot.inject_pose(parse_json_packet(packet, None, received_monotonic=time.monotonic()))


def test_base_to_camera_transform_uses_inverse_current_T_base_camera() -> None:
    T_base_camera = np.eye(4)
    T_base_camera[:3, 3] = [100.0, 20.0, 10.0]
    points_base = np.array([[101.0, 22.0, 15.0]])

    points_camera = points_base_to_camera(points_base, T_base_camera)

    assert np.allclose(points_camera, [[1.0, 2.0, 5.0]])


def test_xyz_deviation_index_preserved_after_filtering() -> None:
    points = np.array([[1, 2, 3], [np.nan, 9, 9], [4, 5, 6]], dtype=float)
    deviations = np.array([0.2, 99.0, 5.0])

    filtered_points, filtered_deviations, input_count = filter_finite_points_and_deviations(points, deviations)

    assert input_count == 3
    assert filtered_points.tolist() == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    assert filtered_deviations.tolist() == [0.2, 5.0]


def test_absolute_deviation_classification() -> None:
    labels = classify_deviations([0.2, 2.0, 5.0], mode="absolute", good_tolerance_mm=1.0, warning_tolerance_mm=3.0)
    assert labels == ["GOOD", "WARNING", "BAD"]


def test_bgra_palette_red_is_not_rgb_order() -> None:
    colors = colors_bgra_for_labels(["BAD"], palette="pure_rgb", opacity=255)
    assert colors[0].tolist() == BGRA_RED.tolist()
    assert colors[0].tolist() == [0, 0, 255, 255]


def test_projector_boundary_filtering() -> None:
    pixels = np.array([[1.0, 1.0], [-1.0, 1.0], [8.2, 1.0], [2.0, 9.0], [3.0, 3.0]])
    z = np.array([5.0, 5.0, 5.0, 5.0, 2.0])

    u, v, visible_z, mask = filter_projector_visible(pixels, z, width=8, height=8)

    assert u.tolist() == [1, 3]
    assert v.tolist() == [1, 3]
    assert visible_z.tolist() == [5.0, 2.0]
    assert mask.tolist() == [True, False, False, False, True]


def test_zbuffer_keeps_nearest_camera_z() -> None:
    far_red = np.array([0, 0, 255, 255], dtype=np.uint8)
    near_green = np.array([0, 255, 0, 255], dtype=np.uint8)

    image = rasterize_bgra_zbuffer(
        5,
        5,
        [2, 2],
        [2, 2],
        [10.0, 3.0],
        np.array([far_red, near_green]),
        point_radius_px=0,
        opacity=255,
    )

    assert image[2, 2].tolist() == near_green.tolist()


def test_rasterizer_keeps_background_opaque_black() -> None:
    image = rasterize_bgra_zbuffer(3, 3, [], [], [], np.empty((0, 4), dtype=np.uint8), point_radius_px=0, opacity=64)
    assert image[0, 0].tolist() == [0, 0, 0, 255]


def test_projection_start_writes_artifacts_and_stop_is_idempotent(tmp_path: Path) -> None:
    config = _config(tmp_path)
    _write_calibration(config)
    robot = RobotUDPServer(config["robot_udp"], config["kuka_pose"])
    _inject_pose(robot)
    camera = FakeCameraManager()
    service = ProjectionService(config, robot, camera)  # type: ignore[arg-type]

    result = service.project_deviation(
        {
            "points_base": [[100.0, 0.0, 20.0], [101.0, 0.0, 20.0]],
            "deviations_mm": [0.2, 5.0],
            "mode": "absolute",
            "good_tolerance_mm": 1.0,
            "warning_tolerance_mm": 3.0,
            "palette": "pure_rgb",
            "point_radius_px": 1,
            "opacity": 255,
        }
    )

    assert result["ok"] is True
    assert result["projection_id"] == "projection_0001"
    assert Path(result["image_path"]).is_file()
    assert (tmp_path / "projections" / "projection_0001" / "projection_manifest.yaml").is_file()
    assert service.status()["active"] is True

    first_stop = service.stop(reason="test")
    second_stop = service.stop(reason="test")

    assert first_stop["active"] is False
    assert second_stop["active"] is False
    assert second_stop["ok"] is True
