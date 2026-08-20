"""Local-only FastAPI bridge for Grasshopper control and small metadata."""

from __future__ import annotations

import argparse
from contextlib import asynccontextmanager
import logging
from pathlib import Path
import time
from typing import Any, AsyncIterator

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict
import uvicorn

from bridge_errors import BridgeError
from camera_robot_calibration import CameraRobotCalibration
from compute_wcs_pointcloud import WCSPointCloudProcessor
from config_loader import load_config
from logging_setup import configure_logging
from pointcloud_capture import PointCloudCapture
from projection_service import ProjectionService
from robot_udp_server import RobotUDPServer
from scan_session import ScanSessionManager
from zivid_camera_manager import ZividCameraManager


LOGGER = logging.getLogger(__name__)


class CaptureRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    request_id: str | None = None


class WCSComputeRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    capture_id: str | None = None
    request_id: str | None = None


class DeviationProjectionRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    points_base: list[list[float]]
    deviations_mm: list[float]
    mode: str | None = None
    good_tolerance_mm: float | None = None
    warning_tolerance_mm: float | None = None
    palette: str | None = None
    point_radius_px: int | None = None
    opacity: int | None = None


def create_app(
    config_path: str | Path = "config.yaml",
    *,
    config_override: dict[str, Any] | None = None,
) -> FastAPI:
    config = config_override if config_override is not None else load_config(config_path)
    configure_logging(config)
    robot = RobotUDPServer(config["robot_udp"], config["kuka_pose"])
    camera = ZividCameraManager(config)
    calibration = CameraRobotCalibration(config, robot, camera)
    capture = PointCloudCapture(config, robot, camera)
    wcs = WCSPointCloudProcessor(config)
    scan = ScanSessionManager(config, robot, camera, capture)
    projection = ProjectionService(config, robot, camera)
    started_monotonic = time.monotonic()

    @asynccontextmanager
    async def lifespan(app_instance: FastAPI) -> AsyncIterator[None]:
        LOGGER.info("Bridge service starting")
        robot.start()
        camera.connect()  # failure is reported in status; HTTP remains available
        try:
            yield
        finally:
            robot.stop()
            camera.close()
            LOGGER.info("Bridge service stopped")

    app = FastAPI(
        title="KUKA-Zivid-Rhino Bridge",
        version="1.0.0",
        lifespan=lifespan,
        docs_url="/docs",
        redoc_url=None,
    )
    app.state.config = config
    app.state.robot = robot
    app.state.camera = camera
    app.state.calibration = calibration
    app.state.capture = capture
    app.state.wcs = wcs
    app.state.scan = scan
    app.state.projection = projection

    @app.exception_handler(BridgeError)
    async def bridge_error_handler(_request: Request, exc: BridgeError) -> JSONResponse:
        return JSONResponse(status_code=exc.status_code, content=exc.as_dict())

    @app.exception_handler(Exception)
    async def unexpected_error_handler(_request: Request, exc: Exception) -> JSONResponse:
        LOGGER.exception("Unhandled HTTP request error", exc_info=exc)
        return JSONResponse(
            status_code=500,
            content={
                "ok": False,
                "error_code": "INTERNAL_ERROR",
                "message": "The bridge encountered an unexpected internal error.",
            },
        )

    @app.get("/health")
    def health() -> dict[str, Any]:
        camera_status = camera.status()
        return {
            "ok": True,
            "service": "ready" if camera_status["connected"] else "degraded",
            "uptime_s": time.monotonic() - started_monotonic,
            "camera_mode": camera_status["mode"],
        }

    @app.get("/status")
    def status() -> dict[str, Any]:
        camera_status = camera.status()
        return {
            "service": "ready" if camera_status["connected"] else "degraded",
            "robot": robot.status(),
            "camera": camera_status,
            "calibration": calibration.status(),
            "capture": capture.status(),
            "wcs": wcs.status(),
            "scan": scan.status(),
            "projection": projection.status(),
        }

    @app.get("/robot/pose")
    def robot_pose() -> dict[str, Any]:
        pose = robot.latest_pose()
        if pose is None:
            raise BridgeError("NO_ROBOT_POSE", "No robot pose has been received.", status_code=404)
        return {
            "ok": True,
            "pose": pose.to_dict(include_raw_message=True),
            "stationary": robot.stationary_status().to_dict(),
        }

    @app.post("/calibration/sample")
    def calibration_sample() -> dict[str, Any]:
        return calibration.add_sample()

    @app.post("/calibration/solve")
    def calibration_solve() -> dict[str, Any]:
        return calibration.solve()

    @app.post("/calibration/reset")
    def calibration_reset() -> dict[str, Any]:
        # The dedicated POST is the explicit reset request. Data is archived.
        return calibration.reset()

    @app.post("/capture")
    def production_capture(request_body: CaptureRequest | None = None) -> dict[str, Any]:
        projection.stop(reason="3d_capture")
        return capture.capture(None if request_body is None else request_body.request_id)

    @app.post("/scan/start")
    def scan_start() -> dict[str, Any]:
        return scan.start()

    @app.post("/scan/capture")
    def scan_capture() -> dict[str, Any]:
        projection.stop(reason="3d_capture")
        return scan.capture_once()

    @app.get("/scan/status")
    def scan_status() -> dict[str, Any]:
        return scan.status()

    @app.post("/scan/merge")
    def scan_merge() -> dict[str, Any]:
        return scan.merge()

    @app.post("/scan/finish")
    def scan_finish() -> dict[str, Any]:
        return scan.finish()

    @app.post("/scan/reset")
    def scan_reset() -> dict[str, Any]:
        return scan.reset()


    @app.post("/projection/deviation")
    def projection_deviation(request_body: DeviationProjectionRequest) -> dict[str, Any]:
        return projection.project_deviation(request_body.model_dump(exclude_none=True))

    @app.post("/projection/stop")
    def projection_stop() -> dict[str, Any]:
        return projection.stop(reason="manual")

    @app.get("/projection/status")
    def projection_status() -> dict[str, Any]:
        return projection.status()

    @app.post("/wcs/compute")
    def compute_wcs(request_body: WCSComputeRequest | None = None) -> dict[str, Any]:
        return wcs.compute(None if request_body is None else request_body.capture_id)

    return app


app = create_app()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config.yaml"))
    args = parser.parse_args()
    runtime_config = load_config(args.config)
    runtime_app = create_app(config_override=runtime_config)
    uvicorn.run(
        runtime_app,
        host=str(runtime_config["api"]["host"]),
        port=int(runtime_config["api"]["port"]),
        log_level=str(runtime_config["api"]["log_level"]).lower(),
    )


if __name__ == "__main__":
    main()
