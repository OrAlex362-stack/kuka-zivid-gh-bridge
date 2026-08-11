# KUKA Zivid Grasshopper Bridge

## Purpose

This repository bridges four parts of an Eye-in-Hand robot-vision workflow:

- KUKA Robot
- Zivid Eye-in-Hand Camera
- External Python Backend
- Rhino 8 / Grasshopper Frontend

The Python backend receives actual KUKA pose/state data over UDP, controls Zivid calibration and capture, transforms captured points into robot Base coordinates, and exposes small control/status responses over HTTP. It does not send robot motion commands. Grasshopper is the HTTP client and geometry frontend, not the hardware backend.

## Architecture

~~~text
KUKA / Fake Robot
        |
        | UDP actual pose/state
        v
Python Service (FastAPI + Zivid + disk storage)
        |
        | HTTP status/commands and file paths
        v
Rhino 8 / Grasshopper
~~~

Point clouds remain on disk. Full clouds are not serialized through HTTP: the API returns IDs, counts, transforms, errors, and file paths. Grasshopper loads preview_cloud.xyz for responsive display, while transformed_cloud.ply remains available for analysis.

## Coordinate Convention

Every transform follows the directional naming convention:

~~~text
T_target_source
p_target = T_target_source @ p_source
~~~

For the Eye-in-Hand chain:

~~~text
T_base_camera =
T_base_flange @ T_flange_camera
~~~

No transform is silently inverted. Translations and point coordinates are in millimetres. Rotation matrices are validated as finite, orthonormal, right-handed rigid transforms.

## Environment

The verified mock/WCS environment is:

- Windows
- Python 3.10.6
- Rhino 8 and Grasshopper
- Zivid SDK 2.18.0
- Zivid Python wrapper 2.18.0
- Open3D 0.19.0
- NumPy 2.2.6
- FastAPI 0.141.1
- Uvicorn 0.52.1

The native Zivid SDK is separate system software. pip installs the Python wrapper, not the native SDK. Install Zivid SDK 2.18.0 from Zivid before using a physical camera, and keep the SDK and Python wrapper versions aligned.

## Repository Layout

~~~text
service.py                         FastAPI entry point
robot_udp_server.py                KUKA/fake-robot UDP receiver
kuka_pose.py                       Explicit KUKA A/B/C conversion
zivid_camera_manager.py            Mock, FileCamera, and hardware capture
camera_robot_calibration.py        Eye-in-Hand workflow
pointcloud_capture.py              Base-frame production capture
compute_wcs_pointcloud.py          Open3D three-plane WCS processing
config.example.yaml                Shareable mock-safe configuration
settings/capture_settings.yml      Zivid capture settings
tools/                             Fake pose and mock-calibration utilities
tests/                             Automated tests
grasshopper/KUKA_Zivid_Bridge.gh  Rhino/Grasshopper frontend
docs/                              Detailed operation guide
sample_data/mock_capture_0001/     Small tracked mock preview example
data/                              Local runtime calibration and captures
logs/                              Local rotating logs
~~~

## First-Time Setup

From the repository root in PowerShell:

~~~powershell
py -3.10 -m venv .venv
..venvScriptsActivate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
Copy-Item config.example.yaml config.yaml
~~~

For development and tests, install requirements-dev.txt instead:

~~~powershell
pip install -r requirements-dev.txt
~~~

config.yaml is local-only and ignored by Git. Review it before connecting hardware. The example starts on localhost, uses UDP port 49152 and HTTP port 8765, and keeps the camera in mock mode.

For a real camera, install the native Zivid SDK 2.18.0 separately, change zivid.mode only after hardware setup, and replace settings/capture_settings.yml with settings validated for the actual camera, material, distance, and lighting.

## Mock Mode Quick Start

Create the test-only identity calibration once if data/calibration/calibration_results.yaml is absent:

~~~powershell
python tools/create_mock_calibration.py
~~~

This utility refuses to run unless zivid.mode is explicitly mock. Its output is synthetic and must never be used with a physical robot or camera.

Terminal 1:

~~~powershell
python service.py
~~~

Terminal 2:

~~~powershell
python tools/send_fake_robot_pose.py
~~~

The fake sender defaults to a stationary flange with origin (1000, 0, 800), identity orientation, and Z axis (0, 0, 1).

Check status:

~~~powershell
Invoke-RestMethod http://127.0.0.1:8765/status
~~~

Trigger a capture:

~~~powershell
Invoke-RestMethod -Method POST -Uri http://127.0.0.1:8765/capture
~~~

With the default mock settings, a capture contains 36,000 source points and a 10,000-point preview. New output is written to the next data/captures/capture_NNNN directory.

Stop the fake sender and service with Ctrl+C in their respective terminals.

## Grasshopper

Open grasshopper/KUKA_Zivid_Bridge.gh in Rhino 8 / Grasshopper and use:

~~~text
http://127.0.0.1:8765
~~~

Confirm Robot Status, confirm the Flange Plane, trigger Capture, then load the returned preview_cloud.xyz path. See grasshopper/README.md for the focused frontend workflow.

## API

| Method | Endpoint | Purpose |
|---|---|---|
| GET | /health | Service liveness and camera mode |
| GET | /status | Aggregate robot, camera, calibration, capture, and WCS state |
| GET | /robot/pose | Latest robot pose and T_base_flange |
| POST | /calibration/sample | Acquire and persist one hand-eye sample |
| POST | /calibration/solve | Solve T_flange_camera |
| POST | /calibration/reset | Archive and reset active calibration data |
| POST | /capture | Capture and transform a point cloud into Base coordinates |
| POST | /wcs/compute | Compute workpiece WCS from a selected/latest capture |

Interactive FastAPI schemas are available at http://127.0.0.1:8765/docs while the service is running.

## Mock vs Real Hardware

| Component | Test mode | Production mode |
|---|---|---|
| Robot pose source | Fake Robot sender | Real KUKA actual-pose UDP |
| Camera | Mock Camera | Real Zivid or diagnostic FileCamera |
| Calibration | Mock Calibration, test-only | Real Zivid Eye-in-Hand calibration |

The modes can be commissioned independently, but mock calibration is rejected outside mock camera mode. Keep kuka_pose.convention null until the physical controller A/B/C convention has been verified against known poses; full matrix UDP packets bypass Euler conversion and remain rigid-transform validated.

## Data Outputs

- data/calibration/ contains active samples, archives, and calibration_results.yaml.
- data/captures/ contains numbered capture directories and point-cloud artifacts.
- logs/ contains the rotating service log.

These locations are ignored by Git because they are machine/cell-specific runtime state and may contain large files or absolute local paths. The application creates the required directories when they are missing. Do not copy a mock calibration into physical production configuration.

The tracked sample_data/mock_capture_0001 directory contains only a sanitized mock preview and metadata. Large PLY, NPZ, ZDF, and ZFC artifacts are intentionally excluded.

## Testing

After creating config.yaml and activating the environment:

~~~powershell
python -m pytest -v
~~~

The suite covers transform direction, UDP parsing and stationarity, mock Zivid capture, calibration persistence, HTTP routes, disk outputs, and Open3D WCS processing.

## Known Commissioning Items

Automated mock tests do not validate:

- real KUKA network/controller integration or robot safety;
- real Zivid camera connectivity and production capture settings;
- the physical controller A/B/C convention;
- real Eye-in-Hand calibration quality;
- lighting, material, working distance, timing, or cell-specific tolerances;
- the Grasshopper definition inside Rhino.

Commission these items in the guarded physical cell. The Python service receives robot state but never sends robot motion commands.

## Detailed Documentation

See docs/KUKA_Zivid_Python_Grasshopper_操作與測試說明.md for the end-to-end operating and verification guide.
