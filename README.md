# KUKA Zivid Grasshopper Bridge

## Purpose

This repository bridges four parts of an Eye-in-Hand robot-vision workflow:

- KUKA Robot
- Zivid Eye-in-Hand Camera
- External Python Backend
- Rhino 8 / Grasshopper Frontend

The Python backend receives actual KUKA pose/state data over UDP, controls Zivid calibration and capture, transforms captured points into robot Base coordinates, and exposes small control/status responses over HTTP. It does not send robot motion commands. Grasshopper is the HTTP client and geometry frontend, not the hardware backend.


## 2026-08-20 Physical Validation Milestone

The physical KUKA + Zivid workflow has been validated with the transform chain unchanged:

~~~text
T_base_camera = T_base_flange @ T_flange_camera
~~~

This direction is physically validated and must not be inverted, silently redefined, or replaced by an ambiguous transform name. The current registration baseline is:

~~~text
Robot kinematics
+
Eye-in-Hand calibration
+
Base-frame transformation
=
Registration baseline
~~~

Current milestone status:

| Area | Status |
|---|---|
| Robot Pose Acquisition | Complete |
| UDP Pose Streaming / History | Complete |
| Zivid Hardware Capture | Complete |
| Eye-in-Hand Calibration | Complete, 10 poses |
| Camera / Flange / Base Transform | Complete |
| Physical Base-frame Validation | Complete |
| Physical Multi-view Scan | Complete, 6 views |
| Base-frame Merge | Complete |
| Physical Registration Validation | Complete |
| Point Cloud / Surface / Mesh | NEXT |
| Design-to-Scan Deviation Field | NEXT |
| Timber fitting / fabrication correction | NEXT |

Physical validation baseline:

- Robot: KUKA KR10 R1100 sixx
- Camera: Zivid Two+ L100
- Coordinate frame: Robot Base, millimetres
- Multi-view captures: 6
- Raw point count: 10,026,462
- Final merged point count: 2,069,804
- Voxel size: 2.0 mm
- Statistical outlier removal: enabled
- ICP: OFF
- Physical Rhino / Robot XYZ deviation: approximately 0.10.2 mm

The `0.10.2 mm` physical deviation is a Rhino/robot reference measurement, not a calibration residual. The public documentation snapshot is under `validation/2026-08-20_physical_base_frame/`. Future registration comparisons should explicitly compare:

- A. Robot + Eye-in-Hand only
- B. Robot + Eye-in-Hand + ICP

The next research focus is Point Cloud -> Surface / Mesh -> Design-to-Scan Deviation Field -> Timber fitting -> Fabrication correction.

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

The verified production target environment is:

- Windows
- Python 3.12.x in a dedicated Conda environment named `kuka-zivid312`
- Rhino 8 and Grasshopper
- Zivid SDK 2.18.0
- Zivid Python wrapper 2.18.0
- Open3D 0.19.0
- NumPy 2.2.6
- FastAPI 0.141.1
- Uvicorn 0.52.1

Do not run the production service from Conda `base` Python 3.13. Open3D 0.19.0 has no Python 3.13 wheel, so `/scan/merge` will report `SCAN_DEPENDENCY_MISSING` in that environment.

The native Zivid SDK is separate system software. pip installs the Python wrapper, not the native SDK. Install Zivid SDK 2.18.0 from Zivid before using a physical camera, and keep the SDK and Python wrapper versions aligned.

## Repository Layout

~~~text
service.py                         FastAPI entry point
robot_udp_server.py                KUKA/fake-robot UDP receiver
kuka_pose.py                       Explicit KUKA A/B/C conversion
zivid_camera_manager.py            Mock, FileCamera, and hardware capture
camera_robot_calibration.py        Eye-in-Hand workflow
pointcloud_capture.py              Base-frame production capture
scan_session.py                    Multi-view scan session and merge workflow
projection_service.py              Deviation-map projector service
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
conda create -n kuka-zivid312 python=3.12 -y
conda activate kuka-zivid312
python -m pip install --upgrade pip
python -m pip install -r requirements-dev.txt
Copy-Item config.example.yaml config.yaml
~~~

`requirements-dev.txt` includes the runtime requirements plus pytest/httpx. For a runtime-only install, use `python -m pip install -r requirements.txt`.

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
conda activate kuka-zivid312
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

With the default mock settings, a capture contains 36,000 source points and a 10,000-point preview. New output is written to the next data/captures/capture_NNNN directory. If RGB data is available, capture also writes preview_cloud.xyzrgb for colored Grasshopper preview.

Stop the fake sender and service with Ctrl+C in their respective terminals.

## Grasshopper

Open grasshopper/KUKA_Zivid_Bridge.gh in Rhino 8 / Grasshopper and use:

~~~text
http://127.0.0.1:8765
~~~

Confirm Robot Status, confirm the Flange Plane, trigger Capture, then load the returned preview_cloud.xyz path. See grasshopper/README.md for the focused frontend workflow.

For multi-view scanning, Grasshopper should act as the HTTP client only: start a scan, trigger one capture at each manually or externally planned robot pose, merge or finish the scan, then load the returned merged_preview.xyzrgb path. The Python backend remains responsible for UDP pose validation, Zivid capture, calibration use, Base-frame transformation, and disk persistence.

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
| POST | /scan/start | Create a numbered active multi-view scan session |
| POST | /scan/capture | Add one production-gated Base-frame capture to the active scan |
| GET | /scan/status | Report active scan id, captures, merge state, and last scan id |
| POST | /scan/merge | Merge active scan captures into raw, downsampled, and preview outputs |
| POST | /scan/finish | Merge if needed, finalize scan_manifest.yaml, and close the scan |
| POST | /scan/reset | Reset only the active in-memory scan; completed scan folders remain on disk |
| POST | /projection/deviation | Render and start a Zivid projector deviation map from Base-frame points |
| POST | /projection/stop | Stop the active projector handle; idempotent |
| GET | /projection/status | Report active projection id, type, image path, and point count |
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
- data/scans/ contains numbered multi-view scan sessions, per-scan capture records, merged PLY files, previews, and scan_manifest.yaml.
- data/projections/ contains numbered deviation projection images and projection_manifest.yaml files.
- logs/ contains the rotating service log.

These locations are ignored by Git because they are machine/cell-specific runtime state and may contain large files or absolute local paths. The application creates the required directories when they are missing. Do not copy a mock calibration into physical production configuration.

The tracked sample_data/mock_capture_0001 directory contains only a sanitized mock preview and metadata. Large PLY, NPZ, ZDF, and ZFC artifacts are intentionally excluded.

## Testing

After creating config.yaml and activating the environment:

~~~powershell
python -m pytest -v
~~~

The suite covers transform direction, UDP parsing and stationarity, mock Zivid capture, calibration persistence, HTTP routes, disk outputs, colored XYZRGB preview I/O, multi-view scan sessions, and Open3D WCS processing.

## Multi-view Scan Session

The scan workflow groups multiple already validated production captures into one immutable scan folder. It does not command robot motion. Move planning and robot execution remain outside this backend; the UDP actual pose is still the only authoritative robot-state input.

Typical sequence:

~~~text
POST /scan/start
move or jog robot externally
POST /scan/capture
move or jog robot externally
POST /scan/capture
POST /scan/merge
POST /scan/finish
~~~

Each scan capture reuses the same safety and correctness gates as POST /capture: fresh pose, stationary robot, valid calibration, acquisition interval coverage, Base-frame transform, committed disk artifacts. The scan folder stores lightweight scan_capture.yaml records that reference the committed capture artifacts under data/captures/ instead of duplicating large original ZDF/NPZ files.

Merge uses Open3D to combine Base-frame PLY files. Robot UDP `T_base_flange` plus Eye-in-Hand `T_flange_camera` remains the authoritative initial alignment, and each `base_ply` is already in Robot Base frame before scan merge. When enabled, bounded Point-to-Plane ICP starts from identity and applies only a small accepted `Delta T` correction to the original full-resolution source cloud before final voxel downsampling and statistical outlier removal.

ICP is a registration-consistency refinement for multi-view overlap. It does not improve Zivid depth accuracy, replace hand-eye calibration, or prove physical frame correctness.

Scan outputs include:

- merged_cloud_raw.ply
- merged_cloud_pre_icp.ply when ICP is enabled
- merged_cloud_downsampled.ply
- merged_preview.xyz
- merged_preview.xyzrgb when color is available
- scan_manifest.yaml

## Deviation Projection

Projection maps Grasshopper-computed Base-frame deviation points onto the physical workpiece through the Zivid projector. Grasshopper sends only `points_base`, `deviations_mm`, thresholds, palette, radius, and opacity. It must not open the Zivid camera, send projector pixels, send camera-frame points, or send a BGRA bitmap.

The service uses the current robot pose at projection time, not old scan capture poses:

~~~text
T_base_camera_current = T_base_flange_current @ T_flange_camera
T_camera_base = inv(T_base_camera_current)
P_camera = T_camera_base @ P_base
~~~

Only then does the backend call `zivid.projection.pixels_from_3d_points(camera, points_camera)`. The projector image is rasterized as BGRA in Python, saved under `data/projections/projection_NNNN/`, and shown through a single retained projection handle. Starting a new projection replaces the previous handle; `/projection/stop` is safe to call repeatedly.

Projection is gated on robot connection, fresh stationary pose, valid non-identity calibration, and camera projection readiness. `/capture` and `/scan/capture` stop active projection before 3D acquisition so service state does not report an image that the camera has already stopped showing.

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

See the operation guide in `docs/` for the end-to-end operating and verification guide.


## UDP Hardening Contract

The UDP stream remains the authoritative actual robot-state source. Motion may be planned or commanded elsewhere (KRL, Jog, Grasshopper, RoboDK, or another planner), but this Python service still does not send robot motion commands.

Preferred JSON pose packet:

~~~json
{
  "type": "POSE",
  "session_id": "kuka-session-001",
  "seq": 1523,
  "timestamp": 12345678,
  "T_base_flange": [[1,0,0,1000],[0,1,0,0],[0,0,1,800],[0,0,0,1]],
  "matrix_name": "T_base_flange",
  "source_frame": "flange",
  "target_frame": "base",
  "units": "mm",
  "state": "READY"
}
~~~

`session_id` is optional for backward compatibility, but real commissioning should send it. Within one session, `seq` must move forward. Duplicate or older same-session packets are ignored and counted in `/status`. A changed `session_id` is treated as sender restart: old pose history is cleared, sequence tracking restarts, and old samples cannot satisfy stationary checks.

Robot-side `timestamp` is metadata only. Freshness and capture timing use the server's `time.monotonic()` receive/acquisition clock unless a separate time-synchronization scheme is explicitly commissioned.

### Pose semantics

A valid 4x4 matrix proves only that the packet contains a rigid transform. It does not prove that the matrix is physically `T_base_flange`. Packets should include at minimum:

~~~text
matrix_name: T_base_flange
source_frame: flange
target_frame: base
units: mm
~~~

The service reports this metadata but does not claim physical validation. Passing software tests does not certify KUKA/RoboDK/Rhino/Zivid transform semantics.

### Networking

Mock/local defaults bind to localhost. For real KUKA commissioning, use a dedicated robot NIC/VLAN, Windows Firewall rules, and set:

~~~yaml
robot_udp:
  expected_sender_ip: <KUKA sender IP>
~~~

Packets from other IPs are rejected and counted. Do not expose HTTP or UDP on `0.0.0.0` unless the cell network/security setup has been reviewed.

## Capture Transaction Model

Each capture directory has a `capture_manifest.yaml` with one of:

~~~text
STARTED -> CAPTURED -> TRANSFORMED -> COMMITTED
FAILED
~~~

Only `COMMITTED` captures are valid for downstream latest-capture selection. On startup, incomplete `STARTED`, `CAPTURED`, or `TRANSFORMED` manifests are marked `FAILED` with recovery metadata. Diagnostic files are preserved.

Capture timing metadata records:

~~~text
capture_request_monotonic
camera_capture_start_monotonic
camera_capture_end_monotonic
post_capture_validation_monotonic
~~~

These bound the software acquisition call. They are not claimed to be physical exposure start/end unless the Zivid SDK explicitly exposes such timing. The robot must be stationary before capture, the pose history spanning the acquisition interval must stay within configured translation/rotation thresholds, and an insufficient UDP pose history rejects the capture instead of assuming stationarity.

`POST /capture` remains bodyless-compatible. Grasshopper or other clients may pass a duplicate-protection body:

~~~json
{"request_id": "gh-capture-123"}
~~~

Repeating the same `request_id` replays the first successful result instead of creating another capture.

## Calibration Provenance

`calibration_results.yaml` preserves the directional transform format:

~~~yaml
transform:
  name: T_flange_camera
  from: camera
  to: flange
  matrix: ...
~~~

New calibration files also include camera, robot, mount, calibration, source, units, classification, and residual summary metadata where available. Unknown hardware fields remain `null`; fake serials should not be invented. Mock/synthetic calibration is still rejected outside mock camera mode. Non-mock camera loading rejects missing production provenance and obvious camera serial mismatches when both serials are known.

## Real KUKA / Zivid Commissioning Checklist

Software tests are necessary but not sufficient. Passing software unit tests does NOT certify physical transform semantics.

1. Keep `kuka_pose.convention: null` until KUKA ABC is physically verified. Prefer full matrix UDP during commissioning.
2. Verify `T_base_flange` known poses:
   - Known Pose 1: translation on one axis.
   - Known Pose 2: translation on another axis.
   - Known Pose 3: rotation-heavy pose.
   - Known Pose 4: combined translation and rotation.
3. For each pose compare:
   - Teach Pendant values.
   - UDP `T_base_flange` reported by `/robot/pose`.
   - Grasshopper flange plane origin and signed axes.
4. Mount and identify the real Zivid camera; record serial/model if available.
5. Perform real Eye-in-Hand calibration; do not copy mock calibration into a physical setup.
6. Scan an asymmetric fiducial target.
7. Verify signed X/Y/Z directions and transformed base-frame fiducial positions.
8. Only after those checks should captures be used as physical fabrication input.

Multi-view scanning is implemented as a first-class `ScanSession` grouping multiple committed capture manifests. Registration, autonomous planning, robot execution, and closed-loop fabrication control remain deferred and must not be added to this perception/data backend without a separate design review.
