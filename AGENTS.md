# AGENTS.md

## Repository purpose

This repository implements a decoupled KUKA–Zivid–Grasshopper robot-vision bridge.

```text
KUKA / Fake Robot
        |
        | UDP actual pose/state
        v
Persistent Python Backend
        |
        |-- RobotUDPServer
        |-- ZividCameraManager
        |-- CameraRobotCalibration
        |-- PointCloudCapture
        |-- WCSPointCloudProcessor
        `-- FastAPI service
        |
        | HTTP metadata/commands + disk file paths
        v
Rhino 8 / Grasshopper
```

The Python backend is a perception/data service. It is not the robot motion controller.
Robot motion may come from KRL, Jog, Grasshopper, RoboDK, or another planner. The actual KUKA state received over UDP is the authoritative robot-state source for capture and calibration.

---

## Architectural invariants

### Transform convention

All transforms use:

```text
T_target_source
p_target = T_target_source @ p_source
```

Eye-in-Hand chain:

```text
T_base_camera = T_base_flange @ T_flange_camera
```

Never silently invert transforms. Never replace explicit names with ambiguous names such as `camera_transform` or `robot_matrix`.
Rigid transforms must remain validated for shape, finite values, homogeneous form, orthonormal rotation, right-handedness, and determinant approximately +1.

Numeric validity does not prove semantic frame direction. Physical commissioning is still required.

### Robot authority

The backend receives robot state but must not send robot motion commands.
Do not add MoveJ, MoveL, RoboDK robot execution, KUKA motion commands, or automatic robot movement inside capture/calibration routes unless the user explicitly changes the architecture.

RoboDK may be an optional planning/simulation layer, but not the authoritative actual-pose source.

### KUKA ABC convention

If `kuka_pose.convention` is null, XYZABC must not be silently converted into `T_base_flange`.
Full 4x4 matrix UDP packets are preferred during commissioning.
Do not guess KUKA A/B/C semantics.

### Point-cloud transport

Do not serialize full point clouds through HTTP.
Large point-cloud data stays on disk. HTTP may return IDs, status, counts, matrices, errors, metadata, and file paths.
Grasshopper loads only the required preview or artifact from disk.

### Mock/production isolation

Mock calibration and mock camera behavior are test-only.
Never weaken checks that prevent synthetic/mock calibration from being used with non-mock hardware.
Do not claim real KUKA, real Zivid, or physical cell safety is verified by software tests.

---

## Important files

Read the relevant files before architecture-level changes:

- `service.py` — FastAPI entry point and service lifecycle.
- `robot_udp_server.py` — UDP robot state receiver, parsing, history, freshness, stationarity.
- `kuka_pose.py` — explicit KUKA XYZABC conversion.
- `zivid_camera_manager.py` — mock/FileCamera/hardware camera abstraction.
- `camera_robot_calibration.py` — Eye-in-Hand sample acquisition and solve.
- `pointcloud_capture.py` — calibrated point-cloud capture and Base-frame transform.
- `compute_wcs_pointcloud.py` — WCS extraction.
- `transform_utils.py` — transform validation/composition.
- `storage_utils.py` — numbered directories and atomic persistence.
- `config_loader.py` — configuration/path loading.
- `config.example.yaml` — shareable safe defaults.
- `tests/` — regression tests.
- `grasshopper/` — Rhino/Grasshopper frontend.
- `docs/` — operation and commissioning documentation.

Do not move core modules into a new package layout unless explicitly requested.

---

## UDP contract

Preferred robot packet:

```json
{
  "type": "POSE",
  "session_id": "kuka-session-001",
  "seq": 1523,
  "timestamp": 12345678,
  "T_base_flange": [
    [1, 0, 0, 1000],
    [0, 1, 0, 0],
    [0, 0, 1, 800],
    [0, 0, 0, 1]
  ],
  "state": "READY"
}
```

Preserve or implement these semantics with backwards-compatible changes and tests:

- `session_id` identifies a sender/controller session.
- `seq` is monotonic within one session.
- duplicate/out-of-order packets must not replace a newer accepted pose.
- a new session must not inherit old stationarity history.
- robot-side timestamps are metadata unless synchronization is explicitly verified.
- freshness uses server-side monotonic receive time.
- sender IP filtering remains configurable for real-cell commissioning.

---

## Capture correctness

A production capture must not rely only on a target pose or fixed sleep.
Preserve or strengthen this sequence:

```text
fresh pose
+
stationary pre-window
        |
        v
capture transaction starts
        |
camera acquisition
        |
        v
pose history across acquisition interval
+
post-capture stationary validation
        |
        v
T_base_camera
        |
        v
point-cloud transform
        |
        v
immutable committed artifacts
```

If pose coverage is insufficient for the acquisition interval, reject or explicitly invalidate the capture.
Do not accept a capture only because `pose_before` and `pose_after` happen to be similar.

---

## Calibration correctness

Eye-in-Hand calibration represents:

```text
T_flange_camera
p_flange = T_flange_camera @ p_camera
```

Calibration artifacts should preserve provenance when available:

- camera serial/model;
- robot/controller identity;
- mounting identity;
- calibration type and timestamp;
- sample count and residuals;
- transform source/target and units;
- Zivid SDK/Python versions;
- software/git revision;
- mock/synthetic classification.

Never invent unavailable hardware IDs. Use null/unknown plus a clear warning.
A valid rigid matrix is not proof that physical frame semantics are correct.

---

## Persistence and capture state

Use unique immutable capture directories:

```text
data/captures/capture_0001/
data/captures/capture_0002/
```

Do not revert to global fixed output filenames.
Where capture manifests exist, downstream consumers should only use committed captures.

Recommended lifecycle:

```text
STARTED
CAPTURED
TRANSFORMED
COMMITTED
FAILED
```

Incomplete or crash-interrupted captures must not be promoted to valid/latest data.
Use atomic writes where practical, especially for manifests, metadata, and preview files.
Do not delete useful failed-capture diagnostics unless explicitly requested.

---

## Grasshopper integration

Grasshopper is a frontend, not a hardware backend.

```text
Grasshopper:
- status display
- command trigger
- Plane/matrix visualization
- preview point loading
- downstream geometry logic

Python:
- UDP
- robot state validation
- Zivid lifecycle
- calibration
- capture transaction
- point-cloud persistence
```

Avoid moving UDP or Zivid lifecycle into Grasshopper.
Mutating HTTP commands should tolerate Grasshopper recomputation safely.
Preserve request-id/idempotency protection if present.
Do not modify `.gh` binary files unless explicitly required.

---

## Safe configuration defaults

Keep shareable defaults conservative:

```yaml
api:
  host: 127.0.0.1

robot_udp:
  bind_ip: 127.0.0.1
```

Do not automatically expose services on `0.0.0.0`.
For real KUKA commissioning, prefer a dedicated robot NIC/VLAN, Windows Firewall rules, configured expected robot sender IP, and explicit local IP binding.
Never overwrite the user's local `config.yaml` when editing `config.example.yaml`.

---

## Runtime data and Git

These are local/runtime artifacts and should not be committed unless explicitly requested:

```text
.venv/
config.yaml
data/calibration/
data/captures/
logs/
*.zdf
*.zfc
```

Do not globally ignore small tracked sample `.xyz` or `.ply` files if intentionally stored under `sample_data/`.

Before finishing code changes:

```powershell
git status
git diff --stat
git diff
```

Do not push automatically unless explicitly requested.
Do not force-push unless explicitly requested and consequences are clear.

---

## Development workflow

Before modifying code:

1. inspect the repository tree;
2. read the relevant implementation and tests;
3. run the existing tests to establish a baseline;
4. identify protections that already exist;
5. avoid duplicate abstractions.

After modifying code, run:

```powershell
python -m pytest -v
```

If UDP behavior changes, test valid, malformed, duplicate, out-of-order, restart/session, stale pose, stationarity reset, timestamp abuse, and transform validation.

If capture behavior changes, test concurrent `/capture`, `/status` during capture, motion during acquisition, insufficient pose coverage, failed artifact writes, immutable IDs, and restart with incomplete capture.

If calibration behavior changes, test mock/non-mock isolation, transform direction, provenance mismatch behavior, and persistence/reload.

Use deterministic hardware-independent tests whenever possible.

---

## Physical commissioning gates

Software tests do not certify physical frame correctness.
Before treating real KUKA pose data as production-valid, perform and document known-pose validation:

```text
Known Pose 1: translation
Known Pose 2: translation on another axis
Known Pose 3: rotation-heavy
Known Pose 4: combined translation + rotation
```

Compare:

```text
Teach Pendant
vs
UDP T_base_flange
vs
Grasshopper Flange Plane
```

Then scan an asymmetric physical fiducial target and verify signed X/Y/Z axes, transformed positions, orientation, and handedness.
Do not mark physical commissioning complete from simulation alone.

---

## Research reproducibility

For research-quality captures, preserve enough metadata to reconstruct a run later:

- capture ID and UTC timestamp;
- UDP session/sequence;
- robot pose and stationarity/freshness metrics;
- capture timing window;
- `T_base_flange`;
- `T_flange_camera`;
- `T_base_camera`;
- calibration artifact/hash;
- Zivid settings/hash;
- camera identity;
- software versions;
- git commit;
- point counts;
- warnings;
- artifact paths.

Avoid secrets and unnecessary machine-specific information.

---

## Future architecture direction

Do not implement closed-loop fabrication unless requested.
The intended scalable direction is:

```text
RoboDK / KRL / GH planner
          |
          | motion intent
          v
        KUKA
          |
          | actual state via UDP
          v
Python Vision Service
          |
          | committed capture/session data
          v
Grasshopper / WCS / Geometry Decision
```

Future multi-view scanning may introduce:

```text
ScanSession
  |- Capture
  |- Capture
  `- Capture
```

Do not base multi-view or fabrication logic on an ambiguous global "latest cloud".

---

## Definition of done

A task is not complete until:

- architectural invariants remain intact;
- affected tests pass;
- mock workflow remains usable unless intentionally changed;
- no runtime data is accidentally staged;
- docs/config examples are updated when behavior changes;
- unsupported hardware claims are not made;
- remaining real-hardware commissioning requirements are stated clearly.

When reporting completion, distinguish:

```text
verified by automated tests
verified by mock integration
not yet verified on real KUKA
not yet verified on real Zivid
not yet verified for physical fabrication
```
