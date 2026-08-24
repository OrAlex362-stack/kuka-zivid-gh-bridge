# KUKA、Zivid、Python 與 Grasshopper 操作與測試說明

## 系統目的

本專案將 KUKA 機器人實際姿態、Zivid Eye-in-Hand 相機、外部 Python 後端，以及 Rhino 8 / Grasshopper 前端串接在一起。

資料流如下：

~~~text
KUKA 或 Fake Robot
        |
        | UDP：實際姿態與狀態
        v
Python Service
        |
        | HTTP：狀態、命令、矩陣、檔案路徑
        v
Grasshopper
~~~

Python 服務只接收機器人狀態，不會送出機器人運動命令。完整點雲保留在磁碟，HTTP 不傳送整份點雲；Grasshopper 使用 preview_cloud.xyz 作幾何預覽。

## 座標與單位約定

所有轉換矩陣都採用：

~~~text
T_target_source
p_target = T_target_source @ p_source
~~~

Eye-in-Hand 串接固定為：

~~~text
T_base_camera =
T_base_flange @ T_flange_camera
~~~

請勿交換矩陣順序或自行隱式反矩陣。平移與點座標單位皆為 mm。

## 第一次安裝

環境需求：

- Windows
- Python 3.10
- Rhino 8 / Grasshopper
- Zivid SDK 2.18.0
- Zivid Python wrapper 2.18.0
- Open3D 0.19.0

在專案根目錄開啟 PowerShell：

~~~powershell
py -3.10 -m venv .venv
..venvScriptsActivate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
Copy-Item config.example.yaml config.yaml
~~~

若要執行測試：

~~~powershell
pip install -r requirements-dev.txt
python -m pytest -v
~~~

Zivid 原生 SDK 必須另外安裝；pip 只會安裝 Python wrapper，不會安裝原生 SDK。

## Mock 模式操作

確認 config.yaml 內：

~~~yaml
zivid:
  mode: mock
~~~

若 data/calibration/calibration_results.yaml 尚不存在，建立測試專用 mock calibration：

~~~powershell
python tools/create_mock_calibration.py
~~~

此檔案只供 Mock 測試。請勿用於實體機器人或實體相機。

第一個 PowerShell 啟動服務：

~~~powershell
python service.py
~~~

第二個 PowerShell 啟動 Fake Robot：

~~~powershell
python tools/send_fake_robot_pose.py
~~~

預設 Fake Robot 會送出：

- Origin = (1000, 0, 800)
- ZAxis = (0, 0, 1)
- T_base_flange 為單位旋轉與上述平移

## HTTP 驗證

健康檢查：

~~~powershell
Invoke-RestMethod http://127.0.0.1:8765/health
~~~

整體狀態：

~~~powershell
Invoke-RestMethod http://127.0.0.1:8765/status
~~~

最新機器人姿態：

~~~powershell
Invoke-RestMethod http://127.0.0.1:8765/robot/pose
~~~

正常 Mock 狀態應包含：

- service = ready
- robot.connected = true
- robot.pose_valid = true
- robot.stationary = true
- camera.mode = mock
- T_base_flange 可用

觸發點雲擷取：

~~~powershell
Invoke-RestMethod -Method POST -Uri http://127.0.0.1:8765/capture
~~~

預設 Mock 擷取預期為 36,000 點，preview 為 10,000 點，並建立新的 data/captures/capture_NNNN 目錄。

## Grasshopper 操作

1. 啟動 Python service。
2. 啟動 Fake Robot，或確認實體 KUKA UDP 已送出實際姿態。
3. 在 Rhino 8 / Grasshopper 開啟 grasshopper/KUKA_Zivid_Bridge.gh。
4. Service Base URL 設為 http://127.0.0.1:8765；若 config.yaml 使用其他埠號，則改用實際設定。
5. 確認 Robot Status。
6. 確認 Flange Plane。
7. 觸發 Capture。
8. 載入回傳路徑中的 preview_cloud.xyz。

Grasshopper 是 HTTP client 與 geometry frontend，不是硬體後端。

## API

| Method | Endpoint | 說明 |
|---|---|---|
| GET | /health | 服務與相機基本健康狀態 |
| GET | /status | 機器人、相機、校正、擷取與 WCS 狀態 |
| GET | /robot/pose | 最新姿態與 T_base_flange |
| POST | /calibration/sample | 新增 Eye-in-Hand 樣本 |
| POST | /calibration/solve | 求解 T_flange_camera |
| POST | /calibration/reset | 封存並重設 active calibration |
| POST | /capture | 擷取並轉換至 Base 座標 |
| POST | /wcs/compute | 從最新或指定 capture 計算 WCS |

## 動態 Eye-in-Hand 校正

實體校正時，先將 KUKA 手動移至穩定且可看到 Zivid calibration board 的姿態，再依序執行：

~~~powershell
Invoke-RestMethod -Method POST -Uri http://127.0.0.1:8765/calibration/sample
Invoke-RestMethod -Method POST -Uri http://127.0.0.1:8765/calibration/solve
~~~

應以多組不同位置與方向取得樣本。target_sample_count 是 UI 引導值，不是強制上限。重設時：

~~~powershell
Invoke-RestMethod -Method POST -Uri http://127.0.0.1:8765/calibration/reset
~~~

既有資料會移至 data/calibration/archive，不會靜默刪除。

## 輸出資料

每次擷取會建立 data/captures/capture_NNNN，內容可能包括：

- original_pointcloud.mock.npz：Mock 原始資料
- original_pointcloud.zdf：實體或 FileCamera 原始資料
- transformed_cloud.ply：Base 座標完整點雲
- preview_cloud.xyz：Grasshopper 預覽點雲
- pose_capture_info.yaml：擷取前後姿態與矩陣

校正、擷取與 log 都是本機 runtime 資料，已由 .gitignore 排除。sample_data 只保留可分享的小型 Mock 範例。

## Mock 與實體硬體界線

| 項目 | Mock / 測試 | 實體 / 生產 |
|---|---|---|
| Robot | Fake Robot | Real KUKA UDP |
| Camera | Mock Camera | Real Zivid 或 FileCamera |
| Calibration | Mock calibration，僅測試 | Real Eye-in-Hand calibration |

實體上線前仍需完成 KUKA 網路、A/B/C convention、Zivid settings、Hand-Eye 精度、光線、材質、工作距離、時間與安全圍籬的 commissioning。

## 停止與故障排查

- 使用 Ctrl+C 分別停止 Fake Robot 與 Python service。
- /health 若為 degraded，先檢查 camera.mode 與 Zivid SDK。
- robot.connected 若為 false，檢查 robot_udp.bind_ip、port 與 Fake Robot/KUKA 傳送端。
- robot.stationary 若為 false，等待 settle time，並檢查姿態新鮮度與移動容差。
- /capture 若回報 CALIBRATION_REQUIRED，先建立 Mock calibration 或完成實體 Hand-Eye calibration。
- WCS 若回報 dependency missing，確認使用 Python 3.10 環境且已安裝 Open3D 0.19.0。

Grasshopper 必須在 Rhino 8 中實際開啟才能確認元件、路徑與 UI；Python 自動測試不代表 Grasshopper 已完成驗證。

## Multi-view Scan Workflow and Colored Preview

This workflow assumes Eye-in-Hand calibration is already solved and `data/calibration/calibration_results.yaml` contains a validated `T_flange_camera` with:

```text
source_frame: camera
target_frame: flange
T_base_camera = T_base_flange @ T_flange_camera
```

The service never derives production pose from A1-A6. Taco Flange Plane remains the source of `T_base_flange` over UDP. A1-A6 may be logged as diagnostics only.

Production scan sequence:

```text
CALIBRATION

Taco Flange Plane
+
Zivid Board Capture

Hand-Eye

T_flange_camera

calibration_results.yaml


MULTI-VIEW SCAN

POST /scan/start

Pose 01
Stationary
POST /scan/capture
T_base_flange_01
T_base_camera_01 = T_base_flange_01 @ T_flange_camera
P_base_01

Pose 02
Stationary
POST /scan/capture
T_base_flange_02
T_base_camera_02 = T_base_flange_02 @ T_flange_camera
P_base_02

...

POST /scan/merge

Concatenate Base-frame clouds
Optional bounded Point-to-Plane ICP refinement
Voxel Downsample
Statistical Outlier Removal
Colored merged cloud
merged_preview.xyzrgb
Grasshopper Colored PointCloud
```

### Scan Client Role

scan client 是一個 Grasshopper 或 HTTP 端的操作介面。它觸發掃描生命週期命令並載入返回的預覽路徑，但它不管理機器人狀態、Zivid 生命週期、標定或點雲持久化。 Python 後端仍然是唯一負責 UDP 位元姿驗證、Zivid 資料擷取、手眼變換、基底座標系點雲輸出和掃描清單提交的後端。機器人運動仍然必須透過 KRL、Jog、Grasshopper 規劃、RoboDK 模擬/規劃或其他委託的運動層在外部實現。
Scan client responsibilities:

- call `/scan/start`, `/scan/capture`, `/scan/status`, `/scan/merge`, `/scan/finish`, and `/scan/reset`;
- display robot/camera/calibration/scan status returned by `/status` and `/scan/status`;
- load small `.xyz` or `.xyzrgb` preview files from disk paths returned by HTTP;
- avoid loading full PLY/ZDF/NPZ files into Grasshopper for responsive preview.

Backend responsibilities:

- reject stale, missing, or moving robot poses;
- run the existing production capture transaction for each scan capture;
- transform points into Robot Base using `T_base_camera = T_base_flange @ T_flange_camera`;
- keep large point clouds and manifests on disk;
- 將基礎幀捕獲與 Open3D 合併，無需透過 HTTP 發送完整的點雲。

### Scan API

| Method | Endpoint | Purpose |
|---|---|---|
| POST | `/scan/start` | Create `scan_NNNN` and make it active. Fails if another scan is active. |
| POST | `/scan/capture` | Run the existing production capture gates, transform to Robot Base, and add the committed capture to the active scan. |
| GET | `/scan/status` | Return active scan id, capture count, capture ids, and merge state. |
| POST | `/scan/merge` | Merge all Base-frame colored captures without reapplying transforms. |
| POST | `/scan/finish` | Merge if needed, finalize `scan_manifest.yaml`, and close the active scan. |
| POST | `/scan/reset` | Reset only the active in-memory session; completed scan folders are not deleted. |

### Scan Output Structure

```text
data/scans/
  scan_0001/
    captures/
      capture_0001/
        scan_capture.yaml
      capture_0002/
        scan_capture.yaml
    merged_cloud_raw.ply
    merged_cloud_pre_icp.ply   # only when ICP is enabled
    merged_cloud_downsampled.ply
    merged_preview.xyz
    merged_preview.xyzrgb
    scan_manifest.yaml
```

Each `scan_capture.yaml` references the committed single-capture artifacts under `data/captures/capture_NNNN/`. Large original ZDF/NPZ files are not duplicated into the scan folder. The scan manifest records `T_base_flange`, `T_flange_camera`, `T_base_camera`, pose age, stationarity, point counts, RGB availability, merge parameters, ICP diagnostics when enabled, and output artifact paths.

Colored preview files:

- `preview_cloud.xyz` remains legacy XYZ-only: `X Y Z`.
- `preview_cloud.xyzrgb` is added when color is available: `X Y Z R G B`.
- `merged_preview.xyz` and `merged_preview.xyzrgb` are produced by `/scan/merge`.
- RGB is filtered and downsampled together with XYZ. XYZ transforms change coordinates only; RGB indexing is preserved.

Merge configuration defaults:

```yaml
scan:
  enabled: true
  merge:
    voxel_size_mm: 2.0
    remove_statistical_outlier:
      enabled: true
      nb_neighbors: 20
      std_ratio: 2.0
    icp:
      enabled: false
      method: point_to_plane
      voxel_size_mm: 2.0
      max_correspondence_distance_mm: 5.0
      max_iterations: 50
      normal_radius_mm: 8.0
      normal_max_nn: 30
      min_fitness: 0.30
      max_inlier_rmse_mm: 3.0
      max_translation_correction_mm: 5.0
      max_rotation_correction_deg: 1.0
      failure_policy: use_initial_alignment
  preview:
    max_points: 100000
```

ICP is not the registration source. Robot UDP `T_base_flange` plus fixed Eye-in-Hand `T_flange_camera` is the global registration, and `base_ply` files are already approximately aligned in Robot Base frame. ICP starts from identity, estimates normals only on downsampled registration clouds, and applies an accepted `Delta T` only to the original source cloud for that capture.

ICP is bounded by fitness, RMSE, translation, rotation, finite-transform, and point-count checks. With `failure_policy: use_initial_alignment`, rejected ICP corrections do not abort a scan; the original robot/hand-eye Base-frame alignment is merged and the manifest records the rejection reason and metrics. ICP improves multi-view registration consistency only. It does not improve Zivid sensor depth accuracy or certify physical frame semantics.

Grasshopper usage:

1. Use HTTP buttons/components to call `/scan/start`.
2. Move the robot externally, wait for Robot Status to show fresh and stationary.
3. Call `/scan/capture`.
4. Repeat motion and capture.
5. Call `/scan/merge` or `/scan/finish`.
6. Load `merged_preview.xyzrgb` with the Load Preview Python component logic in `grasshopper/load_preview.py`.

Troubleshooting:

- `ROBOT_NOT_CONNECTED`: no fresh UDP stream is being received.
- `ROBOT_POSE_STALE`: the latest server-side receive time exceeds `robot_udp.max_pose_age_ms`.
- `ROBOT_NOT_STATIONARY`: wait for the configured settle window or reduce actual robot motion.
- `CALIBRATION_REQUIRED`: create mock calibration for mock tests or complete real Eye-in-Hand calibration.
- `SCAN_ALREADY_ACTIVE`: finish or reset the current scan before starting another.
- `SCAN_DEPENDENCY_MISSING`: run the service in the project `.venv` or another Python environment with Open3D 0.19 installed.
- `COLOR_DATA_INVALID`: a cloud has mismatched XYZ/RGB point counts or invalid RGB values.

Physical commissioning is still required before treating real KUKA/Zivid scan data as production-valid. Automated tests verify software behavior and mock/Open3D integration only; they do not certify physical frame semantics, real KUKA safety, real Zivid acquisition, or fabrication readiness.

### Current Project Change Summary

- Added `scan_session.py` as the multi-view scan session manager.
- Added FastAPI routes `/scan/start`, `/scan/capture`, `/scan/status`, `/scan/merge`, `/scan/finish`, and `/scan/reset`.
- Added scan state to `/status`.
- Added `scan:` configuration defaults and validation in `config_loader.py`.
- Added `paths.scans` and default `data/scans` output location.
- Added colored capture previews via `preview_cloud.xyzrgb` when RGB data is available.
- Added `pointcloud_io.py` helpers for finite XYZ/RGB filtering, `.xyzrgb` writing, and `.xyz`/`.xyzrgb` reading.
- Added Grasshopper Load Preview source at `grasshopper/load_preview.py` to load colored scan/capture previews.
- Updated `grasshopper/KUKA_Zivid_Bridge.gh` to carry the frontend changes.
- Added tests for scan sessions, scan route availability, colored preview I/O, missing calibration, stale pose, and moving robot rejection.
- Updated `settings/capture_settings.yml` from unset placeholders to loadable single-acquisition values: Aperture 8.0, Brightness 1.0, ExposureTime 5000, Gain 1.0, Diagnostics disabled.

Not changed:

- The backend still does not send KUKA motion commands.
- Full point clouds are still not serialized through HTTP.
- Mock calibration remains test-only and must not be used with real hardware.
- Physical KUKA/Zivid/fabrication commissioning is still not certified by software tests.

## Deviation Projection Workflow

This stage projects a deviation map back onto the physical timber with the Zivid L100 projector. Grasshopper remains an HTTP client and geometry frontend. It does not connect to the Zivid camera and does not generate projector pixels or BGRA images.

Formal data flow:

```text
Merged Base-frame Scan
        +
Design Geometry

Deviation Analysis

P_base + deviation
         HTTP
Projection Service

Current T_base_flange
        +
T_flange_camera

T_base_camera
         inverse
P_camera

Zivid pixels_from_3d_points

Projector Image

Zivid L100

Physical Timber
```

### Coordinate Chain

Deviation points sent from Grasshopper must already be in KUKA Robot Base coordinates:

```text
P_base
```

Projection always uses the current robot pose, not the pose saved in an old scan capture:

```python
T_base_camera_current = T_base_flange_current @ T_flange_camera
T_camera_base = np.linalg.inv(T_base_camera_current)
P_camera = T_camera_base @ P_base
```

Only camera-frame points may be sent to Zivid projection:

```python
zivid.projection.pixels_from_3d_points(camera, points_camera)
```

Do not pass `P_base` directly to `pixels_from_3d_points`. Do not swap the transform order. Do not use a stale `T_base_camera_capture_NNNN` for projection.

### Projection API

| Method | Endpoint | Purpose |
|---|---|---|
| POST | `/projection/deviation` | Transform Base-frame points to current Camera frame, rasterize a deviation map, save artifacts, and start projector display. |
| POST | `/projection/stop` | Stop the retained Zivid projection handle and mark projection inactive. Safe to call repeatedly. |
| GET | `/projection/status` | Return active state, projection id, type, image path, projected point count, and last stop/error metadata. |

Request example:

```json
{
  "points_base": [
    [1000.0, 100.0, 800.0],
    [1001.0, 100.0, 800.2],
    [1002.0, 100.0, 800.5]
  ],
  "deviations_mm": [0.3, 1.8, 4.2],
  "mode": "absolute",
  "good_tolerance_mm": 1.0,
  "warning_tolerance_mm": 3.0,
  "palette": "pure_rgb",
  "point_radius_px": 3,
  "opacity": 255
}
```

Response example:

```json
{
  "ok": true,
  "projection_id": "projection_0001",
  "active": true,
  "input_points": 5000,
  "valid_camera_points": 4820,
  "visible_projector_points": 4512,
  "image_path": "data/projections/projection_0001/projector_image.png",
  "T_base_camera": [[1, 0, 0, 1000], [0, 1, 0, 0], [0, 0, 1, 800], [0, 0, 0, 1]]
}
```

Stop example:

```powershell
Invoke-RestMethod -Method POST -Uri http://127.0.0.1:8765/projection/stop
```

### Deviation Modes and Color Palette

`absolute` mode uses `abs(deviation_mm)`:

```text
d <= good_tolerance_mm       GOOD
good < d <= warning          WARNING
d > warning                  BAD
```

`pure_rgb` is the production default because projector brightness is prioritized. The NumPy projector image is BGRA, not RGB:

```python
GOOD    = GREEN = (0, 255, 0, 255)
WARNING = BLUE  = (255, 0, 0, 255)
BAD     = RED   = (0, 0, 255, 255)
BLACK   =        (0, 0, 0, 255)
```

`semantic` is optional and uses green / yellow / red for GOOD / WARNING / BAD.

`signed` mode preserves sign categories. Near-zero values use green, negative values use blue, and positive values use red in `pure_rgb`.

### Rasterization and Visibility

The service creates a projector image with:

```text
shape: height x width x 4
dtype: np.uint8
background: BGRA black
```

Each visible projected point draws a filled circle using `point_radius_px`. Uncovered regions remain black. The first version deliberately avoids image-space interpolation so colors do not smear across physical feature boundaries.

Projection filtering is synchronized across XYZ and deviation arrays:

- finite Base-frame XYZ and finite deviation values;
- Camera-frame `Z > 0`;
- finite projector pixels;
- `0 <= u < width` and `0 <= v < height`.

A projector-space z-buffer is applied during rasterization. If multiple geometry layers hit the same projector pixel or circle neighborhood, the point with the smallest Camera-frame Z wins, preventing obvious back-surface overwrite.

### Point Count Policy

Do not send hundreds of thousands or millions of deviation points through HTTP. The service enforces `projection.deviation.max_points`. If input exceeds the limit, deterministic priority sampling keeps BAD / large-deviation points first, then WARNING, then GOOD. If BAD alone exceeds the limit, BAD points are deterministically downsampled.

Default config:

```yaml
projection:
  enabled: true
  deviation:
    mode: absolute
    good_tolerance_mm: 1.0
    warning_tolerance_mm: 3.0
    palette: pure_rgb
    point_radius_px: 3
    max_points: 20000
  require_robot_stationary: true
  require_fresh_pose: true

paths:
  projections: data/projections
```

### Projection Artifacts

Each projection creates:

```text
data/projections/
  projection_0001/
    projector_image.png
    projection_manifest.yaml
```

The manifest records input frame, units, current `T_base_flange`, calibration `T_flange_camera`, computed `T_base_camera`, deviation settings, point counts, projector resolution, radius, and artifact paths.

### Capture vs Projection State

Zivid capture and projection share the same production camera owner: Python Service.

Before `POST /capture` or `POST /scan/capture` begins a 3D acquisition, the service calls projection stop with reason `3d_capture`. This prevents service state from reporting `projection.active == true` after the camera has stopped projecting for 3D capture.

If `/projection/status` detects that the robot is no longer connected, fresh, valid, or stationary while projection is active, the service stops the projection and records `last_stop_reason: robot_not_stationary_or_stale`.

### Grasshopper Usage

Typical Grasshopper-side source:

```text
Design Mesh / Surface
        +
Scanned Mesh / PointCloud

Closest Point / Distance

Deviation scalar

Point + Deviation

Projection Client
```

Grasshopper sends only:

```text
P_base
deviation_mm
thresholds
palette
point radius
opacity
```

Grasshopper must not send:

```text
projector pixels
BGRA image
camera-frame points
```

If Rhino World and Robot Base are not the same frame, do not silently assume they match. Convert upstream:

```text
Rhino World -> Robot Base
```

Only then call `/projection/deviation`.

### Projection Validation

Do not start with a full deviation map on real hardware. Validate in order:

1. Project 3 known Base-frame points.
2. Project one simple line or feature boundary.
3. Project a categorical low-density deviation map.
4. Compare projected locations against physical timber features.
5. Only after Step 1 point accuracy is correct should full deviation maps be used.

Software tests do not certify physical projector alignment, KUKA safety, real Zivid L100 projection, or fabrication readiness.

### Projection Troubleshooting

- `ROBOT_NOT_CONNECTED`: no current UDP robot stream is available.
- `ROBOT_POSE_STALE`: latest pose receive time is too old for projection.
- `ROBOT_NOT_STATIONARY`: wait until stationarity checks pass before starting projection.
- `CALIBRATION_REQUIRED`: solve Eye-in-Hand calibration first.
- `IDENTITY_CALIBRATION_REJECTED`: projection requires an explicit non-identity `T_flange_camera`.
- `CAMERA_UNAVAILABLE`: the Python Service does not own a connected projection-capable Zivid camera.
- `CAMERA_PROJECTION_UNAVAILABLE`: the installed Zivid wrapper does not expose `zivid.projection`.
- `PROJECTION_INPUT_LENGTH_MISMATCH`: `points_base` and `deviations_mm` lengths differ.
- `PROJECTION_NO_VALID_POINTS`: all rows were non-finite or invalid after filtering.
