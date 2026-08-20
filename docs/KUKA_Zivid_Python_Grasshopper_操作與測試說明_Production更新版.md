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

## 正式 Production Workflow（Hand-Eye Calibration 驗證完成後）

本節定義 **Hand-Eye Calibration 已完成並驗證 `T_flange_camera` 後** 的正式掃描流程。此時不需要每次 Capture 都重新執行 Hand-Eye Solve；只要相機相對於 Robot Flange 的安裝關係沒有改變，已驗證的 `T_flange_camera` 就作為固定 calibration artifact 使用。

正式 Production 的核心關係為：

~~~text
動態 Robot Pose
T_base_flange(t)
        ×
固定 Hand-Eye Calibration
T_flange_camera
        ↓
T_base_camera(t)
=
T_base_flange(t) @ T_flange_camera
        ↓
Zivid Point Cloud
Camera Frame → Robot Base Frame
~~~

### Production 系統資料流

目前以 Grasshopper / Taco 取得 KUKA actual robot state 時，正式資料流為：

~~~text
KUKA KR10 / Taco
        ↓
Grasshopper
        ├── A1-A6
        └── Flange Plane
                ↓
          Robot Pose.py
                ↓
          T_base_flange
                ↓
       GH UDP Publisher
       session_id + seq
                ↓
          UDP :49152
                ↓
       Python RobotUDPServer
                ↓
    freshness / stationary
    pose history / ordering
                ↓
       PointCloudCapture
          + Real Zivid
          + T_flange_camera
                ↓
T_base_camera = T_base_flange @ T_flange_camera
                ↓
      Camera-frame Point Cloud
                ↓
        Robot Base Frame
                ↓
        COMMITTED Capture
        ├── transformed_cloud.ply
        ├── preview_cloud.xyz
        ├── pose_capture_info.yaml
        └── capture_manifest.yaml
                ↓
        /wcs/compute
                ↓
             WCS
                ↓
         Grasshopper
     Geometry / Visualization
~~~

> 若未來 KUKA Controller 可直接輸出符合相同 protocol 的 UDP `T_base_flange`，可省略 Grasshopper telemetry adapter，但 Python `RobotUDPServer`、Calibration、Capture 與 WCS pipeline 不需要改寫。

### Production 前置條件

進入正式 Capture 前，必須同時滿足：

- Real KUKA 的 `T_base_flange` frame semantics 已完成實機驗證。
- `T_base_flange` 與 Grasshopper Flange Plane / Teach Pendant 的已知姿態測試一致。
- Real Eye-in-Hand Calibration 已完成並取得有效 `T_flange_camera`。
- `T_flange_camera` 的方向、相機 serial / provenance 與目前硬體相符。
- Zivid 由 Python Backend 管理；不要同時讓舊 GH Zivid script 與 Backend 搶同一台 Real Zivid。
- GH UDP Publisher 持續發送 ordered robot state。
- `session_id` 在同一 sender lifecycle 內保持固定，`seq` 持續遞增。
- Python `/status` 顯示 Robot pose fresh；Robot 停止並經 settle window 後 `stationary = true`。
- 正式 Production 時不要同時執行 `tools/send_fake_robot_pose.py`。

### Production 啟動順序

#### Step 1 — 啟動 Python Backend

~~~powershell
python service.py
~~~

API 文件：

~~~text
http://127.0.0.1:8765/docs
~~~

先確認：

~~~powershell
Invoke-RestMethod http://127.0.0.1:8765/health
Invoke-RestMethod http://127.0.0.1:8765/status
~~~

正式環境應確認 Camera、Calibration 與 Robot State 均為預期模式。

#### Step 2 — 啟動 KUKA / Taco 與 GH UDP Publisher

Grasshopper Robot State 區：

~~~text
KUKA / Taco
   ├── A1-A6
   └── Flange Plane
          ↓
     Robot Pose.py
          ↓
     T_base_flange
          ↓
     UDP Publisher
          ↓
     127.0.0.1:49152
~~~

建議先以約 10 Hz 持續發送；不要在 GH Python component 中使用 blocking `while True`。

Robot 即使靜止，也要持續送相同姿態的新 packet：

~~~text
seq 100 → Pose A
seq 101 → Pose A
seq 102 → Pose A
seq 103 → Pose A
...
~~~

這是 Backend 建立 stationarity 與 capture interval coverage 所需要的資料。

#### Step 3 — 驗證 Robot State Loopback

~~~powershell
Invoke-RestMethod http://127.0.0.1:8765/robot/pose
Invoke-RestMethod http://127.0.0.1:8765/status
~~~

確認：

~~~text
Taco Flange Plane
=
Robot Pose.py T_base_flange
=
GH UDP packet T_base_flange
=
Python /robot/pose T_base_flange
=
Robot Status GH Plane
~~~

Robot 移動時：

~~~text
stationary = false
~~~

Robot 完全停止並超過 settle window 後：

~~~text
fresh = true
stationary = true
~~~

#### Step 4 — Robot 移至掃描姿態

Robot motion 可由 KRL、Jog、Taco / Grasshopper、RoboDK 或其他 planner 完成。

Python Vision Backend 不發送 Robot motion command。

到達目標姿態後，等待：

~~~text
fresh = true
stationary = true
~~~

不要使用固定 `sleep` 當作 Robot 已穩定的唯一證據。

#### Step 5 — 觸發 Production Capture

Grasshopper `Capture Client` 或 PowerShell 呼叫：

~~~powershell
Invoke-RestMethod -Method POST -Uri http://127.0.0.1:8765/capture
~~~

Capture Client **不需要傳入 `T_base_flange`**。Backend 會直接使用 `RobotUDPServer` 的 pose history。

Backend 在一次 Capture 中負責：

~~~text
pre-capture fresh / stationary check
        ↓
記錄 robot pose / capture timing
        ↓
Zivid acquisition
        ↓
驗證 acquisition interval 的 pose coverage
        ↓
post-capture stationary / motion check
        ↓
讀取已驗證 T_flange_camera
        ↓
T_base_camera = T_base_flange @ T_flange_camera
        ↓
Point Cloud Camera → Base
        ↓
寫入 artifacts
        ↓
COMMITTED
~~~

若 Robot pose coverage 不足、Robot 在 acquisition interval 移動、Calibration 不可用或 artifact 寫入失敗，Capture 不應被視為有效 Production data。

#### Step 6 — 載入 Base-frame Point Cloud

Capture 成功後，使用該次 response 回傳的 exact file path，而不是自行搜尋「latest cloud」。

~~~text
Capture Client
      ↓
preview_path
      ↓
Load Preview
      ↓
preview_cloud.xyz
      ↓
Grasshopper PointCloud
~~~

完整分析資料使用：

~~~text
transformed_cloud.ply
~~~

其中點座標已位於：

~~~text
KUKA Robot Base Frame
~~~

#### Step 7 — 計算 WCS

使用該次明確的 `capture_id`：

~~~powershell
$body = @{ capture_id = "capture_0001" } | ConvertTo-Json
Invoke-RestMethod `
  -Method POST `
  -Uri http://127.0.0.1:8765/wcs/compute `
  -ContentType "application/json" `
  -Body $body
~~~

資料流：

~~~text
COMMITTED Base-frame Point Cloud
        ↓
Crop / Plane / Feature Detection
        ↓
WCS
        ↓
Grasshopper
        ↓
Workpiece Geometry / Localization
~~~

#### Step 8 — 下一個 Robot Pose / 下一次 Capture

Hand-Eye Calibration 不需要重新 Solve。

Robot 移至新的掃描 Pose 後，UDP 會得到新的：

~~~text
T_base_flange(t+1)
~~~

Backend 再使用同一個固定：

~~~text
T_flange_camera
~~~

計算新的：

~~~text
T_base_camera(t+1)
=
T_base_flange(t+1) @ T_flange_camera
~~~

因此可以建立：

~~~text
Capture 0001 → Base Frame
Capture 0002 → Base Frame
Capture 0003 → Base Frame
...
~~~

這也是後續 Multi-view Point Cloud Registration 的基礎。

### Hand-Eye Calibration 何時必須重做

以下任一情況發生後，不應繼續直接使用舊 `T_flange_camera`：

- Zivid Camera 被拆下或重新安裝。
- Camera mount / bracket 被更換或鬆動。
- Camera 或 Robot Flange 發生碰撞。
- Camera serial / hardware identity 改變。
- Robot flange / tool reference 定義改變。
- Calibration provenance 與目前硬體不符。
- Physical fiducial validation 顯示明顯偏移或旋轉誤差。

此時流程應回到：

~~~text
Real Hand-Eye Calibration
        ↓
重新求解 T_flange_camera
        ↓
Physical Validation
        ↓
Production Capture
~~~

### Production 中各元件的責任

| 元件 | 正式 Production 責任 |
|---|---|
| KUKA / Taco | Robot motion 與 actual robot state source |
| Robot Pose.py | Flange Plane → `T_base_flange` |
| GH UDP Publisher | 將 Robot state 送至 UDP :49152 |
| RobotUDPServer | session / seq / freshness / stationarity / pose history |
| Robot Status | Backend → GH 狀態與 pose 驗證 |
| ZividCameraManager | Backend 唯一 Camera owner |
| CameraRobotCalibration | 保存 / 驗證 `T_flange_camera` |
| Capture Client | 觸發 `POST /capture` |
| PointCloudCapture | Zivid acquisition、`T_base_camera`、Base-frame point cloud、transaction |
| Load Preview | 讀取指定 capture 的 `preview_cloud.xyz` |
| WCS | 從指定 COMMITTED capture 計算工作座標 |

### Production 不要做

~~~text
不要同時啟動 Fake Robot sender 與 GH UDP Publisher
不要用 Robot Status Plane 回送 UDP
不要在 GH UDP Publisher 重新做 FK
不要用 Joint Axis 覆蓋 T_base_flange
不要猜 KUKA ABC convention
不要讓 GH 與 Backend 同時控制 Real Zivid
不要讓 Capture Client 自己決定 Robot 是否安全/穩定
不要把完整 Point Cloud 經 HTTP 或 UDP 傳送
不要讓 downstream 使用未 COMMITTED capture
不要在每次 Production Capture 重新 Solve Hand-Eye
~~~

### Production 一句話流程

~~~text
KUKA 移動到掃描 Pose
        ↓
GH UDP Publisher 持續送 T_base_flange
        ↓
Backend 確認 fresh + stationary
        ↓
POST /capture
        ↓
Zivid Capture
        ↓
T_base_camera = T_base_flange @ T_flange_camera
        ↓
Point Cloud → Robot Base
        ↓
COMMITTED Capture
        ↓
Load Preview / WCS
        ↓
Grasshopper Geometry
        ↓
移至下一 Pose 並重複
~~~

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
