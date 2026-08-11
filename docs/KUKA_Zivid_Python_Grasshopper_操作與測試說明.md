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
