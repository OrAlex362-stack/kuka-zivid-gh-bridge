# Grasshopper Frontend

KUKA_Zivid_Bridge.gh is the Rhino 8 / Grasshopper client for this project. Grasshopper is an HTTP client and geometry frontend; it does not receive KUKA UDP packets, operate the Zivid camera directly, perform hand-eye calibration, or act as the hardware backend.

## Start and Connect

1. From the repository root, start the Python service:

   ~~~powershell
   python service.py
   ~~~

2. In another PowerShell, start the fake robot for mock testing:

   ~~~powershell
   python tools/send_fake_robot_pose.py
   ~~~

   For a real cell, use the commissioned KUKA actual-pose UDP stream instead.

3. Open KUKA_Zivid_Bridge.gh in Rhino 8 / Grasshopper.

4. Set the service base URL to:

   ~~~text
   http://127.0.0.1:8765
   ~~~

   If api.port or api.host differs in config.yaml, use that configured URL.

5. Confirm Robot Status reports a connected, fresh, valid, stationary pose.

6. Confirm the Flange Plane matches T_base_flange. The verified fake pose has origin (1000, 0, 800) and Z axis (0, 0, 1).

7. Trigger Capture only after a valid calibration exists. In mock mode, create the test-only calibration with:

   ~~~powershell
   python tools/create_mock_calibration.py
   ~~~

8. Load the preview_cloud.xyz path returned by POST /capture. Full point clouds remain on disk and are not transported through HTTP.

The .gh definition was moved and renamed without changing its internal Grasshopper logic. Rhino/Grasshopper validation still requires opening the file in Rhino 8 and is not covered by the Python tests.
