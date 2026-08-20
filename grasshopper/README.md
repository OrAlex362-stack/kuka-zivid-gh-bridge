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

## Colored Load Preview

The maintainable Python source for the Grasshopper Load Preview component is `grasshopper/load_preview.py`.

Inputs:

```text
preview_path
```

`preview_path` may be a direct `.xyzrgb`/`.xyz` file path or a capture/scan directory. The loader checks colored files first:

```text
merged_preview.xyzrgb
preview_cloud.xyzrgb
merged_preview.xyz
preview_cloud.xyz
```

Outputs:

```text
cloud   Rhino.Geometry.PointCloud
points  list[Rhino.Geometry.Point3d]
colors  list[System.Drawing.Color], empty for legacy XYZ
count   point count
info    load status
```

For `.xyzrgb`, each line is:

```text
X Y Z R G B
```

The component adds each point with `PointCloud.Add(Point3d, Color.FromArgb(r, g, b))`, so Rhino receives per-point color. Legacy `.xyz` remains supported and creates an uncolored PointCloud.

For multi-view scans, call `/scan/merge` or `/scan/finish`, then load the returned `files.preview_xyzrgb` path. Do not load full multi-million point PLY files in Grasshopper for preview.
## Component Scripts To Replace

Do not edit `KUKA_Zivid_Bridge.gh` as a binary file in Git. Open it in Rhino 8 / Grasshopper and replace the matching GhPython component source with these text files:

| Grasshopper component | Source file | Purpose |
|---|---|---|
| UDP Publisher | `grasshopper/udp_publisher.py` | Publishes `T_base_flange` from either `Rhino.Geometry.Plane` or `Rhino.Geometry.Transform` with matrix metadata fixed to flange -> base, units mm. |
| Async Capture Client | `grasshopper/async_capture_client.py` | Starts `POST /capture` on a background thread from a button rising edge. |
| KUKA Zivid Async Scan Client | `grasshopper/async_scan_client.py` | Starts `/scan/start`, `/scan/capture`, `/scan/merge`, `/scan/finish`, `/scan/reset` on one background HTTP worker at a time. |
| Load Preview | `grasshopper/load_preview.py` | Loads `.xyzrgb` first, then legacy `.xyz`, for capture or scan preview. |

The async clients intentionally do not call `ghenv.Component.ExpireSolution()` from worker threads. Use a Grasshopper Timer to refresh the components while `busy` is true so the UDP Publisher keeps sending sequence-incrementing pose packets during long Zivid capture or Open3D merge calls.

Error parsing priority is:

```text
error_code -> error -> code -> details.stationary_during_capture.reason -> UNKNOWN_ERROR
```

Backend JSON `details` are preserved on the `details` output for diagnostics such as `stage`, `capture_directory`, and `stationary_during_capture`.
