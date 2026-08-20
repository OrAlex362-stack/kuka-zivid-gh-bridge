# 2026-08-20 Physical Base-Frame Validation Snapshot

Documentation / validation snapshot only. Do NOT use as runtime calibration.

This public snapshot records the 2026-08-20 physical validation milestone for the KUKA-Zivid-Grasshopper bridge. Runtime artifacts such as `data/calibration/`, `data/captures/`, full `.zdf`, full `.ply`, absolute workstation paths, and physical camera serial numbers are intentionally excluded.

## Validated System

- Robot: KUKA KR10 R1100 sixx
- Camera: Zivid Two+ L100
- Calibration: Eye-in-Hand, 10 poses
- Transform convention: `T_target_source`, `p_target = T_target_source @ p_source`
- Validated composition: `T_base_camera = T_base_flange @ T_flange_camera`
- Coordinate frame for merged points: Robot Base, millimetres

## Physical Result

- Multi-view captures: 6
- All captures stationary: yes
- Raw point count: 10,026,462
- Final merged point count: 2,069,804
- Voxel size: 2.0 mm
- Statistical outlier removal: enabled
- ICP: OFF
- Physical Rhino / Robot XYZ deviation: approximately 0.10.2 mm

The `0.10.2 mm` value is a physical reference measurement against the Rhino/robot XYZ reference. It is not a calibration residual.

## Files

- `calibration_results.sanitized.yaml`: public calibration provenance, `T_flange_camera`, residuals, and quality metrics with camera serial removed.
- `scan_manifest.sanitized.yaml`: public 6-view Base-frame scan manifest with transform matrices and merge metrics, but without absolute workstation paths or large runtime artifacts.

## Baseline Interpretation

Robot kinematics + Eye-in-Hand calibration + Base-frame transformation is the current registration baseline. This validation snapshot is the ICP OFF baseline. Future comparisons should evaluate:

- A. Robot + Eye-in-Hand only
- B. Robot + Eye-in-Hand + ICP

Do not redefine or invert the transform chain while doing those comparisons.
