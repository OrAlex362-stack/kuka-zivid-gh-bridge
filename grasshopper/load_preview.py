"""Grasshopper Load Preview component script.

Inputs expected in Grasshopper:
    preview_path: file path to preview_cloud.xyz, preview_cloud.xyzrgb,
                  merged_preview.xyz, merged_preview.xyzrgb, or a capture/scan folder

Outputs:
    cloud: Rhino.Geometry.PointCloud
    points: list[Rhino.Geometry.Point3d]
    colors: list[System.Drawing.Color] or []
    count: point count
    info: load status text
"""

from __future__ import annotations

import os

try:
    import Rhino.Geometry as rg
    from System.Drawing import Color
except Exception:  # Allows lightweight parser checks outside Rhino.
    rg = None
    Color = None


def _candidate_paths(path):
    if not path:
        return []
    value = os.path.abspath(str(path))
    if os.path.isdir(value):
        return [
            os.path.join(value, "merged_preview.xyzrgb"),
            os.path.join(value, "preview_cloud.xyzrgb"),
            os.path.join(value, "merged_preview.xyz"),
            os.path.join(value, "preview_cloud.xyz"),
        ]
    root, ext = os.path.splitext(value)
    if ext.lower() == ".xyzrgb":
        return [value, root + ".xyz"]
    if ext.lower() == ".xyz":
        return [root + ".xyzrgb", value]
    return [value]


def _read_xyzrgb(path):
    pts = []
    cols = []
    with open(path, "r") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            parts = stripped.split()
            if len(parts) != 6:
                raise ValueError("Line {0} must contain X Y Z R G B.".format(line_number))
            x, y, z = [float(item) for item in parts[:3]]
            r, g, b = [int(item) for item in parts[3:6]]
            if min(r, g, b) < 0 or max(r, g, b) > 255:
                raise ValueError("Line {0} RGB must be in 0..255.".format(line_number))
            pts.append((x, y, z))
            cols.append((r, g, b))
    return pts, cols


def _read_xyz(path):
    pts = []
    with open(path, "r") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            parts = stripped.split()
            if len(parts) < 3:
                raise ValueError("Line {0} must contain X Y Z.".format(line_number))
            pts.append(tuple(float(item) for item in parts[:3]))
    return pts


def load_preview(path):
    selected = None
    for candidate in _candidate_paths(path):
        if os.path.isfile(candidate):
            selected = candidate
            break
    if selected is None:
        return None, [], [], 0, "Preview file not found"

    ext = os.path.splitext(selected)[1].lower()
    if ext == ".xyzrgb":
        xyz, rgb = _read_xyzrgb(selected)
        color_available = True
    else:
        xyz = _read_xyz(selected)
        rgb = []
        color_available = False

    if rg is None:
        return None, xyz, rgb, len(xyz), "XYZRGB preview loaded; Color: available" if color_available else "XYZ preview loaded; Color: unavailable"

    point_items = [rg.Point3d(x, y, z) for x, y, z in xyz]
    color_items = [Color.FromArgb(r, g, b) for r, g, b in rgb]
    point_cloud = rg.PointCloud()
    if color_available:
        for point, color in zip(point_items, color_items):
            point_cloud.Add(point, color)
    else:
        for point in point_items:
            point_cloud.Add(point)
    status = "XYZRGB preview loaded; Color: available" if color_available else "XYZ preview loaded; Color: unavailable"
    return point_cloud, point_items, color_items, len(point_items), status


try:
    cloud, points, colors, count, info = load_preview(preview_path)  # type: ignore[name-defined]
except NameError:
    cloud, points, colors, count, info = None, [], [], 0, "Set preview_path input"
except Exception as exc:
    cloud, points, colors, count, info = None, [], [], 0, "Load Preview failed: {0}".format(exc)
