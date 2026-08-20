from pathlib import Path

import numpy as np

from pointcloud_io import finite_points_and_colors, read_xyz, read_xyzrgb, write_preview_xyz, write_preview_xyzrgb


def test_color_correspondence_survives_finite_filtering() -> None:
    xyz = np.array([[1.0, 2.0, 3.0], [np.nan, 9.0, 9.0], [7.0, 8.0, 9.0]], dtype=np.float32)
    rgba = np.array([[10, 20, 30, 255], [99, 99, 99, 255], [70, 80, 90, 255]], dtype=np.uint8)

    points, colors = finite_points_and_colors(xyz, rgba)

    assert np.allclose(points, [[1.0, 2.0, 3.0], [7.0, 8.0, 9.0]])
    assert colors is not None
    assert colors.tolist() == [[10, 20, 30], [70, 80, 90]]


def test_xyzrgb_round_trip(tmp_path: Path) -> None:
    xyz = np.array([[1023.42, -42.83, 815.21], [1024.32, -40.21, 814.55]], dtype=np.float32)
    rgb = np.array([[132, 94, 61], [141, 98, 65]], dtype=np.uint8)
    path = tmp_path / "preview_cloud.xyzrgb"

    count = write_preview_xyzrgb(path, xyz, rgb, 100)
    loaded_xyz, loaded_rgb = read_xyzrgb(path)

    assert count == 2
    assert np.allclose(loaded_xyz, xyz, atol=1e-5)
    assert loaded_rgb.tolist() == rgb.tolist()


def test_legacy_xyz_preview_round_trip(tmp_path: Path) -> None:
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    path = tmp_path / "preview_cloud.xyz"

    assert write_preview_xyz(path, xyz, 100) == 2
    assert np.allclose(read_xyz(path), xyz)


def test_binary_ply_is_written_as_binary_bytes(tmp_path: Path) -> None:
    from pointcloud_io import write_ply

    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    rgb = np.array([[255, 0, 128], [0, 64, 255]], dtype=np.uint8)
    path = tmp_path / 'cloud.ply'

    assert write_ply(path, xyz, rgb, binary=True) == 2
    raw = path.read_bytes()
    header, body = raw.split(b'end_header\n', 1)

    assert b'format binary_little_endian 1.0' in header
    assert b'property uchar red' in header
    assert len(body) == 2 * (4 + 4 + 4 + 1 + 1 + 1)