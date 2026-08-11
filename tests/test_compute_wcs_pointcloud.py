import numpy as np

from compute_wcs_pointcloud import classify_planes, construct_wcs_from_planes


def test_planes_are_classified_independent_of_detection_order_and_orthogonalized() -> None:
    planes = [
        {"equation": [0.01, 1.0, 0.02, -20.0], "inlier_count": 900, "rmse_mm": 0.2},
        {"equation": [0.02, 0.01, 1.0, -30.0], "inlier_count": 1000, "rmse_mm": 0.1},
        {"equation": [1.0, -0.01, 0.01, -10.0], "inlier_count": 800, "rmse_mm": 0.3},
    ]
    expected = {"top": [0, 0, 1], "side": [0, 1, 0], "end": [1, 0, 0]}
    classified = classify_planes(planes, expected)
    assert classified["top"]["detection_index"] == 2
    assert classified["side"]["detection_index"] == 1
    assert classified["end"]["detection_index"] == 3

    T_base_wcs, T_wcs_base, quality = construct_wcs_from_planes(classified)
    assert np.allclose(T_base_wcs[:3, :3].T @ T_base_wcs[:3, :3], np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(T_base_wcs[:3, :3]), 1.0)
    assert np.allclose(T_wcs_base @ T_base_wcs, np.eye(4), atol=1e-10)
    assert quality["orthogonality_error_deg"] > 0


def test_exact_three_plane_intersection_is_wcs_origin() -> None:
    planes = [
        {"equation": [0, 0, 1, -30], "inlier_count": 1, "rmse_mm": 0},
        {"equation": [1, 0, 0, -10], "inlier_count": 1, "rmse_mm": 0},
        {"equation": [0, 1, 0, -20], "inlier_count": 1, "rmse_mm": 0},
    ]
    classified = classify_planes(
        planes, {"top": [0, 0, 1], "side": [0, 1, 0], "end": [1, 0, 0]}
    )
    T_base_wcs, _, quality = construct_wcs_from_planes(classified)
    assert np.allclose(T_base_wcs[:3, 3], [10, 20, 30])
    assert quality["orthogonality_error_deg"] == 0.0
