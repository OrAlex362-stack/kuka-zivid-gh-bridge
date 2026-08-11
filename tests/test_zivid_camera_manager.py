from config_loader import load_config
from zivid_camera_manager import ZividCameraManager


def test_synthetic_mock_capture_works_without_hardware() -> None:
    manager = ZividCameraManager(load_config())
    assert manager.connect()
    frame = manager.capture_2d_3d()
    assert frame.mock
    assert frame.xyz.shape[1] == 3
    assert frame.rgba is not None
    assert frame.rgba.shape[0] == frame.xyz.shape[0]
    manager.close()
