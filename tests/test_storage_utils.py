from storage_utils import allocate_numbered_directory


def test_capture_directory_naming_does_not_overwrite(tmp_path) -> None:
    first = allocate_numbered_directory(tmp_path, "capture")
    second = allocate_numbered_directory(tmp_path, "capture")
    assert first.name == "capture_0001"
    assert second.name == "capture_0002"
    assert first.is_dir() and second.is_dir()
