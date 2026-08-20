from pathlib import Path

from storage_utils import allocate_numbered_directory


def test_capture_directory_naming_does_not_overwrite(tmp_path) -> None:
    first = allocate_numbered_directory(tmp_path, "capture")
    second = allocate_numbered_directory(tmp_path, "capture")
    assert first.name == "capture_0001"
    assert second.name == "capture_0002"
    assert first.is_dir() and second.is_dir()


def test_ascii_staging_writes_binary_and_cleans_temporary_directory(tmp_path, monkeypatch) -> None:
    from storage_utils import atomic_write_via_ascii_staging

    staging_root = tmp_path / 'ascii_temp'
    monkeypatch.setenv('ZIVID_ASCII_TEMP', str(staging_root))
    destination = tmp_path / 'unicode_destination' / 'original_pointcloud.zdf'
    payload = b'\x00ZDF\xff\x10binary\x00payload'
    seen = {}

    def writer(path: Path) -> None:
        str(path).encode('ascii')
        seen['staging_dir'] = path.parent
        path.write_bytes(payload)

    atomic_write_via_ascii_staging(destination, writer)

    assert destination.read_bytes() == payload
    assert seen['staging_dir'].parent == staging_root
    assert not seen['staging_dir'].exists()



def test_ascii_staging_cleans_up_after_writer_failure(tmp_path, monkeypatch) -> None:
    from storage_utils import atomic_write_via_ascii_staging

    staging_root = tmp_path / 'ascii_temp'
    monkeypatch.setenv('ZIVID_ASCII_TEMP', str(staging_root))
    destination = tmp_path / 'capture.zdf'
    seen = {}

    def writer(path: Path) -> None:
        seen['staging_dir'] = path.parent
        path.write_bytes(b'partial')
        raise RuntimeError('native save failed')

    import pytest

    with pytest.raises(RuntimeError):
        atomic_write_via_ascii_staging(destination, writer)

    assert not destination.exists()
    assert not seen['staging_dir'].exists()