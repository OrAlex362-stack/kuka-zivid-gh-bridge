from __future__ import annotations

from pathlib import Path
import tomllib


ROOT = Path(__file__).parents[1]


def test_project_environment_is_pinned_to_python_312_for_open3d() -> None:
    pyproject = tomllib.loads((ROOT / 'pyproject.toml').read_text(encoding='utf-8'))
    requirements = (ROOT / 'requirements.txt').read_text(encoding='utf-8')
    environment = (ROOT / 'environment.yml').read_text(encoding='utf-8')

    assert pyproject['project']['requires-python'] == '>=3.12,<3.13'
    assert 'open3d==0.19.0' in requirements
    assert "python_version < '3.13'" in requirements
    assert 'name: kuka-zivid312' in environment
    assert 'python=3.12' in environment