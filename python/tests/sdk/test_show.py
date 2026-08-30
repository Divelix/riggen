"""`riggen.show()` (plans/python-sdk step 7) with stub binaries: a Python
script standing in for the window through `RIGGEN_BINARY`. The window
itself is the human's half (by hand on the dev machine)."""

from __future__ import annotations

import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

import riggen
from riggen.show import binary_path

SAVES = """\
    import sys, riggen
    robot = riggen.load(sys.argv[1])
    robot.link("arm").add_link("tip", riggen.Fixed((0, 0, 1)))
    robot.save(sys.argv[1])
    """
QUITS = """\
    import sys, riggen
    riggen.load(sys.argv[1])  # opens fine, closes without saving
    """


@pytest.fixture
def stub(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    def install(body: str) -> Path:
        script = tmp_path / "riggen-stub.py"
        script.write_text(textwrap.dedent(body))
        monkeypatch.setenv("RIGGEN_BINARY", str(script))
        return script

    return install


@pytest.fixture
def pendulum_api(cubes: Path) -> riggen.Robot:
    robot = riggen.Robot("pendulum")
    robot.root.add_mesh(cubes / "cube_binary.stl")
    robot.root.add_link("arm", riggen.Revolute("y", origin=(0, 0, 0.5)), mesh=cubes / "cube_ascii.stl", material="PLA")
    return robot


def test_wait_returns_the_saved_document(stub, pendulum_api: riggen.Robot):
    stub(SAVES)
    viewer = riggen.show(pendulum_api)
    assert viewer.path.name == "pendulum.riggen" and viewer.path.parent.name.startswith("riggen-show-")
    assert viewer.robot is pendulum_api  # until the window closes
    edited = viewer.wait(timeout=60)
    assert viewer.poll() == 0
    assert edited is not pendulum_api and viewer.robot is edited
    assert [l.name for l in edited.links] == ["base_link", "arm", "tip"]
    assert [l.name for l in pendulum_api.links] == ["base_link", "arm"]  # the caller's copy is untouched
    assert edited.link("tip").geoms == [] and edited.link("arm").geoms[0].mesh.is_file()  # paths rebased and back
    assert viewer.wait() is edited  # idempotent
    assert repr(viewer) == "Viewer('pendulum.riggen', exited 0)"


def test_wait_returns_the_original_when_nothing_was_saved(stub, pendulum_api: riggen.Robot):
    stub(QUITS)
    viewer = riggen.show(pendulum_api, block=True)
    assert viewer.poll() == 0
    assert viewer.wait() is pendulum_api


def test_kill_closes_the_window(stub, pendulum_api: riggen.Robot):
    stub("import time; time.sleep(60)")
    viewer = riggen.show(pendulum_api)
    assert viewer.poll() is None and repr(viewer) == "Viewer('pendulum.riggen', open)"
    viewer.kill()
    assert viewer.poll() is not None
    assert viewer.wait() is pendulum_api


def test_missing_binary_says_how_to_get_one(monkeypatch: pytest.MonkeyPatch, pendulum_api: riggen.Robot):
    monkeypatch.setenv("RIGGEN_BINARY", "/nowhere/riggen")
    with pytest.raises(FileNotFoundError, match="RIGGEN_BINARY"):
        riggen.show(pendulum_api)
    result = subprocess.run([sys.executable, "-m", "riggen", "--version"], capture_output=True, text=True, env={**os.environ, "RIGGEN_BINARY": "/nowhere/riggen"})
    assert result.returncode == 1 and "riggen: RIGGEN_BINARY='/nowhere/riggen' is not a file" in result.stderr


def test_binary_path_finds_the_wheel_binary_or_explains(monkeypatch: pytest.MonkeyPatch):
    monkeypatch.delenv("RIGGEN_BINARY", raising=False)
    try:
        found = binary_path()
    except FileNotFoundError as e:
        assert "cargo install --git" in str(e) and "RIGGEN_BINARY" in str(e)  # a develop venv: no binary
    else:
        assert found.is_file() and found.name.startswith("riggen")  # the wheel venv


def test_python_m_riggen_forwards_to_the_binary(stub):
    stub("import sys; print('stub', *sys.argv[1:])")
    result = subprocess.run([sys.executable, "-m", "riggen", "--version"], check=True, capture_output=True, text=True)
    assert result.stdout.strip() == "stub --version"
