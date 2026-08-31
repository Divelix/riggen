"""Shared fixtures for the SDK suite (plans/python-sdk; docs/01 §Testing).

Runs against the installed wheel (the `wheel` CI job) or a `maturin develop`
venv; never against the checkout on `sys.path`, which has no extension
module. `FIXTURES` is `assets/fixtures/` at the repository root.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
FIXTURES = ROOT / "assets" / "fixtures"

IDENTITY = {"t": [0.0, 0.0, 0.0], "r": [0.0, 0.0, 0.0, 1.0]}


def hinge_joint(**overrides):
    """The pendulum's revolute hinge, in the schema's shape."""
    joint = {
        "name": "hinge",
        "kind": "Revolute",
        "origin": {"t": [0.0, 0.0, 0.5], "r": [0.0, 0.0, 0.0, 1.0]},
        "axis": [0.0, 1.0, 0.0],
        "limits": {"lower": -1.5707963267948966, "upper": 1.5707963267948966, "effort": 10.0, "velocity": 3.0},
        "dynamics": {"damping": 0.1, "friction": 0.0, "armature": 0.0},
    }
    joint.update(overrides)
    return joint


def upgraded_from_v1(path: Path) -> dict:
    """A schema-1 corpus file as the current schema holds it: the version
    moves and every joint gains a `mimic` (ADR-0013) and an `actuator`
    (ADR-0014) of `None`. `pendulum.riggen` is frozen at 1 — it is the file
    the upgrade chain reads — so a document the SDK builds and saves is
    compared against this, not against its bytes."""
    doc = json.loads(path.read_text())
    assert doc["schema_version"] == 1, "the upgrade corpus stays at schema 1"
    doc["schema_version"] = 3
    for joint in doc["robot"]["joints"].values():
        joint["mimic"] = None
        joint["actuator"] = None
    return doc


@pytest.fixture
def cubes(tmp_path: Path) -> Path:
    """The two unit-cube STLs copied beside where the test will save, so a
    saved document's mesh paths rebase to bare file names."""
    for name in ("cube_binary.stl", "cube_ascii.stl"):
        shutil.copy2(FIXTURES / name, tmp_path / name)
    return tmp_path


def build_pendulum(cubes: Path):
    """`assets/fixtures/pendulum.riggen` through `_riggen`, in the order the
    app built it: base mesh (m1, g2), then the arm with its hinge (m3, g4,
    l5, j6)."""
    from riggen._riggen import Robot

    robot = Robot("pendulum")
    base = robot.root
    m1 = robot.add_asset(cubes / "cube_binary.stl")
    robot.add_geom(base, m1)
    robot.set_link_material(base, "aluminium")
    arm = robot.add_link("arm", base, hinge_joint(), mesh=cubes / "cube_ascii.stl", material="PLA")
    (geom,) = (g["id"] for g in robot.links()[arm]["visuals"])
    robot.set_geom_pose(arm, geom, {"t": [0.0, 0.0, 0.5], "r": [0.0, 0.0, 0.0, 1.0]})
    return robot


@pytest.fixture
def pendulum(cubes: Path):
    return build_pendulum(cubes)


def find_cli() -> Path:
    """The `riggen` binary the SDK is compared against: `RIGGEN_BINARY`, the
    one bundled in this interpreter's wheel (the `wheel` CI job), else a
    local cargo build; otherwise the test skips."""
    from riggen.show import binary_path

    try:
        return binary_path()
    except FileNotFoundError:
        pass
    for candidate in (ROOT / "target" / "release" / "riggen", ROOT / "target" / "debug" / "riggen"):
        if candidate.is_file():
            return candidate
    pytest.skip("no riggen binary to compare against; set RIGGEN_BINARY")


@pytest.fixture(scope="session")
def cli() -> Path:
    return find_cli()
