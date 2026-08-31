"""Kinematics, inertials, export and import (plans/python-sdk step 5): the
SDK agrees with `riggen --export` — the binary in the same wheel — byte for
byte, and `fk` with the samples the CLI writes for MuJoCo."""

from __future__ import annotations

import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from riggen import errors
from riggen._riggen import Robot

from conftest import FIXTURES, IDENTITY, hinge_joint

ARM = FIXTURES / "arm" / "arm.riggen"
ARM_URDF = FIXTURES / "arm" / "arm.urdf"


def run_cli(cli: Path, *args) -> subprocess.CompletedProcess[str]:
    return subprocess.run([cli, *map(str, args)], check=True, capture_output=True, text=True)


def tree(directory: Path) -> dict[str, bytes]:
    return {str(p.relative_to(directory)): p.read_bytes() for p in sorted(directory.rglob("*")) if p.is_file()}


def wxyz(r: list[float]) -> list[float]:
    x, y, z, w = r
    return [w, x, y, z]


def close(a, b, eps=1e-9) -> bool:
    return all(math.isclose(x, y, abs_tol=eps) for x, y in zip(a, b, strict=True))


@pytest.fixture
def arm() -> Robot:
    robot, warnings = Robot.load(ARM)
    assert warnings == []
    return robot


def test_validate_is_empty_for_any_reachable_document(arm: Robot):
    assert arm.validate() == []
    arm.check()


def test_fk_matches_the_cli_samples(arm: Robot, cli: Path, tmp_path: Path):
    run_cli(cli, "--export", "mjcf", "--fk-samples", "--out", tmp_path, ARM)
    samples = json.loads((tmp_path / "arm.fk.json").read_text())
    assert arm.fk_samples_json() == (tmp_path / "arm.fk.json").read_text()
    names = {id_: link["name"] for id_, link in arm.links().items()}
    joints = [arm.joint(name) for name in samples["joints"]]
    assert None not in joints
    for sample in samples["samples"]:
        world = arm.fk(dict(zip(joints, sample["q"], strict=True)))
        assert set(names[id_] for id_ in world) == set(sample["links"])
        for id_, pose in world.items():
            expected = sample["links"][names[id_]]
            assert close(pose["t"], expected["pos"]), names[id_]
            q = wxyz(pose["r"])
            assert close(q, expected["quat"]) or close(q, [-c for c in expected["quat"]]), names[id_]


def test_fk_defaults_missing_joints_to_zero_and_rejects_unknown_ones(arm: Robot):
    at_rest = arm.fk({})
    assert at_rest[arm.root] == IDENTITY
    assert arm.fk({}) == arm.fk({arm.joint("shoulder_joint"): 0.0})
    with pytest.raises(errors.UnknownId, match="j99"):
        arm.fk({99: 0.1})


def test_origin_for_world_inverts_one_fk_step(arm: Robot):
    upper = arm.link("upper")
    world = arm.fk({})[upper]
    origin = arm.origin_for_world(upper, world)
    assert origin == arm.joints()[arm.parent_joint(upper)]["origin"]
    assert arm.origin_for_world(arm.root, IDENTITY) is None


def test_export_is_byte_identical_to_the_cli(arm: Robot, cli: Path, tmp_path: Path):
    run_cli(cli, "--export", "both", "--fk-samples", "--out", tmp_path / "cli", ARM)
    written = arm.export(tmp_path / "sdk", format="both", fk_samples=True)
    assert [p.name for p in written] == ["arm.xml", "arm.urdf", "base.stl", "fore.stl", "shoulder.stl", "upper.stl", "arm.fk.json"]
    assert all(p.is_file() for p in written)
    assert tree(tmp_path / "sdk") == tree(tmp_path / "cli")


def test_all_three_writers_are_reachable_from_the_sdk(arm: Robot, cli: Path, tmp_path: Path):
    """`format` is a set of writers (ADR-0016), and `all` is the default."""
    run_cli(cli, "--export", "all", "--out", tmp_path / "cli", ARM)
    written = arm.export(tmp_path / "sdk", format="all")
    assert [p.name for p in written][:3] == ["arm.xml", "arm.urdf", "arm.sdf"]
    assert tree(tmp_path / "sdk") == tree(tmp_path / "cli")
    # `all` is what `export` does when asked for nothing in particular.
    arm.export(tmp_path / "default")
    assert tree(tmp_path / "sdk") == tree(tmp_path / "default")
    # SDF alone writes the one file, and it is SDF 1.11 with the mimic the
    # other two also carry (ADR-0016 §1).
    only = arm.export(tmp_path / "sdf", format="sdf")
    assert [p.name for p in only][0] == "arm.sdf"
    assert not (tmp_path / "sdf" / "arm.urdf").exists()
    sdf = (tmp_path / "sdf" / "arm.sdf").read_text()
    assert '<sdf version="1.11">' in sdf
    assert '<mimic joint="upper_joint">' in sdf
    # `model://` is what `package://` is to URDF — one control, one meaning.
    arm.export(tmp_path / "sdf_pkg", format="sdf", mesh_paths="package://arm_description")
    assert "<uri>model://arm_description/meshes/base.stl</uri>" in (
        tmp_path / "sdf_pkg" / "arm.sdf"
    ).read_text()


def test_export_options_reach_the_writers(arm: Robot, tmp_path: Path):
    arm.export(tmp_path / "pkg", format="urdf", mesh_paths="package://arm_description")
    urdf = (tmp_path / "pkg" / "arm.urdf").read_text()
    assert 'filename="package://arm_description/meshes/base.stl"' in urdf
    arm.export(tmp_path / "abs", format="urdf", mesh_paths="absolute")
    assert str(tmp_path / "abs" / "meshes" / "base.stl") in (tmp_path / "abs" / "arm.urdf").read_text()
    # A floating base is a moving body and needs mass: the arm's root is an
    # empty `base_link`, so it is refused (ADR-0008); the pendulum's root
    # has a cube and a material.
    with pytest.raises(errors.ExportError, match='link "base_link" moves but has no mass'):
        arm.export(tmp_path / "float", format="mjcf", floating_base=True)
    # `format` names a set of writers, not a choice (ADR-0016); USD is not
    # one of them and is not going to be.
    with pytest.raises(ValueError, match="format"):
        arm.export(tmp_path / "x", format="usd")
    with pytest.raises(ValueError, match="mesh_paths"):
        arm.export(tmp_path / "x", mesh_paths="package://")


def test_floating_base_adds_a_freejoint(pendulum: Robot, tmp_path: Path):
    written = pendulum.export(tmp_path, format="mjcf", floating_base=True)
    assert [p.suffix for p in written] == [".xml", ".stl", ".stl"]
    assert "<freejoint" in (tmp_path / "pendulum.xml").read_text()


def test_inertial_of_the_base_matches_the_export(arm: Robot, tmp_path: Path):
    mass, com, inertia = arm.inertial(arm.link("base"))
    assert mass > 0 and inertia[0][1] == inertia[1][0]
    arm.export(tmp_path, format="mjcf")
    (body,) = [b for b in ET.parse(tmp_path / "arm.xml").iter("body") if b.get("name") == "base"]
    written = body.find("inertial")
    assert written is not None
    assert math.isclose(float(written.get("mass")), mass, abs_tol=1e-9)
    assert close([float(v) for v in written.get("pos").split()], com)
    ixx, iyy, izz, ixy, ixz, iyz = (float(v) for v in written.get("fullinertia").split())
    assert close([ixx, iyy, izz], [inertia[0][0], inertia[1][1], inertia[2][2]])
    assert close([ixy, ixz, iyz], [inertia[0][1], inertia[0][2], inertia[1][2]])


def test_inertial_errors_are_typed(pendulum: Robot):
    pendulum.set_link_material(5, None)
    with pytest.raises(errors.InertialError, match="no material and no density override"):
        pendulum.inertial(5)
    with pytest.raises(errors.UnknownId):
        pendulum.inertial(99)


def test_export_of_an_unexportable_robot_lists_every_error(pendulum: Robot, tmp_path: Path):
    pendulum.set_link_material(5, None)
    pendulum.add_link("empty", 5, hinge_joint(name="tip", kind="Continuous", limits=None))
    with pytest.raises(errors.ExportError) as info:
        pendulum.export(tmp_path)
    lines = str(info.value).splitlines()
    assert len(lines) == 2 and all(line.startswith("cannot export: ") for line in lines)
    assert 'link "arm": no material and no density override' in lines[0]
    assert 'link "empty" moves but has no mass' in lines[1]
    assert not (tmp_path / "pendulum.xml").exists()


def test_load_urdf_then_export_matches_the_cli(cli: Path, tmp_path: Path):
    result = run_cli(cli, "--export", "both", "--fk-samples", "--out", tmp_path / "cli", ARM_URDF)
    robot, warnings = Robot.load_urdf(ARM_URDF)
    assert robot.name == "arm" and robot.link("fore") is not None
    assert [f"warning: {w}" for w in warnings] == result.stderr.splitlines()
    robot.export(tmp_path / "sdk", format="both", fk_samples=True)
    assert tree(tmp_path / "sdk") == tree(tmp_path / "cli")


def test_load_mjcf_then_export_matches_the_cli(cli: Path, tmp_path: Path):
    # The arm's own MJCF, read back and written out again by both routes.
    run_cli(cli, "--export", "mjcf", "--out", tmp_path / "first", ARM)
    mjcf = tmp_path / "first" / "arm.xml"
    result = run_cli(cli, "--export", "mjcf", "--fk-samples", "--out", tmp_path / "cli", mjcf)
    robot, warnings = Robot.load_mjcf(mjcf)
    assert warnings == [], "our own MJCF holds nothing the document cannot"
    assert result.stderr == ""
    assert robot.name == "arm"
    # The `<site>` → `Frame` symmetry the URDF import does not have (ADR-0012).
    assert sorted(f["name"] for f in robot.frames().values()) == ["camera_mount", "tcp"]
    robot.export(tmp_path / "sdk", format="mjcf", fk_samples=True)
    assert tree(tmp_path / "sdk") == tree(tmp_path / "cli")


def test_load_mjcf_errors_are_typed(tmp_path: Path):
    with pytest.raises(errors.MjcfImportError, match="nowhere.xml"):
        Robot.load_mjcf(tmp_path / "nowhere.xml")
    composite = tmp_path / "composite.xml"
    composite.write_text(
        '<mujoco><worldbody><body name="a"><body name="w">'
        '<joint name="w0"/><joint name="w1"/></body></body></worldbody></mujoco>'
    )
    with pytest.raises(errors.MjcfImportError, match="w0, w1"):
        Robot.load_mjcf(composite)


def test_load_urdf_errors_are_typed(tmp_path: Path):
    with pytest.raises(errors.UrdfImportError, match="nowhere.urdf"):
        Robot.load_urdf(tmp_path / "nowhere.urdf")
    bad = tmp_path / "bad.urdf"
    bad.write_text('<robot name="x"><link name="a"/><link name="b"/></robot>')
    with pytest.raises(errors.UrdfImportError, match="more than one root"):
        Robot.load_urdf(bad, packages={"pkg": tmp_path})
