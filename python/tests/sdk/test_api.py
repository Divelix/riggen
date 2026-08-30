"""The public API (plans/python-sdk step 6): `riggen.Robot` with its
handles and value types over `_riggen`, the examples, and the rule that
every public name is documented."""

from __future__ import annotations

import inspect
import json
import math
import runpy
import subprocess
import sys
import warnings
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import riggen
from riggen import Continuous, Dynamics, Fixed, Limits, Pose, Prismatic, Revolute
from riggen._riggen import Robot as RawRobot

from conftest import FIXTURES, ROOT

ARM = FIXTURES / "arm" / "arm.riggen"
EXAMPLES = ROOT / "examples"


def tree(directory: Path) -> dict[str, bytes]:
    return {str(p.relative_to(directory)): p.read_bytes() for p in sorted(directory.rglob("*")) if p.is_file()}


def close(a, b, eps=1e-9) -> bool:
    return all(math.isclose(x, y, abs_tol=eps) for x, y in zip(a, b, strict=True))


# ---- value types -------------------------------------------------------------


def test_pose_spellings_agree():
    p = Pose((1, 2, 3), rpy=(0, 0, 90), degrees=True)
    assert p.xyz == (1.0, 2.0, 3.0)
    assert close(p.quat, (math.sqrt(0.5), 0, 0, math.sqrt(0.5)))
    assert close(p.rpy, (0, 0, math.pi / 2)) and close(p.rpy_degrees, (0, 0, 90))
    assert Pose(quat=p.quat) == Pose(rpy=p.rpy)
    assert Pose.from_doc(p.to_doc()) == p
    assert Pose() == Pose.IDENTITY and repr(Pose((0, 0, 0.5))) == "Pose((0.0, 0.0, 0.5))"
    doc = p.to_doc()
    assert doc["t"] == [1.0, 2.0, 3.0] and close(doc["r"], [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)])
    with pytest.raises(TypeError):
        Pose(rpy=(0, 0, 0), quat=(1, 0, 0, 0))
    with pytest.raises(ValueError):
        Pose((1, 2))


def test_joint_specs_have_the_app_defaults_and_typed_limits():
    r = Revolute()
    assert (r.kind, r.axis, r.origin) == ("Revolute", (0.0, 0.0, 1.0), Pose.IDENTITY)
    assert r.limits == Limits(-math.pi, math.pi) and r.limits.effort == 0.0
    assert Revolute("-y").axis == (0.0, -1.0, 0.0)
    assert close(Revolute((1, 1, 0)).axis, (math.sqrt(0.5), math.sqrt(0.5), 0))
    assert Revolute(limits=(-90, 90), degrees=True).limits == Limits(-math.pi / 2, math.pi / 2)
    assert Prismatic("x").limits == Limits(-1.0, 1.0)
    assert Continuous().limits is None and Fixed().limits is None
    assert Fixed((0, 0, 1)).origin == Pose((0, 0, 1))
    doc = Revolute("y", origin=(0, 0, 0.5), limits=Limits(-1, 1, effort=10, velocity=3), dynamics=Dynamics(damping=0.1)).to_doc("hinge")
    assert doc["name"] == "hinge" and doc["limits"]["effort"] == 10.0 and doc["dynamics"]["damping"] == 0.1
    assert riggen.JointSpec.from_doc(doc) == Revolute("y", origin=(0, 0, 0.5), limits=Limits(-1, 1, effort=10, velocity=3), dynamics=Dynamics(damping=0.1))
    with pytest.raises(ValueError):
        Revolute("w")
    with pytest.raises(TypeError):
        Revolute(limits=Limits(0, 1), degrees=True)


# ---- the robot and its handles ----------------------------------------------


@pytest.fixture
def pendulum_api(cubes: Path) -> riggen.Robot:
    robot = riggen.Robot("pendulum")
    robot.root.add_mesh(cubes / "cube_binary.stl")
    robot.root.material = "aluminium"
    arm = robot.root.add_link(
        "arm",
        Revolute("y", origin=(0, 0, 0.5), limits=Limits(-90, 90, effort=10, velocity=3, degrees=True), dynamics=Dynamics(damping=0.1)),
        mesh=cubes / "cube_ascii.stl",
        material="PLA",
        joint_name="hinge",
    )
    arm.geoms[0].pose = (0, 0, 0.5)
    return robot


def test_the_api_builds_the_corpus_pendulum(pendulum_api: riggen.Robot, cubes: Path):
    pendulum_api.save(cubes / "pendulum.riggen")
    assert (cubes / "pendulum.riggen").read_text() == (FIXTURES / "pendulum.riggen").read_text()


def test_handles_read_the_document(pendulum_api: riggen.Robot):
    robot = pendulum_api
    base, arm = robot.root, robot.link("arm")
    hinge = robot.joint("hinge")
    assert robot.links == [base, arm] and robot.joints == [hinge]
    assert base.parent is None and base.joint is None
    assert arm.parent == base and arm.joint == hinge and base.children == [arm] and base.joints == [hinge]
    assert base.subtree == [base, arm]
    assert (hinge.parent, hinge.child, hinge.kind) == (base, arm, "revolute")
    assert hinge.origin == Pose((0, 0, 0.5)) and hinge.axis == (0.0, 1.0, 0.0)
    assert hinge.limits == Limits(-math.pi / 2, math.pi / 2, effort=10, velocity=3)
    assert hinge.dynamics == Dynamics(damping=0.1)
    assert hinge.spec == Revolute("y", origin=(0, 0, 0.5), limits=hinge.limits, dynamics=hinge.dynamics)
    assert arm.material == "PLA" and arm.collision == "same_as_visual"
    assert arm.inertial_spec == riggen.ComputedInertial()
    (geom,) = arm.geoms
    assert geom.pose == Pose((0, 0, 0.5)) and geom.mesh.name == "cube_ascii.stl"
    assert robot.materials["PLA"].density == 1240.0
    assert repr(arm) == "Link('arm')" and repr(hinge) == "Joint('hinge': 'base_link' -> 'arm', revolute)"
    assert repr(robot) == "Robot('pendulum': 2 links, 1 joints)"
    with pytest.raises(KeyError):
        robot.link("nobody")


def test_every_setter_is_one_edit(pendulum_api: riggen.Robot):
    robot = pendulum_api
    arm, hinge = robot.link("arm"), robot.joint("hinge")
    hinge.name = "pivot"
    assert robot.joint("pivot") == hinge
    hinge.limits = (-1, 1)
    assert hinge.limits == Limits(-1, 1)
    hinge.axis = "x"
    hinge.origin = Pose((0, 0, 1), rpy=(0, 0, 90), degrees=True)
    assert hinge.axis == (1.0, 0.0, 0.0) and close(hinge.origin.rpy_degrees, (0, 0, 90))
    hinge.dynamics = Dynamics(armature=0.01)
    assert hinge.dynamics.armature == 0.01
    hinge.spec = Continuous("z")
    assert hinge.kind == "continuous" and hinge.limits is None
    hinge.move_frame((0, 0, 0.5))
    assert hinge.origin == Pose((0, 0, 0.5)) and hinge.axis == (0.0, 0.0, 1.0)

    arm.name = "pendulum_arm"
    arm.material = None
    arm.collision = "convex_hull"
    arm.inertial_spec = riggen.HybridInertial(mass=2.5)
    assert (arm.name, arm.material, arm.collision, arm.inertial_spec) == ("pendulum_arm", None, "convex_hull", riggen.HybridInertial(2.5))
    arm.inertial_spec = riggen.OverrideInertial(1.0, (0.1, 0.2, 0.3), ((1, 2, 3), (2, 4, 5), (3, 5, 6)))
    assert arm.inertial_spec == riggen.OverrideInertial(1.0, (0.1, 0.2, 0.3), ((1.0, 2.0, 3.0), (2.0, 4.0, 5.0), (3.0, 5.0, 6.0)))
    assert arm.inertial == riggen.Inertial(1.0, (0.1, 0.2, 0.3), ((1.0, 2.0, 3.0), (2.0, 4.0, 5.0), (3.0, 5.0, 6.0)))
    arm.inertial_spec = riggen.ComputedInertial(density=500)
    arm.material = "PLA"
    assert arm.inertial.mass == pytest.approx(500.0)  # a unit cube at 500 kg/m³

    geom = arm.add_mesh(arm.geoms[0].mesh, pose=(0, 0, 1), scale=0.5, color=(1, 0, 0, 1))
    assert len(arm.geoms) == 2 and geom.pose == Pose((0, 0, 1))
    geom.remove()
    assert len(arm.geoms) == 1

    robot.add_material("brass", 8500, (0.8, 0.6, 0.2, 1.0))
    brass = robot.materials["brass"]
    assert brass.density == 8500.0 and close(brass.color, (0.8, 0.6, 0.2, 1.0), eps=1e-7)  # colours are f32
    robot.remove_material("brass")
    assert "brass" not in robot.materials

    tip = arm.add_link("tip", Fixed((0, 0, 1)))
    assert tip.parent == arm and robot.joint("tip_joint") == tip.joint
    tip.reparent(robot.root)
    assert tip.parent == robot.root and tip.joint.origin == Pose((0, 0, 1.5))  # world pose kept
    tip.place((1, 0, 0))
    assert tip.joint.origin == Pose((1, 0, 0)) and robot.fk()["tip"] == Pose((1, 0, 0))
    tip.remove()
    with pytest.raises(riggen.UnknownId):
        tip.name
    with pytest.raises(riggen.CannotRemoveRoot):
        robot.root.remove()
    with pytest.raises(riggen.InvalidDocument):
        arm.name = "base_link"
    with pytest.raises(ValueError):
        arm.collision = "bouncy"


def test_fk_by_name_and_handle(pendulum_api: riggen.Robot):
    robot = pendulum_api
    hinge = robot.joint("hinge")
    at_rest = robot.fk()
    assert at_rest["base_link"] == Pose.IDENTITY and at_rest["arm"] == Pose((0, 0, 0.5))
    swung = robot.fk({hinge: math.pi / 2})
    assert swung == robot.fk({"hinge": math.pi / 2})
    assert close(swung["arm"].rpy, (0, math.pi / 2, 0))
    with pytest.raises(KeyError):
        robot.fk({"nobody": 0.0})


def test_make_root_and_json_and_copy(pendulum_api: riggen.Robot):
    robot = pendulum_api
    twin = riggen.Robot.from_json(robot.to_json())
    assert twin.to_json() == robot.to_json() and twin is not robot
    other = robot.copy()
    other.name = "other"
    assert robot.name == "pendulum"
    robot.joint("hinge").spec = Fixed((0, 0, 0.5))
    robot.link("arm").make_root()
    assert robot.root.name == "arm" and robot.validate() == []


def test_load_warns_instead_of_failing(cubes: Path, pendulum_api: riggen.Robot):
    path = cubes / "p.riggen"
    pendulum_api.save(path)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        riggen.load(path)  # no warning: nothing changed
    (cubes / "cube_ascii.stl").write_bytes(b"solid nothing\nendsolid nothing\n")
    with pytest.warns(riggen.RiggenWarning, match="changed"):
        robot = riggen.load(path)
    assert robot.link("arm").geoms[0].mesh == cubes / "cube_ascii.stl"
    with pytest.raises(riggen.FileError):
        riggen.load(cubes / "nowhere.riggen")


def test_load_urdf_warns_and_builds(tmp_path: Path):
    with pytest.warns(riggen.RiggenWarning):
        robot = riggen.load_urdf(FIXTURES / "arm" / "arm.urdf")
    assert [l.name for l in robot.links] == ["base_link", "base", "shoulder", "upper", "fore"]
    assert robot.joint("shoulder_joint").kind == "revolute"


# ---- the examples --------------------------------------------------------------


def test_arm_example_exports_the_fixture_byte_for_byte(tmp_path: Path):
    built = runpy.run_path(str(EXAMPLES / "arm.py"))["build"]()
    fixture = riggen.load(ARM)
    built.export(tmp_path / "built", format="both", fk_samples=True)
    fixture.export(tmp_path / "fixture", format="both", fk_samples=True)
    assert tree(tmp_path / "built") == tree(tmp_path / "fixture")


def test_pendulum_example_is_the_corpus_file(tmp_path: Path):
    built = runpy.run_path(str(EXAMPLES / "pendulum.py"))["build"]()
    fixture = riggen.load(FIXTURES / "pendulum.riggen")
    built.export(tmp_path / "built", format="both")
    fixture.export(tmp_path / "fixture", format="both")
    assert tree(tmp_path / "built") == tree(tmp_path / "fixture")


def test_examples_run_from_the_command_line(tmp_path: Path):
    for name in ("pendulum.py", "arm.py"):
        result = subprocess.run([sys.executable, EXAMPLES / name, "--out", tmp_path / name], check=True, capture_output=True, text=True)
        assert (tmp_path / name / f"{name[:-3]}.xml").is_file(), result.stdout
        assert (tmp_path / name / f"{name[:-3]}.fk.json").is_file()


# ---- every public name is documented ----------------------------------------


def public_members(obj) -> list[tuple[str, object]]:
    return [(n, m) for n, m in inspect.getmembers(obj) if not n.startswith("_") and (inspect.isroutine(m) or isinstance(m, property))]


def test_every_public_name_has_a_docstring():
    undocumented = []
    for name in riggen.__all__:
        obj = getattr(riggen, name)
        if not (obj.__doc__ or "").strip():
            undocumented.append(name)
        if inspect.isclass(obj) and not issubclass(obj, BaseException):
            for member, value in public_members(obj):
                doc = (value.fget.__doc__ if isinstance(value, property) else value.__doc__) or ""
                if not doc.strip() and member not in ("to_doc", "from_doc"):
                    undocumented.append(f"{name}.{member}")
    assert not undocumented, undocumented
    assert RawRobot.__doc__  # the extension's class carries its Rust doc comment


def test_convex_decomposition_round_trips_and_exports(tmp_path: Path):
    """The policy the window offers, from Python: assigned as a dataclass,
    read back as one, saved and loaded unchanged, and exported as one mesh
    file and one collision geom per piece (ADR-0011)."""
    import shutil

    from conftest import FIXTURES

    shutil.copy2(FIXTURES / "bracket.stl", tmp_path / "bracket.stl")
    robot = riggen.Robot("bracket")
    link = robot.root.add_link(
        "bracket", riggen.Fixed(), mesh=tmp_path / "bracket.stl", scale=0.001, material="PLA"
    )

    assert link.collision == "same_as_visual"
    link.collision = riggen.ConvexDecomposition(max_hulls=4, resolution=48)
    assert link.collision == riggen.ConvexDecomposition(max_hulls=4, resolution=48, concavity=0.01)
    assert riggen.ConvexDecomposition() == riggen.ConvexDecomposition(8, 64, 0.01)

    # The document's own JSON, and back through `load`.
    doc = json.loads(robot.to_json())
    (stored,) = [l["collision"] for l in doc["robot"]["links"].values() if l["name"] == "bracket"]
    assert stored == {"ConvexDecomposition": {"max_hulls": 4, "resolution": 48, "concavity": 0.01}}
    saved = tmp_path / "bracket.riggen"
    robot.save(saved)
    assert riggen.load(saved).link("bracket").collision == link.collision

    # The export: N pieces, N mesh files, N collision geoms, one visual.
    out = tmp_path / "out"
    robot.export(out, format="urdf")
    pieces = sorted(p.name for p in (out / "meshes").glob("bracket_hull_*.stl"))
    assert 1 < len(pieces) <= 4, pieces
    assert pieces == [f"bracket_hull_{i}.stl" for i in range(len(pieces))]
    assert (out / "meshes" / "bracket.stl").is_file()

    urdf = ET.fromstring((out / "bracket.urdf").read_text())
    (element,) = [l for l in urdf.findall("link") if l.get("name") == "bracket"]
    assert len(element.findall("visual")) == 1
    files = [c.find("geometry/mesh").get("filename") for c in element.findall("collision")]
    assert sorted(files) == sorted(f"meshes/{name}" for name in pieces)


def test_a_bad_decomposition_is_refused_before_the_document(pendulum_api: riggen.Robot):
    arm = pendulum_api.link("arm")
    before = arm.collision
    for bad in (
        riggen.ConvexDecomposition(max_hulls=0),
        riggen.ConvexDecomposition(resolution=-1),
        riggen.ConvexDecomposition(concavity=2.0),
        riggen.ConvexDecomposition(max_hulls=1.5),
    ):
        with pytest.raises(ValueError):
            arm.collision = bad
        assert arm.collision == before
