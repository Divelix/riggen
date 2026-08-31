"""The document half of `riggen._riggen` (plans/python-sdk step 4): the
pendulum built through the bindings is the corpus file, every edit is one
command, and every refused edit is its own exception and leaves the
document untouched."""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

import riggen
from riggen import _riggen, errors
from riggen._riggen import Robot

from conftest import FIXTURES, IDENTITY, hinge_joint, upgraded_from_v1

PENDULUM = FIXTURES / "pendulum.riggen"


def test_version_is_the_package_version():
    assert isinstance(_riggen.__version__, str)
    assert _riggen.__version__ == riggen.__version__


def test_new_robot_has_a_root_and_the_default_materials():
    robot = Robot("x")
    assert robot.name == "x"
    assert robot.root == 0
    assert robot.links()[0]["name"] == "base_link"
    assert robot.joints() == {}
    assert set(robot.materials()) == {"aluminium", "steel", "PLA", "ABS", "nylon", "rubber"}
    assert robot.materials()["PLA"]["density"] == 1240.0
    assert robot.next_id == 1
    assert repr(robot) == "Robot('x': 1 links, 0 joints)"


def test_pendulum_saves_as_the_corpus_file(pendulum: Robot, cubes: Path):
    """Ids, order, hashes, `next_id`, key order — the file is
    `assets/fixtures/pendulum.riggen` once the mesh paths rebase to the
    same bare names, and once that schema-1 corpus is read as schema 2."""
    assert pendulum.next_id == 7
    out = cubes / "pendulum.riggen"
    pendulum.save(out)
    assert json.loads(out.read_text()) == upgraded_from_v1(PENDULUM)


def test_read_access_matches_the_document(pendulum: Robot):
    assert pendulum.link("arm") == 5 and pendulum.link("nobody") is None
    assert pendulum.joint("hinge") == 6 and pendulum.joint("nobody") is None
    assert pendulum.parent_joint(0) is None
    assert pendulum.parent_joint(5) == 6
    assert pendulum.child_joints(0) == [6] and pendulum.child_joints(5) == []
    assert pendulum.subtree(0) == [0, 5]
    hinge = pendulum.joints()[6]
    assert (hinge["parent"], hinge["child"], hinge["kind"]) == (0, 5, "Revolute")
    assert hinge["origin"] == {"t": [0.0, 0.0, 0.5], "r": [0.0, 0.0, 0.0, 1.0]}
    assert hinge["limits"]["effort"] == 10.0
    arm = pendulum.links()[5]
    assert arm["material"] == "PLA"
    assert arm["collision"] == "SameAsVisual"
    assert arm["inertial"] == {"Computed": {"density_override": None}}
    (geom,) = arm["visuals"]
    assert (geom["id"], geom["mesh"], geom["color"]) == (4, 3, None)
    asset = pendulum.assets()[3]
    assert Path(asset["path"]).is_absolute() and asset["path"].endswith("cube_ascii.stl")
    assert asset["content_hash"] == 13076597094302796077
    assert pendulum.frames() == {}


def test_load_gives_the_same_document_and_no_warnings():
    robot, warnings = Robot.load(PENDULUM)
    assert warnings == []
    assert robot.name == "pendulum" and robot.next_id == 7
    assert robot.link("arm") == 5
    assert Path(robot.assets()[1]["path"]) == FIXTURES / "cube_binary.stl"


def test_load_warns_about_a_changed_mesh(cubes: Path, pendulum: Robot):
    out = cubes / "p.riggen"
    pendulum.save(out)
    (cubes / "cube_ascii.stl").write_bytes(b"solid nothing\nendsolid nothing\n")
    _, warnings = Robot.load(out)
    assert len(warnings) == 1 and "m3" in warnings[0] and "changed" in warnings[0]


def test_load_of_a_missing_file_is_a_file_error(tmp_path: Path):
    with pytest.raises(errors.FileError, match="nowhere.riggen"):
        Robot.load(tmp_path / "nowhere.riggen")
    assert issubclass(errors.FileError, errors.RiggenError)


def test_json_round_trip_and_copy(pendulum: Robot):
    text = pendulum.to_json()
    doc = json.loads(text)
    assert doc["schema_version"] == 2 and doc["robot"]["next_id"] == 7
    again = Robot.from_json(text)
    assert again.to_json() == text
    twin = pendulum.copy()
    twin.rename_link(5, "other")
    assert pendulum.link("arm") == 5 and twin.link("other") == 5


def test_from_json_rejects_garbage_and_invalid_documents(pendulum: Robot):
    with pytest.raises(ValueError):
        Robot.from_json("{not json")
    doc = json.loads(pendulum.to_json())
    doc["robot"]["joints"]["j6"]["limits"] = None  # a revolute joint needs limits
    with pytest.raises(errors.ValidationError, match="j6"):
        Robot.from_json(json.dumps(doc))


def test_set_joint_ignores_parent_and_child(pendulum: Robot):
    pendulum.set_joint(6, hinge_joint(name="elbow", kind="Continuous", limits=None, parent=999, child=999))
    hinge = pendulum.joints()[6]
    assert (hinge["name"], hinge["kind"], hinge["limits"]) == ("elbow", "Continuous", None)
    assert (hinge["parent"], hinge["child"]) == (0, 5)


def test_every_edit_is_one_command(pendulum: Robot, cubes: Path):
    pendulum.rename_joint(6, "pivot")
    assert pendulum.joint("pivot") == 6
    pendulum.move_joint_frame(6, {"t": [0.0, 0.0, 1.0], "r": [0.0, 0.0, 0.0, 1.0]}, [1.0, 0.0, 0.0])
    hinge = pendulum.joints()[6]
    assert hinge["origin"]["t"] == [0.0, 0.0, 1.0] and hinge["axis"] == [1.0, 0.0, 0.0]
    # The arm's geom was re-expressed so it did not move in the world.
    (geom,) = pendulum.links()[5]["visuals"]
    assert geom["pose"]["t"] == [0.0, 0.0, 0.0]

    tip = pendulum.add_link("tip", 5, hinge_joint(name="wrist", kind="Fixed", limits=None))
    assert tip == 7 and pendulum.subtree(0) == [0, 5, 7]
    pendulum.reparent(tip, 0, keep_world_pose=True)
    assert pendulum.joints()[8]["parent"] == 0
    pendulum.remove_link(tip)
    assert 7 not in pendulum.links() and 8 not in pendulum.joints()

    m = pendulum.add_asset(cubes / "cube_binary.stl", scale=0.001)
    g = pendulum.add_geom(5, m, pose=IDENTITY, color=[1.0, 0.0, 0.0, 1.0])
    assert [v["id"] for v in pendulum.links()[5]["visuals"]] == [4, g]
    pendulum.set_asset(m, {"path": str(cubes / "cube_ascii.stl"), "scale": 1.0, "fix_up": None})
    assert pendulum.assets()[m]["path"].endswith("cube_ascii.stl")
    pendulum.remove_geom(5, g)
    assert [v["id"] for v in pendulum.links()[5]["visuals"]] == [4]

    pendulum.upsert_material("brass", {"density": 8500.0, "color": [0.8, 0.6, 0.2, 1.0]})
    pendulum.set_link_material(5, "brass")
    assert pendulum.links()[5]["material"] == "brass"
    pendulum.set_link_material(5, None)
    pendulum.remove_material("brass")
    assert "brass" not in pendulum.materials()

    pendulum.set_inertial(5, {"Hybrid": {"mass": 2.5}})
    assert pendulum.links()[5]["inertial"] == {"Hybrid": {"mass": 2.5}}
    pendulum.set_collision(5, "ConvexHull")
    assert pendulum.links()[5]["collision"] == "ConvexHull"
    pendulum.set_joint(6, hinge_joint(kind="Fixed", limits=None))
    pendulum.set_root(5)
    assert pendulum.root == 5 and pendulum.joints()[6]["parent"] == 5


def test_frame_commands_are_one_edit_each(pendulum: Robot):
    tcp = pendulum.add_frame("tcp", 5, pose={"t": [0.0, 0.0, 0.3], "r": [0.0, 0.0, 0.0, 1.0]})
    assert pendulum.frame("tcp") == tcp
    assert pendulum.frames()[tcp] == {
        "name": "tcp",
        "parent": 5,
        "pose": {"t": [0.0, 0.0, 0.3], "r": [0.0, 0.0, 0.0, 1.0]},
    }
    # The arm sits 0.5 m up, so the frame is at 0.8 m; at 90° about Y it
    # swings out along +X. `fk` itself still returns links only.
    assert pendulum.fk_frames({})[tcp]["t"] == [0.0, 0.0, 0.8]
    assert tcp not in pendulum.fk({})
    swung = pendulum.fk_frames({6: math.pi / 2})[tcp]["t"]
    assert swung == pytest.approx([0.3, 0.0, 0.5], abs=1e-12)

    pendulum.rename_frame(tcp, "tool0")
    assert pendulum.frames()[tcp]["name"] == "tool0"
    pendulum.set_frame(tcp, {"name": "tool0", "parent": 0, "pose": IDENTITY})
    assert pendulum.frames()[tcp]["parent"] == 0
    pendulum.remove_frame(tcp)
    assert pendulum.frames() == {}


def unchanged(robot: Robot):
    """A context that asserts the document (and its id counter) survived."""
    before = robot.to_json()

    class _Check:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            assert robot.to_json() == before, "a refused edit changed the document"
            return False

    return _Check()


@pytest.mark.parametrize(
    "exc, edit",
    [
        (errors.CannotRemoveRoot, lambda r: r.remove_link(r.root)),
        (errors.CannotReparentRoot, lambda r: r.reparent(r.root, 5)),
        (errors.UnknownId, lambda r: r.rename_link(99, "x")),
        (errors.UnknownId, lambda r: r.set_joint(99, hinge_joint())),
        (errors.UnknownId, lambda r: r.remove_geom(5, 99)),
        (errors.UnknownId, lambda r: r.subtree(99)),
        (errors.UnknownMaterial, lambda r: r.remove_material("unobtainium")),
        (errors.MaterialInUse, lambda r: r.remove_material("PLA")),
        (errors.MovableJointOnRootPath, lambda r: r.set_root(5)),
        (errors.InvalidDocument, lambda r: r.rename_link(5, "base_link")),
        (errors.InvalidDocument, lambda r: r.add_link("x", 0, hinge_joint(limits=None))),
        (errors.InvalidDocument, lambda r: r.add_geom(0, 99)),
        (errors.InvalidDocument, lambda r: r.set_link_material(5, "unobtainium")),
        (errors.UnknownId, lambda r: r.remove_frame(99)),
        (errors.UnknownId, lambda r: r.rename_frame(99, "x")),
        (errors.UnknownId, lambda r: r.add_frame("tcp", 99)),
        # One namespace: a frame may not take a link's name (ADR-0012).
        (errors.InvalidDocument, lambda r: r.add_frame("arm", 5)),
    ],
)
def test_refused_edits_raise_and_change_nothing(pendulum: Robot, exc, edit):
    with unchanged(pendulum), pytest.raises(exc) as info:
        edit(pendulum)
    assert isinstance(info.value, errors.EditError)
    assert isinstance(info.value, errors.RiggenError)
    assert str(info.value)


def test_would_create_cycle(pendulum: Robot):
    tip = pendulum.add_link("tip", 5, hinge_joint(name="wrist", kind="Fixed", limits=None))
    with unchanged(pendulum), pytest.raises(errors.WouldCreateCycle, match="l5"):
        pendulum.reparent(5, tip)


def test_missing_mesh_file_names_the_file(pendulum: Robot, tmp_path: Path):
    """The by-hand run's first stumble: a bare "No such file or directory
    (os error 2)" said nothing about *which* file."""
    with pytest.raises(FileNotFoundError, match="nowhere.stl"):
        pendulum.add_asset(tmp_path / "nowhere.stl")
    with pytest.raises(FileNotFoundError, match="gone.stl"):
        pendulum.add_link("x", 0, hinge_joint(name="j"), mesh=tmp_path / "gone.stl")
    with pytest.raises(FileNotFoundError, match="moved.stl"):
        pendulum.set_asset(3, {"path": str(tmp_path / "moved.stl"), "scale": 1.0, "fix_up": None})


def test_malformed_values_are_value_errors(pendulum: Robot):
    with unchanged(pendulum), pytest.raises(ValueError, match="joint: missing field"):
        pendulum.set_joint(6, {"name": "h", "kind": "Revolute"})
    with unchanged(pendulum), pytest.raises(ValueError, match="unknown variant"):
        pendulum.set_collision(5, "Bouncy")
    with unchanged(pendulum), pytest.raises(TypeError):
        pendulum.set_joint(6, object())
