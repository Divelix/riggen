"""The M3 acceptance (ADR-0004 §2, plans/m3-sim-ready step 5).

For every directory given on the command line: load its `*.xml` with
`mujoco.MjModel.from_xml_path`, failing on any compiler warning, and — when
a `<name>.fk.json` sits beside it — set each sampled joint configuration,
`mj_forward`, and compare every body's world pose with what `riggen_core::fk`
wrote, to 1e-6. A body carrying convex-decomposition pieces
(`<stem>_hull_0`, `_1`, … — ADR-0011) must carry more than one of them:
MuJoCo hulls a collision mesh itself, so a single piece would mean the
part collides as a solid block and the policy bought nothing.

    uv run --with mujoco --with numpy python python/tests/test_mjcf_load.py target/sample

Plain script, no pytest: the CI job is the four lines in the plan's
Acceptance block and nothing else.
"""

import json
import re
import sys
from pathlib import Path

import mujoco
import numpy as np

TOLERANCE = 1e-6


class WarningError(RuntimeError):
    pass


def fail_on_warning(message: str) -> None:
    raise WarningError(message)


def load(xml: Path) -> mujoco.MjModel:
    mujoco.set_mju_user_warning(fail_on_warning)
    try:
        return mujoco.MjModel.from_xml_path(str(xml))
    finally:
        mujoco.set_mju_user_warning(None)


def check_fk(model: mujoco.MjModel, samples: dict) -> int:
    data = mujoco.MjData(model)
    checked = 0
    for sample in samples["samples"]:
        mujoco.mj_resetData(model, data)
        for name, q in zip(samples["joints"], sample["q"]):
            joint = model.joint(name)
            data.qpos[joint.qposadr[0]] = q
        mujoco.mj_forward(model, data)
        for name, pose in sample["links"].items():
            body = data.body(name)
            pos = np.asarray(pose["pos"])
            quat = np.asarray(pose["quat"])
            dpos = np.abs(body.xpos - pos).max()
            # q and -q are the same rotation.
            dquat = min(np.abs(body.xquat - quat).max(), np.abs(body.xquat + quat).max())
            if dpos > TOLERANCE or dquat > TOLERANCE:
                raise AssertionError(
                    f"body {name!r} at q={sample['q']}: mujoco pos {body.xpos} quat {body.xquat}, "
                    f"riggen pos {pos} quat {quat} (dpos {dpos:.2e}, dquat {dquat:.2e})"
                )
            checked += 1
    return checked


PIECE = re.compile(r"^(?P<stem>.+)_hull_(?P<index>\d+)$")


def check_decomposition(model: mujoco.MjModel) -> int:
    """Every convex decomposition in the model is several geoms on one body.

    `riggen-export` writes `<stem>_hull_0 … _<N-1>` and one collision geom
    per piece, so the pieces are recognisable from the model alone — no
    fixture knowledge here. One piece would mean the export collapsed the
    policy to a hull, which MuJoCo would have taken anyway.
    """
    found: dict[tuple[int, str], set[int]] = {}
    for g in range(model.ngeom):
        mesh_id = model.geom_dataid[g]
        if mesh_id < 0:
            continue
        match = PIECE.match(model.mesh(mesh_id).name or "")
        if match:
            key = (int(model.geom_bodyid[g]), match["stem"])
            found.setdefault(key, set()).add(int(match["index"]))
    for (body, stem), indices in sorted(found.items()):
        name = model.body(body).name
        if len(indices) < 2:
            raise AssertionError(
                f"body {name!r} has {len(indices)} piece of {stem!r}: "
                "a decomposition of one piece is a convex hull"
            )
        if sorted(indices) != list(range(len(indices))):
            raise AssertionError(f"body {name!r}: {stem!r} pieces are {sorted(indices)}, not 0..N")
    return sum(len(i) for i in found.values())


def main(argv: list[str]) -> int:
    dirs = [Path(a) for a in argv] or [Path("target/sample")]
    failures = 0
    for directory in dirs:
        xmls = sorted(directory.glob("*.xml"))
        if not xmls:
            print(f"FAIL {directory}: no .xml in it")
            failures += 1
            continue
        for xml in xmls:
            try:
                model = load(xml)
            except Exception as e:  # noqa: BLE001 — any load problem is the verdict
                print(f"FAIL {xml}: {type(e).__name__}: {e}")
                failures += 1
                continue
            summary = f"{model.nbody} bodies, {model.njnt} joints, {model.nmesh} meshes"
            try:
                pieces = check_decomposition(model)
            except AssertionError as e:
                print(f"FAIL {xml}: {e}")
                failures += 1
                continue
            if pieces:
                summary += f", {pieces} convex-decomposition geoms"
            fk = xml.with_suffix(".fk.json")
            if fk.exists():
                try:
                    n = check_fk(model, json.loads(fk.read_text()))
                except AssertionError as e:
                    print(f"FAIL {xml}: {e}")
                    failures += 1
                    continue
                summary += f", {n} body poses match riggen's FK to {TOLERANCE:g}"
            print(f"ok   {xml}: {summary}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
