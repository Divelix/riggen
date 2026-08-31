"""The M3 acceptance (ADR-0004 §2, plans/m3-sim-ready step 5).

For every directory given on the command line: load its `*.xml` with
`mujoco.MjModel.from_xml_path`, failing on any compiler warning, and — when
a `<name>.fk.json` sits beside it — set each sampled joint configuration,
`mj_forward`, and compare every body's and every **site's** world pose with
what `riggen_core::fk` wrote, to 1e-6. A site the samples name and the model
does not have is a failure, not a skip: it is how a dropped `<site>` would
look (ADR-0012). Every `mjEQ_JOINT` equality — what a mimic joint is written
as (ADR-0013) — must agree with the sampled `qpos`, and a pair of joints the
samples show as exactly coupled must have one, which is how a dropped
`<equality>` or a `polycoef` in the wrong order would look. A body carrying
convex-decomposition pieces
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


def compare(kind: str, name: str, q: list, pos: np.ndarray, quat: np.ndarray, want: dict) -> None:
    """One world pose from MuJoCo against the one riggen's FK wrote."""
    wpos = np.asarray(want["pos"])
    wquat = np.asarray(want["quat"])
    dpos = np.abs(pos - wpos).max()
    # q and -q are the same rotation.
    dquat = min(np.abs(quat - wquat).max(), np.abs(quat + wquat).max())
    if dpos > TOLERANCE or dquat > TOLERANCE:
        raise AssertionError(
            f"{kind} {name!r} at q={q}: mujoco pos {pos} quat {quat}, "
            f"riggen pos {wpos} quat {wquat} (dpos {dpos:.2e}, dquat {dquat:.2e})"
        )


def check_fk(model: mujoco.MjModel, samples: dict) -> int:
    sites = {model.site(i).name for i in range(model.nsite)}
    missing = sorted(set(samples["samples"][0].get("sites", {})) - sites)
    if missing:
        raise AssertionError(
            f"the samples name site(s) {missing} the model does not have "
            f"(it has {sorted(sites)}): a <site> was dropped on the way out"
        )
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
            compare("body", name, sample["q"], body.xpos, body.xquat, pose)
            checked += 1
        for name, pose in sample.get("sites", {}).items():
            site = data.site(name)
            # `site_xmat` is a 3x3 row-major rotation; MuJoCo has no
            # `site_xquat`, so convert it the way it converts its own.
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, site.xmat)
            compare("site", name, sample["q"], site.xpos, quat, pose)
            checked += 1
    return checked


def affine(xs: list[float], ys: list[float]) -> tuple[float, float] | None:
    """`(a0, a1)` if `ys == a0 + a1 * xs` exactly, with `a1` non-zero.

    Exact, not fitted: both series come out of one linear rule in f64, so
    they agree to rounding or they are unrelated.
    """
    base = next(
        (
            (i, j)
            for i in range(len(xs))
            for j in range(i + 1, len(xs))
            if abs(xs[i] - xs[j]) > 1e-12
        ),
        None,
    )
    if base is None:
        return None
    i, j = base
    a1 = (ys[j] - ys[i]) / (xs[j] - xs[i])
    a0 = ys[i] - a1 * xs[i]
    if abs(a1) < 1e-12:
        return None
    if any(abs(y - (a0 + a1 * x)) > 1e-12 for x, y in zip(xs, ys)):
        return None
    return a0, a1


def check_equalities(model: mujoco.MjModel, samples: dict) -> int:
    """Every joint equality agrees with the sampled qpos, and none is missing.

    A mimic joint is an `<equality><joint polycoef>` (ADR-0013), where
    `polycoef` is `y - y0 = a0 + a1 (x - x0) + …` over the two joints'
    deviations from `qpos0`. riggen never writes `ref`, so both references
    are zero and the rule is plain `y = a0 + a1 x`. The samples carry the
    follower's *derived* value, so the two readings of `polycoef` agree
    here or they do not agree at all — a swapped coefficient order fails.
    """
    q_of = {
        name: [s["q"][i] for s in samples["samples"]]
        for i, name in enumerate(samples["joints"])
    }
    # Unordered, because the relation is symmetric: `y = a0 + a1 x` is
    # also `x = -a0/a1 + (1/a1) y`, and one equality covers both readings.
    coupled: set[frozenset[str]] = set()
    equalities = 0
    for e in range(model.neq):
        if model.eq_type[e] != mujoco.mjtEq.mjEQ_JOINT:
            continue
        follower = model.joint(int(model.eq_obj1id[e])).name
        leader = model.joint(int(model.eq_obj2id[e])).name
        a0, a1 = (float(v) for v in model.eq_data[e][:2])
        higher = [float(v) for v in model.eq_data[e][2:5]]
        if any(higher):
            raise AssertionError(
                f"equality {follower!r}/{leader!r} has non-linear polycoef {higher}; "
                "riggen only ever writes the first two coefficients"
            )
        equalities += 1
        coupled.add(frozenset({follower, leader}))
        if follower not in q_of or leader not in q_of:
            raise AssertionError(
                f"equality couples {follower!r} to {leader!r}, which the samples "
                f"do not name (they have {sorted(q_of)})"
            )
        for i, (f, l) in enumerate(zip(q_of[follower], q_of[leader])):
            want = a0 + a1 * l
            if abs(f - want) > TOLERANCE:
                raise AssertionError(
                    f"equality {follower!r} = {a0} + {a1} * {leader!r}: sample {i} has "
                    f"{follower}={f} and {leader}={l}, which polycoef makes {want}"
                )
    # …and nothing was dropped on the way out: a joint whose sampled values
    # are an exact linear function of another's is a mimic, and must have
    # brought its equality with it.
    for follower, ys in q_of.items():
        for leader, xs in q_of.items():
            if leader == follower or frozenset({follower, leader}) in coupled:
                continue
            fit = affine(xs, ys)
            if fit:
                raise AssertionError(
                    f"the samples have {follower!r} = {fit[0]} + {fit[1]} * {leader!r} "
                    "at every configuration, but the model has no equality for it: "
                    "a mimic joint was dropped"
                )
    return equalities


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
            summary = f"{model.nbody} bodies, {model.njnt} joints, {model.nsite} sites, {model.nmesh} meshes"
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
                samples = json.loads(fk.read_text())
                try:
                    n = check_fk(model, samples)
                    equalities = check_equalities(model, samples)
                except AssertionError as e:
                    print(f"FAIL {xml}: {e}")
                    failures += 1
                    continue
                summary += f", {n} body and site poses match riggen's FK to {TOLERANCE:g}"
                if equalities:
                    word = "equality" if equalities == 1 else "equalities"
                    summary += f", {equalities} mimic {word} checked against the samples"
            print(f"ok   {xml}: {summary}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
