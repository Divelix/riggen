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
`<equality>` or a `polycoef` in the wrong order would look. Every actuator
the samples name (ADR-0014, ADR-0023) must be in the model, under its own
name, driving the joint the samples name, with the gains and the two ranges
they give — and the model
may carry no others: `model.nu` is the count, never a `> 0` that a wrong
preset would pass. A body carrying
convex-decomposition pieces
(`<stem>_hull_0`, `_1`, … — ADR-0011) must carry more than one of them:
MuJoCo hulls a collision mesh itself, so a single piece would mean the
part collides as a solid block and the policy bought nothing.

An argument may be `MODEL_DIR=SAMPLES_DIR`, and then the model comes from
the first and its `<name>.fk.json` from the second. That is how the MJCF
round trip is checked (ADR-0015): the arm exported, imported back and
exported again has to reproduce the *original* document's FK, not merely
agree with its own.

An argument may also end in `@ORIGINAL.xml`, a *foreign* MJCF the directory's
model is riggen's re-export of (ADR-0024, plans/actuator-escape-hatch). Then
the model MuJoCo builds from the original is compared with the one it builds
from the re-export, actuator by actuator and in order — transmission, target,
the three types, the three `prm` vectors, `gear`, the two ranges and their
`*limited` flags — and what riggen still drops is `ROUND_TRIP_DROPPED`: a
name and its reason, and a promise that the actuator is *absent* from the
re-export, so the step that starts reading it has to delete its line.

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


def check_actuators(model: mujoco.MjModel, samples: dict) -> int:
    """Every `<actuator>` riggen wrote is in the model, driving the right joint.

    An actuator has a name of its own (ADR-0023) — its joint's by default,
    but a file may have said otherwise — so the check reads the sampled
    `name` and `joint` separately rather than deriving one from the other.
    It is data-driven: the samples say what should be there and an actuator
    they name that the model lacks is a failure — which is how a dropped
    `<actuator>` looks. Several actuators may name one joint, since MuJoCo
    sums their controls. A URDF-imported robot legitimately has none, and
    then `model.nu` must be zero too.

    The samples say which `ctrllimited` / `forcelimited` MuJoCo must end up
    with (ADR-0024) — an actuator's own flag, else `autolimits`' rule over
    the range riggen wrote — and, where it is limited, the range itself.
    Where riggen leaves a range out (a zero effort or velocity is the
    *unfilled* value, not a clamp to zero; an imported actuator that named
    none is unlimited) the flag must be off, so the actuator is unbounded
    rather than stuck.
    """
    want = samples.get("actuators", [])
    have = {model.actuator(i).name for i in range(model.nu)}
    missing = sorted({a["name"] for a in want} - have)
    if missing:
        raise AssertionError(
            f"the samples name actuator(s) {missing} the model does not have "
            f"(it has {sorted(have)}): an <actuator> was dropped on the way out"
        )
    if model.nu != len(want):
        raise AssertionError(
            f"the model has {model.nu} actuator(s) {sorted(have)}, "
            f"the samples name {len(want)}"
        )
    for spec in want:
        name = spec["name"]
        i = int(model.actuator(name).id)
        if int(model.actuator_trntype[i]) != mujoco.mjtTrn.mjTRN_JOINT:
            raise AssertionError(f"actuator {name!r} does not drive a joint")
        driven = model.joint(int(model.actuator_trnid[i][0])).name
        if driven != spec["joint"]:
            raise AssertionError(
                f"actuator {name!r} drives joint {driven!r}, not {spec['joint']!r}"
            )
        for what in ("ctrl", "force"):
            limited = bool(getattr(model, f"actuator_{what}limited")[i])
            got = getattr(model, f"actuator_{what}range")[i]
            wanted = spec.get(f"{what}range")
            wanted_limited = spec[f"{what}limited"]
            if limited != wanted_limited:
                raise AssertionError(
                    f"actuator {name!r} is {what}limited={limited} with {what}range "
                    f"{list(got)}; the samples say {what}limited={wanted_limited} "
                    f"with {wanted}"
                )
            if limited and (wanted is None or np.abs(np.asarray(wanted) - got).max() > TOLERANCE):
                raise AssertionError(
                    f"actuator {name!r} {what}range is {list(got)}, not {wanted}"
                )
        # Where MuJoCo puts each preset's gains: `<position kp kv>` is
        # gainprm[0] = kp with an affine bias (-kp, -kv), `<velocity kv>`
        # is gainprm[0] = kv with bias (0, -kv), and `<motor gear>` is a
        # unit gain with the gear in the transmission.
        gains, kind = spec["gains"], spec["kind"]
        gain, bias = model.actuator_gainprm[i], model.actuator_biasprm[i]
        gear = model.actuator_gear[i][0]
        if kind == "position":
            expected = [("kp", gain[0], gains["kp"]), ("-kp", bias[1], -gains["kp"]),
                        ("-kv", bias[2], -gains["kv"]), ("gear", gear, 1.0)]
        elif kind == "velocity":
            expected = [("kv", gain[0], gains["kv"]), ("-kv", bias[2], -gains["kv"]),
                        ("gear", gear, 1.0)]
        elif kind == "motor":
            expected = [("gain", gain[0], 1.0), ("gear", gear, gains["gear"])]
        else:
            raise AssertionError(f"actuator {name!r} has unknown kind {kind!r}")
        for label, got, wanted in expected:
            if abs(float(got) - wanted) > TOLERANCE:
                raise AssertionError(
                    f"actuator {name!r} ({kind}, gains {gains}): mujoco has {float(got)} "
                    f"where {label} = {wanted} belongs"
                )
    return len(want)


# The round trip of a foreign file (ADR-0024): what MuJoCo holds for an
# actuator, compared field for field between the original and the re-export.
# The two ranges and their flags are the actuator's own since the document
# keeps them (schema 5); a range MuJoCo does not limit by is still its
# numbers, so `ctrlrange="0 0"` and no `ctrlrange` compare unequal.
ROUND_TRIP_FIELDS = (
    "trntype", "target", "dyntype", "gaintype", "biastype",
    "dynprm", "gainprm", "biasprm", "gear",
    "ctrlrange", "forcerange", "ctrllimited", "forcelimited",
)
# What riggen still drops on the way through, by name and with the reason.
# Every entry is checked both ways: the original has it, the re-export does
# not. The step that starts reading one deletes its line here.
ROUND_TRIP_DROPPED = {
    "lift": "a <general>; plans/actuator-escape-hatch step 6 reads it",
    "grip": "drives a tendon, and the document has none until the couplings bullet",
}


def actuator_fields(model: mujoco.MjModel, i: int) -> dict:
    """Actuator `i` as comparable values: names for ids and enums, lists for arrays.

    `trnid` is an index into the model's joints (tendons, sites, bodies),
    and two models need not number them alike, so the target is its
    *name*; `trntype` and the three types are their enum names, so a
    failure reads `filter`, not `2`.
    """
    trntype = mujoco.mjtTrn(int(model.actuator_trntype[i]))
    trnid = int(model.actuator_trnid[i][0])
    target = {
        mujoco.mjtTrn.mjTRN_JOINT: model.joint,
        mujoco.mjtTrn.mjTRN_JOINTINPARENT: model.joint,
        mujoco.mjtTrn.mjTRN_TENDON: model.tendon,
        mujoco.mjtTrn.mjTRN_SITE: model.site,
        mujoco.mjtTrn.mjTRN_BODY: model.body,
    }.get(trntype)
    return {
        "trntype": trntype.name,
        "target": target(trnid).name if target else trnid,
        "dyntype": mujoco.mjtDyn(int(model.actuator_dyntype[i])).name,
        "gaintype": mujoco.mjtGain(int(model.actuator_gaintype[i])).name,
        "biastype": mujoco.mjtBias(int(model.actuator_biastype[i])).name,
        "dynprm": model.actuator_dynprm[i].tolist(),
        "gainprm": model.actuator_gainprm[i].tolist(),
        "biasprm": model.actuator_biasprm[i].tolist(),
        "gear": model.actuator_gear[i].tolist(),
        "ctrlrange": model.actuator_ctrlrange[i].tolist(),
        "forcerange": model.actuator_forcerange[i].tolist(),
        "ctrllimited": bool(model.actuator_ctrllimited[i]),
        "forcelimited": bool(model.actuator_forcelimited[i]),
    }


def same(a, b) -> bool:
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and bool(np.allclose(a, b, rtol=0, atol=TOLERANCE))
    return a == b


def check_round_trip_actuators(original: mujoco.MjModel, model: mujoco.MjModel) -> int:
    """The re-export's `<actuator>` block is the original's, element for element.

    In order, because an actuator's index is its slot in `ctrl`, and a
    policy trained on the original addresses it by that. Returns how many
    actuators agreed.
    """
    names = [original.actuator(i).name for i in range(original.nu)]
    have = [model.actuator(i).name for i in range(model.nu)]
    for name, reason in ROUND_TRIP_DROPPED.items():
        if name not in names:
            raise AssertionError(
                f"ROUND_TRIP_DROPPED names {name!r} ({reason}), which the original "
                f"does not have (it has {names}): a stale entry"
            )
        if name in have:
            raise AssertionError(
                f"actuator {name!r} is in the re-export, but ROUND_TRIP_DROPPED still "
                f"says riggen drops it ({reason}): delete its line"
            )
    want = [n for n in names if n not in ROUND_TRIP_DROPPED]
    if have != want:
        raise AssertionError(
            f"the re-export's actuators are {have}, the original's — less the "
            f"{sorted(ROUND_TRIP_DROPPED)} riggen drops by name — are {want}: "
            "one was dropped, invented or reordered on the way through"
        )
    for name in want:
        a = actuator_fields(original, int(original.actuator(name).id))
        b = actuator_fields(model, int(model.actuator(name).id))
        for field in ROUND_TRIP_FIELDS:
            if not same(a[field], b[field]):
                raise AssertionError(
                    f"actuator {name!r} {field}: the original has {a[field]}, "
                    f"the re-export {b[field]}"
                )
    return len(want)


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


def parse_spec(arg: str) -> tuple[str, str, str | None]:
    """`MODEL_DIR[=SAMPLES_DIR][@ORIGINAL.xml]` → (model dir, samples dir, original)."""
    dirs, _, original = arg.partition("@")
    model_dir, _, samples_dir = dirs.partition("=")
    return model_dir, samples_dir or model_dir, original or None


def main(argv: list[str]) -> int:
    specs = [parse_spec(a) for a in argv] or [("target/sample", "target/sample", None)]
    failures = 0
    for model_dir, samples_dir, original_xml in specs:
        directory, samples_root = Path(model_dir), Path(samples_dir)
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
            fk = (samples_root / xml.name).with_suffix(".fk.json")
            if fk.exists():
                samples = json.loads(fk.read_text())
                try:
                    n = check_fk(model, samples)
                    equalities = check_equalities(model, samples)
                    actuators = check_actuators(model, samples)
                except AssertionError as e:
                    print(f"FAIL {xml}: {e}")
                    failures += 1
                    continue
                summary += f", {n} body and site poses match riggen's FK to {TOLERANCE:g}"
                if equalities:
                    word = "equality" if equalities == 1 else "equalities"
                    summary += f", {equalities} mimic {word} checked against the samples"
                summary += f", {actuators} actuator(s) match what the samples ask for"
            if original_xml:
                try:
                    original = load(Path(original_xml))
                    agreed = check_round_trip_actuators(original, model)
                except (AssertionError, WarningError, ValueError) as e:
                    print(f"FAIL {xml} against {original_xml}: {type(e).__name__}: {e}")
                    failures += 1
                    continue
                summary += (
                    f", {agreed} actuator(s) are {original_xml}'s field for field"
                    f" ({len(ROUND_TRIP_DROPPED)} dropped by name)"
                )
            print(f"ok   {xml}: {summary}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
