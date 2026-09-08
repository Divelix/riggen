"""The M3 acceptance (ADR-0004 §2, plans/m3-sim-ready step 5).

For every directory given on the command line: load its `*.xml` with
`mujoco.MjModel.from_xml_path`, failing on any compiler warning, and — when
a `<name>.fk.json` sits beside it — set each sampled joint configuration
(its `q` is MuJoCo's `qpos`: the document's `q` plus `<joint ref>`, ADR-0025),
`mj_forward`, and compare every body's and every **site's** world pose with
what `riggen_core::fk` wrote, to 1e-6. A site the samples name and the model
does not have is a failure, not a skip: it is how a dropped `<site>` would
look (ADR-0012). Every `mjEQ_JOINT` equality — what a mimic joint is written
as (ADR-0013) — must agree with the sampled `qpos`, and a pair of joints the
samples show as exactly coupled must have one, which is how a dropped
`<equality>` or a `polycoef` in the wrong order would look. Every actuator
the samples name (ADR-0014, ADR-0023) must be in the model, under its own
name, driving the joint — or the **tendon** (ADR-0025 §4) — the samples
name, with the gains and the two ranges
they give — and the model
may carry no others: `model.nu` is the count, never a `> 0` that a wrong
preset would pass. Every `<tendon><fixed>` the samples name must be in the
model too, over the joints and coefficients they give, with the length they
compute at every sampled configuration — `Σ coef · qpos`, absolute, which
is how MuJoCo evaluates one. A body carrying
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
re-export, so the step that starts reading it has to delete its line. The
`<equality>` and `<tendon>` blocks are compared the same way (ADR-0025), and
nothing in either is dropped by name: a coupling or a tendon that went
missing, was flattened or was reordered is a failure.

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
    deviations from `qpos0`, and `qpos0` is each joint's `<joint ref>` —
    which riggen writes as `Joint::qpos_ref` (ADR-0025 §3) — so the
    deviations are read through `model.qpos0` here, and a `ref` the writer
    shifted wrongly would fail. The samples carry the follower's *derived*
    value, so the two readings of `polycoef` agree here or they do not agree
    at all — a swapped coefficient order fails.

    A **chain** is written as it is, not flattened (ADR-0025 §2), so a
    follower's leader may itself be a follower. Each equality is checked on
    its own; the "nothing was dropped" sweep below only exempts a pair
    coupled *transitively*, so a genuinely missing equality still fails.
    """
    q_of = {
        name: [s["q"][i] for s in samples["samples"]]
        for i, name in enumerate(samples["joints"])
    }

    def qpos0(name: str) -> float:
        return float(model.qpos0[model.joint(name).qposadr[0]])

    # Union-find over the coupled joints. Unordered, because the relation
    # is symmetric: `y = a0 + a1 x` is also `x = -a0/a1 + (1/a1) y`, and one
    # equality covers both readings. Transitive, because a chain is legal
    # (ADR-0025 §1): the two ends of `f = 0.5 l`, `l = 0.25 p` are an exact
    # linear function of each other with no equality of their own, which is
    # not a dropped mimic.
    group: dict[str, str] = {}

    def find(j: str) -> str:
        while group.setdefault(j, j) != j:
            j = group[j]
        return j

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
        group[find(follower)] = find(leader)
        if follower not in q_of or leader not in q_of:
            raise AssertionError(
                f"equality couples {follower!r} to {leader!r}, which the samples "
                f"do not name (they have {sorted(q_of)})"
            )
        f0, l0 = qpos0(follower), qpos0(leader)
        for i, (f, l) in enumerate(zip(q_of[follower], q_of[leader])):
            want = f0 + a0 + a1 * (l - l0)
            if abs(f - want) > TOLERANCE:
                raise AssertionError(
                    f"equality {follower!r} - {f0} = {a0} + {a1} * ({leader!r} - {l0}): "
                    f"sample {i} has {follower}={f} and {leader}={l}, which polycoef "
                    f"makes {want}"
                )
    # …and nothing was dropped on the way out: a joint whose sampled values
    # are an exact linear function of another's is a mimic, and must have
    # brought its equality with it.
    for follower, ys in q_of.items():
        for leader, xs in q_of.items():
            if leader == follower or find(follower) == find(leader):
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

    An actuator has a name of its own (ADR-0023) — its target's by default,
    but a file may have said otherwise — so the check reads the sampled
    `name` and `target` separately rather than deriving one from the other.
    It is data-driven: the samples say what should be there and an actuator
    they name that the model lacks is a failure — which is how a dropped
    `<actuator>` looks. Several actuators may name one target, since MuJoCo
    sums their controls. A URDF-imported robot legitimately has none, and
    then `model.nu` must be zero too.

    An actuator drives a joint or a fixed **tendon** (ADR-0025 §4), which
    the samples say as `trntype`; a tendon one has no joint behind it, so
    the samples derive neither range for it and MuJoCo's own unbounded
    defaults are what is checked.

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
        wanted_trn = {"joint": mujoco.mjtTrn.mjTRN_JOINT, "tendon": mujoco.mjtTrn.mjTRN_TENDON}
        kind = spec["trntype"]
        trntype = mujoco.mjtTrn(int(model.actuator_trntype[i]))
        if trntype != wanted_trn[kind]:
            raise AssertionError(
                f"actuator {name!r} has transmission {trntype.name}; "
                f"the samples say it drives a {kind}"
            )
        by_id = model.joint if kind == "joint" else model.tendon
        driven = by_id(int(model.actuator_trnid[i][0])).name
        if driven != spec["target"]:
            raise AssertionError(
                f"actuator {name!r} drives {kind} {driven!r}, not {spec['target']!r}"
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
        # unit gain with the gear in the transmission. A `<general>`
        # (ADR-0024) is MuJoCo's own model spelled out: its three types by
        # name and its three `prm` vectors, which riggen writes trimmed and
        # MuJoCo zero-fills, so each is compared padded to MuJoCo's ten.
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
        elif kind == "general":
            expected = [("gear", gear, gains["gear"])]
            general = spec["general"]
            for what, enum in (("dyntype", mujoco.mjtDyn), ("gaintype", mujoco.mjtGain),
                               ("biastype", mujoco.mjtBias)):
                got = enum(int(getattr(model, f"actuator_{what}")[i])).name
                if got != f"mj{what[:-4].upper()}_{general[what].upper()}":
                    raise AssertionError(
                        f"actuator {name!r} (general): mujoco has {what} {got}, "
                        f"the samples say {general[what]!r}"
                    )
            for what in ("dynprm", "gainprm", "biasprm"):
                got = getattr(model, f"actuator_{what}")[i]
                padded = np.zeros(len(got))
                padded[:len(general[what])] = general[what]
                if np.abs(got - padded).max() > TOLERANCE:
                    raise AssertionError(
                        f"actuator {name!r} (general): mujoco has {what} {got.tolist()}, "
                        f"the samples say {general[what]}"
                    )
        else:
            raise AssertionError(f"actuator {name!r} has unknown kind {kind!r}")
        for label, got, wanted in expected:
            if abs(float(got) - wanted) > TOLERANCE:
                raise AssertionError(
                    f"actuator {name!r} ({kind}, gains {gains}): mujoco has {float(got)} "
                    f"where {label} = {wanted} belongs"
                )
    return len(want)


def check_tendons(model: mujoco.MjModel, samples: dict) -> int:
    """Every `<tendon><fixed>` riggen wrote is in the model, and is its length.

    A fixed tendon is a wrap list of joints with coefficients (ADR-0025
    §4), so the check reads `model.wrap_*` and holds it to the joints and
    coefficients the samples name, in order — a reordered or renamed wrap
    fails, and a wrap that is not a joint means a `<spatial>` got out.
    Then the length: MuJoCo evaluates a fixed tendon over the **absolute**
    `qpos`, not over the deviations from `qpos0` an equality uses, so the
    samples' `Σ coef · (q + ref)` must be `data.ten_length` at every
    sampled configuration — a `qpos_ref` folded in on either side by
    mistake shows up here.

    A tendon the samples name that the model lacks is a failure, which is
    how a dropped `<tendon>` would look, and the model may carry no others.
    """
    want = samples.get("tendons", [])
    have = {model.tendon(i).name for i in range(model.ntendon)}
    missing = sorted({t["name"] for t in want} - have)
    if missing:
        raise AssertionError(
            f"the samples name tendon(s) {missing} the model does not have "
            f"(it has {sorted(have)}): a <tendon> was dropped on the way out"
        )
    if model.ntendon != len(want):
        raise AssertionError(
            f"the model has {model.ntendon} tendon(s) {sorted(have)}, "
            f"the samples name {len(want)}"
        )
    for spec in want:
        name = spec["name"]
        i = int(model.tendon(name).id)
        adr, num = int(model.tendon_adr[i]), int(model.tendon_num[i])
        wraps = []
        for w in range(adr, adr + num):
            wrap = mujoco.mjtWrap(int(model.wrap_type[w]))
            if wrap != mujoco.mjtWrap.mjWRAP_JOINT:
                raise AssertionError(
                    f"tendon {name!r} wraps a {wrap.name}, not a joint: "
                    "riggen only ever writes <fixed>"
                )
            wraps.append((model.joint(int(model.wrap_objid[w])).name, float(model.wrap_prm[w])))
        wanted = [(j["joint"], j["coef"]) for j in spec["joints"]]
        if [n for n, _ in wraps] != [n for n, _ in wanted] or not np.allclose(
            [c for _, c in wraps], [c for _, c in wanted], rtol=0, atol=TOLERANCE
        ):
            raise AssertionError(
                f"tendon {name!r} is over {wraps}, the samples say {wanted}"
            )
        limited = bool(model.tendon_limited[i])
        if limited != spec["limited"]:
            raise AssertionError(
                f"tendon {name!r} is limited={limited} with range "
                f"{model.tendon_range[i].tolist()}; the samples say "
                f"limited={spec['limited']} with {spec.get('range')}"
            )
        if limited and (
            spec.get("range") is None
            or np.abs(np.asarray(spec["range"]) - model.tendon_range[i]).max() > TOLERANCE
        ):
            raise AssertionError(
                f"tendon {name!r} range is {model.tendon_range[i].tolist()}, "
                f"not {spec.get('range')}"
            )
        for what in ("stiffness", "damping", "frictionloss"):
            got = float(getattr(model, f"tendon_{what}")[i])
            if abs(got - spec[what]) > TOLERANCE:
                raise AssertionError(
                    f"tendon {name!r} {what} is {got}, the samples say {spec[what]}"
                )
    if not want:
        return 0
    data = mujoco.MjData(model)
    for s, sample in enumerate(samples["samples"]):
        mujoco.mj_resetData(model, data)
        for jname, q in zip(samples["joints"], sample["q"]):
            data.qpos[model.joint(jname).qposadr[0]] = q
        mujoco.mj_forward(model, data)
        for spec in want:
            i = int(model.tendon(spec["name"]).id)
            got, wanted = float(data.ten_length[i]), spec["lengths"][s]
            if abs(got - wanted) > TOLERANCE:
                raise AssertionError(
                    f"tendon {spec['name']!r} at sample {s} (q={sample['q']}): "
                    f"mujoco has ten_length {got}, riggen wrote {wanted}"
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
ROUND_TRIP_DROPPED: dict[str, str] = {}


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
        if len(a) != len(b):
            return False
        # A wrap list is `(name, coef)` pairs: the names must be equal, the
        # numbers only close.
        if a and isinstance(a[0], tuple):
            return all(x[0] == y[0] and same(x[1], y[1]) for x, y in zip(a, b))
        return bool(np.allclose(a, b, rtol=0, atol=TOLERANCE))
    if isinstance(a, float) or isinstance(b, float):
        return abs(a - b) <= TOLERANCE
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


# The couplings, compared the same way (ADR-0025 §1, §3): what MuJoCo
# holds for a joint equality, between the original and the re-export. The
# `polycoef` is over deviations from `qpos0`, so a `<joint ref>` the writer
# shifted wrongly changes `data` here and not only `qpos0`.
EQUALITY_FIELDS = ("follower", "leader", "data", "active")
# And the tendons (ADR-0025 §4): the wrap list with its coefficients, the
# range and its flag, the passive dynamics. A `<fixed>`'s `springlength`,
# `margin`, `armature` and `sol*` pairs are counted on the way in and not
# carried — the bounded promise ADR-0024 §4 made, and the reason they are
# no more compared here than `actdim` and `lengthrange` are above.
TENDON_FIELDS = ("wraps", "range", "limited", "stiffness", "damping", "frictionloss")


def equality_fields(model: mujoco.MjModel, e: int) -> dict:
    """Joint equality `e` as comparable values, its two joints by name."""
    return {
        "follower": model.joint(int(model.eq_obj1id[e])).name,
        "leader": model.joint(int(model.eq_obj2id[e])).name,
        "data": model.eq_data[e][:5].tolist(),
        "active": bool(model.eq_active0[e]),
    }


def joint_equalities(model: mujoco.MjModel) -> list[int]:
    return [e for e in range(model.neq) if model.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT]


def check_round_trip_equalities(original: mujoco.MjModel, model: mujoco.MjModel) -> int:
    """The re-export's `<equality>` block is the original's, coupling for coupling.

    In order, because MuJoCo evaluates them in order and a chain's two
    links are not interchangeable. Returns how many agreed.
    """
    a, b = joint_equalities(original), joint_equalities(model)
    want = [equality_fields(original, e) for e in a]
    have = [equality_fields(model, e) for e in b]
    pair = lambda f: f"{f['follower']}<-{f['leader']}"  # noqa: E731
    if [pair(f) for f in have] != [pair(f) for f in want]:
        raise AssertionError(
            f"the re-export couples {[pair(f) for f in have]}, the original "
            f"{[pair(f) for f in want]}: an <equality><joint> was dropped, "
            "invented, flattened or reordered on the way through"
        )
    for x, y in zip(want, have):
        for field in EQUALITY_FIELDS:
            if not same(x[field], y[field]):
                raise AssertionError(
                    f"equality {pair(x)} {field}: the original has {x[field]}, "
                    f"the re-export {y[field]}"
                )
    return len(want)


def tendon_fields(model: mujoco.MjModel, i: int) -> dict:
    """Tendon `i` as comparable values, its wraps as `(joint name, coef)`.

    A wrap that is not a joint is a `<spatial>`, which riggen never writes,
    so it is reported as itself rather than compared.
    """
    adr, num = int(model.tendon_adr[i]), int(model.tendon_num[i])
    wraps = []
    for w in range(adr, adr + num):
        wrap = mujoco.mjtWrap(int(model.wrap_type[w]))
        if wrap != mujoco.mjtWrap.mjWRAP_JOINT:
            wraps.append((wrap.name, float(model.wrap_prm[w])))
        else:
            wraps.append((model.joint(int(model.wrap_objid[w])).name, float(model.wrap_prm[w])))
    return {
        "wraps": wraps,
        "range": model.tendon_range[i].tolist(),
        "limited": bool(model.tendon_limited[i]),
        "stiffness": float(model.tendon_stiffness[i]),
        "damping": float(model.tendon_damping[i]),
        "frictionloss": float(model.tendon_frictionloss[i]),
    }


def check_round_trip_tendons(original: mujoco.MjModel, model: mujoco.MjModel) -> int:
    """The re-export's `<tendon>` block is the original's, tendon for tendon.

    In order, because a tendon's index is its slot in `ten_length` and a
    policy reading the original addresses it by that. Returns how many
    agreed.
    """
    names = [original.tendon(i).name for i in range(original.ntendon)]
    have = [model.tendon(i).name for i in range(model.ntendon)]
    if have != names:
        raise AssertionError(
            f"the re-export's tendons are {have}, the original's are {names}: "
            "a <tendon> was dropped, invented or reordered on the way through"
        )
    for name in names:
        x = tendon_fields(original, int(original.tendon(name).id))
        y = tendon_fields(model, int(model.tendon(name).id))
        for field in TENDON_FIELDS:
            if not same(x[field], y[field]):
                raise AssertionError(
                    f"tendon {name!r} {field}: the original has {x[field]}, "
                    f"the re-export {y[field]}"
                )
    return len(names)


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
                    tendons = check_tendons(model, samples)
                except AssertionError as e:
                    print(f"FAIL {xml}: {e}")
                    failures += 1
                    continue
                summary += f", {n} body and site poses match riggen's FK to {TOLERANCE:g}"
                if equalities:
                    word = "equality" if equalities == 1 else "equalities"
                    summary += f", {equalities} mimic {word} checked against the samples"
                summary += f", {actuators} actuator(s) match what the samples ask for"
                if tendons:
                    word = "tendon" if tendons == 1 else "tendons"
                    summary += f", {tendons} fixed {word} match theirs"
            if original_xml:
                try:
                    original = load(Path(original_xml))
                    agreed = check_round_trip_actuators(original, model)
                    coupled = check_round_trip_equalities(original, model)
                    tied = check_round_trip_tendons(original, model)
                except (AssertionError, WarningError, ValueError) as e:
                    print(f"FAIL {xml} against {original_xml}: {type(e).__name__}: {e}")
                    failures += 1
                    continue
                summary += (
                    f", {agreed} actuator(s), {coupled} equality/-ies and {tied} tendon(s)"
                    f" are {original_xml}'s field for field"
                    f" ({len(ROUND_TRIP_DROPPED)} actuator(s) dropped by name)"
                )
            print(f"ok   {xml}: {summary}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
