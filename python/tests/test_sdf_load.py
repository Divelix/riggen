"""The SDF acceptance (ADR-0016 §6, plans/sdf-export step 7).

For every directory given on the command line: load its `*.sdf` with
**libsdformat**, the spec's own parser, through the `sdformat` Python
bindings — `Root.load` raises `SDFErrorsException` on anything illegal, so
"is it legal SDF" is answered by the reference implementation and not by
us. Then, when a `<name>.fk.json` sits beside it, do forward kinematics
from what that parser resolved and compare every link's and every
**frame's** world pose with what `riggen_core::fk` wrote, to 1e-9.

The FK here is deliberately the dumbest possible: libsdformat resolves the
pose graph (`//pose/@relative_to` — ADR-0016 §2) and the axis frames, and
this file only walks the tree and applies `q`. That split is the point.
The parser is held to nothing it does not compute, and everything riggen
could get wrong about *where things are* — a link posed against the wrong
frame, an axis in the wrong frame, a frame attached to the wrong link — is
the parser's arithmetic disagreeing with ours.

The bar is 1e-9 rather than the `mujoco` job's 1e-6 because nothing here is
a simulator with its own integrator and its own float width. A 1 nm error
in one link's pose, a link hung off the wrong parent, a flipped axis and a
joint naming a link that is not there were all checked to fail here.

What this cannot see is the *frame* an `<xyz>` is expressed in, because
every joint in the fixtures is axis-aligned with the model at q = 0 and
`expressed_in` then makes no numeric difference. That convention is pinned
in `sdf.rs`'s golden test instead, which asserts the attribute is absent.

A `<axis><mimic>` — what a mimic joint is written as (ADR-0016 §1) — must
agree with the sampled `qpos`, and a pair of joints the samples show as
exactly coupled must have one, which is how a dropped `<mimic>` would look.
A frame the samples name and the model does not have is a failure, not a
skip: it is how a dropped `<frame>` would look (ADR-0012).

    sudo apt-get install gz-jetty-sdformat-python   # from packages.osrfoundation.org
    python3 python/tests/test_sdf_load.py target/sample-sdf

Plain script, no pytest: the CI job is the lines in the plan's Acceptance
block and nothing else.
"""

import json
import sys
from pathlib import Path

import numpy as np
import sdformat

TOLERANCE = 1e-9

# The version ADR-0016 §1 fixed. Checked in the text, not through the
# parser, because libsdformat silently upconverts what it reads.
VERSION = '<sdf version="1.11">'


def matrix(pose) -> np.ndarray:
    """A `gz.math.Pose3d` as a 4x4 homogeneous matrix."""
    q = pose.rot()
    w, x, y, z = q.w(), q.x(), q.y(), q.z()
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    m[:3, 3] = [pose.x(), pose.y(), pose.z()]
    return m


def quat_of(m: np.ndarray) -> np.ndarray:
    """The `w x y z` of a 4x4's rotation, the order riggen's FK writes."""
    r = m[:3, :3]
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        return np.array([0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s])
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(r[i, i] - r[j, j] - r[k, k] + 1.0) * 2
    out = np.empty(4)
    out[0] = (r[k, j] - r[j, k]) / s
    out[1 + i] = 0.25 * s
    out[1 + j] = (r[j, i] + r[i, j]) / s
    out[1 + k] = (r[k, i] + r[i, k]) / s
    return out


def rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues, as a 4x4."""
    a = axis / np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    m = np.eye(4)
    m[:3, :3] = np.eye(3) * c + s * k + (1 - c) * np.outer(a, a)
    return m


def translation(v: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = v
    return m


def vec(v) -> np.ndarray:
    return np.array([v.x(), v.y(), v.z()])


def edges(model) -> list:
    """`(joint, parent, child)` for every joint between two links.

    A joint whose parent is SDF's reserved `world` frame is the weld that
    fixes the base (ADR-0016 §3); it holds the model in a world, not one
    link to another, so it is not an edge of the tree.
    """
    out = []
    for i in range(model.joint_count()):
        j = model.joint_by_index(i)
        if j.parent_name() == "world":
            continue
        out.append((j, j.parent_name(), j.child_name()))
    return out


def roots(model) -> list[str]:
    links = [model.link_by_index(i).name() for i in range(model.link_count())]
    children = {child for _, _, child in edges(model)}
    return [name for name in links if name not in children]


def fk(model, q: dict[str, float]) -> dict[str, np.ndarray]:
    """Every link's and every frame's world pose at `q`.

    `T_child = T_parent · X_rel · motion(axis, q)`, where `X_rel` and
    `axis` come out of libsdformat resolved into the frames this walk
    needs — the child link's pose *in its parent link's frame* and the
    axis *in the child link's frame*. Nothing here reads a raw `<pose>`.
    """
    root = roots(model)
    if len(root) != 1:
        raise AssertionError(f"expected one root link, found {root}")
    world = {root[0]: matrix(model.link_by_name(root[0]).semantic_pose().resolve("__model__"))}
    pending = edges(model)
    while pending:
        progress = [e for e in pending if e[1] in world]
        if not progress:
            raise AssertionError(f"links {[e[2] for e in pending]} hang off nothing")
        for joint, parent, child in progress:
            rel = matrix(model.link_by_name(child).semantic_pose().resolve(parent))
            motion = np.eye(4)
            axis = joint.axis(0)
            if axis is not None:
                a = vec(axis.resolve_xyz(child))
                value = q.get(joint.name(), 0.0)
                if joint.type() == sdformat.JointType.PRISMATIC:
                    motion = translation(a / np.linalg.norm(a) * value)
                elif joint.type() in (
                    sdformat.JointType.REVOLUTE,
                    sdformat.JointType.CONTINUOUS,
                ):
                    motion = rotation(a, value)
                else:
                    raise AssertionError(
                        f"joint {joint.name()!r} is a {joint.type()} and carries an <axis>"
                    )
            world[child] = world[parent] @ rel @ motion
        pending = [e for e in pending if e not in progress]
    # A frame rides the link it is attached to, at the pose libsdformat
    # resolves in that link's frame (ADR-0012, ADR-0016 §3).
    for i in range(model.frame_count()):
        frame = model.frame_by_index(i)
        attached = frame.attached_to()
        if attached not in world:
            raise AssertionError(
                f"frame {frame.name()!r} is attached to {attached!r}, which is not a link"
            )
        world[frame.name()] = world[attached] @ matrix(
            frame.semantic_pose().resolve(attached)
        )
    return world


def compare(kind: str, name: str, q: list, m: np.ndarray, want: dict) -> None:
    pos, quat = m[:3, 3], quat_of(m)
    wpos, wquat = np.asarray(want["pos"]), np.asarray(want["quat"])
    dpos = np.abs(pos - wpos).max()
    # q and -q are the same rotation.
    dquat = min(np.abs(quat - wquat).max(), np.abs(quat + wquat).max())
    if dpos > TOLERANCE or dquat > TOLERANCE:
        raise AssertionError(
            f"{kind} {name!r} at q={q}: sdformat pos {pos} quat {quat}, "
            f"riggen pos {wpos} quat {wquat} (dpos {dpos:.2e}, dquat {dquat:.2e})"
        )


def check_fk(model, samples: dict) -> int:
    frames = {model.frame_by_index(i).name() for i in range(model.frame_count())}
    missing = sorted(set(samples["samples"][0].get("sites", {})) - frames)
    if missing:
        raise AssertionError(
            f"the samples name frame(s) {missing} the model does not have "
            f"(it has {sorted(frames)}): a <frame> was dropped on the way out"
        )
    checked = 0
    for sample in samples["samples"]:
        q = dict(zip(samples["joints"], sample["q"]))
        world = fk(model, q)
        for kind, key in (("link", "links"), ("frame", "sites")):
            for name, pose in sample.get(key, {}).items():
                if name not in world:
                    raise AssertionError(f"the model has no {kind} {name!r}")
                compare(kind, name, sample["q"], world[name], pose)
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


def check_mimics(model, samples: dict) -> int:
    """Every `<mimic>` agrees with the sampled qpos, and none is missing.

    SDF 1.11's rule is `follower = multiplier * (leader - reference) +
    offset` (ADR-0016 §1); riggen writes `reference` as 0, which makes it
    ADR-0013's `q_y = k*q_x + o` exactly. The samples carry the follower's
    *derived* value, so a swapped multiplier and offset fails here.
    """
    q_of = {
        name: [s["q"][i] for s in samples["samples"]]
        for i, name in enumerate(samples["joints"])
    }
    coupled: set[frozenset[str]] = set()
    found = 0
    for i in range(model.joint_count()):
        joint = model.joint_by_index(i)
        axis = joint.axis(0)
        mimic = axis.mimic() if axis is not None else None
        if mimic is None:
            continue
        found += 1
        follower, leader = joint.name(), mimic.joint()
        coupled.add(frozenset({follower, leader}))
        if follower not in q_of or leader not in q_of:
            raise AssertionError(
                f"<mimic> couples {follower!r} to {leader!r}, which the samples "
                f"do not name (they have {sorted(q_of)})"
            )
        k, o, ref = mimic.multiplier(), mimic.offset(), mimic.reference()
        for n, (f, l) in enumerate(zip(q_of[follower], q_of[leader])):
            want = k * (l - ref) + o
            if abs(f - want) > TOLERANCE:
                raise AssertionError(
                    f"<mimic> {follower!r} = {k} * ({leader!r} - {ref}) + {o}: sample {n} "
                    f"has {follower}={f} and {leader}={l}, which the rule makes {want}"
                )
    # …and nothing was dropped: a joint whose sampled values are an exact
    # linear function of another's is a mimic and must have brought its
    # `<mimic>` with it.
    for follower, ys in q_of.items():
        for leader, xs in q_of.items():
            if leader == follower or frozenset({follower, leader}) in coupled:
                continue
            fit = affine(xs, ys)
            if fit:
                raise AssertionError(
                    f"the samples have {follower!r} = {fit[0]} + {fit[1]} * {leader!r} "
                    "at every configuration, but the model has no <mimic> for it: "
                    "a mimic joint was dropped"
                )
    return found


def main(argv: list[str]) -> int:
    directories = [Path(a) for a in argv] or [Path("target/sample-sdf")]
    failures = 0
    for directory in directories:
        files = sorted(directory.glob("*.sdf"))
        if not files:
            print(f"FAIL {directory}: no .sdf in it")
            failures += 1
            continue
        for path in files:
            try:
                text = path.read_text()
                if VERSION not in text:
                    raise AssertionError(f"does not declare {VERSION} (ADR-0016 §1)")
                root = sdformat.Root()
                root.load(str(path))
                model = root.model()
                if model is None:
                    raise AssertionError("no <model> in the file")
            except Exception as e:  # noqa: BLE001 — any load problem is the verdict
                print(f"FAIL {path}: {type(e).__name__}: {e}")
                failures += 1
                continue
            summary = (
                f"{model.link_count()} links, {model.joint_count()} joints, "
                f"{model.frame_count()} frames"
            )
            fk_json = path.with_suffix(".fk.json")
            if fk_json.exists():
                samples = json.loads(fk_json.read_text())
                try:
                    n = check_fk(model, samples)
                    mimics = check_mimics(model, samples)
                except AssertionError as e:
                    print(f"FAIL {path}: {e}")
                    failures += 1
                    continue
                summary += f", {n} link and frame poses match riggen's FK to {TOLERANCE:g}"
                if mimics:
                    word = "mimic" if mimics == 1 else "mimics"
                    summary += f", {mimics} {word} checked against the samples"
            print(f"ok   {path}: {summary}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
