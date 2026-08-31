# Plan: mimic-joints

- Started: 2026-08-31
- Milestone: v0.2
- Idea: `docs/ideas/mimic-joints-and-actuators.md` option A (partially
  absorbed — the actuator half stays open)
- Idea (verbatim from the human): "About next item in roadmap: mimic joint;
  actuator presets. Elaborate on this idea, or directly write plan if
  everything clear" / "write the plan for mimic joints"

## Goal

A joint can be declared to follow another: `q_follower = multiplier ×
q_leader + offset`. The document holds it, `fk` computes it, `validate`
refuses the combinations that cannot be exported, the properties panel
edits it and the Joints window shows the follower as a derived read-out
rather than a draggable slider. It exports as URDF's native `<mimic>` and
as an MJCF `<equality><joint polycoef>`, imports back from URDF — closing
the `ImportWarning::MimicDropped` dead end that ships today — and the
`mujoco` CI job proves both routes agree with `fk` at five configurations.
The SDK reads and writes it. A gripper URDF survives a round trip with its
fingers still coupled.

## Non-goals

- **Actuators.** The other half of the idea file; a separate plan.
- **Mimic chains** (a follower whose leader also follows). Rejected in
  `validate` with a message that says so — see OPEN 3.
- **Closed kinematic loops** (`SEED.md` §4 non-goal). A mimic is a coupled
  DoF, not a loop: the tree stays a tree and `fk` stays one depth-first
  pass. Nothing here brings `<equality connect>` or a real loop closure.
- **Tendons.** MJCF `<tendon><fixed>` is the right model for a cable-driven
  coupling and a different feature; `<equality joint>` is what `<mimic>`
  means.
- **A follower's own glyph or tint in the viewport.** The overlay does not
  distinguish a driven joint from a free one; backlog.
- **Non-linear coupling.** `polycoef` has five slots and we write two.

## Design deltas

**ADR-0013** (written in step 1) records: mimic as URDF `<mimic>` and MJCF
`<equality joint polycoef>`; the softness caveat; no chains; removal
clears followers while a kind change is refused.

`riggen-core` — `docs/02-data-model.md` §Core types:

```rust
pub struct Joint {
    …
    pub mimic: Option<Mimic>,   // new
}

/// `q(this) = multiplier * q(joint) + offset`. URDF's `<mimic>`.
pub struct Mimic { pub joint: JointId, pub multiplier: f64, pub offset: f64 }
```

§Kinematics: `fk` and `fk::frames` resolve followers. One helper,
`fk::resolve_q(&Robot, &JointState) -> JointState`, is the single
implementation — `fk`, the Joints window and `fk_samples` all read it, so
the derived number cannot drift between them. A follower's entry in the
caller's `JointState` is ignored, not an error.

`validate` gains `ValidationError::` variants for: the leader naming no
joint (`DanglingMimicJoint`), a joint mimicking itself, a leader that is
`Fixed`, a leader that itself mimics (the chain), a non-finite or zero
`multiplier` / non-finite `offset`, and `MimicExceedsLimits` — the leader's
range mapped through `(k, o)` must fit inside the follower's own limits, or
MuJoCo's `range` and the equality fight each other. A follower with no
limits (`Continuous`) makes that check vacuous.

**No new command.** The properties panel already commits a whole edited
`Joint` through `SetJoint` (`panels/properties.rs:1262`), and `SetJoint`
preserves only `parent`/`child`, so `mimic` rides along.

`command::apply` validates and refuses on error, so two removal paths need
care: `RemoveLink` already does `robot.frames.retain(…)` for the doomed
subtree and now does the same for followers whose leader is going away —
clearing the mimic, not refusing the delete. A `SetJoint` that turns a
leader `Fixed` *is* refused, by `validate`, with the follower named.

`riggen-export` — `ResolvedJoint::mimic: Option<ResolvedMimic { joint:
usize, multiplier, offset }>` (an index into `ResolvedRobot::joints`, so
the writers stay dumb serialisers, ADR-0004 §1).
`docs/02-data-model.md` §Format mapping gains a row:

| Concept | URDF | MJCF |
|---|---|---|
| Mimic | `<mimic joint multiplier offset/>` inside the follower's `<joint>` | `<equality><joint joint1="follower" joint2="leader" polycoef="offset multiplier 0 0 0"/></equality>` after `<worldbody>` |

MJCF's `polycoef` is `y − y0 = a0 + a1(x − x0) + …` with `x`, `y` the two
joints' deviations from `qpos0`; we never write `ref`, so both references
are zero and `(offset, multiplier, 0, 0, 0)` is exactly URDF's rule.
**A MuJoCo equality is a soft solver constraint, not a reduction** — under
dynamics the follower tracks to within `solref`/`solimp`, the way joint
limits are soft. This costs the CI proof nothing: `fk_samples` writes
`qpos` for every movable joint, follower included, so `mj_forward` still
matches `fk` to 1e-6.

`fk_samples::samples` keeps every movable joint in `joints`/`q` — the
follower's entry is its **derived** value, so the Python side sets both and
nothing in `test_mjcf_load.py` changes.

`urdf_in` — `docs/02-data-model.md` §URDF import: `<mimic>` is kept.
`ImportWarning::MimicDropped` survives with a `reason` for what we still
refuse (a chain, a `fixed` leader, a leader that is not in the file).

SDK — `Mimic` dataclass in `python/riggen/robot.py`, `mimic` on
`JointInput` / `JointDoc` in `_riggen`, `Joint.mimic` property and setter,
and the import-warning docstring at `robot.py:1101` loses "mimic".

## Steps

- [x] **Step 1 — core: the document holds a mimic and `fk` honours it.**
  `Mimic`, `Joint::mimic`, `fk::resolve_q`, `fk`/`fk::frames` resolving it,
  the `validate` variants above, `RemoveLink` clearing followers, the
  schema move (OPEN 1) with its `upgrade_v1_to_v2` and corpus handling.
  ADR-0013 in the same commit. Tests: a two-joint chain whose follower's
  pose is `k·q + o` at three configurations; one rejection test per new
  `ValidationError`; `RemoveLink` of a leader's subtree leaves a valid
  document; `pendulum.riggen` still opens.
- [x] **Step 2 — export: both writers.** `ResolvedMimic` in `resolve`,
  `<mimic>` in `urdf.rs`, the `<equality>` block in `mjcf.rs`, followers'
  derived `q` in `fk_samples`. A mimic goes into
  `test_util::every_joint_kind()` so both goldens carry one.
- [x] **Step 3 — import: URDF `<mimic>` becomes a `Mimic`.** `urdf_in`
  reads it; `MimicDropped` narrows to a `reason` and keeps its inline test
  coverage for the chain / fixed-leader / unknown-leader cases.
- [x] **Step 4 — the corpus, and MuJoCo proves it.** `assets/fixtures/arm`
  gains a non-degenerate mimic on `fore_joint` — `multiplier="-0.5"
  offset="0.1"`, replacing the `multiplier="1" offset="0"` that was there
  only to warn on — in **both** `arm.urdf` and `arm.riggen`, so the pair
  stays equivalent and the `mujoco` job's two existing models both exercise
  it. `test_mjcf_load.py` additionally asserts the model has an
  `mjEQ_JOINT` equality for every mimic the samples imply. **This is where
  the risk retires** (see Acceptance); nothing before it has asked MuJoCo
  whether `polycoef` means what we think.
- [x] **Step 5 — app: edit it and see it.** Properties › Joint gains a
  Mimic section — a leader combo (movable joints, minus this one and minus
  anything that would make a chain), multiplier and offset fields,
  committed through the existing `SetJoint`. The Joints window draws a
  follower as a disabled slider at its derived value, labelled `= -0.5 ×
  upper_joint + 0.1`. Snapshots: `properties_joint_mimic`,
  `joints_window_mimic` (ADR-0003 — show the human the images).
- [x] **Step 6 — SDK, both layers.** `Mimic` in `python/riggen/robot.py`,
  `mimic` through `_riggen`'s `JointInput`/`JointDoc`, `Joint.mimic` with
  its setter, `_riggen.pyi`, and a test that builds a coupled pair in
  Python, exports MJCF and finds the `<equality>`.

## Acceptance

The existing `mujoco` CI job, unchanged in shape:

```sh
uv run --no-project --with mujoco --with numpy python python/tests/test_mjcf_load.py \
  target/sample target/sample-urdf target/sample-decomp
```

closes the plan when, for the arm exported both from `arm.riggen` and from
the imported `arm.urdf`: the model loads with zero compiler warnings, it
carries an `mjEQ_JOINT` equality coupling `fore_joint` to `upper_joint`,
and every body and site pose from `mj_forward` matches `fk_samples` to 1e-6
at all five configurations with `fore_joint` at `-0.5 × q(upper) + 0.1`.
Plus: importing `arm.urdf` produces **no** `MimicDropped` warning, and
`cargo test` is green including the two new snapshots.

## Docs to update on completion

- `docs/02-data-model.md` §Core types — `Mimic`, `Joint::mimic`.
- `docs/02-data-model.md` §Kinematics — `fk` resolves followers;
  `fk::resolve_q` is the one implementation.
- `docs/02-data-model.md` §Format mapping — the Mimic row above.
- `docs/02-data-model.md` §URDF import — `<mimic>` kept; what
  `MimicDropped` still means.
- `docs/02-data-model.md` §Schema — v2 and its `upgrade_` step, if OPEN 1
  goes that way.
- `docs/03-roadmap.md` §v0.2 — a dated bullet, ADR-0013.
- `AGENTS.md` "Current state" — replace the frames paragraph's position as
  the newest line; keep the file under ~15 lines.
- `docs/adr/0013-*.md` — written in step 1, not at retirement.
- `docs/ideas/mimic-joints-and-actuators.md` — the mimic half is spent;
  either the actuator plan absorbs the rest or the file stays with the
  actuator half only.
- `docs/BACKLOG.md` — add what this plan pushed out: mimic chains if ever
  wanted, `<tendon><fixed>` for cable couplings, a follower's tint in the
  joint glyph overlay.

## Open questions

- ✅ **OPEN 1 — schema 2, or stay at 1?** *Human, 2026-08-31: bump.*
  Done in step 1: `SCHEMA_VERSION = 2`, `OLDEST_SCHEMA_VERSION = 1`, `load`
  walks an `upgrade_vN_to_vN+1` chain, `upgrade_v1_to_v2` is empty.
  `pendulum.riggen` is frozen at v1 as the upgrade corpus and lost its
  byte-for-byte assertion; `bracket.riggen` and `arm/arm.riggen` were
  regenerated at v2 and keep theirs. The original question follows.

  ⚠ **OPEN 1 — schema 2, or stay at 1?** *Human, by step 1.*
  `docs/02-data-model.md` §Schema says a new field is a later version with
  an `upgrade_` step, and `load` currently rejects any `schema_version !=
  1` outright — there is no upgrade machinery yet, so this is the first
  bump and it builds it (cheaply: v1 JSON is valid v2 JSON with `mimic`
  defaulted, so the upgrade is a no-op and the machinery is one `match`).
  The alternative is `#[serde(default, skip_serializing_if =
  "Option::is_none")]` and no bump: documents without a mimic keep opening
  in v0.2.0, and one *with* a mimic fails on an old build with serde
  naming the unknown field — loud, but not the clean `UnsupportedVersion`.
  **Recommend the bump**: half-compatibility is harder to reason about than
  a clean refusal, and this is the cheapest bump we will ever get. It costs
  one fixture decision — `pendulum.riggen` stays v1 forever as the upgrade
  corpus, and `corpus_pendulum_opens` splits into "it opens" plus a v2
  round-trip fixture.
- ✅ **OPEN 2 — the arm carries the mimic. *Human, 2026-08-31.* Step 4 goes
  ahead as written. The original question follows.

  ⚠ **OPEN 2 — the arm carries the mimic, or a new gripper fixture?**
  *Human, by step 4.* Step 4 assumes the arm: `arm.urdf` already has a
  `<mimic>` (put there in M3 to warn on), so making it non-degenerate costs
  no new meshes and puts the coupling in both models the `mujoco` job
  already loads. The cost is a physically odd arm whose forearm follows its
  upper arm. A `assets/fixtures/gripper/` pair built from the cube fixtures
  would read better and is maybe one extra step. **Recommend the arm** —
  the acceptance test is then the milestone's own, which is what a plan
  wants.
- ⚠ **OPEN 3 — chains stay rejected?** *Agent decided; human may flip by
  step 1.* The idea's decision #2 was never answered explicitly. Rejecting
  in `validate` is assumed here: consumer support for chains is a lottery,
  MuJoCo needs them flattened against the free joint anyway, and step 1 can
  grow into resolving them later without a schema change.

- **Finding (step 1): a mimic on a `Fixed` joint needed rejecting too.** The
  plan's `validate` list covered a *leader* that is `Fixed` but not a
  *follower* that is: such a joint has no `q` to drive, and MJCF writes no
  `<joint>` element for it, so the `<equality>` step 2 emits would name
  something that does not exist. Added as `MimicOnFixedJoint`. It also means
  step 5's leader combo is not the only guard — the panel offers Mimic on
  movable joints only, but `validate` is what makes that an invariant.
