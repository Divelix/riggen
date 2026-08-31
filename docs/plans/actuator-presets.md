# Plan: actuator-presets

- Started: 2026-08-31
- Milestone: v0.2
- Idea: `docs/ideas/actuator-presets.md` (absorbed; deleted with this plan's
  first commit)
- Idea (verbatim from the human): "About next item in roadmap: mimic joint;
  actuator presets. Elaborate on this idea, or directly write plan if
  everything clear"

## Goal

A movable joint can carry an actuator, so the MJCF we export is drivable:
`Joint::actuator: Option<ActuatorSpec>` holds one of the three presets an RL
user reaches for — `Position { kp, kv }`, `Velocity { kv }`, `Motor { gear }`
— and the MJCF writer turns it into an `<actuator>` element named after its
joint, with `ctrlrange` from the joint's limits and `forcerange` from
`Limits::effort`. `model.nu` stops being zero, `data.ctrl["shoulder"]` works,
and the apologetic `<!-- … need an <actuator>; not written -->` comment
survives only on joints that really have none. The setting is in the
document, so it survives save/reopen, the SDK and every re-export; it is
edited per joint in the properties panel and applied to a whole arm with one
button.

## Non-goals

- **URDF output changes.** `<transmission>` is a `ros_control` relic
  superseded by `ros2_control` xacro tags; inventing one is the fragile-
  exporter behaviour `SEED.md` §3 complains about. URDF keeps writing
  `<limit effort velocity/>` and gains a comment naming what MJCF got,
  mirroring the existing `armature` comment.
- **URDF import.** URDF has no actuator; `urdf_in` is untouched and no new
  `ImportWarning` appears.
- **MJCF import**, and with it `<general>`, `<adhesion>`, `<muscle>`,
  actuators on a tendon or a site, `<default class>` inheritance of gains,
  and `ctrllimited`/`forcelimited` (`autolimits="true"` is already written).
  A user who needs one of those hand-edits the XML; MJCF import is where
  they come back into the document.
- **`Robot::actuators` as a top-level map** (option F of the idea). D
  promotes into F later with an `upgrade_` step that moves each `Some(spec)`
  into a map keyed by its joint.
- Simulation, gain tuning, or any advice about what `kp` should be.

Decided before the first step, by the human: an actuator on a mimic
follower is **refused** in `validate`, and the whole-model button lives in
the properties panel's actuator section.

## Design deltas

- **`docs/02-data-model.md` §Core types** — `Joint` gains
  `#[serde(default)] pub actuator: Option<ActuatorSpec>`, and
  `ActuatorSpec` is a new enum beside `Limits` / `Dynamics`:

  ```rust
  pub enum ActuatorSpec {
      Position { kp: f64, kv: f64 },   // <position kp kv ctrlrange forcerange>
      Velocity { kv: f64 },            // <velocity kv ctrlrange forcerange>
      Motor    { gear: f64 },          // <motor gear ctrlrange forcerange>
  }
  ```

- **§Schema** — `.riggen` is **schema 3**, with an empty `upgrade_v2_to_v3`
  (a v2 file has no key and serde's default is `None`, exactly as the mimic
  bump was). `assets/fixtures/pendulum.riggen` stays frozen at **1** as the
  upgrade corpus; the byte-for-byte fixtures re-save at 3.
- **§Commands and history** — one new command, `SetActuators(Option<
  ActuatorSpec>)`: the whole-model apply, so "apply to every movable joint"
  is one gesture, one command, one undo. The per-joint edit rides `SetJoint`
  as `mimic` does.
- **`validate`** — `ActuatorOnFixedJoint`, `InvalidActuatorGain` (a
  negative `kp`/`kv`, a zero `gear`; a *non-finite* gain is the existing
  `NonFinite { what }`, as for `mimic`), and `ActuatorOnMimicFollower`: a
  follower is already driven by an `<equality>` (ADR-0013), so actuating it
  too fights that constraint. The message names the leader.
- **§`ResolvedRobot`** — `ResolvedJoint::actuator: Option<ActuatorSpec>`,
  copied through; the writers stay dumb serialisers (ADR-0004 §1) and no
  writer reaches back into `Robot`.
- **§Format mapping** — the "Effort / velocity" row is rewritten (the
  comment is now the *no actuator* case) and an "Actuator" row is added.
- **`docs/01-architecture.md` §Python SDK** — `Position` / `Velocity` /
  `Motor` and `Joint.actuator` join the SDK surface.
- **ADR-0014** (step 1): actuators in the document per joint, MJCF-only,
  named after their joint, three presets; it **amends ADR-0004 §4** — the
  dropped-value comment is written only when the joint has no actuator, or
  it becomes a lie beside the `<actuator>` two lines down.

## Steps

- [x] **Step 1 — ADR-0014, `ActuatorSpec`, schema 3.** The enum on `Joint`,
      the `upgrade_v2_to_v3` no-op, the `validate` rules, the idea file
      deleted. Tests: a v2 fixture opens and re-saves as v3, `pendulum.riggen`
      (v1) still walks the whole chain, and each new `ValidationError` has a
      test naming it.
- [x] **Step 2 — the writers.** `ResolvedJoint::actuator`; an `<actuator>`
      block after `</equality>` with one element per actuated joint, named
      after its joint; `ctrlrange` from `lower upper` (`Position`), from
      `±velocity` (`Velocity`), `-1 1` (`Motor`); `forcerange` `±effort`,
      omitted at effort 0 (OPEN 3). The ADR-0004 §4 comment now only on a
      joint with no actuator; URDF gains the mirror comment. Golden-file
      tests in `mjcf.rs` and `urdf.rs` for all three presets and for a
      `Continuous` joint (no `ctrlrange`).
- [x] **Step 3 — the arm carries real actuators and MuJoCo agrees.**
      `arm.riggen`'s three movable joints take one preset each; `fk_samples`
      writes an `actuators` block (name → kind, target joint, ctrlrange,
      forcerange, gain); `test_mjcf_load.py` grows `check_actuators` —
      `mjTRN_JOINT`, the right `trnid`, the ranges and gains to 1e-6, and a
      sampled actuator the model lacks is a failure, which is how a dropped
      `<actuator>` would look. The URDF-imported arm legitimately has none,
      so the check is data-driven, never "nu > 0". **This retires the risk.**
      *Done, with one correction: the arm has **two** actuators, not three
      — `fore_joint` follows `upper_joint` and a follower may carry none
      (ADR-0014). The third preset, a `<motor>`, went on
      `bracket.riggen`'s hinge, which the same CI job exports, so all
      three are checked against MuJoCo.*
- [x] **Step 4 — the panel.** Properties › Joint gains an actuator section
      (combo none / position / velocity / motor, then its gain fields), and
      — in that same section, beside the thing it copies — the "apply to
      every movable joint" button committing `SetActuators`, which skips
      mimic followers rather than building a document `validate` refuses.
      Snapshot tests per preset and for the applied model (ADR-0003).
      *Two goldens rather than three: `properties_joint_actuator` pictures
      the position rows and `properties_joint_actuator_applied` the motor
      row after the whole-model apply; `velocity` is asserted in the same
      scenario without a third picture that would differ in one row.*
- [x] **Step 5 — the SDK, both layers.** *(taken before step 3, see below)* `ActuatorDoc` in `riggen-py`'s
      `JointDoc` / `JointInput` (passed through `JointSpec.to_doc` as `mimic`
      is, since an actuator belongs to the joint and not to its kind);
      `Position` / `Velocity` / `Motor` dataclasses and the `Joint.actuator`
      property in `python/riggen/robot.py`; `_riggen.pyi` and `__init__`
      exports; a `python/tests/sdk/test_api.py` case that sets one, exports,
      and greps the XML.

## Acceptance

```sh
cargo run -p riggen-app -- --export mjcf --fk-samples --out target/sample \
    assets/fixtures/arm/arm.riggen
uv run --no-project --with mujoco --with numpy python \
    python/tests/test_mjcf_load.py target/sample target/sample-urdf
```

The sample arm loads with **zero compiler warnings** and `model.nu == 2`
— one per movable joint that is not a mimic follower; `bracket.riggen`,
exported by the same CI job, adds the third preset with `model.nu == 1` —
every actuator's target, `ctrlrange`, `forcerange` and gain match what the
`.fk.json` says, and `mj_forward` still agrees with `fk` to 1e-6 — over
both the `.riggen` and the URDF route (which has none, and is checked as
having none).

## Docs to update on completion

- `docs/02-data-model.md` §Core types — `ActuatorSpec`, `Joint::actuator`.
- `docs/02-data-model.md` §Commands and history — `SetActuators`.
- `docs/02-data-model.md` §`ResolvedRobot` — `ResolvedJoint::actuator`.
- `docs/02-data-model.md` §Format mapping — the Effort/velocity row
  rewritten, an Actuator row added.
- `docs/02-data-model.md` §Schema — version 3, the upgrade chain, which
  fixtures sit at which version.
- `docs/01-architecture.md` §Python SDK — the three preset classes.
- `docs/adr/0014-*.md` — written at step 1, and the roadmap notes it amends
  ADR-0004 §4.
- `docs/03-roadmap.md` §v0.2 — "Actuator presets" moves from "Still open"
  to a dated status paragraph.
- `AGENTS.md` current state — actuators done, schema 3; keep under ~15 lines.
- `docs/BACKLOG.md` — anything this plan deliberately left out that is worth
  a line (`<general>`, actuators on a site, gains in a `<default class>`).

## Open questions

**Steps 3 and 5 swapped.** `examples/arm.py` builds the arm through the SDK
and `test_arm_example_exports_the_fixture_byte_for_byte` compares its export
with `arm.riggen`'s, so the moment step 3 gives the fixture actuators the
example must set them too — which needs step 5's API. The SDK went first;
step 3 updates the example in its own commit. (Step 5 also carried the
schema-3 fixes the SDK suite needed after step 1: `conftest.upgraded_from_v1`
and one `schema_version` assertion.)

**Step 1, settled while writing it:** the plan called the gain error
`NonFiniteActuator`, but it also had to cover a negative `kp` — a name that
would lie in the message. Split the way `mimic` already splits: non-finite
is the existing `NonFinite { what }`, and `InvalidActuatorGain { joint,
what }` is the degenerate-but-finite case, beside `ZeroMimicMultiplier`. A
negative `gear` is *accepted*: it reverses the joint, which is what a
mirrored pair of motors is.

`forcerange` at `effort == 0` is **omitted**, not written as `0 0`:
zero is the unfilled value (`default_limits` seeds `effort: 0.0`, and URDFs
in the wild ship `effort="0"`), and `0 0` is a clamp to zero force — an
actuator MuJoCo counts in `model.nu` and accepts `ctrl` for, that cannot
move the joint. Omitted, `autolimits="true"` leaves `forcelimited` off and
the force is unbounded, which is MuJoCo's own default. Step 2's golden test
pins the omission; ADR-0014 records why.
