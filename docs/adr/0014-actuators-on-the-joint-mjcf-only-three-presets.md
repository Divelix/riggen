# ADR-0014: An actuator lives on its joint, is MJCF-only, is named after the joint, and comes in three presets

- Status: Accepted
- Date: 2026-08-31

## Context

Every movable joint in the MJCF we export carries an apology:

```xml
<!-- joint upper_joint: effort 5 velocity 3 need an <actuator>; not written -->
```

MuJoCo loads such a model and `mj_forward` is happy, but `model.nu == 0`:
nothing can drive it. An RL researcher's first act after our export is to
open the XML and hand-write an `<actuator>` block — the "manual XML is
tedious" step `SEED.md` §3 exists to remove, and a direct hit on
differentiator #3, "sim-ready is a feature, not a claim". An MJCF you cannot
actuate is not sim-ready.

An actuator is model *description*, not simulation: the same category as
`damping`, `friction` and `armature`, which `Dynamics` already holds and
both writers already emit. `SEED.md` §4's "physics/dynamics" non-goal is
about *running* the model, and is not in the way.

ADR-0004's rejected alternative "model the document in MJCF's terms" refused
actuators "before the GUI needs them". The GUI needs them now; that clause
was worded as a *timing* objection, and this is the timing.

Four things needed deciding rather than discovering:

1. **Where the actuator lives** — the document, or the export dialog.
2. **How much of MJCF's actuator model we take.**
3. **What URDF does**, having no actuator element.
4. **`forcerange` when `Limits::effort` is the unfilled zero.**

## Decision

**The document holds `Joint::actuator: Option<ActuatorSpec>`**, one of three
presets:

```rust
pub enum ActuatorSpec {
    Position { kp: f64, kv: f64 },   // <position kp kv ctrlrange forcerange>
    Velocity { kv: f64 },            // <velocity kv ctrlrange forcerange>
    Motor    { gear: f64 },          // <motor gear ctrlrange forcerange>
}
```

It is **schema 3**, with an `upgrade_v2_to_v3` step that is empty for the
same reason `upgrade_v1_to_v2` was: a v2 file has no `actuator` key and
serde's default is `None`, which is what a v2 document meant.
`assets/fixtures/pendulum.riggen` stays frozen at v1 as the upgrade corpus;
`bracket.riggen` and `arm/arm.riggen` are the byte-for-byte fixtures and
re-save at 3.

Putting it in the document rather than in `ExportOptions` is what makes a
`.riggen` that says "these are position servos at kp = 100" survive
save/reopen, the SDK and every re-export. `ExportOptions` is deliberately
not saved ("two exports of one document may differ in all of it"), so gains
there would be retyped every time, a gripper could not differ from the arm
it is bolted to, and the SDK — the headline v0.2 feature — could not express
an actuator at all.

**Three presets, and only three.** `<general>`, `<adhesion>`, `<muscle>`,
actuators on a tendon or a site, gains inherited from a `<default class>`,
and `ctrllimited`/`forcelimited` (we write `autolimits="true"`) are out.
They are the escape hatch a user edits by hand, and MJCF import is where
they come back into the document.

**The `<actuator>` element takes the joint's own name.** MJCF namespaces are
per element type, so `model.actuator("shoulder")` and `model.joint("shoulder")`
coexist, and `data.ctrl` is indexable by the name the user already knows —
which `<shoulder>_act` would not be.

**`ctrlrange` comes from the joint, `forcerange` from `Limits::effort`.**
`Position` and `Velocity` are commanded in the joint's own units, so their
`ctrlrange` is `lower upper` and `±velocity` respectively; `Motor`'s `ctrl`
is a normalised `-1 1` scaled by `gear`. A joint with no limits (`Continuous`)
gets no `ctrlrange`.

**`forcerange` is omitted when `effort` is zero**, not written as `0 0`.
Zero is the unfilled value — `default_limits` seeds `effort: 0.0`, and URDFs
in the wild ship `effort="0"` — and `0 0` is a clamp to zero force: an
actuator MuJoCo counts in `model.nu` and accepts `ctrl` for, that cannot
move the joint. Omitted, `autolimits="true"` leaves `forcelimited` off and
the force is unbounded, which is MuJoCo's own default.

**This amends ADR-0004 §4.** The dropped-value comment is written only when
the joint has **no** actuator; beside an `<actuator>` two lines down it would
be a lie. URDF keeps writing `<limit effort velocity/>` — the honest URDF
answer — and gains a comment naming what MJCF got instead, mirroring the
existing `armature` one.

**URDF writes no actuator and imports none.** `<transmission>` is a
`ros_control` relic superseded by `ros2_control` xacro tags; inventing one
would be exactly the fragile-exporter behaviour `SEED.md` §3 complains
about. `urdf_in` is untouched: there is nothing in a URDF to read back.

**`validate` refuses** an actuator on a `Fixed` joint (MJCF writes no
`<joint>` for it to drive), an actuator on a **mimic follower** (already
driven by the `<equality>` of ADR-0013 — actuating it too sets the two
against each other), a non-finite gain, a negative `kp`/`kv`, and a zero
`gear`. A negative `gear` is fine: it reverses the joint.

**One new command, `SetActuators(Option<ActuatorSpec>)`**, the whole-model
apply, so "give every movable joint a position servo" is one gesture, one
command, one undo; it skips mimic followers rather than building a document
`validate` would refuse. The per-joint edit needs no command of its own — it
rides `SetJoint`, which preserves only `parent`/`child`, exactly as `mimic`
does.

## Consequences

- `model.nu` stops being zero. The MuJoCo acceptance grows a data-driven
  `check_actuators`: `--fk-samples` writes what each actuator should be, and
  a sampled actuator the model lacks is a failure — which is how a dropped
  `<actuator>` would look.
- The apologetic comment survives only where it is true, on a joint that
  really has none.
- A `.riggen` from before this ADR opens unchanged, with no actuators, and
  re-saves at schema 3.
- The URDF and MJCF exports of one document are no longer equivalent in what
  they can hold — they already were not (`armature`, `<site>`) — and the
  URDF comment is where the difference is stated.
- Promoting to a top-level `Robot::actuators` map later (the shape MJCF
  itself has, and the shape MJCF *import* will want, since an actuator may
  target a tendon or a site) is an `upgrade_` step that moves each
  `Some(spec)` into a map keyed by its joint. That is a plan, not a
  catastrophe.

## Alternatives considered

- **Actuators as an `ExportOptions` field**, one preset for the whole model:
  zero schema change, ~2 steps, and wrong — see the Decision. It contradicts
  differentiator #2, "a git-friendly document as source of truth".
- **`Robot::actuators: BTreeMap<ActuatorId, Actuator>` with a target enum**,
  the shape MJCF has. More schema, an id kind, a list panel and a namespace
  check, for a generality nothing in v0.2 uses. It becomes right when MJCF
  import lands, and the promotion above is how it gets there.
- **A `<general>` escape hatch beside the presets**, with `dyntype`,
  `gaintype`, `biastype` and their `prm` vectors. That is MJCF's actuator
  model in full, in a properties panel, for a user who by definition already
  knows the XML. Hand-editing is the better answer until MJCF import can
  read it back.
- **Naming the actuator `<joint>_act`.** Unambiguous when reading the XML,
  but `data.ctrl["shoulder_act"]` is not what a user who typed the joint
  name expects, and the per-element-type namespace makes the collision
  impossible anyway.
- **Writing `forcerange="0 0"` at `effort == 0`**, so the attribute is
  always present. It turns "nobody filled the effort in" into "this actuator
  is clamped to zero force", which loads, simulates, and does nothing —
  the worst kind of wrong.
- **Allowing an actuator on a mimic follower.** MuJoCo would accept it and
  the solver would arbitrate between the actuator and the equality
  constraint. A user who wants both fingers driven wants two independent
  joints, not a coupling they are fighting.
