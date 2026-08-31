# Idea: mimic-joints-and-actuators

- Status: mimic half Accepted → plans/mimic-joints; actuator half Open
- Raised: 2026-08-31
- Prompt (verbatim from the human): "About next item in roadmap: mimic joint;
  actuator presets. Elaborate on this idea, or directly write plan if
  everything clear"

## Problem

Two lines of `docs/03-roadmap.md` §v0.2 that the backlog carried as one
sentence. They are not one feature, and only one of them is hard.

**Mimic joints.** A gripper is the common case: two fingers driven by one
DoF, `q_right = -q_left`. URDF says this with `<mimic joint multiplier
offset/>`, and every gripper URDF a researcher imports has it. We drop it
today, loudly — `ImportWarning::MimicDropped`, "no mimic joints yet"
(`crates/riggen-export/src/urdf_in.rs:247`) — so importing a Robotiq or a
Panda hand and re-exporting gives a gripper whose fingers move
independently. The user has no way to put the coupling back, in the GUI or
the SDK, and nothing in the document can hold it.

**Actuators.** Every movable joint in our MJCF carries a comment saying
what we could not write: `<!-- joint upper_joint: effort 1 velocity 1 need
an <actuator>; not written -->` (ADR-0004 §4, `mjcf.rs`). MuJoCo loads such
a model and `mj_forward` is happy, but `model.nu == 0`: nothing can drive
it. An RL researcher's first act after our export is to open the XML and
hand-write an `<actuator>` block — the "manual XML is tedious" step
`SEED.md` §3 exists to remove, and a direct hit on differentiator #3,
"sim-ready is a feature, not a claim". An MJCF you cannot actuate is not
sim-ready.

Frequency: mimic bites anyone importing a gripper (a minority, but a
loud one). Actuators bite *everyone* who exports MJCF to train in.

## Constraints it runs into

- **`SEED.md` §4 non-goal: "closed kinematic loops".** A mimic joint is not
  a loop — the tree stays a tree and `fk` stays one depth-first pass; it is
  a *coupled DoF*, one joint's `q` derived from another's. Users do reach
  for it to approximate a four-bar linkage, and that approximation is
  exactly what keeps us out of loop-closure territory. No conflict, but it
  must be said in the ADR or it will be re-litigated.
- **`SEED.md` §4 non-goal: "physics/dynamics".** An actuator is model
  description, not simulation — the same category as `damping`, `friction`
  and `armature`, which `Dynamics` already holds and both writers already
  emit. No conflict.
- **ADR-0004 §4** ("anything MJCF cannot express without an actuator model
  is written as an XML comment naming the dropped value") is *amended* by
  the actuator half: the comment must appear only when a joint has no
  actuator, or it becomes a lie beside the `<actuator>` two lines down.
- **ADR-0004 §1**: the writers are dumb serialisers over `ResolvedRobot`.
  Both halves must therefore resolve — a `ResolvedJoint::mimic` and a
  `ResolvedActuator` — not reach back into `Robot` from `mjcf.rs`.
- **ADR-0004's rejected alternative** "model the document in MJCF's terms"
  refused actuators "before the GUI needs them". The GUI needs them now;
  this idea is the reason that clause was worded as a *timing* objection.
- **02-data-model §Schema**: a new `Joint` field is `schema_version: 2`,
  an `upgrade_` step and a corpus fixture, since every struct is
  `deny_unknown_fields`.
- **02-data-model §Kinematics / ADR-0004**: `fk`'s
  `BTreeMap<LinkId, Pose>` is the export oracle. Mimic changes what `q`
  means, not what `fk` returns.
- **ADR-0003**: every visible UI change gets a snapshot test.
- **Layer rule**: `riggen-core` owns the mimic (it is kinematics);
  `riggen-export` owns nothing but the mapping.

## Options — mimic joints

### A — `Joint::mimic: Option<Mimic { joint: JointId, multiplier, offset }>`, resolved in `fk`

`fk` reads the leader's `q` from `JointState` and computes the follower's;
`JointState` keeps a value for the follower which `fk` ignores. `validate`
enforces: the leader exists, is movable, is not the joint itself, does not
itself mimic (no chains — see below), `multiplier` is finite and non-zero,
and — the sim-ready check — that the leader's limits mapped through
`(k, o)` fit inside the follower's own limits, so MuJoCo's range and the
coupling do not fight.

Export: URDF gets `<mimic joint="…" multiplier="…" offset="…"/>` inside the
`<joint>`, which is the native spelling. MJCF gets an `<equality>` block
after `<worldbody>`:

```xml
<equality>
  <joint joint1="finger_right" joint2="finger_left" polycoef="0.1 -1 0 0 0"/>
</equality>
```

`polycoef` is `a0 a1 a2 a3 a4` with `y − y0 = a0 + a1(x − x0) + …`, `x` and
`y` the two joints' deviations from their `qpos0` — so `(offset,
multiplier, 0, 0, 0)` is URDF's `q_y = k·q_x + o` when both references are
zero, which ours are. Import: `urdf_in` reads `<mimic>` and the warning
survives only for the cases we reject.

The honest caveat, which belongs in the ADR and in the export dialog: **a
MuJoCo equality is a solver constraint, not a reduction.** In dynamic
simulation the fingers track each other to within `solref`/`solimp`, the
same way joint limits are soft (already a line in the M3 exit gate). It
does **not** weaken our CI proof: the `--fk-samples` harness writes `qpos`
for every joint, follower included, so `mj_forward` and `fk` still agree to
1e-6 exactly as they do today.

Cost: ~6 plan steps (core + schema; command; panel + joints window;
writers; import; SDK + CI gripper fixture). Forecloses nothing.

### B — mimic chains allowed (a mimic whose leader also mimics)

URDF does not forbid it; consumer support is a lottery (KDL and MoveIt
resolve one level reliably, several do not), and MuJoCo needs the chain
flattened into one polycoef against the free joint anyway. Costs a
topological resolve in `fk` and a cycle check in `validate` — half a step —
and buys a case nobody has asked for. Rejecting it in `validate` with a
message that says why is the better trade, and A can grow into B later
without a schema change.

### C — MJCF via `<tendon><fixed>` instead of `<equality joint>`

The right model when the coupling *is* a cable (a real tendon-driven hand),
and it is what a few Menagerie models use. It is a second element with
coefficients per joint, it changes the actuator story (you actuate the
tendon, not the joint), and it is not what `<mimic>` means. A tendon is a
separate future feature that happens to overlap; the backlog already has
"actuators on a site" in that family.

## Options — actuator presets

### D — `Joint::actuator: Option<ActuatorSpec>` in the document, per joint

An enum of the three presets an RL user actually reaches for:

```rust
pub enum ActuatorSpec {
    Position { kp: f64, kv: f64 },   // <position kp kv ctrlrange forcerange>
    Velocity { kv: f64 },            // <velocity kv>
    Motor    { gear: f64 },          // <motor gear ctrlrange forcerange>
}
```

`ctrlrange` comes from the joint's limits (`Position`/`Velocity`) or is the
normalised `-1 1` (`Motor`); `forcerange` comes from `Limits::effort` — the
number the comment currently apologises for. The exported actuator takes
the **joint's own name** (MJCF namespaces are per element type, so
`model.actuator("shoulder")` and `model.joint("shoulder")` coexist and
`data.ctrl` is indexable by the name the user already knows).

The GUI: a section in Properties › Joint, plus one "apply to every movable
joint" button, because the uniform case is the common one and clicking
seven joints is the tedium we are here to remove. URDF writes nothing —
`<transmission>` is a `ros_control` relic superseded by `ros2_control`
xacro tags, and inventing one would be the "fragile exporter" behaviour
`SEED.md` §3 complains about; the effort/velocity it already writes in
`<limit>` is the honest URDF answer, and a comment names what MJCF got that
URDF cannot hold, mirroring the existing `armature` comment.

Cost: ~5 plan steps. Keeps the document the source of truth: a `.riggen`
that says "these are position servos at kp=100" survives save/reopen, the
SDK, and every re-export.

### E — actuators as an `ExportOptions` field, one preset for the whole model

Zero schema change, zero SDK change, ~2 steps: a dropdown and two gain
fields in the export dialog, applied to every movable joint. But
`ExportOptions` is deliberately *not* saved ("two exports of one document
may differ in all of it", `resolve.rs`), so the gains are retyped every
time; a gripper cannot differ from the arm it is bolted to; and the SDK —
the headline v0.2 feature — cannot express an actuator at all. It contradicts
differentiator #2, "a git-friendly document as source of truth". Cheap and
wrong.

### F — `Robot::actuators: BTreeMap<ActuatorId, Actuator>` with a target enum

The shape MJCF actually has, and the shape MJCF *import* will want:
actuators are named top-level elements that may target a joint, a tendon or
a site — the backlog already lists "actuators on a site". More schema, an
id kind, a list panel, and a namespace check, for a generality nothing in
v0.2 uses. D promotes into F later with an `upgrade_` step that moves each
`Some(spec)` into the map keyed by its joint; that is a plan, not a
catastrophe.

### Do nothing

Mimic: imported grippers stay broken and the warning stays a dead end.
Actuators: every exported MJCF needs a hand-written block before it can be
trained in, which is the friction the project exists to remove, and the
comment we emit is a permanent apology.

## Recommendation

**A + D, as two plans, mimic first.**

Mimic first because it is kinematics: it touches `fk`, `validate`, the
schema, the joints window and the SDK, and it closes an import warning that
is already shipping. Actuators are export-only and additive once the schema
bump of the first plan has landed — the second plan pays no migration cost
if it follows the first.

Not one plan: eleven steps in one file is a plan that cannot be retired in
one piece, and `docs/README.md` wants a plan whose acceptance test is one
sentence. Two acceptance tests, one each: *a gripper URDF imports, keeps
its coupling, exports MJCF whose `mj_forward` site and body poses match
`fk` at five configurations*; and *the sample arm exports with `model.nu ==
number of movable joints`, `ctrlrange` from the limits and zero compiler
warnings*.

What would change my mind: if the human wants an MJCF **import** in this
same cycle, F stops being premature and D becomes a schema bump we pay
twice — say so now and the actuator plan starts from the map.

## Decision for the human

1. **Two plans, mimic first?** (preferred) Or one combined plan, or
   actuators first because they hit every user and mimic hits gripper users?
2. **Mimic chains: reject in `validate`?** (preferred) Or resolve them
   topologically for parity with URDF's silence on the matter?
3. **Actuators in the document per joint (D)?** (preferred) Or the
   export-dialog-only preset (E), or the top-level map (F) if MJCF import
   is close?
4. **The three presets `Position` / `Velocity` / `Motor` — enough?**
   (preferred: yes; `<general>` is the escape hatch a user edits by hand,
   and MJCF import is where it comes back)
5. **Actuator named after its joint?** (preferred) Or `<joint>_act`, which
   is unambiguous in a text editor but not what `data.ctrl` users type.

**An ADR is needed** — one, covering both: mimic as URDF `<mimic>` and MJCF
`<equality joint polycoef>` with the softness caveat and the no-chains
rule; actuators in the document, MJCF-only, URDF unchanged; and the
amendment to ADR-0004 §4's comment.
