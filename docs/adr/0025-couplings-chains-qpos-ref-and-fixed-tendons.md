# ADR-0025: Mimic chains resolve, `<joint ref>` is a coordinate offset, and a fixed tendon is a document type

- Status: Accepted
- Date: 2026-09-08
- Amends: [ADR-0013](0013-mimic-joints-as-urdf-mimic-and-mjcf-equality.md),
  [ADR-0023](0023-actuators-are-a-model-level-table-named-in-their-own-namespace.md)

## Context

Three things a Menagerie-style file says about how its joints move
together are warned and dropped today, and each is a decision ADR-0013 or
ADR-0023 deferred by name.

**A mimic chain** — a follower whose leader also follows. ADR-0013
rejected chains in `validate` ("consumer support is a lottery … MuJoCo
wants them flattened against the free joint anyway") and priced the
reversal at "a message": `resolve_q` grows from one pass to a
topological one, no schema change. The premise was wrong on the MuJoCo
side. Probed on 3.12: a file with `l = 0.25·p` and `f = 0.1 − l` as two
`<equality><joint>`s loads with zero warnings, `neq == 2`, and both
constraint residuals are zero at the resolved configuration. MuJoCo does
not flatten and does not ask anyone to; the corpus carries chains, and a
gripper URDF with a coupled finger-tip does too.

**`<joint ref>`.** MJCF lets a joint say what value of `qpos` the
authored body pose corresponds to. The document's `q` has always been the
deviation from the authored pose — Edit's zero configuration (ADR-0021),
the joint frame, every glyph and both writers take `q = 0` there — so a
`ref` had no field to land in, was warned (`<joint ref> × 1: nothing in
the document holds it; not read`), and any `polycoef` over such a joint
was dropped with it. Probed, so the convention below is stated rather
than assumed:

- `qpos0 == ref`, angle-converted for a hinge under `angle="degree"`
  (`ref="10"` → `0.1745`), verbatim for a slide.
- `range` is in `qpos` terms: `range="-30 60"` beside `ref="10"` is
  `jnt_range = [−0.5236, 1.0472]`, not shifted by the compiler.
- At `qpos == ref` the body is at its authored pose (the child site sits
  at `pos`, rotation `0.0`); at `qpos == ref + 0.5` the rotation is `0.5`;
  at `qpos == 0` it is `−ref`.
- `polycoef` deviations are from `qpos0`: over `l` (`ref="0.2"`) and `f`
  (`ref="-0.3"`), the residual of `polycoef="0.1 0.5 0 0 0"` is `0.0` at
  `(f − f0) = 0.1 + 0.5·(l − l0)` and `0.4` at the raw-`qpos` reading.
- A fixed tendon's length is **`Σ coef · qpos`, absolute** — over the
  same `l`, a tendon `1·l − 2·f` at `qpos = (0.6, 0.3)` has
  `ten_length = 0.0`, which is the raw sum, not the deviation sum
  (`−0.2`). Tendons do not see `qpos0`; equalities do.
- A `<position tendon="…">` with no `ctrlrange` under
  `autolimits="true"` loads with zero warnings and `ctrllimited = false`,
  exactly as the joint-targeted case ADR-0024 recorded.

**`<tendon><fixed>`.** ADR-0013 named it as "the right model for a
cable-driven coupling, and a different feature", and ADR-0023 left
`ActuatorTarget` with one variant "so the table's shape does not change
again when they arrive", keeping a tendon-targeted actuator
`ActuatorDropped`. The corpus's `grip` motor is that actuator, the last
name in `ROUND_TRIP_DROPPED`.

## Decision

**1. Chains are accepted; only a cycle is refused.** `fk::resolve_q`
stays the one implementation of the rule (ADR-0013) and becomes a
topological pass with a memo: a follower's value is its leader's
*resolved* value through `(multiplier, offset)`. It is cycle-safe on an
unvalidated document — a member of a cycle keeps its raw `q`, the way
`fk` terminates on a link loop — and `validate` replaces `MimicChain`
with `MimicCycle { joints }`. `MimicExceedsLimits` composes the chain: the
*free* leader's range is mapped through the composed affine map to each
follower's own limits, so the sim-ready promise of ADR-0013 holds for a
chain of any length. `SetJoint` no longer refuses a leader that itself
follows; demoting any leader to `Fixed` is still refused naming its
followers.

**2. Every writer writes a chain as it is, and every reader reads it
back.** The round trip is the identity, in URDF, SDF and MJCF alike.
MJCF was the writer ADR-0013 feared would need flattening and, probed, it
does not; the URDF and SDF specs do not forbid a chain, and consumers
that refuse one vary by consumer. Flattening in one writer would make
that writer's round trip a transformation and the other two's the
identity; a user with a consumer that refuses a chain flattens for it,
one loop in the SDK. *(Recorded from the plan's recommendation; the
human's to confirm by plans/couplings step 3.)*

**3. `<joint ref>` is `Joint::qpos_ref`, a coordinate offset, and the
document's `q` stays the deviation from the authored pose.** The
convention, in one sentence: *`q` is the deviation from the authored
pose; `qpos_ref` is what MJCF adds to it.* MJCF's `qpos = q + qpos_ref`.
Nothing in `fk`, the viewport, the glyphs, Edit's zero configuration, the
URDF writer, the SDF writer or either of their readers learns the field
exists. The three places that do:

- **MJCF import** reads `ref` (angle-converted like every other angle),
  stores it, and shifts `range` by it on the way in, so `Limits::range`
  stays in deviation terms and every check `validate` makes — the limits
  check, the mimic reach — keeps its meaning. `moved_zero` and the
  `polycoef` drop reason "a `<joint ref>` moved one of the two zeros" are
  deleted: MuJoCo's deviations are from `qpos0 = ref`, so a `polycoef`
  over a `ref` joint is *exactly* the document's mimic, untouched.
- **The MJCF writer** writes `ref` when non-zero, shifts `range` back, and
  shifts a `ctrlrange` it *derives* from the joint's range. An
  `ActuatorRanges::ctrl` the file said (ADR-0024) is written verbatim: it
  was in `qpos` terms when read and is held so, because it exists for
  MJCF alone. The comment "we never write `ref`" is rewritten.
- **`--fk-samples`** writes `qpos = q + qpos_ref`, and the Python side
  reads equality deviations through `model.qpos0`.

Schema 6, `#[serde(default)]` `0.0`; `upgrade_v5_to_v6` is empty.

**4. `<tendon><fixed>` is `Robot::tendons`, and `ActuatorTarget::Tendon`
is the second variant ADR-0023 left room for.**

```rust
pub struct Tendon {
    pub name: String,                 // unique among tendons
    pub joints: Vec<TendonJoint>,     // ≥ 1, each joint once, none Fixed
    pub range: Option<[f64; 2]>,
    pub limited: Option<bool>,        // MuJoCo's auto | true | false, as ADR-0024
    pub stiffness: f64,
    pub damping: f64,
    pub frictionloss: f64,
}
pub struct TendonJoint { pub joint: JointId, pub coef: f64 }  // coef ≠ 0
pub enum ActuatorTarget { Joint(JointId), Tendon(TendonId) }
```

`id_type!(TendonId, 't', "tendon")`, allocated from the document's one
`IdGen` (ADR-0005); a `BTreeMap<TendonId, Tendon>` with the same shape as
`Robot::frames` and `Robot::actuators`, edited by the same quartet
(`AddTendon` / `RemoveTendon` / `SetTendon` / `RenameTendon`,
`Created::Tendon`). A tendon is a coupling *the solver enforces through
forces*, not a reduction of the configuration: nothing in `fk` reads it,
and the joint tree shows every joint on a tendon as free. Its two
consumers are the MJCF writer and the SDK.

**A tendon's `range` is in MJCF's own terms — `Σ coef · qpos`, absolute.**
The probe above is why: MuJoCo evaluates a fixed tendon over `qpos`, not
over deviations from `qpos0`, so a tendon over a `ref` joint is held as
the file said it and written back verbatim, with no shift on either side.
The document never evaluates a tendon's length; the one place a length
is computed is `--fk-samples`' `tendons` block, and it computes
`Σ coef · (q + qpos_ref)` for `check_tendons` to hold against
`data.ten_length`. `validate` only asks that `lower < upper`, and needs
no coordinates to ask it.

**Carried and counted**, on the pattern ADR-0024 §4 set. Carried —
document state, round-trips: `name`, the `<joint joint coef>` children,
`range`, `limited`, `stiffness`, `damping`, `frictionloss`. Counted and
warned once per attribute name, silent on nothing: `springlength`,
`solreflimit` / `solimplimit`, `solreffriction` / `solimpfriction`,
`margin`, `armature`, `user`. Decorative and silent, as for geoms:
`group`, `rgba`, `width`. `<tendon><spatial>` (site-routed, wrapping geoms,
pulleys) and `<equality><tendon>` are `TendonDropped` / `ElementDropped`
by name.

**Refusals.** `DuplicateTendonName`, `EmptyTendon`, `DanglingTendonJoint`,
`DuplicateTendonJoint`, `TendonOnFixedJoint`, `ZeroTendonCoef`,
`InvalidTendonRange`, and a non-finite number anywhere in it;
`DanglingActuatorTarget` covers a tendon target as it covers a joint's.
A tendon-targeted actuator of any of the four `ActuatorSpec` kinds is
read and written; `check_actuators`' joint-specific refusals (an actuator
on a `Fixed` joint, on a mimic follower) do not apply to it, because a
tendon's joints are free by construction.

**Two side effects, both the `RemoveLink` precedent (ADR-0006, ADR-0013),
neither a refusal.** `RemoveLink` drops a removed joint's entry from every
tendon, and a tendon left with no joints goes with the subtree, taking
the actuators that targeted it. `RemoveTendon` takes the actuators on it
too — a removal is a gesture about the tendon, and its actuators are
visible in the properties panel of every joint on the tendon and undone
in one keystroke (the plan's OPEN 3, decided here). `SetJoint` to `Fixed`
on a joint some tendon runs over is refused naming the tendon, as
demoting a mimic leader is refused naming the follower: a kind change is
an edit of the coupling's own joint.

**Not decided here**, left with the plan for the human: `<muscle>` and
`<adhesion>` (ADR-0024's addendum sent them to this bullet; the plan's
OPEN 1 recommends they stay out), and whether tendons get a list panel
now that an actuator can exist no joint's panel owns — ADR-0023's own
condition for one — or the read-only rows the plan proposes (OPEN 2).

## Consequences

- ADR-0013's "chains are rejected" is reversed and its "resolving chains"
  alternative is now the decision; the rest of ADR-0013 — the field on
  the follower, `resolve_q` as the one rule, URDF `<mimic>`, MJCF
  `polycoef`, the removed-leader and demoted-leader rules — stands.
  `ImportWarning::MimicDropped` loses two of its reasons (a chain, a
  `ref`) and keeps the others (`fixed` leader, absent leader,
  non-linear, inactive, one-joint).
- ADR-0023's "`ActuatorTarget` gets exactly one variant" is amended to
  two; its site/body boundary is unchanged, and `ActuatorDropped` still
  names a site or body target.
- `validate` gains `qpos_ref` finite; `MimicChain` becomes `MimicCycle`;
  the mimic reach check maps through a composed map, so a document a
  user could build today by hand — a chain whose composed reach leaves a
  follower's range — is refused for the reason ADR-0013 gave.
- `ResolvedJoint::qpos_ref`, `ResolvedTendon`, `ResolvedRobot::tendons`
  and `ResolvedActuator::target: ResolvedTarget { Joint(usize) |
  Tendon(usize) }` join the convention-neutral robot (ADR-0004); the URDF
  and SDF writers ignore all four.
- The `mujoco` job compares the re-export's `<equality>` and `<tendon>`
  blocks with the original's, field for field, the way it compares
  `<actuator>` (ADR-0024); `ROUND_TRIP_DROPPED` empties.
- A `.riggen` from before this ADR opens unchanged and re-saves at schema
  6 with no tendons and every `qpos_ref` zero.
- The escape hatch's `<muscle>` question has the tendon it was waiting
  for, and stays a backlog line unless OPEN 1 decides otherwise.

## Alternatives considered

- **Flattening a chain against the free leader on export**, ADR-0013's
  reason for refusing chains. Rejected once MuJoCo was probed and found
  to accept a chain; flattening would make one writer's round trip a
  transformation, and the document's own rule would differ from the
  file's.
- **Folding `ref` into `q` at import** (`q = qpos`, the authored pose at
  `q = ref`). Every consumer of `q` — Edit's zero configuration, the
  glyph's range arc, the URDF and SDF writers, whose `<limit>` is in
  deviation terms — would learn the field, and a document riggen alone
  has touched would gain a number it never needed. Keeping `q` as the
  deviation confines `ref` to the MJCF reader, the MJCF writer and the
  samples.
- **Folding `ref` into `origin`** (re-authoring the child pose so `qpos0`
  is zero). Lossy in the wrong direction: the file's `ref` is gone, the
  mesh's authored placement is changed, and the `polycoef`, `range` and
  `ctrlrange` of everything around the joint need re-deriving.
- **A tendon's `range` shifted into deviation terms**, symmetric with
  the joint range. Rejected by the probe: MuJoCo evaluates the tendon
  over absolute `qpos`, so a deviation-form range would have to be
  un-shifted on every write, and the document would be holding a number
  the file never said.
- **`Tendon` as a fifth `ActuatorSpec`**, a tendon existing only as an
  actuator's transmission. A tendon with no actuator is a real thing (a
  passive spring between two fingers), and a file may drive one tendon
  with several actuators; a document type keyed by id is what
  ADR-0023 shaped the table for.
- **Refusing `RemoveTendon` while actuators target it.** The
  demote-to-`Fixed` precedent. Rejected: the actuators are an edit of the
  tendon, not of something elsewhere in the tree; the user asked for the
  tendon to go, and the removal is undoable as one gesture.
- **`<tendon><spatial>` in the same document type.** Its length is a
  geometry computation over sites and wrapping geoms riggen does not
  make, and its attributes (`width`, `sidesite`, pulley `divisor`) share
  nothing with a fixed tendon's but the name. Dropped by name.
