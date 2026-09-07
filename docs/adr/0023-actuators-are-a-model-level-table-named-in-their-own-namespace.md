# ADR-0023: Actuators are a model-level table, named in their own namespace

- Status: Accepted
- Date: 2026-09-07
- Amends: [ADR-0014](0014-actuators-on-the-joint-mjcf-only-three-presets.md)

## Context

ADR-0014 put the actuator on `Joint::actuator: Option<ActuatorSpec>`, named
it after the joint, and named its own consequence: "Promoting to a
top-level `Robot::actuators` map later (the shape MJCF itself has, and the
shape MJCF *import* will want, since an actuator may target a tendon or a
site) is a plan, not a catastrophe." MJCF import (ADR-0015) is that plan,
and landing it exposes two silent losses `Option<ActuatorSpec>` cannot
avoid:

**A `<actuator>` whose `name` differs from its joint's.** MJCF gives
`<actuator>` its own per-element namespace precisely so `data.ctrl` can be
indexed by a name distinct from `data.qpos`'s — a real file in the wild
reads `<position name="shoulder_pos" joint="shoulder" .../>`. ADR-0014's
"the `<actuator>` element takes the joint's own name" was a *writing* rule,
argued from what riggen itself produces; read as an *importing* rule it
throws the file's name away and re-derives one MJCF never asked it to.

**A second `<actuator>` on an already-driven joint.** MuJoCo does not
refuse this — it sums the control signals of every actuator that targets
one joint, a pattern real models use (e.g. a coarse and a fine drive on the
same axis). `Option<ActuatorSpec>` has room for exactly one; import would
have to overwrite the first with the second, or refuse the file, either way
losing what the file said with no warning at the loss.

Both losses trace to the same cause: `Joint::actuator` conflates "this
joint has an actuator" with "this actuator has a name and a slot count of
one", because when ADR-0014 was written nothing but the joint could be a
target and nothing but riggen's own writer produced the name. Import
removes both premises at once.

## Decision

**The document holds `Robot::actuators: BTreeMap<ActuatorId, Actuator>`,**
the shape ADR-0014 already named as MJCF's own and deferred:

```rust
pub struct Actuator {
    pub name: String,       // its own; defaults to the joint's, unique among actuators
    pub target: ActuatorTarget,
    pub spec: ActuatorSpec, // the three presets, unchanged
}
pub enum ActuatorTarget { Joint(JointId) }
```

`Joint::actuator` is deleted. This amends ADR-0014 on exactly three points,
each forced by the losses above and by nothing else — the three presets,
`ctrlrange`/`forcerange`, the apologetic comment, and every refusal
`validate` already made stay as ADR-0014 wrote them, now reading "the
actuators targeting this joint" where they read "the joint's actuator":

1. **The table shape ADR-0014 deferred, landed now.** A `BTreeMap` keyed by
   a new `ActuatorId` (`id_type!(ActuatorId, 'a', "actuator")`, allocated
   from the document's one `IdGen`, per ADR-0005) is a model-level
   collection with the same shape as `Robot::frames` (ADR-0012). It is what
   lets several actuators name one joint without the joint holding a list.

2. **The naming rule inverts.** ADR-0014 said "the `<actuator>` element
   takes the joint's own name" — true of what riggen writes, false of what
   a file may say. Now: an actuator carries its own name, defaulting to the
   joint's when riggen creates one (`SetActuators`, the per-joint properties
   control), so nothing changes for a document riggen alone has touched,
   and import keeps whatever name the file gave.

3. **The one-actuator-per-joint assumption, unstated in ADR-0014, is
   named and dropped.** `ActuatorTarget::Joint(JointId)` is not
   `unique_by(target)`: several actuators may target one joint, `validate`
   accepts it, and a MuJoCo file that sums two drives on one axis imports
   without a warning it does not deserve. `check_actuators` gains a
   **duplicate actuator name** refusal instead (unique among actuators, an
   actuator may still share its joint's name) and a **dangling target**
   refusal, both new because a name and a target are now separate fields
   that can each go wrong on their own.

**`ActuatorTarget` gets exactly one variant.** A tendon has no document
type yet (a later bullet) and a site actuator is `refsite` plus six-vector
`gear` semantics, not a retarget of what exists today — building either
now would be speculative. The enum exists so the table's shape does not
change again when they arrive; an actuator that targets a tendon, site or
body keeps its `ActuatorDropped` warning on import, unchanged.

**No Actuators window.** ADR-0014's alternative "a list panel" is still
not earned: the per-joint properties control, now listing every actuator
that targets the selected joint instead of showing at most one, remains
the editing surface. A list panel becomes worth it when an actuator can
exist that no joint's panel can show — with a tendon or site target, not
with this ADR.

**Schema 4.** `upgrade_v3_to_v4` moves each `Some(spec)` on a joint into
the table, allocating an `ActuatorId` per joint (in `JointId` order, from
the document's `next_id`) and naming each after its joint — the same name
a v3 document already implied, so every existing `.riggen` reopens with
identical actuators under the new shape.

## Consequences

- `Joint::actuator` is gone from the `Joint` listing; reading "does this
  joint have an actuator" is a scan of `Robot::actuators` for a matching
  target, same as reading whether a joint is a mimic follower already is.
- The MJCF writer's apologetic comment (ADR-0004 §4, amended by ADR-0014)
  now keys off "no actuator targets this joint" rather than "this joint's
  `actuator` field is `None`" — same trigger, read off the table instead of
  the field.
- `ResolvedJoint::actuator` becomes `ResolvedRobot::actuators: Vec<ResolvedActuator>`
  in `ActuatorId` order; every writer and `--fk-samples` consumer reads the
  vector instead of asking each joint.
- `SetActuators(Option<ActuatorSpec>)` survives as the whole-model apply it
  was — one actuator per free movable joint, named after it, mimic
  followers skipped — now re-expressed as inserts into (or a target-`Joint`
  clear of) the table, still one command, one history entry.
- The four per-actuator commands (`AddActuator`, `RemoveActuator`,
  `SetActuator`, `RenameActuator`) follow the `Frame` quartet's precedent
  (ADR-0012) exactly, rather than inventing a new shape for a keyed
  top-level collection.
- URDF and SDF are untouched: both still write no actuator and import none
  (ADR-0014, ADR-0016 §5); their comments are re-keyed to the table, not
  rewritten.
- A `.riggen` from before this ADR opens unchanged and re-saves at schema
  4, one actuator per joint that had one, under the name that joint already
  implied.

## Alternatives considered

- **`Joint::actuator: Vec<ActuatorSpec>` instead of a map.** Fixes the
  several-actuators loss without a new id kind or a model-level
  collection. Rejected: it has nowhere to hold a name distinct from the
  joint's, so the first loss — a `<actuator name="shoulder_pos">` on joint
  `shoulder` — stays open, and ADR-0014 already named the map as where this
  goes.
- **Keep `Joint::actuator` and only relax the naming rule** (store a
  `name: Option<String>` beside the spec, default to the joint's).
  Rejected: still one slot, so a second `<actuator>` on a driven joint has
  nowhere to go but overwriting the first — the loss this ADR exists to
  close, not the one it can leave for later.
- **`validate` keeps refusing more than one actuator per joint**, treating
  a second `<actuator>` as the same kind of error as one on a `Fixed`
  joint. Rejected: MuJoCo does not refuse it — it sums the controls — so
  refusing it here would make `validate` reject models MuJoCo accepts, and
  import would have to either drop the second actuator silently (the loss
  named in the Context) or refuse otherwise-loadable files.
- **A wider `ActuatorTarget` now** (`Joint`, `Tendon`, `Site`), building the
  escape hatch's targets ahead of the document types they need. Rejected as
  speculative: `Tendon` has no document type until the couplings bullet
  lands, and a site actuator's `gear` semantics are not a straightforward
  retarget — both stay `ActuatorDropped` until their own plan gives the
  enum something real to hold.
