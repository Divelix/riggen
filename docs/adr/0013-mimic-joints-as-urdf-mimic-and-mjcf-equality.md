# ADR-0013: A mimic joint is URDF's `<mimic>` and an MJCF `<equality><joint polycoef>`; no chains; a removed leader frees its followers

- Status: Accepted
- Date: 2026-08-31

## Context

A gripper is one motor and two fingers: `q_right = -q_left`. URDF says this
natively — `<mimic joint="finger_left" multiplier="-1" offset="0"/>` inside
the follower's `<joint>` — and essentially every gripper URDF a researcher
imports carries one. Until now we dropped it, loudly
(`ImportWarning::MimicDropped`), and nothing in the document could hold the
coupling, so importing a Robotiq or a Panda hand and re-exporting gave a
gripper whose fingers moved independently. There was no way to put it back,
in the GUI or the SDK.

A mimic is **not** a closed kinematic loop (`SEED.md` §4 non-goal). The tree
stays a tree, `fk` stays one depth-first pass, and nothing here needs
`<equality connect>` or a real loop closure. It is a *coupled degree of
freedom*: one joint's `q` derived from another's. Users do reach for it to
approximate a four-bar linkage, and that approximation is precisely what
keeps us out of loop territory.

Three things needed deciding rather than discovering:

1. **What MJCF gets**, since it has no `<mimic>`.
2. **Chains** — a follower whose leader also follows.
3. **What happens to a follower when its leader is deleted or demoted.**

## Decision

**The document holds `Joint::mimic: Option<Mimic { joint, multiplier,
offset }>`**, meaning `q(this) = multiplier · q(joint) + offset`. It is
schema 2, with an `upgrade_v1_to_v2` step (a no-op: a v1 file has no key and
serde's default is `None`) and `assets/fixtures/pendulum.riggen` frozen at
v1 as the upgrade corpus.

**`fk::resolve_q(&Robot, &JointState) -> JointState` is the one
implementation of the rule.** `fk` calls it, and so do the Joints window and
`--fk-samples`, so the derived number cannot drift between what the viewport
draws and what the export writes. A follower's own entry in the caller's
`JointState` is ignored, not an error — it is derived state.

**URDF gets `<mimic joint multiplier offset/>`**, the native spelling, and
`urdf_in` reads it back.

**MJCF gets an `<equality>` block after `<worldbody>`:**

```xml
<equality>
  <joint joint1="finger_right" joint2="finger_left" polycoef="0.1 -1 0 0 0"/>
</equality>
```

`polycoef` is `a0 a1 a2 a3 a4` with `y − y0 = a0 + a1(x − x0) + …`, where `x`
and `y` are the two joints' deviations from their `qpos0`. We never write
`ref`, so both references are zero and `(offset, multiplier, 0, 0, 0)` is
exactly URDF's rule. Only the first two slots are ever non-zero: non-linear
coupling is not modelled.

**A MuJoCo equality is a soft solver constraint, not a reduction.** Under
dynamics the fingers track each other to within `solref`/`solimp`, the same
way joint limits are soft. This costs the CI proof nothing: `--fk-samples`
writes `qpos` for every movable joint, the follower included at its derived
value, so `mj_forward` still matches `fk` to 1e-6.

**Chains are rejected**, in `validate`, with a message that says so.

**`validate` also refuses**: a leader that names no joint, a joint mimicking
itself, a mimic on a `Fixed` joint or one whose leader is `Fixed`, a
non-finite or zero `multiplier`, a non-finite `offset`, and — the sim-ready
check — a leader range that, mapped through `(multiplier, offset)`, does not
fit inside the follower's own limits.

**Removing a link clears the mimic of any joint that followed one of the
removed joints; demoting a leader to `Fixed` is refused.** Deletion is a
gesture about one subtree and must not fail because of an edit elsewhere in
the tree; a kind change is an edit of the coupling's own leader, and
refusing it names the follower.

**There is no new command.** The properties panel already commits a whole
edited `Joint` through `SetJoint`, which preserves only `parent`/`child`, so
`mimic` rides along. One gesture is still one command.

## Consequences

- A gripper URDF round-trips with its fingers still coupled, and
  `ImportWarning::MimicDropped` narrows from "no mimic joints yet" to the
  cases we still refuse — a chain, a `fixed` leader, a leader not in the
  file — each with a reason.
- The exported MJCF's follower is driven by a constraint the solver has to
  satisfy, not by construction. A user who needs a hard reduction wants a
  `<tendon><fixed>` or an equality with a stiffer `solref`; both are backlog,
  and this ADR is where they will be re-litigated.
- `MimicExceedsLimits` rejects documents a user could previously have built
  by hand. That is the point: MuJoCo would otherwise be given a `range` and
  an equality that disagree, and the model would look fine and behave badly.
- Chains can be added later without a schema change — `resolve_q` would grow
  from one pass to a topological one — so rejecting them now costs nothing
  but a message.
- `Continuous` cuts both ways in the limits check: a `Continuous` follower
  has no range to leave, so the check is vacuous; a `Continuous` leader has
  an unbounded one, which no bounded follower can contain.

## Alternatives considered

- **A `Mimic` as its own document-level list**, `Robot::mimics: Vec<…>`,
  rather than a field on the follower. It reads like the constraint block it
  becomes in MJCF, but it puts a second place a joint can be named, needs
  its own ids, and makes "is this joint driven?" a scan instead of a field —
  which is the question every one of the UI, `fk` and both writers asks.
- **Resolving chains** rather than rejecting them. Consumer support is a
  lottery: MuJoCo wants them flattened against the free joint anyway, and
  URDF consumers disagree about whether a chain is legal at all. Flattening
  is a transformation we would have to do on export and undo on import, and
  the round-trip would stop being the identity.
- **`<equality><joint>` with an explicit `solref`/`solimp`.** Two numbers we
  would be inventing on the user's behalf, in the one place where MuJoCo's
  defaults are what a MuJoCo user expects. A user who needs a stiffer
  coupling edits the class; a default we picked would be silently wrong for
  everyone else.
- **MJCF `<tendon><fixed>`** with two coefficients. It is the right model
  for a cable-driven coupling, and a different feature: `<mimic>` means a
  gear relation between two joint coordinates, which is what
  `<equality joint>` says.
- **Keeping the schema at 1** with a defaulted `mimic` field. Documents
  without a mimic would keep opening on older builds and documents with one
  would fail with serde naming an unknown field — loud, but not the clean
  `UnsupportedVersion` refusal. Half-compatibility is harder to reason about
  than a clean refusal, and this was the cheapest bump we will ever get: v1
  JSON is valid v2 JSON, so the machinery is one `match` and the first step
  is empty.
- **Refusing `RemoveLink` while something follows the doomed subtree.** The
  user asked to delete a link; making that fail because of a joint they
  cannot see, elsewhere in the tree, trains them to hunt for the coupling
  before every deletion. Clearing the follower is visible in the properties
  panel and undoable in one keystroke.
