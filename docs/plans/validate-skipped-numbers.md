# Plan: validate-skipped-numbers

- Started: 2026-09-13
- Milestone: v0.6 — the import gap's last mile
- Idea (verbatim from the human): "`validate` checks the numbers it skips —
  geom poses and an `Override` inertial's — so a NaN cannot reach a writer"
  (docs/ROADMAP.md §v0.6, its last open line)

## Goal
Every `f64` / `f32` the document holds is finite, and `validate` says so:
geom poses (visual and `CollisionPolicy::Meshes`), `Primitives` poses and
sizes, an `Override`'s mass, CoM and tensor, a `Hybrid` mass, a
`density_override`, a mesh asset's `scale` and `fix_up`, a joint's
`Limits::effort` / `velocity` and `Dynamics`, a decomposition's `concavity`,
and the geom and material colours. A non-finite one is
`ValidationError::NonFinite { what }` naming the slot, like every number
already checked. Because `validate` runs behind every command, before
`to_json` and at the head of `resolve`, a NaN is refused by the edit that
makes it, is never saved as the `null` serde_json writes for one (a
`.riggen` that then does not load), and never reaches a writer. The
DATA-MODEL invariant list loses its "not checked — a backlog line" sentence.

Where a NaN comes from today: not from text. The XML reader refuses a
non-finite word (`xml.rs` `numbers`), the SDK refuses a non-finite Python
float (`doc.rs` `from_py`), JSON cannot spell one, and the properties
panel's `DragValue` filters them. What is left is arithmetic: a command
composing poses, a rotation normalised from a degenerate quaternion, an
overflow. So the check is the safety net ADR-0005 says `validate` is, not
an import-facing refusal, and the Menagerie numbers cannot move.

## Non-goals
- **Physical validity stays at the export gate.** Mass > 0, a symmetric,
  positive-definite tensor obeying the triangle inequality: that is
  `inertial::check` in `resolve` (ADR-0032). `validate` runs after every
  command, and refusing a not-yet-positive-definite tensor there would
  refuse typing one in entry by entry. Finiteness only.
- No positivity for primitive sizes or mesh `scale` — a zero radius is
  finite; whether to refuse it is ⚠ OPEN 2, and a backlog line if not
  taken here.
- No new refusal at import, no SDK change, no UI change: every text entry
  point already refuses non-finite numbers (see Goal). No snapshot moves.
- `Robot.validate()` / `check()` being always empty in the SDK stays its own
  backlog line.

## Design deltas
- `riggen-core/src/validate.rs`: a `check_numbers` pass (or the checks
  folded into `check_references` / `check_joints` where the slot already
  is — the agent's call, by the file's shape) pushing `NonFinite { what }`.
  `what` strings in the existing voice: `pose of geom g2 of link l3`,
  `pose of collision geom g4 of link l3`, `size of primitive 0 of link l3`,
  `mass of the inertial of link l3`, `scale of mesh m1`, `effort limit of
  joint j7`, `damping of joint j7`, `color of material "steel"`. No new
  variant (unless ⚠ OPEN 1 is taken).
- `docs/DATA-MODEL.md` §Core types, the invariants list: the finiteness
  bullet names the whole document and drops the backlog sentence.
- No ADR: finiteness in `validate`, physics at the gate, is the split
  ADR-0024 and ADR-0032 already drew.

## Steps
Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[1]** Step 1 — **Geom poses.** Visual geoms, `Meshes` collision
  geoms and `Primitives` poses and sizes are finite. Tests: one
  `validate.rs` unit test per slot naming the `what`; `SetGeomPose` with a
  NaN translation is refused and leaves the document untouched
  (`command.rs`); `resolve` on a document with a NaN geom pose returns
  `ExportError::Invalid(NonFinite)` and writes nothing; `file::to_json`
  refuses it. DATA-MODEL invariant bullet updated for geoms.
- [x] **[2]** Step 2 — **Inertial numbers.** `Override` mass / CoM / tensor,
  `Hybrid` mass and `density_override` are finite. The case to get right:
  `resolve.rs`'s multi-link inertial test carries a `"nan"` `Override`
  beside `singular` / `lopsided` / `negative` and expects four
  `ExportError::Inertial`s — `resolve` now returns at the `validate` gate
  with one `Invalid` instead, so that case moves to its own assertion and
  the remaining three keep proving `inertial::check`. `inertial::check`'s
  own `NonFinite` stays (it guards a composed value too). DATA-MODEL
  bullet updated for inertials.
- [x] **[1]** Step 3 — **The rest of the document.** Mesh `scale` and
  `fix_up`, `Limits::effort` / `velocity`, `Dynamics`, decomposition
  `concavity`, geom and material colours. One test per group. A test that
  walks a document with *every* slot set to NaN, one at a time, and asserts
  each is refused — the regression net for a field added later. DATA-MODEL
  bullet in its final form: "every number in the document is finite".
- [ ] **[2]** Step 4 — *only if ⚠ OPEN 1 is taken*: a rotation of zero
  length in any pose (frame, joint origin, geom, primitive, `fix_up`) is
  refused. The URDF writer's `to_xyz_rpy` normalises it into NaN rpy and
  the MJCF writer writes `quat="0 0 0 0"`, which MuJoCo refuses; the SDK's
  `set_geom_pose({"t": …, "r": [0,0,0,0]})` reaches it today. Test: that
  SDK-shaped pose through `SetGeomPose` is refused; a URDF export of a
  valid document has no `NaN` in it.

## Acceptance
- `cargo test -p riggen-core -p riggen-export` passes, including: a NaN in a
  geom pose is refused by `validate` and `resolve` returns
  `ExportError::Invalid` instead of exporting (the v0.6 accept line); the
  every-slot walk from step 3 refuses each slot.
- The full pre-commit gate (fmt, clippy `-D warnings`, test) is green and
  the snapshot suite is unchanged.
- `menagerie_style.xml` still imports and exports (the `mujoco` job's
  corpus): nothing an import produces is refused by the new checks.

## Docs to update on completion
- `docs/DATA-MODEL.md` §Core types, invariants — the finiteness bullet
  covers the whole document; the "not checked — a backlog line" sentence
  gone (done step by step, verified at retirement).
- `docs/ROADMAP.md` §v0.6 — the `validate` line marked *Landed*, with the
  one-line result.
- `AGENTS.md` current state — v0.6's last line landed; **Next:**
  `/close-cycle` v0.6.
- `docs/BACKLOG.md` — if ⚠ OPEN 2 is not taken: "Primitive sizes and mesh
  `scale` are finite but may be zero or negative; refuse them in `validate`
  or at the export gate". If ⚠ OPEN 1 is not taken: the zero-length
  rotation as its own line.

## Open questions
- Found in step 3: a `Fixed` joint's `axis` was a slot the Goal did not
  list — `ZeroAxis` only looks at movable joints, yet the file saves it.
  Now `NonFinite { "axis of joint j7" }`; the walk would have caught it.
- OPEN 1 — **answered 2026-09-13: taken.** Step 4 runs.
- OPEN 2 — **answered 2026-09-13: not taken.** Finiteness only; the
  positivity line goes to the backlog at retirement.
- ~~⚠ OPEN 1~~ (human, before step 4): **refuse a zero-length rotation too?**
  It is finite, so it is not "a number it skips", but it is the one way a
  NaN still reaches a writer after steps 1–3, for every pose kind, not only
  geoms. Recommendation: take it — it is what the roadmap line's "so a NaN
  cannot reach a writer" actually promises. Cost: one variant
  (`DegenerateRotation { what }`) or `NonFinite`'s message widened, and a
  check at each pose site steps 1–3 already touch.
- ~~⚠ OPEN 2~~ (human, before step 1): **positivity for primitive sizes and mesh
  `scale`?** MuJoCo refuses a non-positive geom size; a zero `scale`
  collapses a mesh. Recommendation: not here — it is a physical bound, the
  kind this plan leaves to the gate, and no file riggen reads produces one.
  Backlog line.
