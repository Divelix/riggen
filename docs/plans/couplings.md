# Plan: couplings

- Started: 2026-09-08
- Milestone: v0.4, the file half — "Couplings the document cannot hold"
  (`docs/03-roadmap.md` §v0.4), plus the `<muscle>` / `<adhesion>` question
  ADR-0024's addendum sent here
- Idea: none — the roadmap bullet names the three shapes; the decisions
  they need are one ADR (step 1)

## Goal

Three things a Menagerie-style file says about how its joints move
together survive import → edit → export instead of being warned and
dropped. **Mimic chains** — a follower whose leader also follows — are
accepted, resolved by one topological pass of `fk::resolve_q`, written by
all three writers as they are, and read back by both importers; only a
*cycle* is refused. **`<joint ref>`** is kept as `Joint::qpos_ref`, the
MJCF coordinate offset of the joint, while the document's own `q` stays
what it has always been: the deviation from the authored pose. Ranges are
shifted on the way in and back on the way out, a `polycoef` over such a
joint is exactly our mimic (MuJoCo's deviations are from `qpos0 = ref`),
and nothing in `fk`, the viewport, Edit's zero configuration or the
URDF/SDF writers learns the field exists. **`<tendon><fixed>`** is a
document type, `Robot::tendons`, a named linear combination of joint
values with a range and its own dynamics; `ActuatorTarget::Tendon` is the
second variant ADR-0023 left room for, so the corpus's `grip` motor —
the last name in `ROUND_TRIP_DROPPED` — comes back out as what it was.
Schema 6. The corpus gains a chain, a `ref` and a two-joint tendon, and
the `mujoco` job compares its `<equality>` and `<tendon>` blocks with the
original's the way it already compares `<actuator>`.

## Non-goals

- `<tendon><spatial>` (site-routed, with wrapping geoms and pulleys) and
  `<equality><tendon>`: dropped by name, each a unit test, not a feature.
- Non-linear `polycoef` (a2..a4 ≠ 0) stays dropped as it is.
- `springref`, `springlength`, `solref`/`solimp` on tendons and joints:
  counted, not carried — the same "bounded promise" ADR-0024 §4 made.
- `<muscle>` and `<adhesion>` — decided by OPEN 1, default out.
- A Tendons window or an editing surface for tendons in the GUI: the
  SDK and the file are the editors (OPEN 2); the properties panel shows,
  read-only, what a `<general>` already shows read-only (ADR-0024).
- Flattening a chain on export, for any writer: what went in comes out.
- Composition (`<include>` / `<attach>` / `<frame>`): the next bullet.
- SDF import; any new writer.

## Design deltas

- **ADR-0025** (step 1): the three decisions above, amending ADR-0013
  (chains accepted, cycles refused; the limits check composes the chain)
  and ADR-0023 (`ActuatorTarget::Tendon`), and stating the `qpos_ref`
  convention: *the document's `q` is the deviation from the authored pose;
  `qpos_ref` is what MJCF adds to it*. Also decides what a `Tendon` carries
  vs counts, and what `RemoveTendon` does to the actuators on it (OPEN 3).
- `riggen-core::robot`: `Joint::qpos_ref: f64` (`#[serde(default)]`, 0.0);
  `Tendon { name, joints: Vec<TendonJoint { joint, coef }>, range:
  Option<[f64; 2]>, limited: Option<bool>, stiffness, damping,
  frictionloss }`; `Robot::tendons: BTreeMap<TendonId, Tendon>`;
  `id_type!(TendonId, 't', "tendon")`; `ActuatorTarget::Tendon(TendonId)`;
  `Mimic`'s doc comment loses "chains are rejected".
- `riggen-core::fk::resolve_q`: one topological pass with a memo, cycle-safe
  on an unvalidated document (a cycle member keeps its raw `q`, the way
  `fk` terminates on a link loop).
- `riggen-core::validate`: `MimicChain` → `MimicCycle { joints }`;
  `MimicExceedsLimits` maps the *free* leader's range through the composed
  affine map; `qpos_ref` finite; tendons: `DuplicateTendonName`,
  `EmptyTendon`, `DanglingTendonJoint`, `DuplicateTendonJoint`,
  `TendonOnFixedJoint`, `ZeroTendonCoef`, `InvalidTendonRange`, non-finite
  numbers; `DanglingActuatorTarget` covers a tendon target.
- `riggen-core::command`: `AddTendon` / `RemoveTendon` / `SetTendon` /
  `RenameTendon` after the frame quartet, `Created::Tendon`; `RemoveLink`
  drops a removed joint's entries from every tendon and removes a tendon
  left empty with the actuators on it; `SetJoint` to `Fixed` on a tendon's
  joint is refused naming the tendon, as demoting a leader is; a mimic
  leader that itself follows is no longer refused.
- `riggen-core::file`: **schema 6**, `upgrade_v5_to_v6` empty (`tendons`
  and `qpos_ref` both default); `assets/fixtures/driven.riggen` and the
  byte-for-byte fixtures re-saved.
- `riggen-export::resolve`: `ResolvedJoint::qpos_ref`, `ResolvedTendon`
  (joint indices, coefs, range, limited, dynamics),
  `ResolvedRobot::tendons`, `ResolvedActuator::target: ResolvedTarget
  { Joint(usize) | Tendon(usize) }`.
- `riggen-export::mjcf`: `<joint ref>` written when non-zero, `range`
  shifted by it, a derived `ctrlrange` too; `<tendon><fixed>` block after
  `<equality>`; `tendon="…"` transmissions; the "we never write `ref`"
  comment rewritten.
- `riggen-export::mjcf_in`: `ref` read (angle-converted), range shifted,
  `moved_zero` and its drop reason deleted; `<tendon><fixed>` read
  (`<default><tendon>` through the same `Defaults`), `<spatial>` and
  `<equality><tendon>` counted by name; a tendon-targeted actuator of any
  of the four kinds read; `ImportWarning::TendonDropped { tendon, reason }`.
- `riggen-export::urdf_in` / `urdf` / `sdf`: a chain reads and writes
  as-is; nothing else changes (both writers stay in deviation form).
- **Probed in step 1 (ADR-0025 §Context):** a fixed tendon's length is
  `Σ coef · qpos`, *absolute* — MuJoCo evaluates tendons over `qpos`, not
  over deviations from `qpos0` as it does equalities. So `Tendon::range`
  is held in MJCF's own terms and never shifted by a `qpos_ref` on either
  side (steps 7–9), and `fk_samples`' `tendons` block writes the length as
  `Σ coef · (q + qpos_ref)` for `check_tendons` (step 8).
- `riggen-export::fk_samples`: `q` written as `qpos` (`q + qpos_ref`);
  an `actuators` entry names `"joint"` or `"tendon"`; a `tendons` block;
  the derived-range restatement adds `qpos_ref` and derives nothing for a
  tendon.
- `python/tests/test_mjcf_load.py`: `check_equalities` reads deviations
  through `model.qpos0` and accepts transitive coupling; `check_actuators`
  accepts `mjTRN_TENDON`; `check_tendons`; the `@ORIGINAL` comparison
  covers `<equality>` and `<tendon>`; `ROUND_TRIP_DROPPED` empty.
- `riggen-app`: joint properties — the leader combo excludes only what
  would close a cycle; read-only rows for `qpos_ref` (when non-zero) and
  for the tendons through the joint with the actuators driving them; the
  joint tree shows a chain follower's rule as it shows a follower's today.
- `riggen-py`: `Tendon`, `robot.tendons`, `robot.tendon(name)`,
  `add_tendon`, `tendon.remove()`; `Actuator.tendon`; `joint.qpos_ref`;
  `add_actuator` on a tendon.

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[3]** Step 1 — ADR-0025, with the `ref` semantics probed in
  MuJoCo before they are written down (a scratch script, its findings
  pinned in step 6's Python test): `qpos0 == ref`, `range` is in `qpos`
  terms, `polycoef` deviations are from `qpos0`, a `<position tendon>`
  under `autolimits="true"` with no `ctrlrange` loads with zero warnings.
  Decides OPEN 3 and OPEN 4. Row in `docs/adr/README.md`; ADR-0013 and
  ADR-0023 marked amended.
- [x] **[2]** Step 2 — core: chains. `resolve_q` topological and
  cycle-safe; `MimicChain` → `MimicCycle`; the limits check composes the
  chain; `SetJoint` accepts a leader that follows. `fk` test: a chain of
  three equals the hand-resolved free model; `validate` tests for a cycle
  of two and of three and for the composed reach.
- [ ] **[2]** Step 3 — export: chains through every writer and reader.
  `mimic_refusals` stops dropping a chain; `urdf_in` and `mjcf_in` tests
  read one back identically; MJCF / URDF / SDF golden tests gain a chain;
  the corpus gains a `finger` body under `tool` whose joint follows
  `shoulder_lift` (which follows `shoulder_pan`), and the corpus test's
  warning list shrinks; `check_equalities` accepts a pair coupled through
  a third joint.
- [ ] **[1]** Step 4 — app: chains. The leader combo lists every movable
  joint that would not close a cycle; the joint tree shows a chain
  follower read-only at its resolved value. Snapshot
  `joint_tree_chain`; `properties_joint_mimic` re-read if it moves.
- [ ] **[2]** Step 5 — core + file: `Joint::qpos_ref`, schema 6 (the
  bump this plan makes once; `tendons` lands at the same version in step
  7). `validate` finite; `file` test `a_v5_file_opens_as_v6_…`; the v6
  fixtures re-saved; `Command::SetJoint` carries it like `mimic`.
- [ ] **[3]** Step 6 — export: `ref` round trip. Import reads and
  converts it, shifts the range, keeps the `polycoef` it used to drop,
  deletes `moved_zero`; the writer writes it back and shifts what it
  derives; `fk_samples` writes `qpos`; `check_equalities` uses
  `model.qpos0`; the corpus's `shoulder_pan` gains `ref="10"` and the
  `mujoco` job holds the re-export to the original's `fk.json` through it.
- [ ] **[2]** Step 7 — core: tendons. `Tendon`, `TendonId`,
  `Robot::tendons`, `ActuatorTarget::Tendon`, the quartet, `Created::Tendon`,
  the validate refusals, `RemoveLink` and `SetJoint` rules; every
  `target.joint()` caller audited (`SetActuators`, glyphs, properties,
  `fk_samples`, `mjcf`) for what a tendon actuator means to it. Command
  tests for add/remove/set/rename, undo, the two side effects.
- [ ] **[3]** Step 8 — export: tendons out. `ResolvedTendon`,
  `ResolvedTarget`; the MJCF writer's `<tendon>` block and `tendon=`
  transmissions; `fk_samples`' `tendons` block and target naming;
  `check_actuators` accepts `mjTRN_TENDON`, `check_tendons` verifies wrap
  objects and coefficients against the samples; the arm fixture gains no
  tendon — a golden test and a hand-built document through the `mujoco`
  job locally (`uv run … test_mjcf_load.py`) establish that MuJoCo loads
  it with zero warnings.
- [ ] **[2]** Step 9 — export: tendons in. `mjcf_in` reads
  `<tendon><fixed>` and tendon-targeted actuators of all four kinds;
  `<spatial>` and `<equality><tendon>` are `TendonDropped` /
  `ElementDropped` by name (unit tests, not corpus); the corpus's
  `grip_tendon` grows to two joints with `range`, `stiffness`, `damping`;
  `grip` leaves `ROUND_TRIP_DROPPED` (now empty); the `@ORIGINAL`
  comparison gains `check_round_trip_equalities` and
  `check_round_trip_tendons`, field for field like the actuators.
- [ ] **[2]** Step 10 — app: what the panel shows. Joint properties gain
  a read-only `ref` row (non-zero only) and a read-only "tendons" section
  — each tendon through the joint with its coefficient and the actuators
  driving it; the actuator combo's `SetActuators` still skips nothing new
  (a tendon actuator is not "on" the joint). Snapshots
  `properties_joint_ref`, `properties_joint_tendon`; `debug_state()`
  carries both.
- [ ] **[1]** Step 11 — py: the SDK. `Tendon`, the four calls,
  `Actuator.tendon` (`None` for a joint actuator), `joint.qpos_ref`;
  `add_actuator(spec, tendon=…)`; a chain accepted where the SDK test
  today expects `EditError`; `examples/arm.py` untouched; `python/tests`
  cover it; 01 §Python SDK table rows.

## Acceptance

The v0.4 accept, restricted to this bullet:

```sh
cargo test
mkdir -p target/corpus/arm && cp assets/fixtures/menagerie_style.xml target/corpus/ \
  && cp assets/fixtures/arm/{base.stl,shoulder.stl,thing.msh} target/corpus/arm/
cargo run -p riggen-app -- --export mjcf --fk-samples --out target/sample-corpus target/corpus/menagerie_style.xml
uv run --no-project --with mujoco --with numpy python python/tests/test_mjcf_load.py \
  target/sample-corpus@target/corpus/menagerie_style.xml
```

passes with `ROUND_TRIP_DROPPED` empty, the re-export's `<actuator>`,
`<equality>` and `<tendon>` blocks equal to the original's field for
field, MuJoCo loading it with zero warnings, and the corpus test's
warning list naming neither `<joint ref>`, `<tendon>`, `grip` nor any
`MimicDropped`. The `mujoco` CI job is that command.

## Docs to update on completion

- `docs/02-data-model.md` §Core types — `Joint::qpos_ref`, `Tendon`,
  `TendonId`, `Robot::tendons`, `ActuatorTarget::Tendon`; the validate
  list (`MimicCycle` for `MimicChain`, the tendon refusals); §Commands —
  the tendon quartet, `RemoveLink`'s and `SetJoint`'s two new side effects;
  §Kinematics — `resolve_q` topological; §`ResolvedRobot` — the tendon and
  target types, `qpos_ref`; §Format mapping — rows for `ref`, chains,
  tendons; §MJCF import — the read table, the drop table, the `ref` rule;
  §Schema — schema 6.
- `docs/01-architecture.md` §Panels — the properties rows; §Python SDK —
  the table rows; §Testing — the three-block comparison, `check_tendons`.
- `docs/03-roadmap.md` §v0.4 — the couplings bullet *Landed
  (plans/couplings, ADR-0025)*; the escape-hatch bullet's "what it still
  drops is `grip`" sentence.
- `docs/BACKLOG.md` — the `<muscle>` / `<adhesion>` line rewritten per
  OPEN 1; the "things that reference a site" line loses "equality
  constraints" only if OPEN 1 changes nothing there (it does not: those
  are `<equality><connect>`/`<weld>`, still out).
- `docs/adr/README.md` — 0025 row; 0013 and 0023 status "amended by 0025".
- `AGENTS.md` current state — the couplings landed; **Next:** composition.
- `README.md` — only if its feature list names mimics (check at retire).

## Open questions

- `⚠ OPEN 1:` `<muscle>` (on a joint or a tendon) and `<adhesion>` — in
  this plan or a backlog line? **Human, by step 9.** Recommendation:
  out. Neither is a coupling: a `<muscle>` is a fifth actuator vocabulary
  (`timeconst`, `range`, `force`, `scale`, `vmax`, `fpmax`, `fvmax`,
  `lmin`, `lmax`) folded into muscle-typed `prm` vectors, plus the
  `lengthrange` ADR-0024 §4 counts; `<adhesion>` is a body target. With a
  tendon in the document both become *possible*; the backlog line is
  rewritten to say so and stays.
- `⚠ OPEN 2:` a Tendons list panel now that an actuator can exist no
  joint's panel owns (ADR-0023's own condition) — or the read-only rows of
  step 10? **Human, by step 10.** Recommendation: the rows; a panel is a
  backlog line until a user edits tendons in the GUI rather than the SDK.
- ~~`⚠ OPEN 3:`~~ `RemoveTendon` while actuators target it — **decided
  (ADR-0025 §4): take them**, the `RemoveLink` precedent; a removal is a
  gesture about the tendon, and the actuators are visible and undoable.
- `⚠ OPEN 4:` a chain in URDF and SDF — written as-is (the round trip is
  the identity; the spec does not forbid it; consumers vary) or flattened
  against the free leader? **Human, by step 3.** Recommendation: as-is,
  every writer the same; a consumer that refuses a chain is the user's to
  flatten for, and the SDK makes that one loop. *ADR-0025 §2 records
  this recommendation as the decision, marked as the human's to confirm;
  overturning it is an addendum to the ADR before step 3 starts.*
