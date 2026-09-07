# Plan: actuator-table

- Started: 2026-09-07
- Milestone: v0.4 (§The file: nothing silently lost, first bullet)
- Idea (verbatim from the human): "good, /plan it" — on the recommendation
  that the actuator table is the next thing to plan, ADR-0014's deferred
  alternative being already blessed and the `<general>` bullet blocked on it

## Goal
`Joint::actuator: Option<ActuatorSpec>` is gone; the document holds
`Robot::actuators: BTreeMap<ActuatorId, Actuator>`, each with its **own**
name, an `ActuatorTarget` saying what it drives, and one of the three
presets — the shape MJCF itself has, which ADR-0014 named and deferred
("it becomes right when MJCF import lands"). Two silent losses close with
it: an `<actuator>` whose `name` differs from its joint's keeps its name
through import → export, and a second `<actuator>` on an already-driven
joint is kept instead of overwriting the first without so much as a
warning. `.riggen` is **schema 4**, and its `upgrade_v3_to_v4` is the
first upgrade step that is not empty — which is why the chain moves off
the parsed `Robot` and onto the JSON before the field does. Nothing about
the three presets, the naming default, `ctrlrange`/`forcerange` or the
apologetic comment changes; where they read "the joint's actuator" they
now read "the actuators targeting this joint".

## Non-goals
- **The escape hatch.** `<general>`, `<adhesion>`, `<muscle>`, `dyntype` /
  `gaintype` / `biastype`, gains from a `<default class>`, explicit
  `ctrllimited` / `forcelimited` — the next bullet of the cycle. This plan
  builds the table they land in; it does not widen `ActuatorSpec`.
- **Targets other than a joint.** `ActuatorTarget` gets exactly one
  variant, `Joint(JointId)`. A tendon has no document type yet (the
  couplings bullet) and a site actuator is `refsite` plus six-vector `gear`
  semantics, not a retarget. An actuator driving a tendon, site or body
  keeps its `ActuatorDropped` warning; the enum is the seam those variants
  arrive at without a second schema bump.
- **An Actuators window.** ADR-0014's alternative mentioned "a list panel";
  the per-joint control in the properties panel stays the editing surface.
  A list panel becomes worth it when an actuator can exist that no joint's
  panel can show — i.e. with the escape hatch, not here.
- URDF and SDF: both still write no actuator and import none (ADR-0014,
  ADR-0016 §5). The comment each writes is re-keyed, not rewritten.
- Anything on the cycle's "out" list: no fourth format, no SDF import.

## Design deltas

**ADR-0023** (step 1), amending ADR-0014 on three points: the table shape
it deferred, its naming rule ("the `<actuator>` element takes the joint's
own name" → an actuator carries its own name, defaulting to the joint's),
and its unstated one-actuator-per-joint assumption (MuJoCo sums several,
foreign files ship them, `validate` must not refuse what the import may
not drop).

`docs/02-data-model.md` **§Core types** — `Robot::actuators`, `Actuator`,
`ActuatorTarget`; `Joint::actuator` deleted from the `Joint` listing:

```rust
pub struct Robot { …, pub actuators: BTreeMap<ActuatorId, Actuator> }  // schema 4

pub struct Actuator {
    pub name: String,               // its own; defaults to the joint's, unique among actuators
    pub target: ActuatorTarget,
    pub spec: ActuatorSpec,         // the three presets, unchanged
}
pub enum ActuatorTarget { Joint(JointId) }
```

`crates/riggen-core/src/ids.rs` — `id_type!(ActuatorId, 'a', "actuator")`,
allocated from the one `IdGen` like every other kind (ADR-0005).

`docs/02-data-model.md` **§Commands** — `AddActuator(Actuator)` →
`Created::Actuator(ActuatorId)`, `RemoveActuator(ActuatorId)`,
`SetActuator(ActuatorId, Actuator)`, `RenameActuator(ActuatorId, String)`:
the `Frame` quartet's shape (ADR-0012's precedent for a top-level keyed
collection). `SetActuators(Option<ActuatorSpec>)` survives as the
whole-model apply it was, re-expressed over the table.

`docs/02-data-model.md` **§ResolvedRobot** — `ResolvedJoint::actuator`
becomes `ResolvedRobot::actuators: Vec<ResolvedActuator { name, joint:
usize, spec }>`, in `ActuatorId` order. The writers read the vector;
ADR-0004 §4's apologetic comment is written when **no** actuator targets
the joint.

`docs/02-data-model.md` **§Schema** — schema 4, and the paragraph that
says both upgrade steps are empty gains the one that is not. **The chain
moves onto `serde_json::Value`**: `load_from`'s comment ("every version so
far parses into today's `Robot`") stops being true the moment
`Joint::actuator` is deleted, because `deny_unknown_fields` rejects a v3
file's `"actuator"` key before any step could run. `upgrade_vN_to_vN+1`
takes `&mut serde_json::Value`; the two existing steps stay empty; the
`deny_unknown_fields` promise (a typo fails loudly with the field's name)
is preserved and tested.

`crates/riggen-core/src/validate.rs` — `check_actuators` walks the table:
the same five refusals, now naming the actuator; a **dangling target**;
and a **duplicate actuator name** (unique among actuators — an actuator
may share a joint's name, which is the whole point of MJCF's per-element
namespaces). Several actuators on one joint is legal and has a test
saying so.

`crates/riggen-export/src/mjcf_in.rs` — `read_actuators` inserts into the
table instead of writing `j.actuator`; the `name` attribute survives; a
second actuator on one joint is a second entry.
`drop_what_validate_refuses` removes by `ActuatorId`.

`crates/riggen-export/src/fk_samples.rs` — `actuators()` walks the table in
`ActuatorId` order; `SampledActuator::name` stops being "which is its
joint's name". `python/tests/test_mjcf_load.py`'s `check_actuators` reads
the sampled name rather than assuming the joint's.

`crates/riggen-py` + `python/riggen/robot.py` + `_riggen.pyi` —
`robot.actuators()`, `robot.actuator(name)`; `Joint.actuator` becomes a
read of the table.

`docs/01-architecture.md` §SDK table (the `.actuator` row, the `Actuator`
row) and §`--fk-samples`.

## Steps
Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[1]** Step 1 — **ADR-0023: actuators are a model-level table,
      named in their own namespace.** The three amendments to ADR-0014
      above, with the two silent losses as the context and the target enum's
      single variant as the stated seam. `docs/adr/README.md` gains its row
      and ADR-0014's row gains "amended by 0023". No code.
- [x] **[3]** Step 2 — **the upgrade chain runs on the JSON.** `load_from`
      parses to `serde_json::Value`, walks `upgrade_vN_to_vN+1(&mut Value)`,
      then deserializes into `Robot` and validates as before.
      `upgrade_v1_to_v2` / `upgrade_v2_to_v3` become empty `Value` steps
      with their reasons intact. Green on the corpus unchanged:
      `corpus_pendulum_opens` (v1), `a_v2_file_opens_as_v3_with_no_actuators`
      (v2), the byte-for-byte v3 fixtures. New test: an unknown key in a
      v3 file still fails with the field's name, so `deny_unknown_fields`
      survives the move. *No schema bump in this step* — the mechanism
      lands alone, revertible alone.
- [x] **[2]** Step 3 — **`Robot::actuators`, `ActuatorId`, schema 4**
      (and step 5's four commands, which it turned out to need — see
      *Open questions*).
      `id_type!(ActuatorId, 'a', "actuator")`; `Actuator`, `ActuatorTarget`;
      `Joint::actuator` deleted; `upgrade_v3_to_v4` moves each `Some(spec)`
      into the table, allocating from `next_id` in `JointId` order and
      naming each after its joint. A **small purpose-made** v3 document
      with actuators is frozen in `assets/fixtures/` as the upgrade corpus
      (beside `pendulum.riggen` at v1) and its test pins the migration
      entry by entry, `next_id` included; `bracket.riggen` and
      `arm/arm.riggen` re-save at 4 as the byte-for-byte fixtures. Everything downstream is
      compiled through in this step by the shortest correct edit — the
      readouts follow in their own steps.
- [x] **[2]** Step 4 — **`validate` re-keyed.** `check_actuators` over the
      table: `Fixed` target, mimic-follower target, non-finite or negative
      gain, zero `gear` — each naming the actuator — plus a dangling
      target and a duplicate actuator name. A test that two actuators on
      one joint validate, and one that an actuator may share a joint's
      name.
- [x] **[2]** Step 5 — **the four commands.** *Folded into step 3*: the
      properties panel and the SDK both make a **per-joint** actuator edit,
      and the moment `Joint::actuator` is deleted there is no command that
      can express one — `SetActuators` is the whole model. `AddActuator`,
      `RemoveActuator`, `SetActuator`, `RenameActuator` with
      `Created::Actuator`, following `AddFrame`/`SetFrame`/`RenameFrame`
      exactly; `SetActuators(Option<ActuatorSpec>)` re-expressed: one
      actuator per free movable joint, named after it, mimic followers
      skipped; `None` removes every actuator whose target is a joint. One
      gesture, one history entry, undo unchanged.
- [x] **[2]** Step 6 — **`ResolvedRobot` and the MJCF writer.**
      `ResolvedActuator` vector in, `ResolvedJoint::actuator` out;
      `write_actuator` writes the actuator's own `name` and its joint's as
      `joint`; the apologetic comment keys off "no actuator targets this
      joint" (ADR-0004 §4, as amended by 0014 and now 0023). `fk_samples`
      walks the table; `test_mjcf_load.py`'s `check_actuators` reads the
      sampled name. The MJCF golden gains two cases: two actuators on one
      joint, and one whose name differs from its joint's.
- [x] **[2]** Step 7 — **MJCF import fills the table.** `read_actuators`
      inserts an entry per element, keeping the file's `name`; a second
      actuator on a driven joint is kept, not silently swallowed;
      `drop_what_validate_refuses` removes by `ActuatorId`.
      `ActuatorDropped` stays for a tendon / site / body target and for a
      tag outside the three presets. `menagerie_style.xml` grows a second
      actuator on an already-driven joint and one whose name is not its
      joint's; the corpus test's warning list shrinks by exactly what this
      step now keeps.
- [x] **[2]** Step 8 — **the app.** The properties panel's actuator
      control **lists** the actuators targeting the selected joint, one
      control each — the existing combo is the one-actuator case; `glyphs.rs`
      (`driven_marks`, the amber of an actuated joint) reads the table;
      `debug_state()` reports it. Snapshots `properties_joint_actuator` and
      `properties_joint_actuator_applied` refreshed, plus one new scenario
      for a joint an imported file gave two actuators — images shown to the
      human before `UPDATE_SNAPSHOTS=1` (ADR-0003).
- [x] **[2]** Step 9 — **the SDK.** `robot.actuators()`,
      `robot.actuator(name)`, `Actuator` gaining its `name` and `joint`;
      `Joint.actuator` reads the table, and its setter **raises** when the
      joint has several, naming `robot.actuators()`; `_riggen.pyi` and
      `docs/01-architecture.md`'s SDK table updated; `python/tests/sdk/`
      covers the new surface and `conftest.py`'s `joint["actuator"] = None`
      stripping goes.

## Acceptance
`assets/fixtures/menagerie_style.xml` — grown in step 7 — imported and
re-exported loads in MuJoCo with **zero compiler warnings**, agrees with
`fk` to 1e-6, and its `<actuator>` block names what the original's did:
the differently-named actuator keeps its name, and the joint with two
keeps both. `python/tests/test_mjcf_load.py::check_actuators` matches the
`--fk-samples` block element for element, and the `mujoco` job's
round-trip model stays held to the *original* document's `fk.json`. Every
fixture reopens: `pendulum.riggen` at v1, the new frozen v3 file through
the non-empty step, `bracket.riggen` and `arm/arm.riggen` byte-for-byte at
v4. `cargo test` green, snapshot suite included.

## Docs to update on completion
- `docs/02-data-model.md` §Core types — `Robot::actuators`, `Actuator`,
  `ActuatorTarget`; `Joint::actuator` removed from the `Joint` listing
- `docs/02-data-model.md` §Commands — the four commands and `Created::Actuator`;
  the `SetActuators` paragraph re-expressed over the table
- `docs/02-data-model.md` §ResolvedRobot — `ResolvedActuator`
- `docs/02-data-model.md` §Round trip table — the Actuator and
  Effort/velocity rows, on the actuator's own name
- `docs/02-data-model.md` §MJCF import — what `ActuatorDropped` is still for
- `docs/02-data-model.md` §Schema — schema 4, the first non-empty upgrade
  step, and the chain running on the JSON
- `docs/01-architecture.md` §SDK table and §`--fk-samples`
- `docs/adr/README.md` — the ADR-0023 row, ADR-0014's "amended by 0023"
- `docs/03-roadmap.md` §v0.4 — the actuator bullet marked *Landed*
- `AGENTS.md` current state — the file half's first line done

## Open questions
None left open. Three were asked at planning time and answered by the
human on 2026-09-07, each the recommendation; they are written into the
steps above and repeated here as the reasons. One finding was made while
executing.

- **Steps 3 and 5 are one step** (found 2026-09-07, executing step 3).
  Step 3's "compile everything downstream through by the shortest correct
  edit" is impossible for the two *editing* surfaces: the properties
  panel's actuator combo and the SDK's `Joint.actuator` setter both make a
  per-joint edit, and with `Joint::actuator` gone the only actuator command
  left is the whole-model `SetActuators`. The alternatives were a throwaway
  `SetJointActuator` command that step 5 would delete, or a UI that is
  read-only for one commit; both are worse than landing the quartet with
  the table it keys. Step 5's content is in step 3's commit.
- **`SetJoint` drops the joint's actuators when it retypes it to `Fixed`**
  — a delta the plan did not name. The panel used to clear
  `edited.actuator` itself when the kind combo went to `Fixed`; that line
  has nowhere to live now, and without it `validate` refuses the retype
  and the edit silently does nothing. The rule moves into the command,
  beside `RemoveLink` dropping the actuators of the joints it removes.

- **The schema-3 upgrade corpus is a small purpose-made document** beside
  `pendulum.riggen`, not a frozen copy of `arm/arm.riggen`. The arm is
  large, carries mesh assets whose hashes would have to stay valid
  forever, and is one of the byte-for-byte fixtures that must re-save at
  v4 — a frozen copy of it would drift against its living twin. A small
  file pins the migration entry by entry and stays readable in a diff.
  (Step 3.)
- **The properties panel lists every actuator targeting the joint**, one
  control per actuator, the existing combo being the one-actuator case.
  Showing the first with a note naming the rest re-creates exactly the
  feeling this cycle exists to remove: the user edits what the panel shows
  and never learns what it did not. (Step 8.)
- **The SDK's `Joint.actuator` setter raises** when the joint has several,
  and names `robot.actuators()` as the way through. Replacing them all
  would silently destroy what an imported file brought — the same loss
  step 7 closes on the import side, re-opened in Python. (Step 9.)
