# Plan: actuator-escape-hatch

- Started: 2026-09-07
- Milestone: v0.4 (§The file: nothing silently lost, second bullet)
- Idea (verbatim from the human): "plan" — on the recommendation that the
  escape hatch is the next thing to plan, the table it lands in having
  landed (ADR-0023) and the roadmap's own order putting `<general>` before
  the couplings

## Goal
An `<actuator>` that is not one of the three presets survives import →
export instead of being named in a warning and thrown away.
`ActuatorSpec` gains a fourth variant, `General` — MJCF's own actuator
model: `dyntype` / `gaintype` / `biastype` and their three `prm` vectors,
plus the `gear` a joint transmission scales by — and `Actuator` gains the
`ctrlrange` / `forcerange` / `ctrllimited` / `forcelimited` the file said,
which until now were re-derived from the joint on every write and so came
back out as different numbers than went in. `.riggen` is **schema 5**,
whose `upgrade_v4_to_v5` is empty for the same reason v1→v2 and v2→v3 are:
a v4 file has neither key and serde's defaults are what a v4 document
meant. The boundary stays where ADR-0023 drew it — **the target, not the
tag**: anything driving a joint comes in, anything driving a tendon, site
or body keeps its `ActuatorDropped` warning until the type it needs
exists. And the corpus stops being checked only by its warning list: it
joins the `mujoco` job, where the model MuJoCo builds from the original
file is compared with the one it builds from riggen's re-export, actuator
by actuator.

## Non-goals
- **A target that is not a joint.** `<general tendon=…>`, `<general
  site=…>`, `<adhesion body=…>` — `ActuatorTarget` keeps its one variant.
  A tendon has no document type until the couplings bullet; a body target
  is the first actuator no joint's properties panel could show, which is
  the Actuators window ADR-0023 declined to earn yet.
- **`<muscle>` and `<adhesion>`** — see the first open question. The
  recommendation is that both stay dropped and travel with the couplings
  bullet, which is where a tendon target and a written-back `<muscle>`
  belong together.
- **Authoring a `<general>` in the GUI.** The panel carries one and reads
  it back out; the three presets stay what the combo offers. ADR-0014's
  "a user who by definition already knows the XML" is still true — what
  changes is that riggen no longer *loses* their XML.
- **Writing a `<default class>`.** The class tree is resolved on import and
  dropped (01 §MJCF import): riggen writes flat, class-free MJCF and keeps
  doing so. The roadmap's "gains in a `<default class>`" is a *reading*
  obligation, and this plan proves it with a corpus that inherits actuator
  gains two levels up rather than by inventing a writer for classes.
- URDF and SDF: both still write no actuator and import none (ADR-0014,
  ADR-0016 §5). A `General` changes neither writer's comment beyond the
  preset name it prints.
- Anything on the cycle's "out" list: no fourth format, no SDF import.

## Design deltas

**ADR-0024** (step 1), amending ADR-0014's "Three presets, and only
three", which named `<general>`, the `prm` vectors, `<default class>`
gains and explicit `ctrllimited` / `forcelimited` as out and said why:
"hand-editing is the better answer **until MJCF import can read it back**".
It can now. Four things the ADR decides:

1. **`General` is a fourth `ActuatorSpec` variant, not a verbatim
   attribute blob.** The blob is cheaper and carries every attribute, but
   nothing could validate it, the SDK would hand back strings, and the
   document stops being the typed thing every other import vocabulary
   lands in (ADR-0015 §4). Typed also means the writer can trim what
   MuJoCo zero-fills, so a `gainprm="200"` round-trips as `gainprm="200"`
   and the document stays diff-friendly.
2. **The presets stay sugar and are not desugared into it.** MuJoCo
   defines `<position>` / `<velocity>` / `<motor>` as `<general>`
   shortcuts, so folding all four into one variant is tempting and wrong:
   the combo, the glyph's preset marks, the SDK's `Position` / `Velocity`
   / `Motor` and every golden would lose the name the user typed, to buy
   nothing the round trip needs. An element is read as a preset **iff
   every attribute it carries is one the preset can express**; otherwise
   it is a `General`. That rule is checkable and it keeps a document
   riggen alone has touched byte-identical to what it writes today.
3. **The ranges move onto the actuator, and `*limited` is a tri-state.**
   `Limits::effort` / `Limits::velocity` keep coming back to the *joint*,
   because URDF has nowhere else to read them; the actuator additionally
   keeps what the element said, and the writer prefers it. `Option<bool>`
   is exactly MuJoCo's `auto | true | false` under the `autolimits="true"`
   we write, and it is what lets an imported `<position>` with **no**
   `ctrlrange` come back out unlimited instead of clamped to the joint's
   range — today's silent change of meaning, which step 2 measures before
   step 3 fixes it.
4. **What a `<general>` carries, and what it counts.** Carried:
   `dyntype`, `gaintype`, `biastype`, `dynprm`, `gainprm`, `biasprm`,
   `gear` (the first of six, as `Motor` already reads it), and the four
   range fields. Counted and warned once per attribute name, the way an
   unread element already is (02 §Nothing is dropped silently):
   `actdim`, `actearly`, `actrange`, `lengthrange`, `cranklength`.
   `group` stays decorative and silent. The promise is bounded and
   stated, rather than "everything" and untrue.

`docs/02-data-model.md` **§Core types** — the fourth variant and the
ranges:

```rust
pub struct Actuator {
    pub name: String,
    pub target: ActuatorTarget,
    pub spec: ActuatorSpec,
    pub ranges: ActuatorRanges,   // what the file said; schema 5
}

/// `None` is "the writer derives it" — a riggen-authored actuator — and
/// `None` on a flag is MJCF's own `auto` under `autolimits="true"`.
pub struct ActuatorRanges {
    pub ctrl: Option<[f64; 2]>,
    pub force: Option<[f64; 2]>,
    pub ctrl_limited: Option<bool>,
    pub force_limited: Option<bool>,
}

pub enum ActuatorSpec {
    Position { kp: f64, kv: f64 },
    Velocity { kv: f64 },
    Motor    { gear: f64 },
    General(General),                 // schema 5
}

pub struct General {
    pub dyntype: DynType,             // None|Integrator|Filter|FilterExact|Muscle|User
    pub gaintype: GainType,           // Fixed|Affine|Muscle|User
    pub biastype: BiasType,           // None|Affine|Muscle|User
    pub dynprm: Vec<f64>,             // ≤ 10 each (mjNDYN / mjNGAIN / mjNBIAS)
    pub gainprm: Vec<f64>,
    pub biasprm: Vec<f64>,
    pub gear: f64,
}
```

**`ActuatorSpec` stops being `Copy`** — three `Vec`s in a variant — and
keeps `Clone` + `PartialEq`. That is a mechanical edit across
`properties.rs`, `glyphs.rs`, `fk_samples.rs`, `mjcf.rs` and
`riggen-py`, and it is why the variant lands in a step of its own.
`Vec` rather than `[f64; 10]`: the fixed array is MuJoCo's own shape and
would keep `Copy`, but it puts 240 bytes in one variant beside `Position`'s
16 — `clippy::large_enum_variant` under `-D warnings` — and it writes ten
numbers where the file wrote one.

`crates/riggen-core/src/validate.rs` — `check_actuators` gains, for a
`General`: every `prm` entry and `gear` finite (`NonFinite`), each vector
at most ten long (`ActuatorPrmTooLong`). The existing five refusals are
unchanged; `InvalidActuatorGain`'s "kp/kv ≥ 0, gear ≠ 0" is a statement
about the *presets* and does not extend to a `General`, whose gains are
MuJoCo's to interpret.

`crates/riggen-core/src/file.rs` — schema **5**, `upgrade_v4_to_v5` empty
with its reason, `ActuatorRanges` `#[serde(default)]` so a v4 actuator
deserialises. `bracket.riggen` and `arm/arm.riggen` re-save at 5;
`pendulum.riggen` (v1) and `driven.riggen` (v3) stay frozen as the
upgrade corpus.

`crates/riggen-export/src/resolve.rs` + `mjcf.rs` — `ResolvedActuator`
carries `ranges`; `write_actuator` writes `General` as `<general>` with
the three type names and the trimmed `prm` vectors, and for **every**
preset prefers `ranges` over the joint-derived numbers, emitting an
explicit `ctrllimited` / `forcelimited` whenever the field is `Some`.

`crates/riggen-export/src/fk_samples.rs` — `SampledActuator` gains the
same four range fields and the `general` kind. It keeps deriving them
from the `Robot` **independently** of `mjcf.rs`, which is the point of
the file (its own doc comment: "two statements of one intent"); the two
statements now both begin "the actuator's own, else the joint's".

`crates/riggen-export/src/mjcf_in.rs` — `read_actuators` reads a
`<general>` driving a joint into the new variant, records the effective
`ctrllimited` / `forcelimited` MuJoCo would compute, and counts the five
unread attributes. The target-attribute list gains `jointinparent`, so
an element that uses it is dropped naming *it* rather than "nothing".

`crates/riggen-app/src/app/panels/properties.rs` — a `General` actuator's
row, per the second open question.

`crates/riggen-py` + `python/riggen/robot.py` + `_riggen.pyi` — a
`General` spec class and `Actuator.ranges`.

`assets/fixtures/menagerie_style.xml` + `.github/workflows/ci.yml` — the
corpus joins the `mujoco` job (step 2) and grows the elements the ADR's
four points need (steps 2 and 6).

## Steps
Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[1]** Step 1 — **ADR-0024: the escape hatch beside the three
      presets.** The four decisions above, amending ADR-0014's
      three-presets-only clause on its own stated condition ("until MJCF
      import can read it back"). `docs/adr/README.md` gains its row and
      ADR-0014's row gains "amended by 0024". No code.
- [x] **[3]** Step 2 — **the corpus joins the `mujoco` job, and the round
      trip is compared actuator by actuator.** Nothing has ever run
      `menagerie_style.xml` through MuJoCo in CI: the Rust test pins its
      *warnings*, and the job's round trip is riggen's own arm export read
      back. This step imports the corpus, re-exports it, and adds
      `check_round_trip_actuators` to `test_mjcf_load.py` — the model
      MuJoCo builds from the **original file** against the one it builds
      from the re-export, per actuator: `trntype`, `trnid`, `dyntype`,
      `gaintype`, `biastype`, `dynprm`, `gainprm`, `biasprm`, `gear`. The
      four range fields are **not** compared yet; they arrive with the
      fields that make them agree (step 3). What riggen still drops is a
      **named allowlist in the test with its reason** — `lift` (a
      `<general>`, step 6) and `grip` (a tendon target, the couplings
      bullet) — and later steps empty it. Unproven twice over: whether the
      corpus re-exported loads at all with zero compiler warnings, and
      whether the three presets already agree field for field. Needs
      `arm/thing.msh` to become a real solid — it is a single triangle
      today and MuJoCo refuses fewer than four vertices, the caveat
      plans/actuator-table's acceptance run had to work around by hand.
      *Landed:* both unprovens held — the re-export loads clean and the
      presets agree field for field. The ranges do not, as designed: the
      check *reports* `pan` clamped to the joint's ±π `ctrlrange` and
      `pan_damp` given the joint's `forcerange` where the file had
      neither — step 3's measurement, taken. One finding: MuJoCo's own
      `LoadMSH` refused the triangle too ("invalid sizes in MSH file"), so
      the **original** corpus had never loaded either; the tetrahedron is
      what lets the comparison have an original at all.
- [x] **[2]** Step 3 — **`Actuator::ranges`, `ctrllimited` /
      `forcelimited`, schema 5.** The struct, the empty `upgrade_v4_to_v5`,
      the writer preferring the actuator's own numbers over the
      joint-derived ones and emitting the explicit flags,
      `fk_samples` stating the same rule in its own words, and
      `read_actuators` recording the effective flags MuJoCo would compute.
      `Limits::effort` / `Limits::velocity` keep being back-filled from
      the first actuator: URDF has nowhere else to read them. Step 2's
      check grows the four range fields, and a `<position>` that named no
      `ctrlrange` comes back out unlimited rather than clamped to the
      joint's range. The two byte-for-byte fixtures re-save at 5; the v1
      and v3 corpus files still open.
      *Landed.* One refinement to "records the effective flags": the
      import records a flag only where the writer's own
      `autolimits="true"` would not reproduce what the file meant — a
      written `true` / `false` as written, `auto` beside a range left
      `None`, and `Some(false)` where the file named **no** range (or had
      `autolimits` off). So a document riggen alone touched and a foreign
      `<position ctrlrange>` both come back out as today's one-attribute
      line, and only the actuator step 2 measured (`pan`, no `ctrlrange`)
      gains an explicit `ctrllimited="false"`. MuJoCo's own `auto` rule,
      probed rather than assumed, is *lower < upper*, not "not `0 0`";
      the joint's `limited` in `read_joint` still uses the latter, which
      differs only for an inverted range — noted, not touched. The
      `mujoco` job's six models pass locally with the four fields
      compared; the SDK suite and pyright are green.
- [x] **[2]** Step 4 — **`ActuatorSpec::General` in the document.** The
      variant, `General`, `DynType` / `GainType` / `BiasType`,
      `kind_name()` = `"general"`, `validate`'s two new refusals, and the
      `Copy`-to-`Clone` edit everywhere the enum is passed by value. No
      reader and no writer yet: the document can hold one, a hand-built
      test says `validate` accepts it and refuses a non-finite `prm` and
      an eleventh entry, and the step is revertible alone.
      *Landed.* The three type enums carry their MJCF spellings
      (`mjcf_name` / `from_mjcf` / `ALL`) and `General::MAX_PRM` here,
      being the types' own; `General::default()` is MuJoCo's bare
      `<general>`. Until step 5 the MJCF writer names a `General` in a
      comment rather than dropping it silently; the URDF and SDF comments
      print its three type names where a preset prints its gains; the
      panel's gain grid shows nothing for one and its combo can still
      replace it (step 7 adds the row).
- [x] **[2]** Step 5 — **the MJCF writer writes `<general>`.** The tag,
      the three type names, the `prm` vectors with MuJoCo's zero-fill
      trimmed off the end, `gear`, and the ranges from step 3. The golden
      gains a `<general>` case beside the three presets; the apologetic
      comment (ADR-0004 §4) is unaffected — a `General` drives the joint
      like any other actuator.
      *Landed.* The trim keeps at least one entry: `dynprm="0"` is zero
      where an absent `dynprm` is MuJoCo's default of one, checked against
      MuJoCo rather than assumed. The `--fk-samples` block gains a
      `general` sub-block (the three type names, the three vectors) so
      `check_actuators` holds a riggen-authored `<general>` to its model
      too, padded to MuJoCo's ten; the golden's fourth case sits beside
      the three presets in `each_preset_writes_its_element…`, not in the
      shared `GOLDEN` constant, which the importer still reads as
      preset-only until step 6.
- [x] **[2]** Step 6 — **the MJCF import reads a `<general>` driving a
      joint.** The preset rule of ADR-0024 §2 — a preset iff every
      attribute fits it, else a `General` — the five counted attributes,
      and `jointinparent` named in the drop message. The corpus grows an
      `<actuator>` whose gains are inherited from a `<default class>` two
      levels up, and one with an explicit `ctrllimited="false"` beside a
      range; the warning test loses the `lift` line and step 2's allowlist
      loses `lift`, leaving `grip` and its reason. This is where the
      round-trip check first has something to prove.
      *Landed.* `lift` moved from `shoulder_lift` — a mimic follower, so
      `validate` would have refused it the moment it was read — to
      `wrist_slide`, with its gains through `class="arm_drive"` → `arm`.
      The preset rule desugars a `<position gear|timeconst>` and a
      `<velocity gear>` into MuJoCo's own reading (fixed gain, affine
      bias, an exact filter); MuJoCo's other shortcuts (`<intvelocity>`,
      `<damper>`, `<cylinder>`) are named once and dropped like
      `<muscle>`. One bound stated in 02 §MJCF import: a
      `<default><general>` applies to `<general>` elements only, while
      MuJoCo also lets a preset inherit it. Two names step 5's Python
      check shadowed (`want`) are fixed here; the count it printed was
      wrong, the verdict was not. The `properties_joint_two_actuators`
      snapshot moved — the slide's glyph gains its ring and a `general`
      mark, the status bar one warning fewer — image shown.
- [ ] **[2]** Step 7 — **the app.** The properties panel's actuator list
      shows a `General` per the second open question; `glyphs.rs`'s
      `driven_marks` prints `general` beside the preset names it already
      prints, and the amber of an actuated joint is unchanged;
      `debug_state()` reports it. One new snapshot scenario for a joint an
      imported file gave a `<general>`, and the two existing actuator
      scenarios refreshed only if they move — images shown to the human
      before `UPDATE_SNAPSHOTS=1` (ADR-0003).
- [ ] **[2]** Step 8 — **the SDK.** A `General` spec class beside
      `Position` / `Velocity` / `Motor`, `Actuator.ranges`, `_riggen.pyi`
      and `docs/01-architecture.md`'s SDK table; `python/tests/sdk/`
      covers building one, round-tripping it through `to_json`, and
      exporting it. `Joint.actuator` reads a `General` back like any other
      spec.

## Acceptance
The cycle's own, one bullet earlier than it will finally be run:
`assets/fixtures/menagerie_style.xml`, imported and re-exported, loads in
MuJoCo with **zero compiler warnings**, agrees with `fk` to 1e-6, and its
`<actuator>` block *is the original's* — `check_round_trip_actuators`
compares the two models element for element over `trntype`, `trnid`,
`dyntype`, `gaintype`, `biastype`, `dynprm`, `gainprm`, `biasprm`, `gear`,
`ctrlrange`, `forcerange`, `ctrllimited` and `forcelimited`, and its
allowlist is down to `grip`, the tendon target the couplings bullet owns.
No `ActuatorDropped` warning is left for anything driving a joint. The
`mujoco` job's round-trip model stays held to the *original* document's
`fk.json`. Every fixture reopens: `pendulum.riggen` at v1,
`driven.riggen` at v3, `bracket.riggen` and `arm/arm.riggen`
byte-for-byte at v5. `cargo test` green, snapshot suite included; `uvx
pyright` clean; `python/tests/sdk` green.

## Docs to update on completion
- `docs/02-data-model.md` §Core types — `ActuatorSpec::General`, `General`,
  the three type enums, `Actuator::ranges` / `ActuatorRanges`, and the note
  that `ActuatorSpec` is no longer `Copy`
- `docs/02-data-model.md` §Invariants — the two `General` refusals, and why
  `InvalidActuatorGain` does not extend to one
- `docs/02-data-model.md` §ResolvedRobot — `ResolvedActuator::ranges`
- `docs/02-data-model.md` §Round trip table — the Actuator row: `<general>`
  out as well as in, and the ranges reading the actuator before the joint
- `docs/02-data-model.md` §MJCF import — what `ActuatorDropped` is still
  for (a tendon, site or body target, and nothing else), the preset-iff
  rule, and the five counted attributes
- `docs/02-data-model.md` §Schema — schema 5 and its empty upgrade step
- `docs/01-architecture.md` §SDK table and §`--fk-samples`
- `docs/adr/README.md` — the ADR-0024 row, ADR-0014's "amended by 0024"
- `docs/03-roadmap.md` §v0.4 — the escape-hatch bullet marked *Landed*
- `docs/BACKLOG.md` — whatever the first open question leaves behind
- `AGENTS.md` current state — the file half's second line done

## Open questions
- **Resolved by the human at step 6 (2026-09-08): `<muscle>` and
  `<adhesion>` are out**, on the recommendation below; both travel with
  the couplings bullet. Original question: **`<muscle>` and `<adhesion>`
  — in or out?** The roadmap bullet names both. Recommendation: **out**, and the reasons
  are different for each. `<adhesion>` drives a *body*, so it is already
  out by ADR-0023's target rule with no special case, and the body target
  is the first one no joint's panel can show — the Actuators window
  question, not this plan's. `<muscle>` is a tendon-length model: on a
  joint it is rare, and writing one back as `<general dyntype="muscle">`
  means MuJoCo must compute an `actuator_lengthrange` we did not write,
  which can fail the load outright. Both belong with the couplings bullet,
  where a tendon exists and a `<muscle>` can be written back as a
  `<muscle>`. If the human says in, `<muscle joint=…>` is a pure attribute
  map into `General` and step 2's check is exactly what would catch the
  `lengthrange` problem before it lands.
- **Resolved by the human at step 6 (2026-09-08): a read-only row**, on
  the recommendation below. Original question: **what the properties
  panel does with a `General`.** Recommendation: **a read-only row** — the actuator's name, the
  word `general`, and its three type names — with the combo still able to
  remove it or replace it with a preset, which is a deliberate and visible
  act. Editing ten `prm` numbers in a panel is MJCF's actuator model in a
  properties grid, which ADR-0014 rejected on its merits and this plan
  does not reopen; showing nothing at all re-creates the loss the cycle
  exists to remove — the user would see a joint the file drives and no
  sign of what drives it.
- **Resolved in step 2 (on the recommendation, unasked): `arm/thing.msh`
  is a tetrahedron**, regenerated by `msh.rs`'s ignored
  `write_thing_msh_fixture` and pinned to its generator like `cube.msh`.
  It had to be: MuJoCo's own `.msh` reader refuses the three-vertex file,
  so the *original* corpus could not be loaded for comparison, not only
  the re-export. Mesh bytes only — every pose in that model is written
  explicitly. If the human wants the fixture left a triangle, this step's
  comparison has no original to run against and the plan's acceptance
  falls with it.
