# Plan: mjcf-import

- Started: 2026-08-31
- Milestone: v0.2 (`docs/03-roadmap.md` § "Still open" — first half of
  "MJCF import; SDF export")
- Idea (verbatim from the human): "MJCF import; SDF export"

## Goal

An MJCF file opens as a document by every route a URDF already does —
`riggen robot.xml`, File › Import MJCF…, a dropped `.xml`, `riggen --export
… robot.xml`, `riggen.load_mjcf()` — through a new
`riggen_export::mjcf_in::load(path) -> Result<(Robot, Vec<ImportWarning>),
ImportError>`. Our own exported MJCF round-trips **exactly**: links, joint
kinds, axes, limits, dynamics, inertials, meshes, `<site>` → `Frame`
(ADR-0012's promised symmetry), `<equality><joint polycoef>` →
`Joint::mimic` (ADR-0013), `<actuator>` → `ActuatorSpec` (ADR-0014). A
model written by somebody else — Menagerie-shaped: `<default>` classes with
`childclass`, `angle="degree"`, `euler` / `axisangle` / `xyaxes` / `zaxis`
orientations, `meshdir` — imports too, with an `ImportWarning` naming every
element the document cannot hold and an `ImportError` for the shapes it
must refuse. The `mujoco` CI job gains a fourth route: export the arm,
import it back, export it again, and MuJoCo agrees with `fk` on the result.

## Non-goals

- **SDF anything** — the other half of the roadmap line is
  `docs/plans/sdf-export.md`.
- **`<include>`, `<replicate>`, `<attach>`** — a file that composes other
  files is an `ImportError`, not a resolver we write.
- Everything MJCF has that the document has no field for: tendons,
  sensors, cameras, lights, `<contact>`, `<keyframe>`, `<flag>`/`<option>`,
  hfields, skins, composites, `<general>` / `<muscle>` / `<adhesion>`
  actuators, `solref`/`solimp`, friction and mass-from-geom. Each is a
  warning that names it; none is read.
- `.msh` meshes and inline `<mesh vertex face>` — a warning, no geometry.
- Promoting `Joint::actuator` to `Robot::actuators` (a backlog line since
  ADR-0014; an actuator whose target is not a joint is a warning here).
- No new UI beyond the menu item and the status-bar warnings the URDF
  import already shows.

## Design deltas

- **`riggen-export/src/xml.rs` grows a reading half** (or a sibling
  `xml_in.rs`): a ~60-line read-only DOM — element name, attributes,
  children — over `quick-xml`, which is **already in the tree** at 0.36
  under `urdf-rs`, so the workspace gains a `[dependencies]` line and no
  compiled crate. Beside it, MJCF's five orientation spellings collapsing
  to one `DQuat` (`quat`, `euler` under `<compiler eulerseq angle>`,
  `axisangle`, `xyaxes`, `zaxis`) — the mirror of `xml::quat_wxyz`, and the
  same "one place, tested" rule.
- **`riggen-export/src/mjcf_in.rs`**, the module the plan is about, beside
  `urdf_in.rs`. `ImportWarning` and `ImportError` are the **same two enums**
  the URDF import uses, grown MJCF variants — the app's status bar, the
  SDK's `RiggenWarning` and the CLI's stderr already speak them, and one
  import vocabulary is what ADR-0015 decides.
- `docs/02-data-model.md` gains **§MJCF import** beside §URDF import, and
  §Format mapping's table gets an "read back as" note wherever the two
  directions differ.
- `docs/01-architecture.md` §Crates (line 75, the tree comment) and §Routes
  in (line 583, 624–627) name `mjcf_in` and the `.xml` extension.
- **ADR-0015** — the subset MJCF import reads, the parser, the shared
  import vocabulary, and the shapes that are errors rather than warnings.
  Listed as step 1.

## Steps

- [x] 1 — **ADR-0015: what MJCF import reads.** The subset (the non-goals
  above as a table), `quick-xml` over a new dependency, one shared
  `ImportWarning` / `ImportError` vocabulary with the URDF import, the
  `<default>` tree resolved at import rather than stored, and which shapes
  are `ImportError` (`<include>`, several `<joint>`s in one body — see
  OPEN 1, several `<worldbody>` bodies, a body cycle). Docs-only commit;
  the ADR README row lands with it.
- [x] 2 — **The reader half of `xml.rs`.** `xml::parse(&str) -> Result<Node,
  ParseError>`; attribute helpers for `f64`, `[f64; N]`, `DVec3`; the five
  orientation spellings → `DQuat`. Test: every spelling of one rotation
  agrees to 1e-12, `angle="degree"` flips them, and re-parsing the golden
  of `mjcf::tests::GOLDEN` yields the elements the writer wrote. *Retires
  the parsing-and-rotation unknown before anything depends on it.*
- [x] 3 — **`<compiler>` and the `<default>` tree.** A `Defaults` keyed by
  class with `childclass` inheritance, resolving any element's effective
  attributes; `angle`, `eulerseq`, `meshdir`, `assetdir`, `autolimits`.
  Test: a hand-written file with nested defaults and a `childclass` on a
  body resolves each `<geom>` and `<joint>` to the attribute set MuJoCo
  would.
- [x] 4 — **`<worldbody>` → the link tree.** Bodies → `Link`s, the body
  pose → the parent joint's `origin`, one `<joint>` → `Joint` (`hinge`
  with `range` → `Revolute`, without → `Continuous`, `slide` →
  `Prismatic`, none → `Fixed`), `range`/`damping`/`frictionloss`/
  `armature` → `Limits`/`Dynamics`, `<inertial>` (`fullinertia`, or
  `diaginertia` + `quat`) → `InertialSpec::Override`, no `<inertial>` →
  `NoInertial`, `<freejoint>` → a warning (`floating_base` is an export
  option, not a document field). Test: `every_joint_kind`'s exported MJCF
  imports to a `Robot` whose `fk` matches the original at five
  configurations.
- [x] 5 — **`<asset><mesh>` and `<geom>`.** Mesh assets by name → `MeshAsset`
  (file through `meshdir`, uniform `scale`, `MeshNotFound` when absent),
  the visual / collision split (OPEN 2), mesh geoms → `Geom`s, primitives →
  `CollisionPolicy::Primitives` with the half-extents undone, `plane` /
  `ellipsoid` / `hfield` / a primitive visual → warnings, `<site>` →
  `Frame` on its body (ADR-0012). Test: the arm's own export comes back
  with the same geoms, and `bracket`'s decomposition comes back as N
  collision meshes.
- [x] 6 — **`<equality>` and `<actuator>`.** `<joint polycoef="o m 0 0 0">`
  → `Joint::mimic` (a non-zero `a2..a4`, a `ref`, or a coupling `validate`
  refuses → `MimicDropped` with the reason, as the URDF import phrases it);
  `<position kp kv>` / `<velocity kv>` / `<motor gear>` on a joint →
  `ActuatorSpec`, everything else → `ActuatorDropped`. Test: the arm's
  mimic and both actuators survive the round trip; a `<general>` and an
  actuator on a site are each warned about by name.
- [ ] 7 — **Every route in.** `mjcf_in::load`; File › Import MJCF… and a
  dropped `.xml` in `app/file_io.rs`; `.xml` input in `cli.rs`;
  `Robot.load_mjcf` in `riggen-py` and `riggen.load_mjcf` in the public
  layer, with `_riggen.pyi` and an SDK test. Snapshot: the File menu with
  the new item.
- [ ] 8 — **The round trip in CI, and the foreign corpus.**
  `assets/fixtures/menagerie_style.xml` — hand-written, defaults +
  `childclass` + degrees + every orientation spelling + a `<general>` — with
  a Rust test asserting its warnings by name. The `mujoco` job gains
  `target/sample-mjcf-in`: export the arm to MJCF, import that `.xml`,
  export it again with `--fk-samples`, and `test_mjcf_load.py` holds the
  result to the same poses, sites, `mjEQ_JOINT` and `model.nu` as
  `target/sample`.

## Acceptance

`cargo test --workspace` green, and the `mujoco` CI job's new route:

```sh
riggen --export mjcf --out target/sample assets/fixtures/arm/arm.riggen
riggen --export mjcf --fk-samples --out target/sample-mjcf-in target/sample/arm.xml
uv run --no-project --with mujoco --with numpy \
  python python/tests/test_mjcf_load.py target/sample-mjcf-in
```

loads with zero MuJoCo compiler warnings, and every body pose, site pose,
`mjEQ_JOINT` row and `model.nu` matches `target/sample`'s own `arm.fk.json`
to 1e-9 — the exact round trip ADR-0012 promised. Plus
`mjcf_in::tests::menagerie_style_imports_with_the_warnings_it_should`.

## Docs to update on completion

- `docs/02-data-model.md` — new **§MJCF import** (the `load` signature, the
  subset, the warning list, the corpus fixtures); §Format mapping gains the
  read-back column note; §URDF import's closing paragraph gains a sibling
  sentence pointing at it.
- `docs/01-architecture.md` — §Crates tree comment (`mjcf_in` beside
  `urdf_in`), §Routes in (`.xml` opens as a new untitled document),
  §Python SDK's table (`load_mjcf`).
- `docs/adr/README.md` — the ADR-0015 row; ADR-0012's "once MJCF import
  exists" and ADR-0014's "until MJCF import can read one back" are left
  as written (ADRs are append-only) — ADR-0015 states that it closes them.
- `docs/03-roadmap.md` — the v0.2 "Still open" line loses "MJCF import"
  and a done paragraph is added.
- `docs/BACKLOG.md` — remove the two lines that were waiting on this
  (`<general>` hand-editing, the `Robot::actuators` promotion) or re-word
  them to what is still true.
- `AGENTS.md` — current state, one line.
- `README.md` — the import sentence and `--export`'s `INPUT` description.

## Findings while executing

- **Step 4, `<joint pos>`.** MJCF anchors a joint at `pos` in the *body*
  frame; the document's joint frame **is** the child link frame. Neither
  the plan nor ADR-0015 said what to do. Ignoring it changes the
  kinematics, which ADR-0015 §5 forbids, and refusing it would refuse most
  of Menagerie, so the import **rebases**: the child link frame is the body
  frame moved to the anchor, the parent joint's `origin` carries the move,
  and everything inside the body (the inertial CoM, the child bodies' poses,
  and the geoms and sites of step 5) is re-expressed by subtracting it.
  Our own writer emits no `pos`, so the round trip is untouched.
- **Step 4, `<frame>`.** MuJoCo 3's grouping element is a transform wrapper
  whose child bodies are nobody's children. Reading around it would lose
  them silently, so it joins `<include>` / `<replicate>` / `<attach>` in
  `ImportError::UnsupportedElement` — ADR-0015 §5's rule applied to an
  element the ADR did not enumerate.
- **Step 4, `<joint ref>`.** It moves a joint's zero, which the document has
  no field for, so it is warned as an `ElementDropped` — the "an attribute
  that changes the robot rather than decorating it" exception of ADR-0015
  §1, on an attribute that section's list did not name.
- **Step 4, an unlimited `slide`.** The document has no unlimited
  `Prismatic` (`validate` requires finite limits), so one gets ±1 m and a
  new `ImportWarning::LimitsInvented` that names it.
- **Step 4, `Limits::effort` / `velocity` do not round-trip on an
  unactuated joint.** MJCF keeps them on the `<actuator>` (ADR-0004 §4 as
  amended by ADR-0014), so a joint with one gets them back from
  `forcerange` / `ctrlrange` in step 6 and a joint without one — where the
  writer emits only the "not written" comment — cannot. The Goal's "limits
  round-trip exactly" is true of `lower`/`upper` and of `effort`/`velocity`
  only where an `<actuator>` carries them; §MJCF import says so. The CI
  acceptance is unaffected: MuJoCo sees neither number. Step 6 narrowed it
  further: `effort` comes back from `forcerange` on any preset, but
  `velocity` only from a **velocity** servo's `ctrlrange` — a position
  servo's `ctrlrange` is the joint's position range and says nothing about
  rate.
- **Step 6, the order of the two refusal passes.** `mimic_refusals` runs
  before `actuator_refusals`: an actuator on a mimic follower is refused
  because the `<equality>` drives it, so a coupling that is itself dropped
  must not take an actuator down with it.

- **Step 5, `CollisionPolicy::None` vs `SameAsVisual`.** ADR-0015 §6's
  third step (everything is a visual, the link is `SameAsVisual`) is for a
  file that never distinguished. A link in a file that *did* — by class or
  by `contype` — and has nothing on the collision side means
  `CollisionPolicy::None`, so our own round trip keeps it. Which of the two
  rules applies is decided once per link, not per geom: in a file that uses
  `contype`, a geom that omits it collides at MuJoCo's default.
- **Step 5, `<geom fromto>`** names the two ends of a cylinder or capsule
  and replaces its pose; Menagerie is full of it, and it is read.

## Open questions

- ⚠ OPEN 1: **Several `<joint>`s in one `<body>`** — MuJoCo's way of
  spelling a ball or planar DoF, and Menagerie is full of them; the
  document has one joint per tree edge. Refuse the file
  (`ImportError::CompositeJoint`), or synthesise massless intermediate
  links so the chain imports and re-exports identically? *Human decides, by
  step 4.* Agent's recommendation: refuse in this plan and put the
  synthesis in the backlog — a synthesised link is a link the user did not
  draw, and ADR-0006's "a drop is a link" rule says the tree is the user's.
  **Decided (step 1, human): refuse** — `ImportError::CompositeJoint`,
  with `JointOnRoot` and a non-root `type="free"` beside it; ADR-0015 §5
  carries the reasoning, and the synthesis is a backlog line.
- ⚠ OPEN 2: **The visual / collision split** for a file that uses neither
  our `class="visual"` / `class="collision"` nor `contype`/`conaffinity`.
  *Human decides, by step 5.* Agent's recommendation: our two class names
  first; else `contype == 0 && conaffinity == 0` is visual and everything
  else collision; else every geom is a visual and the link is
  `SameAsVisual` — never a silent loss of geometry.
  **Decided (step 1, human): as recommended**, ADR-0015 §6.
- ⚠ OPEN 3: **A body whose `<geom>`s are its only mass** (`<inertial>`
  absent, MuJoCo's `inertiafromgeom`). Warn and leave `Computed` — which
  needs a material the imported document has not got — or compute the
  inertial at import from the meshes we just loaded? *Agent decides, by
  step 4*, unless it turns out to change the round trip.
  **Decided (step 1, agent): warn and leave `Computed`**, as the URDF
  import already does — ADR-0015 §7. MuJoCo's fallback density is a number
  the user never chose, and the round trip is untouched because our writer
  emits an `<inertial>` for every link that has mass.
