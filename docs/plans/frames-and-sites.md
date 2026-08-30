# Plan: frames-and-sites

- Started: 2026-08-31
- Milestone: v0.2 (`docs/03-roadmap.md` §v0.2, "Named frames / MJCF sites")
- Idea (verbatim from the human): "plan next item from roadmap"

## Goal

A **named frame** — a TCP, a sensor mount, a grasp pose — is a first-class
child of a link: created in the app, posed with the same gizmo and snapping
that place a joint, listed in the tree under its link, saved in `.riggen`
(the `frames` map has been in the schema since M1 and is finally non-empty),
read and written from the Python SDK, and exported as an MJCF `<site>` and
a URDF massless dummy link on a fixed joint. CI proves the number is right
the way M3 proved joints: `riggen --export --fk-samples` writes each frame's
world pose per configuration and MuJoCo's `mj_forward` site poses match it
to 1e-6.

## Non-goals

- Anything that *references* a site: sensors, actuators, equality
  constraints, cameras, `<touch>`/`<force>`. A frame is a pose with a name.
- Mimic joints and actuator presets — the other two items on the same
  roadmap bullet; they stay backlog lines.
- MJCF import, SDF export.
- Turning arbitrary massless URDF links back into frames on import
  (decision 1) — the import keeps them as links.
- Frames as a snap source for *other* frames, or frame-relative geom poses.
  A frame's parent is a link, always.

## Design deltas

- **`docs/02-data-model.md` §Core types** — `Frame { name, parent, pose }`
  stays as it is; its "post-MVP, always empty" notes go. `Robot::frames` is
  live.
- **§Commands** — three new: `AddFrame { frame: Frame }` (allocates the
  `FrameId`, returns it), `RemoveFrame(FrameId)`, `SetFrame(FrameId, Frame)`
  (name, parent and pose in one value, the properties panel's commit) plus
  `RenameFrame(FrameId, String)` for the tree's inline rename, mirroring
  `RenameLink` / `RenameJoint`. `RemoveLink`, `MoveJointFrame` and
  `Reparent` already carry frames (`command.rs:223`, `:281`) — this plan
  adds the tests that pin that.
- **§Invariants** — frame names are unique among frames, valid XML names,
  and — because the URDF writer turns each one into a `<link>` — must not
  collide with a link name (decision 5). `DanglingFrameLink` already exists.
- **§Kinematics** — `fk::frames(robot, &JointState) -> BTreeMap<FrameId,
  Pose>`, one `world(parent) ∘ frame.pose` per frame over an `fk` result;
  `fk` itself keeps returning links only (decision 4).
- **§`ResolvedRobot`** — `ResolvedLink::sites: Vec<ResolvedSite { name,
  pose }>`, filled by `resolve` in `FrameId` order. No new `ExportError`
  beyond the validation ones.
- **§Format mapping** — a new row. MJCF: `<site name pos quat/>` inside the
  body, at the class defaults (decision 3). URDF: a `<link name="tcp"/>` with
  no visual, collision or inertial, plus `<joint name="tcp_fixed"
  type="fixed">` from the parent — the ROS convention, so `tf` and MoveIt
  see the frame.
- **§URDF import** — one sentence recording that the asymmetry is
  deliberate (decision 1).
- **`docs/01-architecture.md` §Panels and menus** — the tree gains a frame
  row under its link; `Selection::Frame(FrameId)`; the properties panel
  gains a frame section; a frame glyph (triad + name label) in the
  viewport; Move/Rotate and the snap ladder target a frame.
- **`docs/01-architecture.md` §Python SDK** — a `Frame` handle beside
  `Link` / `Joint` / `Geom`: `Link.add_frame(name, pose)`,
  `Link.frames()`, `Robot.frames()`, `Robot.frame(name)`, `frame.pose`,
  `frame.name`, `frame.parent`, `frame.remove()`, `frame.world(q)`.
- **ADR-0012 — how a frame reaches each format.** Why MJCF gets a `<site>`
  and URDF a massless dummy link on a fixed joint; why a URDF import does
  not reverse the second (a massless childless link is not
  distinguishable from a real one, and guessing would silently delete
  links); the name-collision rule that falls out of it. Written in step 2,
  with the code it justifies.

## Steps

- [x] **Step 1 — `resolve` carries sites.** `ResolvedSite`,
  `ResolvedLink::sites`, filled in `FrameId` order; frames on a link
  removed from the tree cannot exist (validation). Unit tests in
  `resolve.rs` for order, for a frame on the root, and for a document with
  no frames resolving byte-identically to today.
- [x] **Step 2 — both writers, and ADR-0012.** MJCF `<site>` inside the
  body after its geoms; URDF dummy link + fixed joint, emitted after every
  real link so the file still reads root-first. Golden-XML tests in
  `mjcf.rs` and `urdf.rs` extended with a two-frame fixture; the
  name-collision rule from decision 5 enforced in `validate` with its own test.
  ADR-0012 in the same commit.
- [ ] **Step 3 — the CI acceptance.** `fk_samples` writes `sites: {name:
  {pos, quat}}` beside `links` per sample; `test_mjcf_load.py` sets each
  configuration and compares `data.site(name)` to 1e-6, and fails a file
  whose `fk.json` has sites the model does not; the arm fixture generator
  (`write_arm_fixtures`) gives `arm.riggen` a `tcp` frame on the last link
  and a `camera_mount` on the base, so `assets/fixtures/arm/arm.urdf` gains
  the dummy links too and the existing `mujoco` job covers both routes.
  **This step retires the risk**: MuJoCo agreeing with our site poses.
- [ ] **Step 4 — core commands.** `AddFrame`, `RemoveFrame`, `SetFrame`,
  `RenameFrame` in `command.rs`; `fk::frames`; tests for each, plus the
  three that pin existing behaviour (`RemoveLink` takes the subtree's
  frames — already there —, `MoveJointFrame` re-expresses them, `Reparent`
  leaves them on their link). A `.riggen` corpus fixture with a frame,
  saved and reopened byte-for-byte; `schema_version` stays 1.
- [ ] **Step 5 — the tree knows frames.** `Selection::Frame(FrameId)`
  through the ~33 match sites; a frame row under its link (indented, its
  own icon, click selects, F2 / double-click renames, Delete removes);
  hovering a row highlights its glyph and vice versa, as joints do. The
  frame glyph in `glyphs.rs`: a small triad in the triad colours plus the
  name as a label, drawn for every frame, brightened when selected or
  hovered. Snapshot `frames_tree`.
- [ ] **Step 6 — the properties panel edits a frame.** Name, parent link
  (a combo — changing it keeps the world pose, decision 2), xyz + RPY in
  degrees through the same draft-buffer fields the link pose uses, all
  committed as one `SetFrame`; "+ Frame" in the tree header adds one at
  the selected link's origin and starts the inline rename. Snapshot
  `frame_properties`.
- [ ] **Step 7 — place a frame with the mouse.** The Move and Rotate
  gizmos take `Selection::Frame` (`GizmoTarget::Frame`, drag previews,
  release commits one `SetFrame`), and the snap ladder — pick point,
  vertex, AABB corner, face normal — works under them, so a TCP lands on
  a picked feature without a coordinate typed. Snapshot
  `gizmo_move_frame`.
- [ ] **Step 8 — the SDK.** `_riggen` gains `add_frame` / `remove_frame` /
  `set_frame`; `python/riggen/robot.py` gains the `Frame` handle and the
  `Link` / `Robot` accessors above; `_riggen.pyi` updated; pytest cases in
  `test_document.py` and `test_api.py` (add, move, remove, export and read
  the site back out of the MJCF), pyright clean. `examples/arm.py` gains
  the TCP so the wheel job's MuJoCo load covers it.

## Acceptance

```sh
cargo test --workspace                     # golden XML, resolve, commands, snapshots
cargo run -p riggen-app -- --export mjcf --fk-samples --out target/sample \
    assets/fixtures/arm/arm.riggen
cargo run -p riggen-app -- --export mjcf --fk-samples --out target/sample-urdf \
    assets/fixtures/arm/arm.urdf
uv run --no-project --with mujoco --with numpy \
    python python/tests/test_mjcf_load.py target/sample target/sample-urdf
```

The arm exports with a `tcp` and a `camera_mount`; MuJoCo loads it with zero
compiler warnings; every **site** pose from `mj_forward` matches
`riggen_core::fk` at all five sampled configurations to 1e-6, over both the
`.riggen` and the URDF-import route; the SDK suite and pyright pass in the
`wheel` job; `frames_tree`, `frame_properties` and `gizmo_move_frame`
snapshots pass on the CPU adapter.

## Docs to update on completion

- `docs/02-data-model.md` §Core types — drop "post-MVP; always empty" from
  `Robot::frames` and `Frame`.
- `docs/02-data-model.md` §Commands — the four new commands in the
  `Command` enum listing and the prose after it.
- `docs/02-data-model.md` §Kinematics — `fk::frames`.
- `docs/02-data-model.md` §`ResolvedRobot` — `ResolvedSite`,
  `ResolvedLink::sites`.
- `docs/02-data-model.md` §Format mapping — the frame row (MJCF `<site>`,
  URDF dummy link + fixed joint), and the name-uniqueness rule under
  §Invariants.
- `docs/02-data-model.md` §URDF import — the one sentence on the
  deliberate asymmetry.
- `docs/01-architecture.md` §Panels and menus — the frame row, the frame
  glyph, the gizmo target.
- `docs/01-architecture.md` §Python SDK — the `Frame` handle.
- `docs/adr/0012-frames-as-mjcf-sites-and-urdf-dummy-links.md` — new
  (written in step 2, not at retirement).
- `docs/03-roadmap.md` §v0.2 — a "Done so far" entry; the "Named frames /
  MJCF sites" half of the still-open bullet removed.
- `docs/BACKLOG.md` — remove "Named frames / MJCF sites (TCP, sensor
  mounts)"; keep the mimic-joints line.
- `README.md` — the feature list gains frames if it names joints (check at
  retirement).
- `AGENTS.md` current state — one line, the oldest milestone line dropped
  to stay under ~15.

## Decisions

Taken at planning time; the human agreed to all five on 2026-08-31, so no
step is blocked. Decision 1 is the one ADR-0012 records — the others are
implementation shape and live here only.

1. **A URDF import does not turn a massless childless link back into a
   frame.** Nothing distinguishes a real massless link (a bracket the user
   has not weighed) from our dummy, and guessing wrong deletes a link on
   re-export. Round-tripping our own file therefore gains two links, which
   is the accepted cost. → ADR-0012, step 2.
2. **`SetFrame` may change `parent`, and the panel keeps the world pose.**
   The panel computes the new pose through `fk` before committing, so the
   command stays dumb and writes what it is given, like `SetJoint`. → step 6.
3. **An MJCF site is written bare**: `<site name pos quat/>`, no size or
   group. MuJoCo's default 0.005 m sphere renders as a visible dot in
   `mujoco.viewer` — a TCP marker one can see is a feature, and an override
   is a `<default>` the user adds. → step 2.
4. **`fk()` keeps returning links only**; frames get `fk::frames` (and
   `frame.world(q)` in the SDK), because `fk`'s `BTreeMap<LinkId, Pose>` is
   the export oracle and the round-trip test's contract. → step 4.
5. **One namespace.** `validate` requires a frame name to be unique among
   frames *and* distinct from every link name: sites and bodies are
   separate namespaces in MJCF but both are `<link>` in URDF, and renaming
   behind the user's back at export time is worse than one rule checked
   once. → step 2.

## Open questions

- **Found in step 2, decided there:** decision 5 closes the frame-vs-link
  collision but not the one it creates itself — the URDF writer's
  `<frame>_fixed` joint can land on a real joint's name, and two `<joint
  name="grip_fixed">` is an invalid URDF from a document that validated.
  `validate` now rejects that too (`FrameJointNameCollision`), same rule,
  same reason: refuse at the moment the name is typed rather than rename
  behind the user's back. Recorded in ADR-0012.
