# Plan: m1-document-tree-joints

- Started: 2026-08-29
- Milestone: M1
- Idea (verbatim from the human): "plan m1"

## Goal

A two-link pendulum you can save, reopen, and swing. `riggen-core` holds the
`Robot` document of 02-data-model with `validate`, `fk`, a snapshot
`History`, and `.riggen` v1 serde (relative mesh paths, content hash).
`riggen-app` owns one `Robot` and derives everything else from it: the
viewport draws one instance per `(LinkId, GeomId)` visual at the FK pose for
the current joint values. A link-tree panel (add / remove / rename /
reparent by drag), a properties panel with numeric xyz + RPY entry
(degrees in the UI, radians in the document), a joint-sliders window that
swings the robot within its limits, a materials table with density and a
per-link material, New / Open / Save / Save As with a dirty marker and an
unsaved-changes confirm, and Ctrl+Z / Ctrl+Shift+Z / Ctrl+Y undo/redo. Every
visible change has a snapshot scenario; `fk` is unit-tested against
hand-computed poses for a 3-joint chain.

## Non-goals

- Gizmos, snapping, joint glyphs in the overlay, tree↔3D hover highlight (M2).
- Inertials, mass properties, collision policy UI, export, URDF import (M3).
  `Link.inertial` and `Link.collision` exist in the schema with their
  defaults (`Computed { density_override: None }`, `SameAsVisual`) and are
  not shown in any panel.
- Frames (`Robot.frames` is present, always empty, never shown).
- An import-units dialog: `MeshAsset.scale` is editable in the properties
  panel and defaults to a single app-wide "import units" setting (see
  ⚠ OPEN below); no per-drop prompt.
- Async mesh loading (`jobs`), decimation, ground grid, MSAA — backlog.
- Python, wasm beyond the existing build check.

## Design deltas

**`riggen-core` (new, replaces the placeholder).** Modules: `ids`, `pose`,
`robot` (types of 02), `validate`, `fk`, `command` + `history`, `file`
(`.riggen` v1). Depends on `riggen-mesh` (for `glam` and `TriMesh` in
`file::load`'s hash check only — no geometry is stored in the document),
`serde`, `serde_json`. No egui, no wgpu (crate doc comment already says so).

- **Ids.** `LinkId`, `JointId`, `GeomId`, `MeshId`, `FrameId` are `u32`
  newtypes handed out by a per-document `next_id` counter, stored in
  `BTreeMap<Id, T>` and serialised as `"l3"` / `"j7"` / `"g2"` / `"m1"`
  strings. Stable across edits and across save/load, never reused within a
  document's life. This replaces 02's "Ids are `slotmap` keys": a slotmap
  key carries a version that a readable `"l3"` cannot round-trip without
  a remap-on-load pass, and the document is small enough that a `BTreeMap`
  costs nothing. → 02 §Core types, §Schema. ⚠ OPEN below.
- **Joints are tree edges.** A link is always added together with its parent
  joint, so no command can produce an orphan link and `validate`'s tree
  invariant is an invariant of the command layer, not just of the file.
  `Command` becomes:
  ```rust
  AddLink { link: Link, parent: LinkId, joint: Joint }   // joint.parent/child are set by the command
  RemoveLink(LinkId)                                     // removes the whole subtree; root is refused
  RenameLink(LinkId, String), RenameJoint(JointId, String)
  AddGeom(LinkId, Geom), RemoveGeom(LinkId, GeomId), SetGeomPose(LinkId, GeomId, Pose)
  SetJoint(JointId, Joint)                               // one gesture = one SetJoint; parent/child immutable here
  Reparent { link: LinkId, new_parent: LinkId, keep_world_pose: bool }
  SetLinkMaterial(LinkId, Option<String>), UpsertMaterial(String, Material), RemoveMaterial(String)
  SetAsset(MeshId, MeshAsset)                            // scale / fix_up edits; adding an asset is not a command (below)
  SetInertial, SetCollision, SetRoot                     // present in the enum for M3, validated, no UI in M1
  ```
  `AddJoint` / `RemoveJoint` are gone: "create a joint between two selected
  links" is `Reparent` (the joint is the edge). → 02 §Commands. Recorded in
  **ADR-0005** together with the id decision (step 1).
- **Assets are registered, not commanded.** `Robot::add_asset(MeshAsset) ->
  MeshId` is a plain method; an asset that no geom references is pruned on
  save. Undoing "drop a mesh" undoes the `AddLink`/`AddGeom`, and the asset
  stays registered for the session, so redo does not reload the file.
- **`Pose`.** `Pose { t: DVec3, r: DQuat }` with `compose`, `inverse`,
  `to_mat4`, `from_xyz_rpy` / `to_xyz_rpy` (URDF convention: `R = Rz(yaw)
  · Ry(pitch) · Rx(roll)`, radians). The RPY helpers live in core because the
  URDF writer needs the same ones in M3. → 02 §Conventions.
- **`Material { density: f64, color: [f32; 4] }`**, `Robot::default_materials()`
  (aluminium 2700, steel 7850, PLA 1240, ABS 1040, nylon 1150, rubber 1100)
  seeded into every new document. Density is stored only; M3 consumes it.
- **`History`** is `{ undo: Vec<Robot>, redo: Vec<Robot>, saved_depth:
  Option<usize> }`. `apply` validates the command against a clone, then
  pushes the pre-state and commits; `undo` / `redo` swap; `mark_saved()` /
  `is_dirty()` compare the undo depth to `saved_depth`, which becomes `None`
  when an edit branches past it. `EditError` wraps `ValidationError` plus
  `UnknownId`, `WouldCreateCycle { link, ancestor }`, `CannotRemoveRoot`.
- **`.riggen` v1.** `file::save(&Robot, path)` writes `{ "schema_version":
  1, "robot": … }` with every `MeshAsset.path` rebased **relative to the
  file** (forward slashes); `file::load(path) -> Result<(Robot, Vec<Warning>),
  FileError>` resolves them back to absolute against the file's directory,
  so the in-memory document always holds absolute paths and the file never
  does. `content_hash` is FNV-1a 64 over the mesh file bytes, computed at
  registration; `load` recomputes it and reports a mismatch as a `Warning`
  (the mesh still loads). `#[serde(deny_unknown_fields)]` on every struct;
  `assets/fixtures/pendulum.riggen` is the first corpus file and must open
  forever. → 01 §File format, 02 §Schema.
- **`riggen-app`** — the document is now the only state
  (01 §The document is the only state becomes true):
  ```rust
  pub struct RiggenApp {
      robot: Robot, history: History, file: Option<PathBuf>,
      mesh_store: HashMap<MeshId, Arc<TriMesh>>,          // scaled + fix_up applied once at load
      instances: BTreeMap<(LinkId, GeomId), InstanceId>,  // the viewport's instance key (02 §Geom)
      q: JointState, selection: Selection,                // Selection = None | Link(LinkId) | Joint(JointId)
      viewport, next_instance, status, …                  // as M0
  }
  ```
  `sync_scene()` runs after every history change: adds / removes instances
  so the table matches the document's visual geoms, then writes
  `fk(robot, q)[link] ∘ geom.pose` into every instance model. A viewport
  click selects the link owning the hit instance; selecting in the tree
  selects the link's instances in the viewport (needs `Viewport::set_selected
  (Option<InstanceId>)`, new). `place_instance` is deleted: scenarios move
  parts through commands.
  New modules: `app/document.rs` (apply / undo / redo / sync / selection),
  `app/panels/{tree,properties,joints,materials}.rs`, `app/shortcuts.rs`,
  `app/file_io.rs` grows `.riggen` open/save and the confirm modal.
  Mesh files still enter through `load_files`: a `.riggen` replaces the
  document (after the dirty confirm); an STL/OBJ becomes a `MeshAsset` plus a
  new link named after the file stem, `Fixed` joint at identity, child of the
  selected link or the root. "Add mesh to this link…" in the link properties
  is the route for a second geom on one link.
- **Camera range follows the scene.** `OrbitCamera.near/far` are fixed at
  0.01 / 100 m today; a part imported at mm→m scale is 1000× smaller than
  M0's unit cube and clips. `frame_scene` / `animate_frame_scene` set
  near/far from the fitted radius (e.g. `r/1000`, `r*1000`, clamped). → 01
  §Frame loop (camera paragraph).
- **`debug_state()`** gains a `document` section: file name, dirty flag,
  links (id, name, parent joint, geom count, material), joints (id, name,
  kind, parent, child, q), selection, and which windows are open. Existing
  goldens change (`startup` grows two panels) — one `snapshots:` commit,
  images shown to the human.
- **Shortcuts.** Ctrl+N / Ctrl+O / Ctrl+S / Ctrl+Shift+S fire always;
  Ctrl+Shift+Z is matched **before** Ctrl+Z (egui's `consume_key` matches
  modifiers logically, so a bare Ctrl+Z pattern swallows the shift variant),
  Ctrl+Y is the alternative redo, and undo/redo yield while a text field has
  focus so `TextEdit`'s own undo keeps working; Delete removes the selected
  link/joint outside text fields. Property edits commit on Enter / lost
  focus, never per keystroke, and a commit equal to what the document holds
  is dropped before it reaches `History`.

## Steps

- [x] Step 1 — `riggen-core` foundation: `ids` (`u32` newtypes, `"l3"` serde,
  `IdGen`), `Pose` (compose / inverse / mat4 / RPY both ways, with tests
  against hand-computed rotations and a random-round-trip test), the `Robot` /
  `Link` / `Geom` / `Joint` / `JointKind` / `Limits` / `Dynamics` /
  `Material` / `MeshAsset` / `InertialSpec` / `CollisionPolicy` / `Frame`
  types with serde derives, `Robot::new(name)` (root `base_link`, default
  materials), `Robot::add_asset`. `validate()` with `ValidationError` (tree
  rooted at `root`, single parent joint, no cycles, dangling ids, unique
  XML-valid names, non-zero axis, limits present and ordered for
  Revolute/Prismatic) and a test per error. **ADR-0005**: ids as counters and
  joints as edges, with the reasons above.
- [x] Step 2 — `fk`: `JointState`, `motion(kind, axis, q)`, one depth-first
  pass from the root returning `BTreeMap<LinkId, Pose>`. Tests: the 3-joint
  chain against hand-computed poses (revolute about Z, revolute about Y with
  an offset origin, prismatic), a fixed joint is identity, a joint absent
  from `q` reads as 0, chain order independent of insertion order. This is
  the acceptance's `fk` test.
- [x] Step 3 — `Command`, `History`, `EditError`. Every variant validated
  and applied; `RemoveLink` takes the subtree; `Reparent` refuses the root
  and any descendant of `link` (cycle), and with `keep_world_pose` rewrites
  the joint origin from `fk` so world poses are unchanged (tested against
  `fk` before/after). Undo / redo / `mark_saved` / `is_dirty` tests including
  "edit past the saved mark → dirty until saved again".
- [x] Step 4 — `.riggen` v1: `file::save` / `file::load` with path rebasing,
  content hash, warnings, `deny_unknown_fields`. `assets/fixtures/
  pendulum.riggen` (base + arm from the cube fixtures, one revolute joint
  with limits) hand-written as the corpus file. Tests: round-trip equality,
  relative paths in the written JSON, hash mismatch is a warning not an
  error, unknown field is an error naming it, the corpus file opens.
- [x] Step 5 — `riggen-app` owns a `Robot`: `document.rs` (apply / undo /
  redo / `sync_scene` / selection mapping), `mesh_store`, instance table,
  `load_files` dispatches on extension (`.riggen` → replace document; mesh →
  `AddLink` under the selection or root), CLI `riggen robot.riggen`,
  camera near/far from the fit radius, `Viewport::set_selected`,
  `debug_state().document`. `place_instance` removed; `three_parts` becomes
  three links placed through `SetJoint`. Goldens re-baselined; a new
  `pendulum` scenario opens the fixture and asserts two instances at the FK
  poses for `q = 0`, plus an `mm_scale_part` scenario (cube at `scale
  0.001`, fitted, not clipped). `snapshots:` commit with the images shown.
- [x] Step 6 — Tree panel (left `SidePanel`): links as a collapsible tree
  with the parent joint's name and kind on each row, click selects, F2 /
  double-click renames inline, "+ Link" adds an empty link under the
  selection, Delete removes, `dnd_drag_source` / `dnd_drop_zone` reparents
  (`keep_world_pose: true`); viewport ↔ tree selection sync both ways.
  Snapshots: `tree_pendulum` (selection on `arm`, viewport tinted),
  `tree_reparent` (reparent through the command API — kittest cannot drag —
  then the tree re-drawn; the world pose assertion lives in `debug_state`).
- [x] Step 7 — Properties panel (right `SidePanel`): for a link — name,
  material combo, geoms list with xyz + RPY (degrees) and the asset's scale /
  fix-up, "Add mesh to this link…", remove geom; for a joint — name, kind
  combo (limits appear / vanish with the kind), origin xyz + RPY (degrees),
  axis (normalised on commit), limits (degrees for revolute, meters for
  prismatic), dynamics. Commit on Enter / lost focus; no-op commits dropped
  (test: clicking through every field adds no history entry). Snapshots:
  `properties_link`, `properties_joint`; app test: typing an origin and RPY
  moves the arm's instance to the expected FK pose.
- [x] Step 8 — Joint sliders `Window`: one slider per non-fixed joint,
  bounded by its limits (Continuous: ±π), driving `q` → `sync_scene` every
  frame (not a command; repaint while dragging). Reset-all button. Snapshot
  `pendulum_swing` at `q = 45°` asserting the arm's instance position from
  `fk`; app test: `q` is clamped when a limit is edited below it.
- [ ] Step 9 — Materials table `Window`: rows of name / density / colour,
  add / remove / edit through `UpsertMaterial` / `RemoveMaterial` (removal
  refused while a link uses it, with the reason in the status bar); the link
  material combo reads the same table; the viewport tints instances with the
  material colour. Snapshot `materials`.
- [ ] Step 10 — File menu: New, Open…, Save (Ctrl+S; Save As when untitled),
  Save As…, Quit; `.riggen` filter in the dialogs; window title
  `name.riggen — riggen` with `*` when dirty; the confirm modal (Save /
  Don't save / Cancel) on New / Open / Quit / dropped `.riggen` when dirty,
  including the eframe `close_requested` / `CancelClose` path. Snapshots:
  `dirty_title` (status bar and title show the marker), `unsaved_confirm`
  (the modal). App test: save → reopen → `Robot` equal, `is_dirty` false.
- [ ] Step 11 — Edit menu + shortcuts (`shortcuts.rs`): undo / redo /
  delete with the ordering and text-focus rules above, `handle_shortcuts`
  called before the panels each frame. App tests: Ctrl+Shift+Z redoes and
  does not undo; Ctrl+Z inside a focused `TextEdit` leaves the document
  alone; Ctrl+S with a text field focused still saves. Then the acceptance
  run below (`/retire-plan` does the docs and the `m1` tag).

## Acceptance

Roadmap M1: build a base + arm from two STLs with a revolute joint typed
numerically; the slider swings it within limits; save, reopen, undo/redo
survive; `fk` unit tests against hand-computed poses for a 3-joint chain.

Executable form, all green under `cargo test --workspace` on the CPU adapter:

- `riggen-core`: `fk::tests::three_joint_chain_matches_hand_computed_poses`,
  `history::tests::*`, `file::tests::corpus_pendulum_opens`.
- `riggen-app --test visual`: scenarios `startup`, `pendulum`,
  `mm_scale_part`, `tree_pendulum`, `tree_reparent`, `properties_link`,
  `properties_joint`, `pendulum_swing`, `materials`, `dirty_title`,
  `unsaved_confirm` pass; app tests `build_pendulum_numerically` (two cube
  fixtures dropped → joint typed in the properties panel → slider at 45° →
  save to a temp dir → reopen → equal document → undo twice → redo twice →
  same document) and the shortcut tests pass.
- By hand, once: `cargo run -- assets/fixtures/pendulum.riggen`, swing the
  slider, reparent by drag, save, reopen. The human reports what was
  annoying; that list goes to the backlog, not this plan.

## Docs to update on completion

- `docs/02-data-model.md` §Core types — ids paragraph (counters, `"l3"`
  serde), `Material` struct, `Robot::add_asset`; §Commands and history — the
  enum as shipped, `History` fields, `saved_depth`; §Conventions — the RPY
  convention line; §Schema — corpus fixture named.
- `docs/01-architecture.md` §Cargo workspace — `riggen-core` no longer a
  placeholder, `riggen-app` module list; §The document is the only state —
  present tense, `sync_scene`, `Selection`; §Frame loop — camera near/far
  from the fit; §File format — absolute in memory / relative on disk, hash
  warning; §Testing — the M1 scenario list and the "kittest cannot drag,
  reparent through the command API" harness fact.
- `docs/03-roadmap.md` §M1 — status line `done <date>, tag m1`, and note
  that `Reparent { keep_world_pose }` landed here (M2's line becomes "wired
  to the gizmo").
- `docs/adr/README.md` — ADR-0005 in the index.
- `docs/BACKLOG.md` — add: import-units dialog per drop; tree↔3D hover
  highlight (M2 already); anything from the by-hand run.
- `AGENTS.md` current state — M1 line, next: M2.

## Open questions

- ~~⚠ OPEN~~ **Decided 2026-08-29 (step 1): ids as `u32` counters +
  `BTreeMap`**, one counter for every kind (`Robot::next_id`). ADR-0005.
- ~~⚠ OPEN~~ **Decided 2026-08-29 (step 1): joints are edges** (`AddLink`
  carries its joint; `AddJoint` / `RemoveJoint` dropped; "connect two links"
  = `Reparent`). ADR-0005.
- ~~⚠ OPEN~~ **Decided 2026-08-29 (step 5, human): app-wide import scale,
  mm default.** `RiggenApp::import_scale` (`DEFAULT_IMPORT_SCALE = 0.001`)
  is what a dropped mesh's `MeshAsset::scale` gets; the status bar shows it
  as `import: mm`. The File-menu choice and its eframe-storage persistence
  land with the menu in step 10; the asset row in the properties panel
  (step 7) edits it per asset. The harness sets `1.0` for every scenario
  (the fixtures are unit cubes meant as meters); `mm_scale_part` sets the
  default back explicitly.
- ~~⚠ OPEN~~ **Decided 2026-08-29 (step 5, human): a dropped mesh is a new
  link** under the selected link (a selected joint's child; else the root),
  `Fixed` joint at identity, named after the file stem made XML-valid
  (`my part` → `my_part`, `3d` → `_3d`) and deduplicated (`arm_2`); the
  joint is `<name>_joint`, deduplicated the same way.
- ~~⚠ OPEN~~ **Decided 2026-08-29 (step 3): `RemoveLink` removes the
  subtree** (links, joints, and any frame on them) — one undo, matches every
  tree UI. Splicing children onto the parent can be a later command if wanted.
- Findings from step 3 (shipped as written; the human can object):
  - `Reparent { keep_world_pose }` preserves world poses in the **zero
    configuration** (`q = 0`): commands do not see the slider state. A drag
    in the tree panel (step 6) while sliders are non-zero can therefore
    jump. If that is annoying in the by-hand run, `Reparent` grows a
    `JointState` and the origin is corrected for the ancestors' `q`.
  - `History::apply` drops a command whose result equals the document, so
    "a commit equal to what the document holds is dropped" (Design deltas,
    Shortcuts) is guaranteed in core rather than by every panel; step 7's
    no-history-entry test still stands as a check of the panels' commit
    path.
  - `History::apply` returns `Option<LinkId>` — the link `AddLink` made, so
    step 5 can select a dropped mesh. `AddLink.link` is `Box<Link>` (clippy
    `large_enum_variant`); geom ids inside it and in `AddGeom` come from
    `robot.next_id.alloc()` at the caller.
  - `SetRoot` reverses fixed joints on the path only and refuses a movable
    one (`EditError::MovableJointOnRootPath`): a reversed revolute joint's
    pivot is not expressible in the swapped child frame. M3 decides whether
    to relax this.
  - `EditError` as shipped: `Invalid(ValidationError)`, `UnknownId { kind,
    id }`, `UnknownMaterial`, `WouldCreateCycle { link, new_parent }`,
    `CannotRemoveRoot`, `CannotReparentRoot`, `MaterialInUse { material,
    link }`, `MovableJointOnRootPath`. `SetJoint` ignores `parent` / `child`
    in the value rather than erroring.
  - Step 4: `file::load` validates (a hand-edited file that breaks an
    invariant is `FileError::Invalid`, not a half-open document) and
    `file::save` refuses an invalid robot before writing; the write goes
    through `<name>.riggen.tmp` + rename so a crash leaves the old file.
    Besides `HashMismatch` there is a `Warning::MeshUnreadable` for a mesh
    file that moved. The corpus fixture was produced by `save` itself and
    `corpus_pendulum_opens` re-saves it and compares bytes, so a formatting
    drift in `save` fails there rather than silently rewriting fixtures.
    Pendulum geometry: unit cubes, `hinge` revolute about Y at (0, 0, 0.5),
    arm geom at (0, 0, 0.5) in the arm frame → at `q = 0` the arm cube sits
    on the base cube at world (0, 0, 1); limits ±90°.
  - Step 5: `open_path` returns `Option<LinkId>` (`None` for a `.riggen`,
    the new link for a mesh); registering the asset happens before the
    `AddLink`, so the pre-state snapshot holds it and undo/redo never
    reloads the file (`drop_undo_redo` app test). `sync_scene` also
    re-uploads an instance whose asset scale / fix-up changed (`LoadedMesh`
    keeps the raw file mesh) and clamps `q` to freshly edited limits;
    `set_joint_value` clamps too. The camera's depth range is set on every
    fit (`OrbitCamera::set_depth_range_for`: near `r/100`, far `r·1000`,
    clamped to `[1e-6, 1]` / `[100, 1e6]`) and the zoom range follows
    `[2·near, far/2]` instead of M0's fixed `[0.02, 50]` m; `CameraDebug`
    reports `near` / `far`. A `.riggen` dropped or opened replaces the
    document without a confirm until step 10. `Viewport::set_selected`
    marks triangle `0`: selection is per instance, the triangle is a
    readout only. The viewport→document selection sync runs once per frame
    after `Viewport::ui`; a click on empty space with a *joint* selected in
    the tree leaves that selection alone (the viewport reports `None` →
    `None`, nothing to notice) — revisit if it annoys.
  - Step 6: **`dnd_drag_source` swallows clicks.** egui's hit test
    (`hit_test.rs`, "the top thing senses only drags, so we ignore the
    click-widget") gives a press to the drag-only widget the helper lays
    over the row, so the label under it never sees `clicked()`. The row's
    name is a `Button::selectable(..).sense(click_and_drag())` that calls
    `dnd_set_drag_payload` itself; `dnd_drop_zone` is fine (hover-only).
    The harness fact for the docs: `harness.get_by_label("arm").click()`
    + one `step()` clicks a tree row. `shortcuts.rs` exists already with
    Delete / F2; "a text field has focus" is `TextEdit::load_state(ctx,
    focused_id).is_some()`, because a clicked button holds focus too and
    must not block Delete (tested: Delete inside the rename field edits
    text, Delete after Enter removes the link). "+ Link" selects the new
    link and starts the rename; a dropped mesh does *not* move the
    selection, so a multi-file drop lands side by side instead of chained.
    A selected joint counts as its child for both "+ Link" and drops. The
    `⟂` glyph is not in egui's default font — rows say `hinge · revolute`.
  - Step 7: numeric fields are `TextEdit`s with a per-field draft buffer
    (`PropertiesState.drafts`, keyed by widget id, alive only while the
    field has focus); commit on `lost_focus` (Enter surrenders focus, so
    it is the same path), Escape reverts, and a commit that formats to the
    shown value is dropped before it becomes a command. Drafts are cleared
    when the selection changes rather than committed to a field that is
    no longer drawn. Every field is `labelled_by` its label, so kittest's
    `get_all_by_label("x").nth(0)` is the origin's x and `.nth(1)` the
    axis's — the harness fact for the docs. A closed `ComboBox` does not
    expose its selected text as a label (the golden pins it instead).
    Switching a joint to Revolute/Prismatic with no limits gives ±π /
    ±1 m; switching away keeps the limits in the document (hidden), so
    switching back restores them. `add_mesh_to_link(link, path)` is the
    API behind "Add mesh to this link…"; `register_mesh` is shared with
    the drop path. Panel default width 380 px so the `x y z` rows fit.
  - Step 8: the sliders window is closed by default and toggled from a
    new **Window** menu (`Window › Joints`; step 9's materials table joins
    it) — a floating window over an empty document is noise. It opens at
    the viewport's top-right and is draggable from there. Slider units:
    degrees for Revolute (limits) and Continuous (±180°), meters for
    Prismatic (limits); each change writes `q` through `set_joint_value`
    (clamped) and re-syncs the scene; "Reset all" zeroes every joint.
    `debug_state().ui.windows` lists open windows. Harness facts: a menu
    is driven with `get_by_label("Window").click()` + `step()` then the
    item; a slider's value is read with `NodeT::accesskit_node()
    .numeric_value()` (import `egui_kittest::kittest::NodeT`).
  - `validate` also checks material names (`InvalidName { kind:
    "material" }`) and that densities are finite and non-negative, so
    `UpsertMaterial` cannot smuggle an unexportable name into the table.
