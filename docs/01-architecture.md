# 01 — Architecture

## Layer map

Strictly layered; dependencies point downward only. `riggen-core` and
`riggen-export` must compile without egui or wgpu — they are what the v0.2
Python module links against, and they are where the tests that matter live.

```
┌────────────────────────────────────────────────────────────────┐
│  riggen-app       eframe shell, panels, gizmos, drag-drop,     │  binary
│                   selection, snapping, jobs, snapshot suite    │  (maturin bin)
├────────────────────────────────────────────────────────────────┤
│  riggen-viewport  wgpu renderer via egui_wgpu callbacks:       │
│                   instances, camera, ID-buffer picking,        │
│                   project / cursor_ray, Overlay (world-space   │
│                   segment, polyline, arc, point, label)        │
├──────────────────────────────┬─────────────────────────────────┤
│  riggen-export               │  (riggen-py, v0.2)              │
│  ResolvedRobot, MJCF + URDF  │  PyO3 over core + export        │
│  writers, URDF import,       │                                 │
│  validation, round-trip FK   │                                 │
├──────────────────────────────┴─────────────────────────────────┤
│  riggen-core      Robot document (links, joints, frames,       │
│                   inertial spec, collision policy), FK,        │
│                   undo history, serde, schema versioning       │
├────────────────────────────────────────────────────────────────┤
│  riggen-mesh      TriMesh, STL/OBJ loaders, mass properties,   │
│                   convex hull, primitive fits, ray/triangle    │
└────────────────────────────────────────────────────────────────┘
      egui / eframe / wgpu appear ONLY in riggen-viewport and riggen-app
      glam is the one math library, re-exported by riggen-mesh
```

The viewport owns the projection, so it owns the overlay: everything drawn
on top of the scene — the joint glyphs, snap markers, readouts — arrives as
world-space `OverlayItem`s and is projected through the same
`camera.view_proj` the wgpu pass rasterized with, so an overlay cannot
disagree with the geometry about where a point is. It is drawn with egui's
painter after the paint callback and is **not** depth-tested; for a joint
glyph inside a part that is the wanted behaviour. The viewport never sees a
`Joint` — the app builds the items (`app/glyphs.rs`).

`riggen-core` depends on `riggen-mesh` today only for the `glam` re-export
(no geometry is stored in the document); M3 adds mass properties (a link's
computed inertial is a function of its meshes). `riggen-export` depends on
both. Nothing below `riggen-app` knows about selection, hover, or
gizmos.

## Cargo workspace

```
riggen/
├── Cargo.toml              # [workspace], resolver 3, edition 2024, every dep version
├── rust-toolchain.toml     # stable + rustfmt, clippy, wasm32-unknown-unknown
├── kittest.toml            # snapshot thresholds (ADR-0003); found by walking up from the crate
├── crates/
│   ├── riggen-mesh/        # TriMesh, Aabb, Ray, load_stl / load_obj / load_mesh
│   ├── riggen-core/        # ids, pose, robot, validate, fk, command, history, file
│   ├── riggen-export/      # placeholder until M3
│   ├── riggen-viewport/    # camera/, scene, pick_id, gpu_mesh, viewport/, shaders/
│   └── riggen-app/         # bin "riggen"; cdylib for the wasm build check; tests/visual
│       └── src/app/        # document, file_io, file_menu, shortcuts, status_bar,
│                           # panels/{tree, properties, joints, materials}
├── assets/fixtures/        # cube_binary.stl, cube_ascii.stl, cube.obj — the unit cube
│                           # (TriMesh::cube(0.5)) in every format; pendulum.riggen, the
│                           # .riggen v1 corpus file (02 §Schema); arm/*.stl in mm, the
│                           # M2 acceptance's four parts (an ignored generator test writes them)
├── python/                 # v0.2: pyproject.toml, riggen/ package, riggen-py crate
├── docs/
├── SEED.md
└── AGENTS.md, CLAUDE.md
```

Dependency policy (ADR-0001): egui/eframe/egui-wgpu 0.36.x from crates.io,
wgpu version dictated by egui-wgpu — never depend on a different wgpu.
`glam` 0.30 with the `serde` feature, re-exported as `riggen_mesh::glam`;
no other crate lists it. Every version lives once in
`[workspace.dependencies]`; crates say `.workspace = true`. Local checkouts
of egui and rerun under `~/Documents/code/rust/` are reference reading only;
no `path =` or `[patch]` unless an unreleased fix is needed, and then with a
comment saying which one. Profile settings carried from RoboCAD:
`opt-level = 1` for our crates in dev, `3` for dependencies — an unoptimized
wgpu is felt. A dependency's default features are checked against the wasm
build (`tobj`'s `ahash` default pulled `getrandom` in, which does not compile
for `wasm32-unknown-unknown`; it is off).

`riggen-export` is an empty `lib.rs` that already carries the "no egui/wgpu"
rule in its doc comment; `riggen-core` keeps the same rule and depends on
`serde` / `serde_json` and nothing else. `eframe` is built with its
`persistence` feature so the import-units choice (and egui's own window
layout) survive a restart through eframe storage.

## The document is the only state

`riggen-app` owns one `Robot` (02-data-model) plus derived, never-saved
state:

```rust
pub struct RiggenApp {
    robot: Robot, history: History, file: Option<PathBuf>,
    mesh_store: HashMap<MeshId, LoadedMesh>,            // raw file mesh + the scaled/fixed-up Arc<TriMesh>
    instances: BTreeMap<(LinkId, GeomId), InstanceId>,  // the only map between document and scene
    q: JointState, selection: Selection,                // Selection = None | Link(LinkId) | Joint(JointId)
    tool: Tool, gizmo_state: GizmoState,               // Select | Move | Rotate | PlaceJoint | Align; the gizmo and its drag
    preview_world: Option<(LinkId, Pose)>,             // a link's pose while a gizmo drag previews it
    import_scale: f64, pending: Option<PendingAction>,  // File › Import units; New/Open/Quit awaiting the dirty answer
    tree, props, joints_window, materials_window,       // transient panel state (a rename in progress, drafts)
    viewport, next_instance, status, …
}
```

Every user edit is a `Command` applied through `History`, which is
**snapshot-based**: `Robot` is a few kilobytes of ids, poses and numbers
(meshes are referenced by id, never copied), so `History` keeps
`Vec<Robot>` and undo is a swap. This is deliberately simpler than RoboCAD's
reversible commands; it stays correct as long as `Robot` never holds bulky
data. Mesh geometry lives in the mesh store beside the document, keyed by
`MeshId`, loaded once per file (`LoadedMesh` keeps the raw mesh and the
`scale` / `fix_up` derivative, so a scale edit re-derives without re-reading
the file) and shared across snapshots by `Arc`.

`RiggenApp::apply` / `undo` / `redo` wrap `History` and then run
`sync_scene()`, which makes the viewport match the document: an instance
per `(LinkId, GeomId)` visual is added or removed, a mesh whose asset scale
changed is re-uploaded, every model matrix is written from
`fk(robot, q)[link] ∘ geom.pose`, and every instance's colour from the
geom's own colour, else the link's material, else the viewport default.
`q` is pruned of vanished joints and clamped to freshly edited limits on
the way, and `preview_world` — a link's pose while a gizmo drag is in
flight — is applied as a correction to that link and its whole subtree, so
the parts follow the handle exactly as they will after the commit while the
document stays untouched. Opening a document (`replace_document`) resets history, selection,
`q` and the mesh store, then syncs; the meshes come from the assets' paths.

Selection is document-level and mirrored both ways: a viewport click
selects the link owning the hit instance (`sync_selection_from_viewport`,
once per frame after `Viewport::ui`), and selecting in the tree calls
`Viewport::set_selected` with the link's first instance. A selected joint
has no instance; a click on empty viewport space therefore leaves a joint
selection alone (the viewport reports `None → None`).

A `Tool` is modal: it decides what a viewport click and drag mean. `Select`
is the M1 behaviour and the resting state, and `Esc` always returns to it
(consumed only while a tool is active, so the rename / modal / field-revert
uses of Escape still see it). The four editing tools commit frame-rewriting
commands, which work in the **zero configuration**, so `set_tool` resets `q`
first when something is off zero and says so in the status bar
(plans/m2-placement-ux OPEN 1). Resetting `q` is not an edit and adds no
history entry.

Granularity rule, kept from RoboCAD: **one gesture = one command.** A gizmo
drag mutates a *preview* pose every frame and commits once on release; a
slider preview of a joint angle is not a command at all (joint values are
derived state, not document state — the document stores limits, not the
current `q`). Concretely: properties-panel numbers are text fields with a
draft buffer that commit on Enter or lost focus, never per keystroke, and a
commit equal to the shown value never becomes a command (`History` drops
the remaining no-ops); the materials table's colour picker keeps a draft,
tints the viewport live and sends one `UpsertMaterial` when the popup
closes.

### Panels and menus

- **Links** (left): one row per link with its parent joint's name and kind
  (`hinge · revolute`); click selects the link, click the joint label
  selects the joint, double-click or F2 renames inline, "+ Link" adds an
  empty link under the selection, Delete / "− Remove" removes the subtree
  (root refused, reason in the status bar), dragging a row onto another
  reparents with `keep_world_pose`. Every row is a `dnd_drop_zone` around a
  `Button::selectable(..).sense(click_and_drag())` that sets its own
  payload with `dnd_set_drag_payload` — egui's `dnd_drag_source` lays a
  drag-only widget over its content and the hit test then swallows clicks
  (`hit_test.rs`: a top-most widget that senses only drags hides the
  click-widget under it). The panel draws from the document and applies
  its actions after drawing.
- **Toolbar**: five buttons — Select / Move / Rotate / Place joint /
  Align — in a popup frame floating over the viewport's top-left corner.
  Drawn *after* the viewport in the same layer, which is what gives it the
  pointer: egui's hit test prefers the widget registered last.
- **Joint glyphs** (in the viewport): a joint has no geometry, so without
  one it exists only in the tree and "which way does this hinge turn?" has
  to be read off two number fields. Each glyph is an axis segment through
  the **pivot** (`world(parent) ∘ origin`, which unlike the child link
  frame has not slid away by `q`), an origin triad in the axes triad's
  colours, and a limit arc (revolute; the full circle for `Continuous`) or
  an offset travel segment with end stops (prismatic), each with a tick at
  the current `q`. Sized from the child link's own world bounds, so a
  glyph is the size of the part it belongs to; the scene radius, then one
  metre, are the fallbacks. Drawn for every movable joint plus the
  selected one whatever its kind (plans/m2-placement-ux OPEN 4) — every
  weld in a big assembly would be noise. **Hover runs both ways**: a
  hovered tree row (the link's name or the joint's label) draws that
  joint's glyph hot, and a glyph under the cursor — nearest axis segment
  within `GLYPH_HOVER_RADIUS` screen points, measured in screen space
  because what the user aims at is the line they can see — brightens the
  tree row and names the joint in the status bar. While a glyph is
  hovered the viewport's own pick is suppressed, so the part behind it is
  not highlighted as well and a click selects the *joint*.
- **Properties** (right): a link's name, material, and per geom the pose
  (xyz m, RPY °), asset scale and fix-up, "Add mesh to this link…"; a
  joint's name, kind (limits appear with Revolute/Prismatic, defaulting to
  ±π / ±1 m), origin, axis (normalised on commit), limits in ° or m,
  dynamics. Fields are `labelled_by` their labels for the accessibility
  tree.
- **Window › Joints** / **Materials**: floating windows, closed by default.
  Joints: one slider per movable joint in its limits (Continuous ±180°),
  writing `q` and syncing every frame, "Reset all". Materials: name /
  density / colour rows, add and remove (refused while a link uses it).
- **Debug**: egui's `DebugOptions` overlays (debug on hover, widget hits,
  interactive widgets, width / height expansion, resize, unaligned) toggled
  in both themes at once, then **Copy state (JSON)** / **Save state
  (JSON)…** — the runtime route to `debug_state()` (§Testing) for a state
  reached by hand rather than by a scenario.
- **File**: New, Open…, Save (Save As when untitled), Save As…, Import
  units, Quit; **Edit**: Undo, Redo, Delete, greyed out when idle. The
  window title is `name.riggen* — riggen`. Every route that would drop a
  dirty document — New, Open, a dropped `.riggen`, Quit, the OS close button
  (refused with `CancelClose` until answered) — goes through one
  `PendingAction` and the Save / Don't save / Cancel modal.
- **Shortcuts** (`shortcuts.rs`, run before the panels each frame): Ctrl+N
  / O / S / Shift+S fire always; Delete, F2, Ctrl+Z, Ctrl+Shift+Z and Ctrl+Y
  yield while a `TextEdit` has focus (`TextEdit::load_state` on the focused
  id — a clicked button holds focus too and must not block Delete), and the
  shifted pattern is consumed before the bare one because egui matches
  modifiers logically.

## Frame loop

```
input ──► viewport.set_input_suppressed(gizmo owned the cursor last frame)
       ──► egui panels (tree, properties, joint sliders, status)
       ──► viewport.set_overlay(joint glyphs, from the document and q)
       ──► viewport.ui ──► gizmo ──► toolbar   (registration order = pointer precedence)
       ──► gizmo / snapping / pick handling  ──► Commands ──► History ──► Robot
Robot ──► fk(robot, q) ──► world pose per link
       ──► for each visual geom: viewport.set_instance_model(instance, link_pose * geom.pose)
       ──► viewport.ui(...)   (records the wgpu callback; picks resolve next frame)
```

The viewport draws **instances**, not links: one instance per `(LinkId,
GeomId)` visual, with the uploaded `TriMesh` shared by `MeshId` (scale and
fix-up applied once at load through `TriMesh::transform`). Moving a joint
writes matrices; nothing is re-uploaded. Collision geometry will render as
a second, toggleable instance set (translucent) sharing the same camera
(M3).

`InstanceId(u32)` is handed out by the app and never reused in a session.
`Scene<M>` keeps one entry per instance — payload (`GpuMesh`), `DMat4`
model, linear RGBA colour (the material tint; `DEFAULT_INSTANCE_COLOR`
until told otherwise), visibility, model-space `Aabb` — in insertion
order, which is draw order; `bounds()` unions the transformed boxes for
zoom-to-fit. Model matrix and colour go to the GPU as one dynamic-offset
uniform buffer (80 bytes per instance, grown by `next_power_of_two`), one
`draw_indexed` per visible instance, no CPU merge; the pick and highlight
shaders declare only the matrix, which is valid against the larger
binding.
The scene renders into an offscreen colour + `Depth32Float` pair (egui's
own pass has no depth attachment) and is blitted in `paint()`; the axes
triad draws last in its own corner viewport with a rotation-only camera.
`f64` → `f32` happens in `GpuMesh::upload` and the model-uniform pack, and
nowhere else.

Camera: `OrbitCamera` is robocad's turntable camera on glam — `f32`, radians,
Z-up with Y standing in as the up hint at the poles. glam's
`perspective_rh` / `orthographic_rh` produce wgpu's `[0, 1]` clip depth
directly, so `view_proj = proj * view` with no OpenGL-to-wgpu remap; a
camera test pins near → 0 and far → 1 in both projections. Every fit sets
the depth range from the fitted radius (`set_depth_range_for`: near
`r/100`, far `r·1000`, clamped to `[1e-6, 1]` / `[100, 1e6]`) and the zoom
range follows `[2·near, far/2]`, so a part imported at mm → m scale is
neither clipped nor lost and a room-sized scene still fits. Wheel input is
read from the raw events, not egui's smoothed delta (the smoothing reads as
the camera coasting), and zooms toward the cursor. Numpad 1/3/7/0 (+ctrl)
snap views, Num5 or `P` toggles projection, Home animates a fit; the
`persp`/`ortho` label sits in the viewport corner, the wall-clock frame time
in the status bar (hidden by `set_frame_hud_visible(false)` in tests).

Repaint policy: egui repaints on input; request continuous repaint only
during camera motion, gizmo drags, slider drags and joint animation. A hover
pick is issued only when the cursor pixel or the camera matrix changed
(RoboCAD's `last_pick` rule — otherwise pick + readback + repaint loop at
vsync while the pointer merely rests).

## Picking and snapping

The ID buffer (`R32Uint`) encodes `(slot: 12 bits, triangle + 1: 20 bits)`
per pixel; `0` is the clear value and means "miss", which is why the
triangle is stored offset by one. The **slot** is the `Scene`'s draw slot,
recycled through a free-list (`MAX_INSTANCES = 4096` live at once), not the
`InstanceId`, so a long session never runs out of encodable ids;
`Scene::instance_at_slot` turns a readback into an `InstanceId` and answers
`None` for a slot whose instance was removed while the pick was in flight.
The 20-bit triangle field is where `riggen_mesh::MAX_TRIANGLES = 2^20 − 1`
comes from; the loaders refuse bigger meshes with an error that names the
file. Pick vertices are one per triangle corner, drawn non-indexed, because a
welded vertex cannot carry two triangles' ids; the shaded pass stays indexed.

The pick pass runs on its own encoder after the scene pass, copies the 5×5
pixel region around the cursor into a `MAP_READ` buffer and registers a
`map_async`; a request still unanswered after `MAX_PICK_FRAMES` frames is
abandoned and the memo cleared, because the readback is asynchronous and
nothing guarantees it lands — a frame whose paint callback never ran would
otherwise wedge hovering and selection for the rest of the session; a non-blocking `device.poll` at the top of the next
`Viewport::ui` lets it resolve — the readback never stalls a frame. The hit
nearest the cursor in the region wins (there is no B-Rep, so no vertex >
edge > face ladder). At most one pick is in flight; a click's select pick
beats a hover; a hover whose `(pixel, view_proj)` equal the last pick's is
not re-issued (`last_pick`), otherwise a resting cursor would re-render the
ID buffer at vsync rate forever; `PointerGone` clears the hover. The policy
is the pure `decide_pick`, unit-tested without a GPU. The result is a
`PickHit { instance, triangle }`; hover and selection tint the **whole
instance** (a "face" on an STL is one triangle, so a face outline would
trace a single triangle) and the status bar reads `arm (i1/t120)`.
`Viewport::set_selected(Option<InstanceId>)` is the other direction, for
the tree; it records triangle `0`, since selection is per instance and the
triangle is a readout only.

`riggen_mesh::ray_triangle` (Möller–Trumbore, two-sided — the ID buffer has
already chosen the triangle) recovers the exact hit point by intersecting
`Viewport::cursor_ray` with that one triangle, taken into mesh space, so
snap targets never need a spatial index. The 5×5 region means the named
triangle can be the cursor's *neighbour*, and the exact ray then misses it
by a pixel; its plane is the fallback. `app/snap.rs` builds the
candidates and picks among them by a fixed ladder — **vertex > box >
circle > point** — with the winner, its axis and its readout in
`debug_state().snap`. Only the placement tools snap (`Tool::snaps`);
markers under the cursor while merely selecting would be noise.

- **vertex**: a corner of the hit triangle within `SNAP_PIXEL_RADIUS`
  screen points;
- **box**: a corner or face centre of the instance's AABB, also within the
  pixel radius, from the `Scene` bounds already kept for zoom-to-fit — a
  part with no modelled features still has somewhere obvious to grab;
- **circle**: `feature::fit_circle_with` on the smooth region around the
  hit triangle (02 §Mesh features) → centre, axis, radius, residual,
  segments. **No pixel radius**: the centre of a bore is nowhere near the
  wall the user is pointing at, which is the point. This is the mechanic
  that makes "click the bore, get the joint axis" work on STL data with no
  B-Rep, and it was the M2 risk item;
- **point**: the ray/triangle hit itself, which always exists, with the
  triangle's normal for "axis = face normal".

**Align** is the mouse-only route for a part that came out of CAD at the
wrong origin: first click a feature on the **selected link**, second click
a feature anywhere. Two circles are made concentric — the minimal rotation
taking the first axis onto the second (the target axis is flipped first
when that is the shorter way round, since a circle's axis has no preferred
direction), about the first centre, then the centres together — and
anything else is a plain point → point translation, because a vertex says
nothing about orientation. The result is one `SetJoint` on the link's
parent joint through `fk::origin_for_world`, so the link and its subtree
move and the gesture costs one history entry. The pending first pick is
drawn in magenta, not remembered silently, and a tool change or another
selection abandons it.

A box target on the **far** side of the part is dropped before the ladder
runs: the overlay is not depth-tested and a bounding box floats around the
geometry rather than lying on it, so a hidden corner would otherwise win a
snap to something the user cannot see. A vertex of the hit triangle is on
the surface by construction and is not filtered.

**Place joint** turns a candidate into one `MoveJointFrame` on the selected
joint: the frame keeps its orientation and moves to the feature, and the
axis is the feature's, re-expressed in that frame. A circle gives both
(centre and axis); a plain point on a face gives the hit and the face
normal; a vertex or a box corner gives a position and **nothing** about
direction, so the axis is left alone — inventing one from a corner is a
decision the user cannot see. Nothing in the world moves; only the pivot
does, and the status bar repeats the fit it placed on.

While a placement tool is active the viewport's *select* click is
suppressed (`set_select_suppressed`) while its hover keeps running — the
click means "put it here", and the hover is what the snap is computed from
— and a glyph never takes the pointer, because the selected joint's own
glyph sits exactly where the user is aiming.

The marker is cyan and carries the fit's own confidence —
`circle r 12.0 mm · 24 seg · res 0.01 mm` — so a bad fit is obvious rather
than silent. The fit is memoised per `(instance, triangle)` and the welded
adjacency is cached beside the loaded mesh, so a resting cursor fits once.

Gizmos come from `transform-gizmo-egui` (ADR-0007), behind
`app/gizmo.rs` — the only file that names the crate — fed the viewport's
view/projection matrices as `mint` matrices and drawing with egui's painter
over the viewport, not depth-tested. Both it and the ID buffer want the
mouse, and the gizmo wins: its interaction widget is registered *after* the
viewport's rect in the same layer, so egui's hit test gives it the click,
and `Viewport::set_input_suppressed(bool)` — driven from
`Gizmo::is_focused()`, one frame late — turns the viewport's camera input
and picking off wholesale while it owns the cursor. The toolbar is
registered after the gizmo in turn: viewport < gizmo < toolbar.

What the gizmo edits follows the selection (plans/m2-placement-ux OPEN 2): a
**link** moves through its parent joint's `origin` (one `SetJoint` via
`fk::origin_for_world`; the subtree follows), a **joint** moves its pivot
alone (one `MoveJointFrame`; the axis is expressed in the child frame, which
is the frame the gizmo just moved, so it rides along unchanged and nothing
in the world moves). Drag previews through `preview_world`, release commits.

`Viewport::project(DVec3) -> Option<Pos2>` is the one projection everything
drawn over the viewport goes through — glyphs, snap markers, a scripted
click aimed at a part (`RiggenApp::project_world`) — so an overlay can never
disagree with the wgpu pass about where a point is: both start from
`camera.view_proj`.

## Jobs and threads

There is no evaluator. The only long-running work is mesh loading, convex
hull / primitive fitting, and export. Mesh loading currently runs
synchronously on the UI thread: every route in — CLI arguments,
drag-and-drop, File › Open — ends in `RiggenApp::load_files` (through the
dirty check when a `.riggen` is among the files) and fits the view
afterwards, and `sync_scene` loads a document's assets from their paths;
`riggen-app::jobs` arrives with the hull work and runs them on a
`std::thread` with an `mpsc` channel and a `wake` callback bound to
`ctx.request_repaint()`; results are drained once per frame. On wasm the
same API runs inline (no threads from eframe on the web); nothing else in
the app cares which. This is RoboCAD's `EvalExecutor` shape without the
generation machinery — a job carries the `MeshId` or export request it was
made for, and a result for an id that no longer exists is dropped.

## File format

`robot.riggen` is JSON: `{ "schema_version": 1, "robot": Robot }`
(02 §Schema). Mesh paths are **absolute in memory and relative to the
`.riggen` file on disk**, forward slashes: `riggen_core::save` rebases
them on the way out and `load` resolves them on the way in, so nothing
outside `file.rs` ever meets a relative path. `riggen_core::absolute`
(absolute + lexical normalization) is the one way a path enters the
document — `std::path::absolute` alone keeps `a/../b`, and two spellings of
one file would compare unequal. Every asset carries an FNV-1a 64 hash of
its mesh bytes, taken at registration; `load` recomputes it and reports a
mismatch or an unreadable file as a `Warning` (shown in the status bar),
never an error — the document opens, the user is told. Geometry is never
embedded; assets no geom references are dropped on save; the write goes
through `<name>.riggen.tmp` and a rename so a crash leaves the old file. A
schema bump comes with an `upgrade_vN_to_vN+1` and a corpus test that keeps
every old version opening forever (RoboCAD's rule).

In the app, `save_to` marks the history depth as saved and the status bar
and window title show `name.riggen*` until then; the unsaved-changes modal
guards every route that would drop a dirty document (§Panels and menus).

Export writes a directory: `<name>.xml` / `<name>.urdf` plus `meshes/`,
with mesh paths in the style the target expects (`package://`, relative, or
absolute — a user setting).

## Python distribution (ADR-0002)

MVP: maturin `bindings = "bin"` packages the `riggen` executable into the
wheel and generates the `riggen` console script. No PyO3, no extension
module, no GIL — the process is the app, exactly as if installed with
`cargo install`. `python -m riggen` is a two-line `__main__.py` that execs
the bundled binary.

v0.2: `riggen-py` is a PyO3 `cdylib` over `riggen-core` + `riggen-export`
exposing `Robot`, `Link`, `Joint`, `fk`, `validate`, `export_mjcf`,
`export_urdf`, `load_urdf`. `riggen.show(robot)` serialises to a temp
`.riggen` and spawns the bundled binary on it (the `rr.spawn()` model). The
GUI is never entered from inside a Python call.

## Testing

- `riggen-mesh`, `riggen-core`, `riggen-export`: plain unit tests; no GPU,
  no egui. This is where correctness lives.
- **Round-trip FK test** (`riggen-export/tests`): build a reference arm →
  export URDF → parse with `urdf-rs` → compute FK independently from the
  parsed file → compare end-effector poses over a joint-space grid against
  `riggen-core::fk`. Catches frame-convention bugs mechanically.
- **MuJoCo load test** (`python/tests`, CI only): `mujoco.MjModel.from_xml_path`
  on the exported MJCF must succeed with zero compiler warnings, and
  `mj_forward` body positions must match our FK.
- **Visual snapshots** (`riggen-app/tests/visual`, ADR-0003): `egui_kittest`
  drives the real `eframe::App` headlessly through wgpu (CPU adapter via
  lavapipe, so local and CI agree) and diffs PNGs. This is how an agent sees
  the window. A `debug_state()` JSON dump of what the app believes it drew
  (camera with near/far, the document — file, dirty, links, joints with
  `q`, selection — the `ui` section — rename in progress, open windows,
  modal, title — instances with their link/geom key, position and colour,
  viewport selection, status, viewport rect) accompanies every snapshot as
  a golden of its own; every float in it is rounded to six decimals and
  `-0.0` normalised so goldens never churn. At runtime the same JSON is
  under Debug › Copy / Save state (JSON), beside egui's layout overlays.
  The scenarios: `startup`, `cube`, `hover_cube`, `select_cube`,
  `three_parts`, `pendulum`, `mm_scale_part`, `tree_pendulum`,
  `tree_reparent`, `properties_link`, `properties_joint`, `pendulum_swing`,
  `materials`, `toolbar`, `gizmo_move_link`, `gizmo_rotate_joint`,
  `glyph_revolute`, `glyph_prismatic`, `glyph_hover`, `snap_vertex`,
  `snap_circle`, `place_joint_bore`, `align_concentric`, `five_minute_arm`,
  `dirty_title`, `unsaved_confirm`, `debug_menu`, plus
  golden-less app tests including `build_pendulum_numerically`, the M1
  acceptance in executable form.
  The harness sets the import scale to `1.0` (the fixtures are unit cubes
  meant as meters; the app's default is mm). Harness facts that must not be
  rediscovered:
  - `Harness::step()` runs egui's logic pass only, no GPU work; anything that
    depends on the GPU having run (the ID-buffer pick) needs `pump_rendered`.
    `step()` also drains every queued event in one go, running one logic
    pass each, so a viewport click is one raw event per rendered frame
    (`click_at`), not `drag_at`/`drop_at`; an egui widget needs only
    `get_by_label("arm").click()` + `step()`.
  - A new popup or window is laid out invisibly on its first frame: its
    AccessKit nodes exist at once, its pixels one frame later. `settle()`
    (or one more `step()`) before a capture, or the menu is missing from
    the PNG while every query on it passes.
  - `click_widget(harness, label)` clicks a widget that floats **over** the
    viewport (the toolbar, later the gizmo) with a real pointer, through
    `click_at`. `Node::click()` cannot: it queues press and release
    together, `step()` runs one unrendered logic pass per queued event, and
    a pick issued by the pointer moving in is then recorded by a frame that
    is never rendered — it stays in flight forever and the next `settle`
    waits for a readback that cannot arrive.
  - `synthetic_drag(harness, from, to, steps)` presses, walks the pointer
    and releases with a rendered frame between each event. A gizmo drag is
    *frames*, not a pair of queued events: `step()` runs every queued event
    in one unrendered logic pass, and the press, the moves and the release
    would never be seen apart. `RiggenApp::project_world` aims it — for the
    gizmo, `debug_state().gizmo.screen` is its view-plane handle.
  - kittest cannot drag a tree row onto another: `tree_reparent` reparents
    through the command API and only draws the result. A synthetic drag
    (press, `PointerMoved` in steps, release) does work for a one-off check.
  - Fields are found by their label (`labelled_by`): `get_all_by_label("x")
    .nth(0)` is the origin's x, `.nth(1)` the axis's; a `ComboBox` by
    `Role::ComboBox`, its items by label once open; a closed combo does not
    expose its selected text. Menus: `get_by_label("Window").click()`,
    `step()`, then the item. A slider is set exactly through the AccessKit
    `SetValue` action (`harness.event(AccessKitActionRequest { .. })` with
    the node's `locate()`) — its accessibility rect spans the value box, so
    a rail click is not exact; its value is read with
    `NodeT::accesskit_node().numeric_value()`.
  - `settle()` pumps until `RiggenApp::settled()` — no camera animation, no
    pick in flight — has held for four frames.
  - Scenarios serialise on a global `Mutex`: parallel lavapipe devices at
    1440×900 segfault inside the driver.
  - `UPDATE_SNAPSHOTS=1` refreshes the PNG **and** the JSON golden; look at
    the `.diff.png` before committing, and the commit says `snapshots:` why.
  - `kittest.toml` at the workspace root: `threshold = 0.6`,
    `max_failed_pixels = 64` — driver-revision tolerance, not a place to
    hide a regression.
  - `tests/visual_scratch.rs` is `test = false` and run by name
    (`cargo test -p riggen-app --test visual_scratch -- --nocapture`); it
    writes `target/visual-scratch/scratch.{png,json}` and compares against
    nothing — the "show me the app right now" path. `RIGGEN_SCRATCH_OPEN=
    <path>` (relative to the workspace root) opens a document or mesh and
    fits the view before the capture, so no tracked file is edited to look
    at one. `with_app()` runs a body against the real app with no goldens
    at all. The `visual-debug` skill is the how-to for all of this.
  - A scenario prints `SKIPPING` when no wgpu adapter exists. That is an
    environment failure, not a pass; CI installs `mesa-vulkan-drivers`.
- CI: `cargo fmt --check`, `clippy -D warnings`, `cargo test`, and a
  `wasm32-unknown-unknown` **build** of `riggen-app` (build check only; the
  web build is not a product in v1).
