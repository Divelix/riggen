# 01 — Architecture

## Layer map

Strictly layered; dependencies point downward only. `riggen-core` and
`riggen-export` must compile without egui or wgpu — they are what the
Python extension module links against, and they are where the tests that
matter live.

```
┌────────────────────────────────────────────────────────────────┐
│  riggen-app       eframe shell, panels, gizmos, drag-drop,     │  binary
│                   selection, snapping, export dialog, the CLI  │  (the wheel's
│                   (--export, --example, --version), snapshots, │   scripts/)
│                   and the wasm cdylib the web demo loads       │  + cdylib
├────────────────────────────────────────────────────────────────┤
│  riggen-viewport  wgpu renderer via egui_wgpu callbacks:       │
│                   instances, camera, ID-buffer picking,        │
│                   project / cursor_ray, Overlay (world-space   │
│                   segment, polyline, arc, point, label)        │
├──────────────────────────────┬─────────────────────────────────┤
│  riggen-export               │  riggen-py                      │  cdylib
│  resolve → ResolvedRobot,    │  PyO3 abi3 extension module     │  (the wheel's
│  MJCF + URDF + SDF writers,  │  riggen._riggen over core +     │   riggen/)
│  URDF + MJCF import, export  │  export; never egui or wgpu     │
│  dir, FK samples, round trip │  (ADR-0009)                     │
├──────────────────────────────┴─────────────────────────────────┤
│  riggen-core      Robot document (links, joints, frames,       │
│                   inertial spec, collision policy), FK,        │
│                   undo history, serde, schema versioning       │
├────────────────────────────────────────────────────────────────┤
│  riggen-mesh      TriMesh, STL/OBJ loaders, mass properties,   │
│                   convex hull, convex decomposition (decomp:   │
│                   parry3d-f64's V-HACD), primitive fits,       │
│                   ray/triangle                                 │
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

`riggen-core` depends on `riggen-mesh` for the `glam` re-export and for
`mass_properties`: a link's computed inertial is a function of its meshes,
which core reaches through the `MeshLookup` trait (02 §Inertials) rather
than storing — no geometry is in the document. `riggen-export` depends on
both plus `urdf-rs` (the URDF parser) and `quick-xml` (the MJCF one,
already in the tree under `urdf-rs`). Nothing below `riggen-app` knows
about selection,
hover, or gizmos.

No layer reaches the filesystem directly either. Every reader takes a
`riggen_core::FileSource` — `read`, plus `exists` and `hash` over it —
whose implementations are `Disk` (`std::fs`) and `MemorySource` (bytes by
path), and `riggen-app` adds `DroppedSet`, the files of one browser drop
resolved by name (ADR-0017). That is what lets the same `riggen-core` and
`riggen-export` run in a page with no filesystem behind them (§File format).

## Cargo workspace

```
riggen/
├── Cargo.toml              # [workspace], resolver 3, edition 2024, every dep version;
│                           # [profile.release] strip + thin LTO: the wheel's binary;
│                           # [profile.web] opt-level "s" + fat LTO: the wasm download
├── pyproject.toml          # the `riggen` wheel: maturin `bindings = "pyo3"` over
│                           # crates/riggen-py, python-source = python/, the binary from
│                           # riggen._riggen.data/ (ignored; the build fills it), version
│                           # from Cargo, readme = README.md (§Python distribution)
├── README.md               # user-facing, uv first; also the PyPI page
├── .cargo/config.toml      # one rustflag, wasm32 only: getrandom's backend (ADR-0007)
├── rust-toolchain.toml     # stable + rustfmt, clippy, wasm32-unknown-unknown
├── kittest.toml            # snapshot thresholds (ADR-0003); found by walking up from the crate
├── .github/workflows/      # ci.yml (§Testing), release.yml (§Python distribution),
│                           # pages.yml (the web demo, §The web build)
├── web/                    # the demo's page: index.html, main.js (the WebGPU probe,
│                           # the canvas, the panic sheet), build.sh → web/dist/
│                           # (gitignored: the wasm-bindgen bundle plus the page)
├── crates/
│   ├── riggen-mesh/        # TriMesh, Aabb, Ray, load_stl / load_obj / load_mesh, feature/,
│   │                       # mass, hull (quickhull), decomp (V-HACD, ADR-0011), fit
│   ├── riggen-core/        # ids, pose, robot, validate, fk, command, history, file, inertial
│   ├── riggen-export/      # resolve, mesh_store, mjcf, urdf, sdf, export, fk_samples, xml
│   │                       # (both halves: the writer and a quick-xml DOM), import (the
│   │                       # warning and error vocabulary both imports speak), urdf_in, mjcf_in
│   ├── riggen-viewport/    # camera/, scene, pick_id, gpu_mesh, overlay, viewport/, shaders/
│   ├── riggen-app/         # bin "riggen"; the cdylib the web demo loads; tests/visual,
│       │                   # tests/cli.rs (the built binary from a shell)
│       ├── src/example.rs  # the bundled sample arm's bytes: --example arm unpacks
│       │                   # them, the web build opens them as a drop (ADR-0017)
│       ├── src/download.rs # the browser's way out: a stored zip and a Blob
│       │                   # download (ADR-0017 §6); wasm and cfg(test) only
│       ├── build.rs        # RIGGEN_GIT_HASH / RIGGEN_BUILD_DATE for `--version`
│       ├── src/jobs.rs     # the job thread: Jobs, Job, JobKey, JobResult (§Jobs and threads)
│       ├── src/cli.rs      # the flag table, --help, --version, --example, `riggen
│       │                   # --export …` headless (ADR-0008)
│       ├── src/app/        # document, file_io, file_menu, export_dialog, debug_menu,
│       │                   # shortcuts, status_bar, tool, gizmo, glyphs, snap, align,
│       │                   # panels/{tree, properties, joints, materials}
│       └── src/debug/      # debug_state(): what the app thinks it drew, as JSON (ADR-0003)
│   ├── riggen-py/          # cdylib `_riggen`, the PyO3 abi3 extension module `riggen._riggen`
│   │                       # over core + export; `test = false`, tested from Python (ADR-0009)
│   └── riggen/             # the crates.io name reservation: an empty 0.0.1 lib with its
│                           # own README; publishing the app under this name is a backlog
│                           # line (SEED.md §5)
├── assets/fixtures/        # cube_binary.stl, cube_ascii.stl, cube.obj — the unit cube
│                           # (TriMesh::cube(0.5)) in every format; pendulum.riggen, the
│                           # .riggen v1 corpus file (02 §Schema); arm/*.stl in mm, the
│                           # M2 acceptance's four parts plus fore_hull.stl (an ignored
│                           # generator test writes them); arm/arm.riggen, the M3 sample
│                           # robot (`write_arm_sample`), and arm/arm.urdf, the hand-written
│                           # URDF import corpus file (02 §URDF import). arm.riggen and its
│                           # four STLs are also `include_bytes!`d for `--example arm`;
│                           # bracket.stl (a U-channel) and bracket.riggen, the convex
│                           # decomposition fixture and the `mujoco` and `sdf` jobs'
│                           # third model;
│                           # menagerie_style.xml, the foreign MJCF import corpus
│                           # (02 §MJCF import) — hand-written, not ours
├── python/riggen/          # the wheel's Python half: __init__ (the public names,
│                           # __version__), robot.py (the API), show.py (the window,
│                           # binary_path), errors.py, __main__ (execs the bundled
│                           # binary), _riggen.pyi + py.typed
├── examples/               # pendulum.py (the README's ten lines), arm.py (the M2 arm
│                           # from its STLs) — the SDK's worked examples (§Python SDK)
├── python/build_wheel.py   # the one build recipe: cargo build riggen-app → the data
│                           # directory → maturin build (§Python distribution)
├── python/tests/           # test_mjcf_load.py (MuJoCo load + FK), test_sdf_load.py
│                           # (libsdformat load + FK) and test_wheel.py (the installed
│                           # wheel, headless) — plain scripts; sdk/ the SDK's pytest
│                           # suite, run on the built wheel (§Testing)
├── scratch/                # gitignored: personal notebooks on the dev build (README §Developing)
├── LICENSE-MIT, LICENSE-APACHE   # "MIT OR Apache-2.0"; the wheel's license-files
├── docs/                   # 0N design docs, adr/, ideas/, plans/, assets/arm.png (the README hero)
├── SEED.md
└── AGENTS.md, CLAUDE.md
```

Dependency policy (ADR-0001): egui/eframe/egui-wgpu 0.36.x from crates.io,
wgpu version dictated by egui-wgpu — never depend on a different wgpu.
`glam` 0.30 with the `serde` and `mint` features, re-exported as
`riggen_mesh::glam`; no other crate lists it. `transform-gizmo` pins its own
`glam ^0.32` and `parry3d-f64` — the V-HACD convex decomposition, at its
default features, `riggen-mesh`'s dependency alone (ADR-0011) — pins
`glam 0.33` through its `glamx` bridge, so **three** versions are in the
lock file. They never meet: `mint` is the gizmo's boundary and
`riggen_mesh::decomp` is parry's, converting component-wise through `f64`,
and nothing of ours names 0.32 or 0.33 (ADR-0007, ADR-0011). Every version
lives once in `[workspace.dependencies]`; crates say `.workspace = true`. Local checkouts
of egui and rerun under `~/Documents/code/rust/` are reference reading only;
no `path =` or `[patch]` unless an unreleased fix is needed, and then with a
comment saying which one. Profile settings carried from RoboCAD:
`opt-level = 1` for our crates in dev, `3` for dependencies — an unoptimized
wgpu is felt. A dependency's default features are checked against the wasm
build (`tobj`'s `ahash` default pulled `getrandom` in, which does not compile
for `wasm32-unknown-unknown`; it is off).

`riggen-export`'s `lib.rs` carries the "no egui/wgpu" rule in its doc
comment and nothing else — the crate is its modules; `riggen-core` keeps
the same rule and depends on `serde` / `serde_json` and nothing else. `eframe` is built with its
`persistence` feature so the import-units choice (and egui's own window
layout) survive a restart through eframe storage.

## The document is the only state

`riggen-app` owns one `Robot` (02-data-model) plus derived, never-saved
state:

```rust
pub struct RiggenApp {
    robot: Robot, history: History, file: Option<PathBuf>,
    mesh_store: HashMap<MeshId, LoadedMesh>,            // raw file mesh + the scaled/fixed-up Arc<TriMesh>,
                                                        // its welded adjacency and convex hull, cached on first use
    instances: BTreeMap<(LinkId, GeomId), InstanceId>,  // the only map between document and scene
    collision_instances: BTreeMap<(LinkId, usize), (InstanceId, CollisionSource)>, // translucent shapes,
    show_collision: bool,                               // per link and shape index, while View › Collision geometry is on
    jobs: Jobs,                                         // the job thread (§Jobs and threads), drained once per frame
    decomp: HashMap<(MeshId, DecompParams), DecompState>, // convex pieces it produced; the document holds the
                                                        // parameters and never the pieces (ADR-0011)
    q: JointState, selection: Selection,                // None | Link(LinkId) | Joint(JointId) | Frame(FrameId)
                                                        // a mimic joint's slot in `q` is ignored: `joint_value`
                                                        // answers with `fk::resolve_q`'s derived one (ADR-0013)
    tool: Tool, gizmo_state: GizmoState,               // Select | Move | Rotate | PlaceJoint | Align; the gizmo and its drag
    preview_world: Option<(LinkId, Pose)>,             // a link's pose while a gizmo drag previews it
    hovered_joint, glyph_hover, snap_candidate,        // resolved every frame from the pointer
    hovered_frame, frame_glyph_hover,                  // the same pair for a frame's triad glyph
    snap_cache, align_source, toolbar_rect,            // the memoised fit, the align gesture's first pick
    import_scale: f64, pending: Option<PendingAction>,  // File › Import units; New/Open/Quit awaiting the dirty answer
    export_dialog: ExportDialog,                        // File › Export…: options, directory, the resolve errors
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
With View › Collision geometry on, `sync_collision` derives one translucent
instance per collision shape the link's policy resolves to — a cached hull
per visual mesh for `ConvexHull`, **every piece** of the cached
decomposition for `ConvexDecomposition` (nothing until its job lands, and
`drain_jobs` re-syncs on the frame it does), a generated mesh per
primitive, each `Meshes` geom; nothing extra for `None` / `SameAsVisual` —
at the same FK poses, re-uploading only when a shape's source changed
(`CollisionSource`, whose `Piece(MeshId, DecompParams, usize)` makes an
edited parameter a different source); with it off they are all removed. `q` is pruned of vanished joints and clamped to freshly edited limits on
the way, and `preview_world` — a link's pose while a gizmo drag is in
flight — is applied as a correction to that link and its whole subtree, so
the parts follow the handle exactly as they will after the commit while the
document stays untouched. Opening a document (`replace_document`) resets
history, selection, `q` and the mesh store, then syncs; the meshes come
from the assets' paths.

Selection is document-level and mirrored both ways: a viewport click
selects the link owning the hit instance (`sync_selection_from_viewport`,
once per frame after `Viewport::ui`), and selecting in the tree calls
`Viewport::set_selected` with the link's first instance. A selected joint
or frame has no instance; a click on empty viewport space therefore leaves
such a selection alone (the viewport reports `None → None`).

A `Tool` is modal: it decides what a viewport click and drag mean. `Select`
is the M1 behaviour and the resting state, and `Esc` always returns to it
(consumed only while a tool is active, so the rename / modal / field-revert
uses of Escape still see it). The four editing tools commit frame-rewriting
commands, which work in the **zero configuration**, so `set_tool` resets `q`
first when something is off zero and says so in the status bar
(plans/m2-placement-ux OPEN 1). Resetting `q` is not an edit and adds no
history entry. A tool **says what it needs**: on entry and on every
selection change while it is active, the status bar carries the
selection it is waiting for — Move / Rotate with nothing or the root
selected, Place joint without a joint, Align without a (non-root) link —
and drops the line once the selection satisfies it, so a click the tool
would ignore has its reason beside it (`MOVE_NEEDS_TARGET` and the other
public constants in `tool.rs`; the zero-configuration line takes
precedence on the entry that rewound the sliders).

Granularity rule, kept from RoboCAD and sharpened in v0.3: **one gesture =
one history entry.** A gizmo drag mutates a *preview* pose every frame and
commits one command on release; a scrubbed number field previews *through*
the document, one command per frame, and `History::apply_in_gesture`
coalesces them into the one entry (02 §Commands and history); a slider
preview of a joint angle is not a command at all (joint values are derived
state, not document state — the document stores limits, not the current
`q`). Concretely: properties-panel numbers are scrubbers (egui's
`DragValue` with our formatter): a horizontal drag changes the value at one
percent of its magnitude per point, never less than the field's unit floor
(a millimetre, a tenth of a degree, a gram, a nano-kg·m²), a tenth of that
with Ctrl, one `Set…` per frame under the field's `GestureId` and
`end_gesture` on release; **Ctrl+wheel** over a field steps it by one unit
of the last digit it shows (`1240` → 1, `0.5` → 0.1, a field at `0` by its
unit floor), a burst of notches within 0.4 s being one entry — Ctrl because
egui routes a wheel with its zoom modifier away from scrolling, so a plain
wheel keeps scrolling the panel wherever the cursor is (plans/panels-and-
numbers OPEN 2); a click opens the text editor, which commits on
Enter or lost focus, never per keystroke, and reverts on Escape; a commit
equal to the shown value never becomes a command (`History` drops the
remaining no-ops); the materials table's colour picker keeps a draft,
tints the viewport live and sends one `UpsertMaterial` when the popup
closes.

### Panels and menus

- **Links** (left): one row per link with its parent joint's name and kind
  (`hinge · revolute`), and under it — before its child links — a row per
  named frame (`⌖ tcp   frame`, ADR-0012); click selects the link, the
  joint or the frame, double-click or F2 renames a link or a frame inline,
  "+ Link" adds an empty link under the selection and "+ Frame" a named
  frame at that link's origin (both start the new row's rename), Delete /
  "− Remove"
  removes the subtree (root refused, reason in the status bar) or, for a
  selected frame, that frame alone, dragging a row onto another
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
  hovered the viewport's own **picking** is suppressed
  (`set_pick_suppressed`), so the part behind it is not highlighted as well
  and a click selects the *joint* — the camera keeps the pointer, and the
  wheel still zooms (ADR-0010).
- **Frame glyphs** (in the viewport): a frame has no geometry either, so
  each is drawn as a triad in the axes triad's colours at its world pose
  (`world(parent) ∘ frame.pose`) with its name as a label beside it. Every
  frame is drawn, always — there are a handful and the user placed each on
  purpose, unlike a weld. Sized from its link's glyph size. Hover runs both
  ways as it does for joints — row ↔ glyph, nearest triad arm within
  `GLYPH_HOVER_RADIUS`, `tcp (frame)` in the status bar beside a joint's
  `hinge (joint)`, picking
  suppressed so a click selects the frame — and a frame glyph wins the
  pointer over a joint's, whose long axis line often runs straight through
  it.
- **Properties** (right): a link's name, material, and per geom the pose
  (xyz m, RPY °), asset scale and fix-up, "Add mesh to this link…"; then
  **Inertial** — the `InertialSpec` mode combo (Computed / Override /
  Hybrid) with its fields (density override; mass, CoM and the six tensor
  entries; mass) and, beside them, what the meshes say (mass, CoM,
  principal moments) or why they say nothing ("open mesh: <file>" in
  warning colour) — and **Collision** — the policy combo (None / Same as
  visual / Convex hull / Convex decomposition / Primitives; `Meshes` is
  not offered but, when an import carries it, edited geom by geom: the
  file, pose rows, Remove, and "Add file…" through the file seam, each
  commit one `SetCollision`), for Primitives the list
  with "+ Box / Cylinder / Sphere / Capsule" (each fitted to the link's
  meshes on creation), Fit to mesh, Remove, pose and size fields, and for
  the decomposition its three parameters (max pieces, voxel grid,
  concavity — capped at 64 pieces and a 256³ grid, so a typo cannot ask
  for a thousand geoms) beside the job thread's own "pieces: N", a spinner
  while it runs, or the reason there are none. Every
  commit is one `SetInertial` / `SetCollision`. A joint's name, kind
  (limits appear with Revolute/Prismatic, defaulting to ±π / ±1 m),
  origin, axis (normalised on commit), limits in ° or m, dynamics. A
  frame's name, the link it hangs on (a combo — changing it keeps the frame
  where it is in the world, the panel re-expressing the pose through `fk`
  in the zero configuration so `SetFrame` stays as dumb as `SetJoint`) and
  its pose in that link's frame as xyz (m) and RPY (°); whatever changed,
  one `SetFrame` (ADR-0012). Fields
  are `labelled_by` their labels for the accessibility tree. Every number
  field scrubs (§Tools above). Every number, field or readout, is shown by
  one `fmt_num`: six significant figures,
  scientific below `1e-3` (`2.86e-5`, never `0.000029`), zero below
  `1e-12` (round-off; the writers keep twelve decimals, so nothing
  smaller reaches a file), and a field sized to its text accepts either
  spelling — "changed" means changed at that precision.
- **Window › Joints** / **Materials**: floating windows. Joints: one
  slider per movable joint in its limits (Continuous ±180°), writing `q`
  and syncing every frame, "Reset all". It **opens itself**: when a
  document with a movable joint replaces the current one (and closes when
  one without does), and when a command creates the document's first
  movable joint; the user closing it — the title bar, the menu — is
  respected until the next document (plans/panels-and-numbers OPEN 3).
  Materials is closed until asked for. Materials: name /
  density / colour rows, add and remove (refused while a link uses it),
  and the name renamed inline — double-click it or press F2 over it, the
  tree's idiom; Enter commits one `RenameMaterial` (refused onto a taken
  name), Escape leaves it — every link that used it following.
- **Debug**: egui's `DebugOptions` overlays (debug on hover, widget hits,
  interactive widgets, width / height expansion, resize, unaligned) toggled
  in both themes at once, then **Copy state (JSON)** / **Save state
  (JSON)…** — the runtime route to `debug_state()` (§Testing) for a state
  reached by hand rather than by a scenario.
- **File**: New, Open…, Save (Save As when untitled), Save As…, Import
  URDF…, Import MJCF…, Export…, Import units, Quit; **Edit**: Undo, Redo, Delete, greyed
  out when idle; **View**: Collision geometry (off by default, remembered
  through eframe storage). The window title is `name.riggen* — riggen`.
  Every route that would drop a dirty document — New, Open, a dropped
  `.riggen`, `.urdf` or `.xml`, Import URDF…, Import MJCF…, Quit, the OS
  close button (refused
  with `CancelClose` until answered) — goes through one `PendingAction`
  and the Save / Don't save / Cancel modal. **Export…** is a second modal
  (`export_dialog.rs`): format — three checkboxes, MJCF / URDF / SDF, all
  ticked by default because the format is a *set* (ADR-0016) — directory
  (`rfd`), mesh path style (URDF and SDF), MJCF floating base; it resolves
  the document on open and on every option change and lists each
  `ExportError` with the link it names, the Export button disabled while
  any exist or while no format is ticked; success is the status bar's
  `exported N files to <dir>`. In a browser there is no dialog and no
  directory to choose: Open and the two Imports point at the drop gesture
  instead, the export row reads `download   <name>.zip`, and Save, Save As,
  Export and Debug › Save state each hand the browser a file (§The web
  build, ADR-0017).
- **Shortcuts** (`shortcuts.rs`, run before the panels each frame): Ctrl+N
  / O / S / Shift+S fire always; Delete, F2, Ctrl+Z, Ctrl+Shift+Z and Ctrl+Y
  yield while a `TextEdit` has focus (`TextEdit::load_state` on the focused
  id — a clicked button holds focus too and must not block Delete), and the
  shifted pattern is consumed before the bare one because egui matches
  modifiers logically.

## Frame loop

```
input ──► shortcuts ──► menu bar, status bar, tree, properties
       ──► central panel:
             joint + frame glyphs from (Robot, q) ──► glyph hover ──► snap candidate
             viewport.set_overlay(glyphs + frame triads + align pick + snap marker)
             viewport.set_pick_suppressed / set_pointer_blocked / set_select_suppressed
             viewport.ui ──► gizmo ──► toolbar   (registration order = pointer precedence)
             a click ──► select a joint or frame / place a joint or frame / align
       ──► Commands ──► History ──► Robot
Robot ──► fk(robot, q) ──► world pose per link
       ──► for each visual geom: viewport.set_instance_model(instance, link_pose * geom.pose)
       ──► viewport.ui(...)   (records the wgpu callback; picks resolve next frame)
```

The viewport draws **instances**, not links: one instance per `(LinkId,
GeomId)` visual, with the uploaded `TriMesh` shared by `MeshId` (scale and
fix-up applied once at load through `TriMesh::transform`). Moving a joint
writes matrices; nothing is re-uploaded. Collision geometry is a second
instance set in the same scene with `RenderGroup::Translucent`: drawn
after every opaque instance through a second scene pipeline (alpha-blended,
depth-tested, no depth write) and skipped by the pick pass, so a hull over
a part never takes the part's hover or click.

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
triangle is a readout only. A resolved select pick is also an **event**:
`take_select_result() -> Option<Option<PickHit>>` hands the app each
click's verdict once, hit or miss, so a click on empty space is
distinguishable from no click and **clears whatever is selected** — a
joint's or a frame's selection included, which the viewport never held.
Under a snapping tool (or a frame under Move / Rotate) the select pick is
suppressed altogether, so that click clears nothing.

`riggen_mesh::ray_triangle` (Möller–Trumbore, two-sided — the ID buffer has
already chosen the triangle) recovers the exact hit point by intersecting
`Viewport::cursor_ray` with that one triangle, taken into mesh space, so
snap targets never need a spatial index. The 5×5 region means the named
triangle can be the cursor's *neighbour*, and the exact ray then misses it
by a pixel; its plane is the fallback. `app/snap.rs` builds the
candidates and picks among them by a fixed ladder — **vertex > box >
circle > point** — with the winner, its axis and its readout in
`debug_state().snap`. Only the placement tools snap (`Tool::snaps`);
markers under the cursor while merely selecting would be noise. Move and
Rotate join them for a **selected frame** (`RiggenApp::placing_frame`,
`snapping`): a frame is the one thing the gizmo edits that nothing hangs
off, so a click puts it on the picked feature — Move takes the point and
keeps the orientation, Rotate keeps the point and turns the frame's +Z onto
the feature's axis — and a TCP lands on a bore or a corner without a
coordinate typed (ADR-0012). One `SetFrame` per click, and the gizmo is
still there for the rest of the gesture.

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
  B-Rep, and it was the milestone's risk item;
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

Whenever the snap ladder runs — a placement tool, or Move / Rotate with a
frame selected (`RiggenApp::snapping`) — the viewport's *select* click is
suppressed (`set_select_suppressed`) while its hover keeps running: the
click means "put it here", and the hover is what the snap is computed from.
A glyph never takes the pointer then either, because the selected joint's
or frame's own glyph sits exactly where the user is aiming.

The marker is cyan and carries the fit's own confidence —
`circle r 12.0 mm · 24 seg · res 0.01 mm` — so a bad fit is obvious rather
than silent. The fit is memoised per `(instance, triangle)` and the welded
adjacency is cached beside the loaded mesh, so a resting cursor fits once.

Gizmos come from `transform-gizmo-egui` (ADR-0007), behind
`app/gizmo.rs` — the only file that names the crate — fed the viewport's
view/projection matrices as `mint` matrices and drawing with egui's painter
over the viewport, not depth-tested. The egui glue is **ours**, not the
crate's `GizmoExt::interact` (ADR-0010): that one registers a
click-and-drag widget at the cursor on every frame, and egui's hit test
prefers the widget registered last, so any gizmo on screen took the whole
pointer — hover, click and wheel — from the viewport under it.
`app/gizmo.rs` hit-tests the handles itself with `Gizmo::pick_preview`,
which asks the subgizmos directly and needs no widget, and registers one
only while a handle is under the cursor or a drag it started is in flight —
and that widget senses **clicks only**. It exists to deny the viewport the
click under a handle, nothing more; the gizmo reads the raw pointer, never
the widget's response. Sensing drags as well would take the *middle* drag
too, because egui sets `potential_drag_id` from `hits.drag` on a press of
any button, and orbit would land on a widget that does not orbit.
Click-only, the hit test reports `click: gizmo, drag: viewport` — so orbit
and pan start from a handle like anywhere else. Everywhere else the
viewport keeps the pointer it always had. The toolbar
is registered after the gizmo in turn: viewport < gizmo < toolbar.

The viewport takes that policy through **three** switches, because "the
pointer is busy" has three different meanings:

| Switch | Off | Set by |
|---|---|---|
| `set_pick_suppressed` | both picks; the camera stays live | a gizmo handle, or a joint or frame glyph under the cursor — something drawn *in front of* the geometry that would answer |
| `set_select_suppressed` | the select pick; the hover keeps running | `snapping()`: a placement tool, or Move / Rotate on a frame — the click means "put it here" |
| `set_pointer_blocked` | camera **and** picks | the toolbar, which floats in the viewport's own egui layer; a gizmo drag in flight, which is solved against the projection it started in |

The gizmo's two are one frame late — it cannot say whether it owns the
cursor until it has run, and the viewport runs first — which is the same lag
egui's own interaction has. Camera input keys on `Response::contains_pointer`
rather than `hovered`: `contains_pointer` filters *layers* covering the
cursor but not same-layer widgets, so a floating window still takes the wheel
while a gizmo handle no longer freezes the camera — and the toolbar, being
same-layer, is what `set_pointer_blocked` is for.

What the gizmo edits follows the selection (plans/m2-placement-ux OPEN 2): a
**link** moves through its parent joint's `origin` (one `SetJoint` via
`fk::origin_for_world`; the subtree follows), a **joint** moves its pivot
alone (one `MoveJointFrame`; the axis is expressed in the child frame, which
is the frame the gizmo just moved, so it rides along unchanged and nothing
in the world moves), a **frame** moves on its link (one `SetFrame`; nothing
else moves, because nothing hangs off a frame — ADR-0012). Drag previews
through `preview_world`, or on the glyph itself for a frame; release commits.

`Viewport::project(DVec3) -> Option<Pos2>` is the one projection everything
drawn over the viewport goes through — glyphs, snap markers, a scripted
click aimed at a part (`RiggenApp::project_world`) — so an overlay can never
disagree with the wgpu pass about where a point is: both start from
`camera.view_proj`.

## Jobs and threads

There is no evaluator, and one job thread. `riggen-app::jobs` is RoboCAD's
`EvalExecutor` shape: a `std::thread`, an `mpsc` request/result pair, a
`wake` callback the app binds to `ctx.request_repaint()` so an idle window
does not sit on an undrained result, and `Jobs::drain` called once per
frame at the top of `ui` — before anything reads what it filled. A request
carries a `JobKey`, and `Jobs::request` drops a key already in flight, so
the frame may ask every frame. On wasm there is no thread: `request` runs
the job inline and queues the result, and app code does not branch.

**What runs on it: convex decomposition, and nothing else yet.** V-HACD is
seconds of work on a real part (ADR-0011), which is the first thing here a
frame could not absorb. `RiggenApp::request_decompositions` asks for every
`(MeshId, DecompParams)` the document's `ConvexDecomposition` links want
and do not have; `drain_jobs` moves results into `RiggenApp::decomp`;
`decompositions_pending()` is part of `settled()`, so a snapshot is never
taken over a half-computed collision view.

On wasm the run is inline and stops the page for seconds, so it is
*consented to* rather than merely allowed: `RiggenApp::decomp_consent`
starts false in a browser, `request_decompositions` returns while it is,
and the properties panel says the tab will stop responding and offers the
button that answers. Asked once per session, not once per link. A document
that wants a decomposition while the answer is outstanding is **not**
`decompositions_pending` — nothing is running (ADR-0011, ADR-0017).

**What still runs on the UI thread:** mesh loading, convex hulls and
export. Every route in — CLI arguments, drag-and-drop, File › Open, Import
URDF, Import MJCF — ends in `RiggenApp::load_files`, or in
`load_dropped` for the bytes of a browser drop, through the dirty check
when a document is among the files, and fits the view afterwards;
`sync_scene` loads a document's assets through `RiggenApp::files`
(§File format); a hull is computed the first
time the collision view or a fit asks for it and cached beside the loaded
mesh (`LoadedMesh::hull`); the export resolves in the dialog — on open and
again on every option change, ticking a format among them — and writes on
the button. The meshes riggen is built against make none of this
noticeable, so "async mesh loading via `jobs`" stays a backlog line.

## File format

`robot.riggen` is JSON: `{ "schema_version": 3, "robot": Robot }`
(02 §Schema). Mesh paths are **absolute in memory and relative to the
`.riggen` file on disk**, forward slashes: `riggen_core::save` rebases
them on the way out and `load` resolves them on the way in, so nothing
outside `file.rs` ever meets a relative path. On disk `riggen_core::absolute`
(absolute + lexical normalization) is what a path passes through on the way
in — `std::path::absolute` alone keeps `a/../b`, and two spellings of one
file would compare unequal. In a browser there is no working directory to
be absolute against, so a dropped file is given the synthetic `/dropped/`
(ADR-0017); either way what the document holds is absolute and normalized,
and nothing downstream learns a second kind of path. Every asset carries an FNV-1a 64 hash of
its mesh bytes, taken at registration; `load` recomputes it and reports a
mismatch or an unreadable file as a `Warning` (shown in the status bar),
never an error — the document opens, the user is told. Geometry is never
embedded; assets no geom references are dropped on save; the write goes
through `<name>.riggen.tmp` and a rename so a crash leaves the old file. A
schema bump comes with an `upgrade_vN_to_vN+1` and a corpus test that keeps
every old version opening forever (RoboCAD's rule).

Reading and writing are split from the filesystem, one function deep
(ADR-0017). `load(path)` reads the bytes and calls `load_from(text, base,
source)`; `save(robot, path)` writes what `to_json(robot, base)` returned,
through a temp file and a rename. `base` is where the document *is* — its
directory is what relative mesh paths are relative to, and it needs no
filesystem behind it. The same split runs through `MeshStore::load`,
`urdf_in::load`, `mjcf_in::load` and `riggen_mesh::load_mesh_bytes`, and
through `riggen_export::export_files(robot, options, dir)`, the export
directory as `(path, contents)` with `export()` the atomic writer over it.
So the desktop and the browser run the *same* readers and writers, with
`Disk` or a `DroppedSet` under them, and the browser is not a thinner
riggen.

In the app, `save_to` marks the history depth as saved and the status bar
and window title show `name.riggen*` until then; the unsaved-changes modal
guards every route that would drop a dirty document (§Panels and menus).

Export writes a directory (ADR-0008, ADR-0016): any of `<name>.xml`,
`<name>.urdf` and `<name>.sdf`
plus one `meshes/<stem>.stl` — every mesh baked to meters as binary STL,
`scale` and `fix_up` applied, `<stem>_hull.stl` beside it for hulls and
`<stem>_hull_0.stl …` for a convex decomposition (`<stem>_hull2_0 …` for a
second parameter set on the same mesh), no `scale` attribute anywhere —
each file through a `.tmp` sibling and a rename. A named frame needs no
file: it is a `<site>` in the MJCF, a `<frame attached_to>` in the SDF and
a massless link on a `_fixed` joint in the URDF (ADR-0012, ADR-0016). The
mesh path style is a dialog option read by the URDF and SDF writers
(`MeshPathStyle`: relative, `package://<name>/` — `model://<name>/` in the
SDF — absolute); MJCF has `meshdir`. `riggen --export
mjcf|urdf|sdf|both|all [--fk-samples] --out DIR INPUT` does the same
headlessly (`INPUT` is a `.riggen`, a `.urdf` or an `.xml`), returning
before eframe starts, which is what CI's `mujoco` and `sdf` jobs run. A `.urdf` opens as a new, untitled document through
`riggen_export::urdf_in` (02 §URDF import) and an `.xml` through
`riggen_export::mjcf_in` (02 §MJCF import, ADR-0015); both share one
warning vocabulary, so both reach the status bar the same way.

## Python distribution (ADR-0002, ADR-0009)

`pyproject.toml` at the repository root, build backend maturin (`>=1.8,<2`),
`dynamic = ["version"]` — the version is read from the workspace
`Cargo.toml` and lives once. `readme = "README.md"` is the root README, so
the GitHub page and the PyPI page are one file. One wheel per platform,
with **two halves**:

- **The extension module** `riggen._riggen`, built by maturin: `bindings
  = "pyo3"`, `manifest-path = crates/riggen-py/Cargo.toml`, `module-name
  = "riggen._riggen"`, `features = ["extension-module"]` (the crate
  feature that stops PyO3 linking libpython; off, the workspace checks
  build the crate like any other), `python-source = python`. PyO3 is
  `abi3-py310`, so the wheel is `cp310-abi3-<platform>`: one per platform
  for every CPython ≥ 3.10. `python/riggen/_riggen.pyi` and `py.typed`
  ship beside it.
- **The binary**, built by cargo from `crates/riggen-app` and copied by
  `python/build_wheel.py` into maturin's wheel data directory
  `riggen._riggen.data/scripts/` (maturin's default `<module-name>.data/`
  beside `pyproject.toml`; gitignored; deliberately not a `data = …`
  setting, which must exist or maturin fails — the default is skipped
  when absent). maturin packages it as `riggen-<ver>.data/scripts/
  riggen[.exe]`, which installs to the environment's `bin/` (`Scripts\`
  on Windows), exactly where M4's `bindings = "bin"` put it.
- **The command is the binary.** There is *no* `[project.scripts]` entry —
  a console script of the same name would shadow it and put a Python
  interpreter in front of the startup budget. `python -m riggen`
  (`python/riggen/__main__.py`) finds the binary through
  `riggen.show.binary_path` — `RIGGEN_BINARY`, else
  `sysconfig.get_path("scripts")` (the user-site layout second) — and
  `os.execv`s it; on Windows, which has no exec, `subprocess.call` +
  `sys.exit`. `riggen.__version__` is `importlib.metadata.version
  ("riggen")`; `_riggen.__version__` is `CARGO_PKG_VERSION` mapped to PEP
  440 (`0.2.0-dev` → `0.2.0.dev0`), so the two agree.
- **The recipe** is `python python/build_wheel.py [--target <triple>]
  [--binary-only]`: `cargo build --release -p riggen-app [--target T]`, the
  copy into the data directory, then `maturin build --release --out dist`
  (`maturin` from PATH, else `uvx maturin`). `--binary-only` stops before
  maturin — for the CI containers, where maturin-action runs maturin.
  Built in the tree, `build.rs` asks git for the hash, so no
  `RIGGEN_GIT_HASH` is needed locally; the workflows set it because their
  containers cannot ask git about a checkout another uid owns.
- **The sdist** carries `crates/riggen-py` and the three crates below it
  (maturin packages the extension's path dependencies; `riggen-app` is
  not one) and no data directory, so `pip install` from it — any platform
  outside the five, or `pip install .` — builds `import riggen` with a
  Rust toolchain and a `python3` and gets **no binary**; `python -m
  riggen` and `show()` say so and how to get one (a wheel, `cargo install
  --git`, or `RIGGEN_BINARY`; §Python SDK). Targets with wheels: linux x86_64 and
  aarch64 (manylinux 2_28), macOS arm64 and x86_64, Windows x86_64.
- Free-threaded CPython (3.13t / 3.14t) has no wheel: abi3 does not
  install there (BACKLOG).
- `[profile.release]`: `strip = true`, `lto = "thin"`, `codegen-units =
  1`. linux x86_64 at 0.2.0: the binary 21.9 MB, the extension 1.3 MB,
  the wheel 10.1 MB (M4's binary-only wheel was 9.6).
- `riggen --version` prints `riggen <cargo version> (<hash> <date>)`;
  `build.rs` takes the hash and date from `RIGGEN_GIT_HASH` /
  `RIGGEN_BUILD_DATE` when set, else from git (`-dirty` when the tree
  is), else `unknown` / today.
- `release.yml`: a `build` matrix of the five targets named as full
  triples, so one `${{ matrix.target }}` reaches maturin-action and
  `build_wheel.py --target` and both cargo runs agree on
  `target/<triple>/release/`. Linux runs both halves inside
  maturin-action's container (manylinux 2_28 for x86_64, the cross
  container for aarch64) — the binary half in `before-script-linux`, which
  the action runs after installing Rust and the target and putting
  `/opt/python/*/bin` on PATH. macOS (both architectures on the arm64
  runner) and Windows run `build_wheel.py --binary-only` as a step before
  maturin-action, after `dtolnay/rust-toolchain` and `setup-python`. Then
  the sdist; `smoke` installs the wheel into a fresh venv on ubuntu /
  macos / windows and runs `test_wheel.py`; `publish-testpypi` on
  `workflow_dispatch` (`skip-existing`: a version TestPyPI already holds
  is a silent no-op, so the workspace version is a `-dev` pre-release
  between releases and a final one only for the tag) and `publish-pypi` + a GitHub Release on a
  `v*` tag push, both through PyPI trusted publishing (environments
  `testpypi` / `pypi`, no token in the repository).

`riggen.show(robot)` (§Python SDK) serialises to a temp
`.riggen` and spawns the bundled binary on it — the `rr.spawn()` model.
The GUI is never entered from inside a Python call (ADR-0002).

## Python SDK (ADR-0009)

Two layers. `riggen._riggen` (`crates/riggen-py`) is the extension module:
a thin, typed layer over `riggen-core` and `riggen-export`, one method per
`Command`, no sugar. `riggen` (`python/riggen/`) is the public API over it
(`python/riggen/robot.py`). This section is the extension module.

**Values cross in the schema's shape.** A joint, a pose, an inertial spec
is the same dict the `.riggen` file spells (02 §Schema) — `{"t": [x, y, z],
"r": [x, y, z, w]}` for a `Pose`, `"Revolute"` for a kind, `{"Computed":
{"density_override": None}}` for an inertial — produced and consumed
through `serde_json::Value` (`doc.rs`), with one difference: **ids are
ints**. The file writes `"l5"`, Python sees `5`; the keys that hold an id
are fixed by the schema (`id` a geom, `mesh` a mesh, `parent` / `child` a
link, `joint` a mimic's leader), so the rule is by key and a link *named*
`"l5"` is untouched. A
malformed value is a `ValueError` naming the field (`joint: missing field
\`axis\``); a wrong Python type is a `TypeError`.

**Every edit runs on a clone** and replaces the document only on success —
`Command::apply` can leave a half-edited robot behind a validation
failure (02 §Commands) — so a refused edit raises and changes nothing,
the id counter included. No `History`: a script has no undo.

| Python (`riggen._riggen.Robot`) | Rust |
|---|---|
| `Robot(name)` | `Robot::new` |
| `Robot.load(path) -> (robot, warnings)` | `file::load`; `Warning`s as strings |
| `robot.save(path)` | `file::save` (paths rebased, unreferenced assets dropped) |
| `robot.to_json()` / `Robot.from_json(text)` | the current-schema envelope (3), paths as held (absolute); `from_json` validates and refuses any other version — it walks no upgrade chain, unlike `file::load` |
| `robot.copy()` | `Robot::clone` |
| `robot.name`, `.root`, `.next_id` | the fields; `next_id` is `IdGen::peek` |
| `robot.links()`, `.joints()`, `.frames()`, `.assets()`, `.materials()` | `{id: doc}` dicts, materials by name |
| `robot.link(name)`, `.joint(name)` | a name lookup, `None` when absent |
| `robot.parent_joint(l)`, `.child_joints(l)`, `.subtree(l)` | `Robot::{parent_joint, child_joints, subtree}`; an unknown link is `UnknownId` |
| `robot.add_asset(path, *, scale, fix_up)` | `absolute` + `hash_file` + `Robot::add_asset` (not a command) |
| `robot.add_link(name, parent, joint, *, mesh, scale, fix_up, material)` | `Command::AddLink`; with `mesh`, `add_asset` and one geom at identity first — the app's drop |
| `remove_link`, `rename_link`, `rename_joint` | `RemoveLink`, `RenameLink`, `RenameJoint` |
| `add_geom(link, mesh, *, pose, color)`, `remove_geom`, `set_geom_pose` | `AddGeom` (the geom id allocated here), `RemoveGeom`, `SetGeomPose` |
| `set_joint(joint, doc)` | `SetJoint`; `parent` / `child` in the dict ignored |
| `add_frame(name, link, *, pose)`, `remove_frame`, `rename_frame`, `set_frame(frame, doc)`, `frame(name)` | `AddFrame` (the id allocated there and returned), `RemoveFrame`, `RenameFrame`, `SetFrame`, a name lookup (ADR-0012) |
| `move_joint_frame(joint, origin, axis)` | `MoveJointFrame` |
| `reparent(link, new_parent, *, keep_world_pose)` | `Reparent` |
| `set_root(link)` | `SetRoot` |
| `set_link_material`, `upsert_material`, `remove_material`, `rename_material(from, to)` | `SetLinkMaterial`, `UpsertMaterial`, `RemoveMaterial`, `RenameMaterial` |
| `set_asset(mesh, doc)` | `SetAsset`; the path absolutised, the hash recomputed |
| `set_inertial(link, doc)`, `set_collision(link, doc)` | `SetInertial`, `SetCollision` |
| `validate() -> list[str]`, `check()` | `validation_errors`; `check` raises `ValidationError`. Empty for any document the edit methods, `load` or `from_json` let through — they validate |
| `fk({joint: q}) -> {link: pose}` | `fk` with a `JointState`; a missing joint is at zero, an unknown one `UnknownId`, a mimic joint's own entry ignored — `fk::resolve_q` derives it (ADR-0013) |
| `fk_frames({joint: q}) -> {frame: pose}` | `fk::frames`; `fk` itself stays links only |
| `origin_for_world(link, world) -> pose \| None` | `origin_for_world` |
| `inertial(link) -> (mass, com, inertia rows)` | `MeshStore::load` + `compose_inertial`; `InertialError` (mesh load errors appended) |
| `export(dir, *, format, mesh_paths, floating_base, fk_samples) -> [Path]` | `MeshStore::load` + `resolve` + `export` (+ `fk_samples::to_json`), exactly `cli::run`: every resolve error joined one per line as `cannot export: …` into `ExportError`; `format` names a **set** of writers — `"mjcf" \| "urdf" \| "sdf" \| "both"` (the first two) `\| "all"`, the default — and `mesh_paths` (URDF and SDF) is `"relative" \| "absolute" \| "package://<name>"` |
| `fk_samples_json()` | `fk_samples::to_json` |
| `Robot.load_urdf(path, packages=None) -> (robot, warnings)` | `urdf_in::load` with a `PackageMap`; `UrdfImportError`, `ImportWarning`s as strings |
| `Robot.load_mjcf(path) -> (robot, warnings)` | `mjcf_in::load`; `MjcfImportError`, the same `ImportWarning`s as strings |

**Exceptions** live in Python (`python/riggen/errors.py`) and Rust raises
them by name (`errors.rs`), so `except riggen.EditError` is a plain Python
class: `RiggenError` ← `EditError` ← one subclass per `EditError` variant
(`InvalidDocument`, `UnknownId`, `UnknownMaterial`, `WouldCreateCycle`,
`CannotRemoveRoot`, `CannotReparentRoot`, `MaterialInUse`,
`MaterialExists`, `MovableJointOnRootPath`), and beside it `ValidationError`, `FileError`,
`ExportError`, `UrdfImportError`, `MjcfImportError`, `InertialError`. The
message is the Rust `Display`.

**`show()`** (`python/riggen/show.py`): the GUI is never entered from
inside a Python call (ADR-0002, ADR-0009). `riggen.show(robot, *,
block=False)` saves the robot to `tempfile.mkdtemp(prefix="riggen-show-")
/<name>.riggen` (mesh paths rebased by `save`, resolved again by `load`),
`subprocess.Popen`s the bundled binary on it and returns a `Viewer`
(`path`, `process`, `poll()`, `kill()`, `robot`). `Viewer.wait(timeout)`
blocks until the window exits and returns the document re-read from the
file if its SHA-256 changed — the GUI saved — else the very robot passed
in; idempotent. `binary_path()` is the one lookup for `show()` and
`python -m riggen`: `RIGGEN_BINARY` if set (a `.py` path runs under the
interpreter — the SDK suite's stub windows), else `sysconfig`'s scripts
directory (user-site second); when nothing is found the
`FileNotFoundError` says the install has no binary (a build from the
sdist) and names the three ways to get one.

**The public layer** (`python/riggen/robot.py`, re-exported from
`riggen`; pure Python, typed, `pyright` clean) is handles and value types
over that table — no logic of its own beyond spelling:

| `riggen` | Over `_riggen` |
|---|---|
| `Robot(name)`, `load(path)`, `load_urdf(path, packages)`, `load_mjcf(path)` | `Robot`, `Robot.load`, `Robot.load_urdf`, `Robot.load_mjcf`; the warnings become `RiggenWarning`s through `warnings.warn` |
| `robot.root`, `.links`, `.joints`, `.link(name)`, `.joint(name)` (`KeyError`), `.materials` | handles by id; `Material(density, color)` |
| `robot.add_link(name, parent, spec, *, mesh, scale, fix_up, material, joint_name)` = `link.add_link(name, spec, …)` | `add_link` with `spec.to_doc(joint_name or f"{name}_joint")` |
| `link.name`, `.material`, `.collision`, `.inertial_spec` (get/set) | `rename_link`, `set_link_material`, `set_collision`, `set_inertial` — one edit per assignment |
| `link.joint`, `.parent`, `.joints`, `.children`, `.subtree`, `.geoms` | `parent_joint`, `child_joints`, `subtree`, `links()[id]["visuals"]` |
| `link.add_mesh(path, *, pose, scale, fix_up, color)` → `Geom`; `geom.pose`, `.mesh`, `.remove()` | `add_asset` + `add_geom`; `set_geom_pose`, `remove_geom` |
| `link.remove()`, `.reparent(parent, keep_world_pose=True)`, `.place(world)`, `.make_root()` | `remove_link`, `reparent`, `origin_for_world` + `set_joint`, `set_root` |
| `link.inertial` → `Inertial(mass, com, inertia)` | `inertial` |
| `joint.name`, `.kind`, `.parent`, `.child`; `.origin`, `.axis`, `.limits`, `.dynamics`, `.mimic`, `.actuator`, `.spec` (get/set); `.move_frame(origin, axis)` | `set_joint` with the one field changed; `move_joint_frame` |
| `Mimic(joint, multiplier, offset)` — `joint` is the leader's handle | the `mimic` dict, the leader as an id; a coupling is not part of a `JointSpec`, so assigning `.spec` carries it over (and a `Fixed` spec drops it, ADR-0013) |
| `Position(kp, kv=0)`, `Velocity(kv=1)`, `Motor(gear=1)` → `Actuator` | the `actuator` dict (`{"Position": {…}}`); every default is MuJoCo's own. Like a coupling it belongs to the joint, not to its kind, so `.spec` carries it over and a `Fixed` spec drops it (ADR-0014) |
| `link.add_frame(name, pose)` → `Frame`; `link.frames`, `robot.frames`, `robot.frame(name)` (`KeyError`) | `add_frame`, `frames()`, `frame(name)` |
| `frame.name`, `.parent`, `.pose` (get/set), `.world(q)`, `.remove()` | `rename_frame`, `set_frame`, `fk_frames`, `remove_frame`; setting `.parent` keeps the *stored* pose, so the frame moves — the app's panel is the one that keeps the world pose (ADR-0012) |
| `Pose(xyz, rpy= \| quat=, degrees=)`, `.rpy`, `.rpy_degrees`, `.to_doc()` | `rpy_to_quat` / `quat_to_rpy` (the core's convention, never re-derived); `quat` is `(w, x, y, z)` |
| `Fixed(origin)`, `Revolute(axis, *, origin, limits, dynamics, degrees)`, `Continuous`, `Prismatic` → `JointSpec` | the joint dict; `axis` is `"x" \| "-y" \| (x, y, z)`; `limits` a `Limits` or `(lower, upper)`; the app's defaults (`±π`, `±1`, effort and velocity 0) |
| `ComputedInertial(density)`, `OverrideInertial(mass, com, rows)`, `HybridInertial(mass)` | the `InertialSpec` dict (the tensor column-major in the file, rows here) |
| `ConvexDecomposition(max_hulls, resolution, concavity)` | the `{"ConvexDecomposition": {…}}` `set_collision` already takes — no new `Command` method; `link.collision` reads it back as the dataclass, the three simple policies as their names, anything else as the document value (ADR-0011) |
| `robot.fk({name \| joint: q})` → `{name: Pose}`, `.frame_poses({…})` → `{name: Pose}`, `.validate()`, `.save()`, `.export(dir, *, format, mesh_paths, floating_base, fk_samples)`, `.to_json()` / `from_json`, `.copy()` | the same names, ids ↔ names |

`examples/pendulum.py` (the README's ten lines; the corpus pendulum) and
`examples/arm.py` (the M2 arm from its STLs, joints typed, a `tcp` and a
`camera_mount` frame on top and the forearm coupled to the upper arm; its
export is byte-identical to `arm.riggen`'s) are the API's worked examples and the
`wheel` job's MuJoCo input.

## The web build

The same app, in a browser, at
[divelix.github.io/riggen](https://divelix.github.io/riggen/). `riggen-app`
is a `cdylib` whose `WebHandle` starts eframe on a full-window canvas and
opens the bundled sample arm; `web/index.html` and `web/main.js` are the
page around it and `web/build.sh` produces `web/dist/` — cargo at
`[profile.web]`, then a `wasm-bindgen-cli` pinned to `Cargo.lock`'s own
`wasm-bindgen`, because the two halves of that ABI have to agree.

**WebGPU only** (ADR-0017 §7). The viewport's picking reads an `R32Uint`
target back with `copy_texture_to_buffer`, which wgpu's GL backend will not
do, so `main.js` asks for a real adapter before it starts anything and
otherwise writes a page naming the browsers that have one. It also polls
`WebHandle::has_panicked`: a panic poisons eframe's runner and the canvas
then simply stops repainting, which reads as a hang.

**No filesystem.** Files arrive as the bytes of one drop gesture, read
asynchronously into an inbox the frame loop drains, and are resolved by
**file name** against that gesture's own set (`DroppedSet`, ADR-0017 §3).
A gesture carrying a document replaces the set; meshes alone join it. Out
is a download: the `.riggen` text, the export directory as one stored zip,
the debug state's JSON. There is no dialog and no path, so a document
opened in a browser is untitled and Save behaves as Save As.

`pages.yml` builds and deploys on every push to `main` — `main` is always
green, and the demo should be what riggen is now — and the `wasm` CI job
builds the same bundle on every push, so a break shows up in CI first. The
measured size is in 03 §v0.2.

## Testing

- `riggen-mesh`, `riggen-core`, `riggen-export`: plain unit tests; no GPU,
  no egui. This is where correctness lives.
- **Round-trip FK test** (`riggen-export/src/urdf.rs`): the sample arm →
  export URDF → parse with `urdf-rs` → compute FK independently from the
  parsed `xyz rpy axis` with glam matrices alone → compare end-effector
  poses over a 5³ joint grid against `riggen-core::fk` to 1e-9. Catches
  frame-convention bugs mechanically. The URDF import has the mirror test:
  `arm.urdf` imported has `arm.riggen`'s FK.
- **SDF load test** (`python/tests/test_sdf_load.py`; the `sdf` CI job):
  `libsdformat`'s `Root.load` — the spec's own parser — must accept the
  exported SDF, raising `SDFErrorsException` on anything illegal, and then
  FK built from **what that parser resolved** (`semantic_pose().resolve`
  for every link and frame, `resolve_xyz` for every axis) must match the
  `<name>.fk.json` to **1e-9**, tighter than the MJCF bar because nothing
  in the loop is a simulator with its own integrator. The parser resolves
  the pose graph and the axis frames; the script only walks the tree and
  applies `q`, so everything riggen could get wrong about *where things
  are* is the reference implementation disagreeing with ours. Every
  `<axis><mimic>` must reproduce the sampled `q`, and a pair of joints the
  samples show as exactly coupled must have one, so a dropped `<mimic>`
  and a swapped multiplier fail alike; a frame the samples name and the
  model lacks fails the file. Run over the sample's export, the export of
  its URDF import, and `bracket.riggen`.
- **MuJoCo load test** (`python/tests/test_mjcf_load.py`; the `mujoco` CI
  job, and locally `uv run --with mujoco --with numpy python …`):
  `mujoco.MjModel.from_xml_path` on the exported MJCF must succeed with
  zero compiler warnings (a `set_mju_user_warning` hook fails on any), and
  `mj_forward` body **and site** poses must match the `<name>.fk.json` the
  export wrote with `--fk-samples` to 1e-6 at five joint configurations (a
  site the samples name and the model lacks fails the file, which is what a
  dropped `<site>` looks like) — for the
  sample's export, the export of its URDF import,
  `assets/fixtures/bracket.riggen`, the decomposition acceptance
  (ADR-0011), and the arm's export imported back and exported again, the
  MJCF round trip (ADR-0015). An argument may be `MODEL_DIR=SAMPLES_DIR`,
  and the round trip uses it to hold that second model to the *original*
  document's `fk.json`: agreeing with its own samples would not catch an
  import that lost something. Every `mjEQ_JOINT` equality — a mimic joint (ADR-0013) —
  must reproduce the sampled `qpos` through its `polycoef`, and a pair of
  joints the samples show as exactly coupled must have one, so a swapped
  coefficient order and a dropped `<equality>` both fail. The `.fk.json`'s
  `actuators` block says what the `<actuator>` block should hold
  (ADR-0014) — the driven joint, the gains where MuJoCo keeps them
  (`gainprm` / `biasprm` / `gear`), and the two ranges, an omitted one
  having to leave `ctrllimited` / `forcelimited` off — and `model.nu` must
  be exactly that many, so a dropped or invented actuator fails and the
  URDF import's actuator-less model is checked as such, not skipped. The
  three presets are covered by the two fixtures: the arm's shoulder is a
  `<position>` and its upper arm a `<velocity>`, the bracket's hinge a
  `<motor>`. The script also fails any body whose `<stem>_hull_N` pieces
  do not number at least two and run 0..N: MuJoCo hulls a collision mesh
  itself, so one piece would mean the policy bought nothing. It reads that
  off the model, not off the fixture.
- **SDK suite** (`python/tests/sdk/`, pytest; the `wheel` CI job after the
  smoke, and locally `uv venv target/sdk-venv && VIRTUAL_ENV=$PWD/target/
  sdk-venv uvx maturin develop --uv`, `uv pip install --python target/
  sdk-venv pytest`, `target/sdk-venv/bin/python -m pytest python/tests/
  sdk`): the pendulum built through `_riggen` saves byte-identical to
  `assets/fixtures/pendulum.riggen` (ids, order, hashes, `next_id`);
  `load` of it has no warnings and a changed mesh has one; every edit
  method runs once; every `EditError` variant is raised as its class and
  leaves `to_json()` unchanged; `set_joint` ignores `parent` / `child`;
  malformed values are `ValueError` / `TypeError`. Against the `riggen`
  binary (`RIGGEN_BINARY`, else the one bundled beside the interpreter —
  in the `wheel` job the same wheel's — else a local `target/` build, else
  skipped): `fk` of the arm equals `--fk-samples`' JSON to 1e-9 and
  `fk_samples_json()` is that file; `export` of `arm.riggen` and of
  `load_urdf(arm.urdf)` are byte-identical to `riggen --export both
  --fk-samples`, warnings included, and `--export all` is byte-identical
  through both routes too; `inertial` of the arm's base is the
  `<inertial>` the MJCF carries; an unexportable pendulum's `export`
  raises `ExportError` with every reason. The public layer: the corpus
  pendulum built through `riggen.Robot`; every setter runs once; `fk` by
  name and handle; `load` / `load_urdf` / `load_mjcf` warn, and
  `load_mjcf` of the arm's own MJCF export writes the same directory the
  CLI does from that `.xml`; `examples/arm.py` exports
  byte-identical to `arm.riggen`'s export and `examples/pendulum.py` to
  the corpus file's; both examples run from the command line; every name
  in `riggen.__all__` and every public method and property has a
  docstring. `show()` with stub windows (`RIGGEN_BINARY` at a Python
  script): one that loads the file through `riggen`, adds a link and
  saves → `wait()` returns the edited document and the caller's robot is
  untouched; one that exits without saving → the same object back;
  `kill()`; `RIGGEN_BINARY` pointing nowhere → `FileNotFoundError` and
  `python -m riggen` exits 1 with the message; `python -m riggen`
  forwards its arguments. The window itself is the human's half. `uvx pyright` (pyproject `[tool.pyright]`: the package, the
  stubs, the examples) is clean; the `wheel` job runs it, then
  `examples/arm.py` through the wheel and `test_mjcf_load.py` on its
  output. Never against the checkout on `sys.path`, which has no
  extension module.
- **Wheel smoke** (`python/tests/test_wheel.py`; the `wheel` CI job and
  `release.yml`'s `smoke` jobs): given a venv the wheel is installed into,
  `riggen --version` matches `riggen \d+.\d+.\d+ (… …)`, `python -m
  riggen --version` prints the same line, `--help` has a usage block, and
  `riggen --export mjcf` of `arm.riggen` writes `arm.xml` and its meshes.
  Headless on purpose: no runner has a display, so the window is the
  human's half of the M4 acceptance.
- **CLI** (`riggen-app/src/cli.rs` unit tests, `tests/cli.rs`): every
  `FLAGS` entry parses in its long and short form; `help()` is generated
  from `FLAGS`, and a test greps it for every spelling and doc line, so a
  flag cannot exist without help; `--version`'s shape; `--example arm`
  extracts exactly five files that `riggen_core::load` + `MeshStore` read
  back. `tests/cli.rs` runs the built binary (`CARGO_BIN_EXE_riggen`) for
  exit codes and streams.
- **Startup budget** (`startup_first_frame_under_budget`, through
  `with_app`): `RiggenApp::new` to the end of the first `ui` pass under
  500 ms, 2000 ms when `CI` is set (lavapipe on a shared runner). It
  guards a regression in `new` — a font atlas, a pipeline, a persistence
  load — not the number the user sees: the real window's clock starts in
  `main` (`riggen --timing` prints it) and holds the OS window and the
  wgpu device too; see 03 §M4 for the measured figure.
- **The web bundle** (the `wasm` CI job): `web/build.sh` on every push, so
  the bundle `pages.yml` deploys is built and checked for its three files
  before it is deployed. A break that only shows under wasm-bindgen breaks
  here first. The `FileSource` seam under it is covered by plain unit
  tests: `arm.riggen`, `arm.urdf` and `menagerie_style.xml` opened out of
  an in-memory set rooted at a directory that **does not exist** — so any
  read that leaked to the filesystem would fail — and compared field for
  field against the on-disk load; `export_files` against what `export`
  wrote; the export zip's entry names, stored method and bytes.
  What only a real browser can answer — WebGPU comes up, the pick readback
  works, a drop opens, a download lands — is the by-hand half, and the
  agent runs it rather than asking the human what the screen says: a
  headless-Chromium CDP driver loads the page, collects the console,
  clicks, drops files built from `fetch`, and reads the canvas back with
  `toDataURL` (`Page.captureScreenshot` does not composite a WebGPU
  canvas). It has to run **headed** on a display: headless Chromium's GPU
  process fails `requestDevice`, which a minimal clear-to-red WebGPU page
  confirms is the environment and not riggen.
- **Visual snapshots** (`riggen-app/tests/visual`, ADR-0003): `egui_kittest`
  drives the real `eframe::App` headlessly through wgpu (CPU adapter via
  lavapipe, so local and CI agree) and diffs PNGs. This is how an agent sees
  the window. A `debug_state()` JSON dump of what the app believes it drew
  (camera with near/far, the document — file, name, dirty, import scale,
  links, joints with `q`, frames, selection — the `ui` section — rename in
  progress, open windows, modal, title, collision view — instances with
  their link/geom key, position and colour, viewport selection, the gizmo,
  the joint glyphs, the frame glyphs, the snap candidate, the viewport's
  pointer policy (`input`, omitted while nothing is suppressed), status,
  viewport rect) accompanies every snapshot as
  a golden of its own; every float in it is rounded to six decimals and
  `-0.0` normalised so goldens never churn. At runtime the same JSON is
  under Debug › Copy / Save state (JSON), beside egui's layout overlays.
  The scenarios: `startup`, `cube`, `hover_cube`, `select_cube`,
  `three_parts`, `pendulum`, `mm_scale_part`, `tree_pendulum`,
  `tree_reparent`, `properties_link`, `properties_joint`, `pendulum_swing`,
  `materials`, `toolbar`, `gizmo_move_link`, `gizmo_rotate_joint`,
  `glyph_revolute`, `glyph_prismatic`, `glyph_hover`, `snap_vertex`,
  `snap_circle`, `place_joint_bore`, `align_concentric`, `five_minute_arm`,
  `dirty_title`, `unsaved_confirm`, `file_menu`, `debug_menu`, and M3's
  `collision_hull`,
  `collision_primitives` (a pick through a translucent box hits the part),
  `properties_inertial`, `properties_inertial_open_mesh`,
  `properties_collision`, `export_dialog`, `export_blocked`, `import_urdf`,
  v0.2's `collision_decomposition`, `properties_collision_decomposition`,
  the mimic and actuator set — `joints_window_mimic`,
  `properties_joint_mimic`, `properties_joint_actuator`,
  `properties_joint_actuator_applied` —
  and the frame set — `frames_tree`, `frame_properties`,
  `add_frame_button`, `gizmo_move_frame` — and `decomp_needs_consent`, the
  browser's half of the Collision block, which a native runner renders by
  turning `set_decomp_consent` off,
  plus golden-less app tests including `build_pendulum_numerically` (the
  M1 acceptance in executable form), `example_arm_opens_from_the_bundle`,
  `startup_first_frame_under_budget`, and the pointer-sharing set behind
  ADR-0010 — `gizmo_leaves_the_pointer_alone`,
  `camera_works_while_the_gizmo_is_up`, `orbit_works_from_a_gizmo_handle`,
  `the_toolbar_does_not_zoom_the_camera`,
  `a_hovered_glyph_leaves_the_camera_alone` and the acceptance run
  `gizmo_shares_the_viewport`, which orbits, zooms, re-selects and drags a
  handle in one session because those stopped working together. `debug_state().timing`
  (`first_frame_ms`, `frame_dt`) is present only while the frame HUD is
  on, which the harness turns off, so no golden holds a wall-clock number.
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
  - `scroll_at(harness, pos, lines)` and
    `middle_drag(harness, from, to, modifiers)` drive the camera. The wheel
    is read off `InputState::raw.events`, which holds one frame's worth, so
    the event needs a frame of its own after the hover has settled; and a
    modifier is carried by `Event::ModifiersChanged`, because egui keeps the
    previous pass's modifiers until an event changes them — one at each end
    holds shift down across every frame of a pan.
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
- CI (`ci.yml`): `cargo fmt --check`, `clippy -D warnings`, `cargo test`, the
  `wasm` job — `web/build.sh`, the whole wasm-bindgen bundle the demo is
  served from (§The web build), not a build check — the `mujoco` job — the app's
  `--export` of `arm.riggen`, of `arm.urdf`, of `bracket.riggen` and of
  the first of those exports read back as MJCF (with `rust-cache`), then
  `python/tests/test_mjcf_load.py` on all four through `uv` (ADR-0008 §3)
  —
  the `sdf` job — the same three documents exported as SDF, then
  `python/tests/test_sdf_load.py` under `libsdformat`'s own Python
  bindings, installed from `packages.osrfoundation.org` (the workflow's
  only third-party apt repository, ADR-0016 §6) —
  and the `wheel` job: in maturin-action's manylinux 2_28 container,
  `build_wheel.py --binary-only` (`before-script-linux`) then maturin for
  the extension, a fresh `uv venv` installs the wheel, `test_wheel.py`
  runs — about 6.5 min wall time, the same as the `mujoco` job (a release
  build of the app from cold either way). The binary links only libc and
  the container has a `python3`, so it needs no package installed first.
  The `clippy` job runs the *latest* stable, so a new lint reaches CI
  before the local hook, and pins the layer rule for `riggen-py` with
  `cargo tree` (ADR-0009). `release.yml` is §Python distribution.
