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
│                   overlays (axes triad, joint glyphs, frames)  │
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

`riggen-core` depends on `riggen-mesh` for `TriMesh` and mass properties
(a link's computed inertial is a function of its meshes). `riggen-export`
depends on both. Nothing below `riggen-app` knows about selection, hover, or
gizmos.

## Cargo workspace

```
riggen/
├── Cargo.toml              # [workspace], resolver 3, edition 2024, every dep version
├── rust-toolchain.toml     # stable + rustfmt, clippy, wasm32-unknown-unknown
├── kittest.toml            # snapshot thresholds (ADR-0003); found by walking up from the crate
├── crates/
│   ├── riggen-mesh/        # TriMesh, Aabb, Ray, load_stl / load_obj / load_mesh
│   ├── riggen-core/        # placeholder until M1
│   ├── riggen-export/      # placeholder until M3
│   ├── riggen-viewport/    # camera/, scene, pick_id, gpu_mesh, viewport/, shaders/
│   └── riggen-app/         # bin "riggen"; cdylib for the wasm build check; tests/visual
├── assets/fixtures/        # cube_binary.stl, cube_ascii.stl, cube.obj — the unit cube
│                           # (TriMesh::cube(0.5)) in every format; sample robots later
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

`riggen-core` and `riggen-export` are empty `lib.rs` files that already
carry the "no egui/wgpu" rule in their doc comment.

## The document is the only state

`riggen-app` owns one `Robot` (02-data-model) plus derived, never-saved
state: the FK pose for the current joint values, the viewport's instance
table, selection, hover, gizmo interaction, and job results in flight.

Every user edit is a `Command` applied through `History`, which is
**snapshot-based**: `Robot` is a few kilobytes of ids, poses and numbers
(meshes are referenced by id, never copied), so `History` keeps
`Vec<Robot>` and undo is a swap. This is deliberately simpler than RoboCAD's
reversible commands; it stays correct as long as `Robot` never holds bulky
data. Mesh geometry lives in a `MeshStore` beside the document, keyed by
`MeshId`, loaded once per file and shared across snapshots by `Arc`.

Granularity rule, kept from RoboCAD: **one gesture = one command.** A gizmo
drag mutates a *preview* pose every frame and commits once on release; a
slider preview of a joint angle is not a command at all (joint values are
derived state, not document state — the document stores limits, not the
current `q`).

## Frame loop

```
input ──► egui panels (tree, properties, joint sliders, status)
       ──► gizmo / snapping / pick handling  ──► Commands ──► History ──► Robot
Robot ──► fk(robot, q) ──► world pose per link
       ──► for each visual geom: viewport.set_instance_model(instance, link_pose * geom.pose)
       ──► viewport.ui(...)   (records the wgpu callback; picks resolve next frame)
```

The viewport draws **instances**, not links: one instance per `(LinkId,
GeomId)` visual, with the uploaded `TriMesh` shared by `MeshId`. Moving a
joint writes matrices; nothing is re-uploaded. Collision geometry renders as
a second, toggleable instance set (translucent) sharing the same camera.

`InstanceId(u32)` is handed out by the app and never reused in a session.
`Scene<M>` keeps one entry per instance — payload (`GpuMesh`), `DMat4`
model, visibility, model-space `Aabb` — in insertion order, which is draw
order; `bounds()` unions the transformed boxes for zoom-to-fit. Model
matrices go to the GPU as one dynamic-offset uniform buffer (grown by
`next_power_of_two`), one `draw_indexed` per visible instance, no CPU merge.
The scene renders into an offscreen colour + `Depth32Float` pair (egui's
own pass has no depth attachment) and is blitted in `paint()`; the axes
triad draws last in its own corner viewport with a rotation-only camera.
`f64` → `f32` happens in `GpuMesh::upload` and the model-uniform pack, and
nowhere else.

Camera: `OrbitCamera` is robocad's turntable camera on glam — `f32`, radians,
Z-up with Y standing in as the up hint at the poles. glam's
`perspective_rh` / `orthographic_rh` produce wgpu's `[0, 1]` clip depth
directly, so `view_proj = proj * view` with no OpenGL-to-wgpu remap; a
camera test pins near → 0 and far → 1 in both projections. Wheel input is
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
`map_async`; a non-blocking `device.poll` at the top of the next
`Viewport::ui` lets it resolve — the readback never stalls a frame. The hit
nearest the cursor in the region wins (there is no B-Rep, so no vertex >
edge > face ladder). At most one pick is in flight; a click's select pick
beats a hover; a hover whose `(pixel, view_proj)` equal the last pick's is
not re-issued (`last_pick`), otherwise a resting cursor would re-render the
ID buffer at vsync rate forever; `PointerGone` clears the hover. The policy
is the pure `decide_pick`, unit-tested without a GPU. The result is a
`PickHit { instance, triangle }`; hover and selection tint the **whole
instance** (a "face" on an STL is one triangle, so a face outline would
trace a single triangle) and the status bar reads `i3/t120`.

`riggen_mesh::ray_triangle` (Möller–Trumbore, two-sided — the ID buffer has
already chosen the triangle) recovers the exact hit point by intersecting
the mouse ray with that one triangle on the CPU, so snap targets never need
a spatial index:

- **point**: the hit point, or the nearest triangle vertex when within a
  pixel radius;
- **face normal**: the hit triangle's normal (used for "axis = normal");
- **circle / cylinder axis**: grow the hit triangle's fan by near-coplanar
  neighbours around a boundary loop, fit a circle → center + axis. This is
  the mechanic that makes "click the bore, get the joint axis" work on STL
  data with no B-Rep, and it is the M2 risk item;
- **bounding box**: per-instance AABB corners/face centers, from the
  `Scene` bounds already kept for zoom-to-fit.

Gizmos come from `transform-gizmo-egui`, fed the viewport's view/projection
matrices; they draw with egui's painter over the viewport. If the gizmo's
hit-testing fights the ID buffer (both want the mouse), the gizmo wins while
it is hovered.

## Jobs and threads

There is no evaluator. The only long-running work is mesh loading, convex
hull / primitive fitting, and export. Mesh loading currently runs
synchronously on the UI thread in `RiggenApp::load_files` (every route in —
CLI arguments, drag-and-drop, File › Open — ends there and fits the view
afterwards); `riggen-app::jobs` arrives with the hull work and runs them on a
`std::thread` with an `mpsc` channel and a `wake` callback bound to
`ctx.request_repaint()`; results are drained once per frame. On wasm the
same API runs inline (no threads from eframe on the web); nothing else in
the app cares which. This is RoboCAD's `EvalExecutor` shape without the
generation machinery — a job carries the `MeshId` or export request it was
made for, and a result for an id that no longer exists is dropped.

## File format

`robot.riggen` is JSON: `{ "schema_version": 1, "robot": Robot }`. Mesh
paths are stored **relative to the `.riggen` file**, with a content hash so a
changed STL is noticed on open. Geometry is never embedded. A schema bump
comes with an `upgrade_vN_to_vN+1` and a corpus test that keeps every old
version opening forever (RoboCAD's rule).

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
  (camera, instances, selection, status, viewport rect) accompanies every
  snapshot as a golden of its own; every float in it is rounded to six
  decimals and `-0.0` normalised so goldens never churn. Harness facts that
  must not be rediscovered:
  - `Harness::step()` runs egui's logic pass only, no GPU work; anything that
    depends on the GPU having run (the ID-buffer pick) needs `pump_rendered`.
    `step()` also drains every queued event in one go, so a click is one raw
    event per rendered frame (`click_at`), not `drag_at`/`drop_at`.
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
    nothing — the "show me the app right now" path. `with_app()` runs a body
    against the real app with no goldens at all.
  - A scenario prints `SKIPPING` when no wgpu adapter exists. That is an
    environment failure, not a pass; CI installs `mesa-vulkan-drivers`.
- CI: `cargo fmt --check`, `clippy -D warnings`, `cargo test`, and a
  `wasm32-unknown-unknown` **build** of `riggen-app` (build check only; the
  web build is not a product in v1).
