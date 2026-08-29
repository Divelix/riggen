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
│                   instances, camera, ID-buffer picking, grid,  │
│                   overlays (axes, joint glyphs, frames)        │
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
├── Cargo.toml              # [workspace], resolver 3, edition 2024
├── crates/
│   ├── riggen-mesh/
│   ├── riggen-core/
│   ├── riggen-export/
│   ├── riggen-viewport/
│   └── riggen-app/         # bin "riggen"; cdylib for the wasm build check
├── python/                 # v0.2: pyproject.toml, riggen/ package, riggen-py crate
├── assets/                 # sample meshes + a reference .riggen used by tests
├── docs/
├── SEED.md
└── AGENTS.md, CLAUDE.md
```

Dependency policy (ADR-0001): egui/eframe/egui-wgpu 0.36.x from crates.io,
wgpu version dictated by egui-wgpu — never depend on a different wgpu.
`glam` 0.30 with the `serde` feature. Local checkouts of egui and rerun under
`~/Documents/code/rust/` are reference reading only; no `path =` or
`[patch]` unless an unreleased fix is needed, and then with a comment saying
which one. Profile settings carried from RoboCAD: `opt-level = 1` for our
crates in dev, `3` for dependencies — an unoptimized wgpu is felt.

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

Repaint policy: egui repaints on input; request continuous repaint only
during camera motion, gizmo drags, slider drags and joint animation. A hover
pick is issued only when the cursor pixel or the camera matrix changed
(RoboCAD's `last_pick` rule — otherwise pick + readback + repaint loop at
vsync while the pointer merely rests).

## Picking and snapping

The ID buffer (`R32Uint`) encodes `(instance, triangle)`; readback is async
and resolves the following frame. A pick gives an exact hit by intersecting
the mouse ray with that one triangle on the CPU (`riggen-mesh::ray`), so
snap targets never need a spatial index:

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
hull / primitive fitting, and export. `riggen-app::jobs` runs them on a
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
  accompanies every snapshot.
- CI: `cargo fmt --check`, `clippy -D warnings`, `cargo test`, and a
  `wasm32-unknown-unknown` **build** of `riggen-app` (build check only; the
  web build is not a product in v1).
