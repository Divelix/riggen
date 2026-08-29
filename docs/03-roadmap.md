# 03 — Roadmap

Every milestone ends with something you can run and show, and each retires
the scariest remaining unknown first. A milestone's "out" list is as binding
as its "in" list. Calibration: RoboCAD went from empty to 58k lines in three
weeks; this roadmap is smaller than that.

Spine: M0 → M1 → M2 → M3 → M4, then v0.2.

---

## M0 — Skeleton and the ported viewport

*Goal: `riggen part.stl` opens a window with the part in it.*

- Workspace of 01-architecture; CI (fmt, clippy, test, wasm build check).
- `riggen-mesh`: `TriMesh`, STL (binary + ASCII) and OBJ loaders, AABB,
  ray/triangle.
- `riggen-viewport` ported from `robocad-viewport`: cgmath → glam, `BodyId`
  → `InstanceId`, `RenderMesh` → `TriMesh`, `TopoRef` → `(InstanceId, tri)`,
  sketch-plane code and edge/vertex pick passes removed. Camera, low-latency
  surface config, hover/select restyle, axes triad, grid, zoom-to-fit kept.
- `riggen-app`: eframe shell, file drop and `rfd` open, one instance per
  dropped file, status bar with frame-time HUD.
- `egui_kittest` snapshot harness and `debug_state()` from day one.

**Out:** any document; any panel beyond the status bar.

**Accept:** drop three STLs → three orbitable, pickable parts; a `startup`
snapshot test passes on the CPU adapter; wasm target builds.

## M1 — Document, tree, joints, FK

*Goal: a two-link pendulum you can save, reopen, and swing.*

- `riggen-core`: types of 02-data-model, `validate`, `fk`, snapshot
  `History`, `.riggen` v1 serde with relative mesh paths and content hash.
- Link tree panel (add/remove/rename/reparent by drag), properties panel
  with numeric pose entry (xyz + RPY, editable in degrees, stored in radians),
  joint creation between selected links, joint sliders window driving FK.
- Materials table with density; per-link material choice.
- New/Open/Save/Save As with dirty marker and confirm; undo/redo shortcuts
  (the RoboCAD egui `consume_key` ordering lesson applies verbatim).

**Out:** gizmos, snapping, inertials, export.

**Accept:** build a base + arm from two STLs with a revolute joint typed
numerically; the slider swings it within limits; save, reopen, undo/redo
survive; `fk` unit tests against hand-computed poses for a 3-joint chain.

## M2 — Placement UX (the risk milestone)

*Goal: assemble a 3-DoF arm from a folder of STLs using only the mouse, in
under five minutes, without typing a coordinate.*

- `transform-gizmo-egui` for geom pose and joint origin; drag = preview,
  release = one command.
- Snapping: pick-point, triangle-vertex, AABB corners/centers, face normal.
- **Circle fit from a picked triangle fan** → joint axis and origin from a
  bore or shaft in one click; a visible confidence readout (residual, number
  of segments) so a bad fit is obvious.
- Joint glyphs in the overlay: axis line, limit arc, origin triad; hover a
  joint in the tree → highlight it in 3D and vice versa.
- Reparent with `keep_world_pose`.
- Snapshot tests for every gizmo/glyph state; iterate on this milestone with
  the snapshots open, not after.

**Out:** inertials, export.

**Accept:** the five-minute arm, recorded as a scripted `egui_kittest`
scenario that replays the clicks and asserts the resulting joint axes
within 1 mm / 0.5°. A human does it once for real and reports what was
annoying; that list is the M2 exit gate.

## M3 — Sim-ready: inertials, collision, export, import

*Goal: the exported MJCF loads in MuJoCo with zero warnings and moves like
the viewport does.*

- Mass properties ported from `robocad-kernel/src/mass.rs`; `compose_inertial`;
  `InertialSpec` modes and the comparison readout; closed-mesh detection.
- `CollisionPolicy`: convex hull (`riggen-mesh::hull`, quickhull), fitted
  box/cylinder/sphere/capsule, translucent collision view.
- `ResolvedRobot`; MJCF writer; URDF writer; export dialog with mesh path
  style; validation errors block export with the reason.
- URDF import via `urdf-rs`; `riggen robot.urdf` on the command line.
- Round-trip FK test in CI; MuJoCo load test in CI (Python job).
- Sample robot in `assets/` used by the tests and the README screenshot.

**Out:** convex decomposition, actuators, MJCF import.

**Accept:** the sample arm exports; `mujoco.MjModel.from_xml_path` succeeds
with no compiler warnings and `mj_forward` body poses match `fk` to 1e-6;
`urdf-rs` round-trip passes; importing a Menagerie-style URDF and
re-exporting it as MJCF loads too.

## M4 — Distribution

*Goal: `uv add riggen && riggen` on a machine that has never seen Rust.*

- `python/pyproject.toml` with maturin `bindings = "bin"`; `python -m riggen`.
- CI wheels for linux x86_64/aarch64, macOS arm64/x86_64, Windows x86_64;
  `uv build` locally; TestPyPI first, then PyPI. Reserve the crates.io name
  with a placeholder publish of `riggen-core`.
- README with a 30-second screencast, install line, and the sample robot;
  `--help`; a `--version` that prints the git hash.
- Startup time budget: window visible in < 500 ms on the dev machine, measured
  and asserted in a test.

**Accept:** a clean VM installs the wheel and opens the sample arm; the
release workflow is a tag push.

---

## v0.2 — Python SDK and the harder mesh work

- `riggen-py` (PyO3 over core + export): `Robot`, `Link`, `Joint`, `fk`,
  `validate`, `export_mjcf`, `export_urdf`, `load_urdf`; `riggen.show()`.
- Convex decomposition (CoACD port or a bundled binary — decide with an ADR).
- Named frames / MJCF sites; mimic joints; actuator presets.
- MJCF import; SDF export.
- Web demo build if the wasm check has stayed green.

## What not to spend agent time on

Physics, collision checking, parametric primitives, docking UI, a second
renderer, USD, or theming. Re-open any of these only through an ADR.
