# 03 — Roadmap

Every milestone ends with something you can run and show, and each retires
the scariest remaining unknown first. A milestone's "out" list is as binding
as its "in" list. Calibration: RoboCAD went from empty to 58k lines in three
weeks; this roadmap is smaller than that.

Spine: M0 → M1 → M2 → M3 → M4, then v0.2.

---

## M0 — Skeleton and the ported viewport

*Goal: `riggen part.stl` opens a window with the part in it.*

**Status: done 2026-08-29, tag `m0`.**

- Workspace of 01-architecture; CI (fmt, clippy, test, wasm build check).
- `riggen-mesh`: `TriMesh`, STL (binary + ASCII) and OBJ loaders, AABB,
  ray/triangle.
- `riggen-viewport` ported from `robocad-viewport`: cgmath → glam, `BodyId`
  → `InstanceId`, `RenderMesh` → `TriMesh`, `TopoRef` → `PickHit
  (InstanceId, triangle)`, sketch-plane code, edge/vertex pick passes and
  the face-outline pass removed. Camera, low-latency surface config,
  whole-instance hover/select restyle, axes triad, gradient background,
  zoom-to-fit kept. (robocad never had a ground grid or MSAA; both are
  backlog items, not ports.)
- `riggen-app`: eframe shell, CLI args, file drop and `rfd` open, one
  instance per dropped file, File › Open/Quit menu, status bar with
  frame-time HUD.
- `egui_kittest` snapshot harness and `debug_state()` from day one.

**Out:** any document; any panel beyond the status bar and the File menu.

**Accept:** drop three STLs → three orbitable, pickable parts; the
`startup`, `cube`, `hover_cube`, `select_cube`, `three_parts` snapshot
scenarios pass on the CPU adapter; wasm target builds.

## M1 — Document, tree, joints, FK

*Goal: a two-link pendulum you can save, reopen, and swing.*

**Status: done 2026-08-29, tag `m1`.** `Reparent { keep_world_pose }`
landed here, as a drag in the tree, and stayed there — M2's gizmo moves a
link within its parent instead. Decisions: ADR-0005 (ids, joints as edges),
ADR-0006 (drops, removal, import scale).

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

**Status: done 2026-08-29, tag `m2`.** The risk — a circle fit good enough
to place a joint from one click on STL data with no B-Rep — came out
cheaper than feared: exact-position welding plus a local dihedral walk plus
a Kåsa least-squares fit, and the segment count is what tells a bore from a
cube face. Decisions: ADR-0007 (the gizmo comes from `transform-gizmo-egui`,
bridged through `mint`). The four open questions were settled by the human
before the work started: editing tools work in the zero configuration; a
gizmo on a link moves the link and on a joint moves the pivot; the crate
before an own gizmo; glyphs for movable joints plus the selection.

The by-hand exit gate was run and came back "generally fine", with nine
lines then in `docs/BACKLOG.md`. The largest — the gizmo swallowing all
viewport pointer input while it was near the cursor, which read as a dead
camera and clicks that did not select — is fixed (ADR-0010, 2026-08-30);
eight lines remain.

- `transform-gizmo-egui` on a link (its parent joint's origin; the subtree
  follows) or on a joint (its pivot, the geometry staying put); drag =
  preview, release = one command. A geom's own pose stayed
  properties-panel-only.
- Snapping: pick-point, triangle-vertex, AABB corners/centers, face normal,
  behind the Place joint and Align tools.
- **Circle fit from a picked triangle fan** → joint axis and origin from a
  bore or shaft in one click; a visible confidence readout (residual, number
  of segments) so a bad fit is obvious.
- Joint glyphs in the overlay: axis line, limit arc, origin triad; hover a
  joint in the tree → highlight it in 3D and vice versa.
- Align: two clicks bring a part exported out of place onto a feature, as
  one `SetJoint` through `fk::origin_for_world`. Reparenting stayed the
  tree drag it was in M1 — the gizmo moves a link *within* its parent, and
  the two gestures turned out not to want the same handle.
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

**Status: done 2026-08-29, tag `m3`.** The risk — MuJoCo loading our
MJCF with zero compiler warnings and agreeing with `fk` — was retired at
step 5 and held through the URDF import: both `arm.riggen`'s export and
`arm.urdf`'s re-export load clean with 25 matching body poses, the `mujoco`
CI job runs both. Decisions (ADR-0008): meshes baked to meters as binary
STL with no `scale`, `<inertial fullinertia>` so MuJoCo decomposes, the
headless `riggen --export` on the app binary; `CollisionPolicy::Meshes`
keeps imported collision meshes losslessly; floating base is an export
option, not a document field; `SetRoot` across a movable joint stays
refused. No job thread: hulls are synchronous and cached per `MeshId`.
The by-hand half was done headlessly (both arms swing 10 s under gravity
without a NaN; the `mujoco.viewer` look is still the human's); what was
annoying is under an M3 heading in the backlog — narrow tensor fields,
overlapping shells counted twice, no `PackageMap` UI, imports without a
material, soft joint limits.

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

**Status: done 2026-08-30, tag `m4`** (plans/m4-distribution). The risk —
a maturin `bin` wheel from this workspace that installs and runs on a
clean venv — was retired at step 1 and held through the container
matrix. Decisions: `pyproject.toml` at the repository root so the README
is the PyPI page too (OPEN 3); the binary in the wheel's `scripts/`, no
console script, so `riggen` is the executable with no interpreter in
front of it; `python -m riggen` execs it; the version lives once, in
Cargo; native wgpu backends only (eframe's GL enumeration cost 100–150
ms for an adapter never picked); `--example arm` from `include_bytes!`
of the tracked fixtures, no new mesh in git (OPEN 4); no screencast until
the GUI is polished (OPEN 2); crates.io publishing is its own later plan
(OPEN 1). Wheel sizes from the `v0.1.0` release run: linux x86_64
9.1 MB (binary 22 MB, stripped + thin LTO; 566 KB of it is the CycloneDX
SBOM maturin adds), linux aarch64 8.6 MB, macOS x86_64 6.2 MB, macOS
arm64 5.8 MB, Windows x86_64 7.0 MB, sdist 0.3 MB. Startup on the dev machine (RTX 5090, X11): `RiggenApp::new`
to the first frame 8 ms; launch to the first frame 380–500 ms, of which
~200 ms is the NVIDIA Vulkan device creation and the rest the X11 window
— the budget test pins the part that is ours. Verified headlessly against
the published indexes: `uvx --index-url https://test.pypi.org/simple/ …
riggen --version` on the dev machine and `pip install riggen` in
`python:3.12-slim` (no Rust, no checkout) both print `riggen 0.1.0
(fbd26be 2026-08-30)`; `release.yml` was green for the TestPyPI dispatch
and for the `v0.1.0` tag (all five wheels, three smokes, PyPI + GitHub
Release). PyPI's CDN served the old 0.0.1 for a few minutes after the
upload — a first `pip install` right after a release can lag. The
clean-VM window run is still the human's. What was
annoying is under an M4 heading in the backlog.

**Accept:** a clean VM installs the wheel and opens the sample arm; the
release workflow is a tag push.

---

## v0.2 — Python SDK and the harder mesh work

- **Done 2026-08-30** — the Python SDK (plans/python-sdk, ADR-0009): one
  wheel per platform with two halves, `crates/riggen-py` (PyO3 abi3
  `riggen._riggen`, one method per `Command`, values in the schema's
  shape) plus the app binary as wheel data; `python/riggen/` the API —
  `Robot` with `Link` / `Joint` / `Geom` handles, `Pose`, `Fixed` /
  `Revolute` / `Continuous` / `Prismatic`, the inertial specs, `load`,
  `load_urdf`, `fk`, `export`, `show()` → `Viewer.wait()`. 37 public
  names, 53 methods and properties on 18 classes, all documented, pyright
  clean; 57 pytest tests on the built wheel. Decisions: the data-directory
  layout and abi3 (ADR-0009), no numpy (tuples and nested lists), no live
  link (a file and `show()`), the `_riggen` layer speaks the v1 schema.
  Wheel sizes at 0.2.0.dev0: linux x86_64 9.7 MB (M4: 9.6; 10.1 with the
  full extension), linux aarch64 9.2, macOS arm64 6.2, macOS x86_64 6.6,
  Windows 7.4; the extension 1.3 MB. The by-hand notebook run
  (2026-08-30, the TestPyPI `0.2.0` wheel in a fresh `uv` project with
  ipykernel): the pendulum from the README, `show()`, a joint placed by
  hand and saved, `wait()`, MuJoCo — worked; it found one message with
  no file name in it (fixed before the tag) and that a TestPyPI
  pre-release needs an explicit index in a `uv` project (README
  §Developing). The `v0.2.0` tag is the release.
- **Done 2026-08-30** — the viewport pointer, the M2 exit gate's first
  line (ADR-0010): the gizmo's egui glue is ours instead of
  `GizmoExt::interact`, it hit-tests its own handles with
  `Gizmo::pick_preview` and registers a click-only widget solely on the
  frames it wants the pointer, and the viewport's one suppression switch
  became `set_pick_suppressed` (picks off, camera live) beside
  `set_pointer_blocked` (both off) with camera input on
  `contains_pointer()`. With Move or Rotate active the viewport now orbits,
  pans, zooms, tints and selects exactly as with no tool — everywhere but
  on a handle. `gizmo_shares_the_viewport` is the acceptance run; no golden
  PNG changed.
- Convex decomposition (CoACD port or a bundled binary — decide with an ADR).
- Named frames / MJCF sites; mimic joints; actuator presets.
- MJCF import; SDF export.
- Web demo build if the wasm check has stayed green.

## What not to spend agent time on

Physics, collision checking, parametric primitives, docking UI, a second
renderer, USD, or theming. Re-open any of these only through an ADR.
