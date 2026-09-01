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

**Status: done 2026-08-29, tag `m1`.** Decisions: ADR-0005 (ids, joints as
edges), ADR-0006 (drops, removal, import scale).

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
cheaper than feared; the method is 02-data-model §Mesh features. Decisions:
ADR-0007 (the gizmo from `transform-gizmo-egui`, bridged through `mint`),
amended by ADR-0010 (its egui glue is ours, the pointer shared per handle).
The by-hand exit gate came back "generally fine" with nine backlog lines;
eight remain.

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

**Status: done 2026-08-29, tag `m3`.** The risk — MuJoCo loading our MJCF
with zero compiler warnings and agreeing with `fk` — was retired at step 5
and held through the URDF import; the `mujoco` CI job runs both arms.
Decisions: ADR-0008 (export conventions). The `mujoco.viewer` look is still
the human's; the exit gate's findings are an M3 heading in the backlog.

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

**Status: done 2026-08-30, tag `m4`.** The risk — a wheel from this
workspace that installs and runs on a clean venv — was retired at step 1
and held through the container matrix. Decisions: ADR-0002, amended by
ADR-0009 (one wheel: the abi3 extension plus the binary as data); the
layout is 01-architecture §Python distribution.

Two measurements this file is the only record of. **Startup** on the dev
machine (RTX 5090, X11): `RiggenApp::new` to the first frame 8 ms — the
part the budget test pins — and launch to the first frame 380–500 ms, of
which ~200 ms is NVIDIA's Vulkan device creation and the rest the X11
window. **Wheels** at `v0.2.0`: linux x86_64 9.7 MB, linux aarch64 9.2,
macOS arm64 6.2, macOS x86_64 6.6, Windows 7.4, sdist 0.3; the abi3
extension is 1.3 MB of that.

PyPI's CDN can serve the previous version for some minutes after an
upload, so a `pip install` straight after a release may lag. The clean-VM
window run is still the human's; the exit gate's findings are an M4
heading in the backlog.

**Accept:** a clean VM installs the wheel and opens the sample arm; the
release workflow is a tag push.

---

## v0.2 — Python SDK and the harder mesh work

**Status: in progress.** Done so far:

- **2026-08-30, tag `v0.2.0`** — the Python SDK (ADR-0009): `import
  riggen` beside the app in one wheel, `python/riggen/` the API over the
  `riggen._riggen` extension. 01-architecture §Python distribution and
  §Python SDK have the shape; the by-hand notebook run passed.
- **2026-08-30** — the viewport pointer (ADR-0010): with Move or Rotate
  active the viewport orbits, pans, zooms, tints and selects as it does
  with no tool, everywhere but on a gizmo handle. Closed the M2 exit
  gate's largest line.
- **2026-08-31** — convex decomposition (ADR-0011). Not the CoACD port or
  bundled binary this line guessed at: `parry3d-f64` has V-HACD in pure
  Rust at f64, so `riggen_mesh::decompose` is a module beside the
  quickhull and the wasm check stayed green. `CollisionPolicy::
  ConvexDecomposition { max_hulls, resolution, concavity }` resolves to N
  collision geoms and `<stem>_hull_N.stl` in both writers, computed once
  per `(MeshId, params)` on `riggen-app::jobs` — the first job thread,
  which 01 §Jobs and threads had specified since M3 — offered in the
  properties panel and as `riggen.ConvexDecomposition`. parry implements
  V-HACD's split half and not its merge half, so `decomp::merge` is ours
  and `max_hulls` is a real ceiling. The `mujoco` job loads a decomposed
  model as its third.

- **2026-08-31** — named frames (ADR-0012). `Robot::frames`, in the schema
  since M1 and always empty, is live: a frame is created in the tree
  ("+ Frame"), posed with the same gizmo and snap ladder that place a joint,
  edited in the properties panel and read and written from the SDK. It
  exports as an MJCF `<site>` and — URDF having no such element — a massless
  dummy link on a fixed joint, the ROS convention; the import deliberately
  does not reverse the second, because nothing tells our dummy from a real
  unweighed link. Frames and links share one namespace, checked in
  `validate`, since URDF spells both `<link>`. The `mujoco` job compares
  every site pose from `mj_forward` against `fk::frames` at five
  configurations, over both the `.riggen` and the URDF route.

- **2026-08-31** — mimic joints (ADR-0013). `Joint::mimic` couples one
  joint's `q` to another's, `q = multiplier · q(leader) + offset`, and
  `fk::resolve_q` is the one place that rule lives, read by `fk`, the
  Joints window and `--fk-samples` alike. `validate` refuses the seven
  shapes that would export to a model MuJoCo loads and simulates wrongly —
  chains among them. It writes as URDF's native `<mimic>` and as an MJCF
  `<equality><joint polycoef>`, a *soft* solver constraint rather than a
  reduction, and — closing the `ImportWarning::MimicDropped` dead end that
  shipped in M3 — the URDF import keeps it, dropping with a reason only
  what the document cannot hold. The first schema bump: `.riggen` is
  version 2, `load` walks an `upgrade_vN_to_vN+1` chain and
  `pendulum.riggen` is frozen at 1 as the file it reads. The `mujoco` job
  checks every `mjEQ_JOINT` against the sampled `qpos`, so a swapped
  `polycoef` order or a dropped `<equality>` fails.

- **2026-08-31** — actuator presets (ADR-0014). `Joint::actuator` holds
  one of `Position { kp, kv }`, `Velocity { kv }` or `Motor { gear }`, and
  the MJCF writer turns it into an `<actuator>` element named after its
  joint, `ctrlrange` from the joint's limits (or the normalised `-1 1` of a
  motor) and `forcerange` from `Limits::effort`, each attribute omitted
  where the number is the unfilled zero. `model.nu` stops being zero and
  `data.ctrl["shoulder_joint"]` works, which is what "sim-ready is a
  feature, not a claim" was missing. It **amends ADR-0004 §4**: the
  apologetic "need an `<actuator>`" comment survives only on a joint that
  has none. URDF invents no `<transmission>` and names the preset in a
  comment instead, beside the `armature` one. `validate` refuses an
  actuator on a fixed joint, on a mimic follower (already driven by its
  `<equality>`), and any gain MuJoCo cannot use. The panel edits one per
  joint and `SetActuators` gives the whole model the same one in a single
  undo; the SDK has `Position` / `Velocity` / `Motor`. `.riggen` is schema
  3 — the second bump, an empty step on the chain ADR-0013 built — and the
  `mujoco` job holds `MjModel` to an `actuators` block `--fk-samples`
  writes, `model.nu` included, so a dropped or invented actuator fails.

- **2026-09-01** — MJCF import (ADR-0015). `mjcf_in::load` opens an MJCF
  by every route a URDF already did — `riggen robot.xml`, File › Import
  MJCF…, a dropped `.xml`, `riggen --export … robot.xml`,
  `riggen.load_mjcf()` — over a `quick-xml` DOM that is the reading half
  of `xml.rs`, where MJCF's five spellings of one rotation collapse to one
  `DQuat`. `<compiler>` and the `<default>` class tree are resolved at
  read and dropped, so the document holds numbers rather than a second
  MJCF-shaped description of itself. Our own export round-trips: bodies,
  joint kinds, axes, limits, dynamics, inertials, meshes, `<site>` →
  `Frame` — the symmetry ADR-0012 promised and the URDF import cannot have
  — `<equality>` → `Joint::mimic`, `<actuator>` → the three presets. A
  foreign, Menagerie-shaped file imports too, with an `ImportWarning`
  naming every element it holds that the document has no field for and an
  `ImportError` for the shapes the link tree cannot represent at all — a
  body with several joints among them. One warning vocabulary now serves
  both imports. The `mujoco` job gains a fourth model: the arm exported,
  imported and exported again, held to the *original* document's `fk.json`.

- **2026-09-01** — SDF export (ADR-0016). `sdf.rs` is the third dumb
  serialiser of one `ResolvedRobot`, and the one that apologises least: a
  capsule stays a capsule, a `Frame` is SDF's own `<frame attached_to>`
  rather than a dummy link, and a mimic is `<axis><mimic>`, which is why
  the file declares spec **1.11**. It costs no arithmetic, because SDF's
  defaults are riggen's conventions: a link's `<pose relative_to="«parent»">`
  *is* `ResolvedJoint::origin`, a joint carries no `<pose>` because SDF
  already expresses one in the child link frame (ADR-0004), and `<xyz>`
  carries no `expressed_in` because its default is that same frame. Only
  the actuator still becomes a comment — Gazebo's `<plugin>` names a C++
  class and a version of Gazebo, so ADR-0014's URDF reasoning holds word
  for word. `Format` stopped being a three-valued enum and became a set of
  three booleans, spelled `mjcf|urdf|sdf|both|all` on the command line and
  in the SDK and three checkboxes in the dialog. The `sdf` CI job holds the
  file to **libsdformat itself**: the spec's own parser raises on anything
  illegal and resolves the pose graph, and FK over what it resolved matches
  `fk` to 1e-9 — a tighter bar than the `mujoco` job's, because nothing in
  that loop is a simulator. `pybullet` is not in CI and reads our SDF
  wrong, measured and stated: it ignores `relative_to` in silence, and its
  users want the `.urdf` the same export writes.

- **2026-09-02** — the web demo (ADR-0017), at
  [divelix.github.io/riggen](https://divelix.github.io/riggen/). Not a
  thinner web riggen: the browser runs the *same* `riggen_core::load_from`,
  the same URDF and MJCF imports and the same three writers, over one
  read-only `FileSource` seam — `Disk` on the desktop, the drop gesture's
  files in a page. Bytes in (`load_mesh_bytes`, `load_from`, a `DroppedSet`
  that resolves a mesh reference by file name), bytes out (`export_files`
  under `export`, and Save / Export / Debug › Save state as downloads, the
  export a stored zip). The sample arm is in the viewport on load, out of
  the same `include_bytes!` `--example arm` unpacks. WebGPU only, with a
  plain-English page for a browser that has none, and V-HACD asks before it
  freezes the tab, because `jobs` has no thread there. `pages.yml` deploys
  on every push to `main`; the `wasm` CI job builds the same bundle, so a
  break shows up in CI first.

  A third measurement this file is the only record of. **The wasm bundle**
  at the deploy: 10.40 MB raw, **3.35 MB gzipped**, which is what a visitor
  downloads. The `web` profile (`opt-level = "s"`, fat LTO) is worth 0.32 MB
  gzipped over `--release`, and `wasm-opt` is *not* used: `-O2`, `-Os` and
  `-Oz` each take ~1 MB off the raw file and put ~0.12 MB **back on** the
  gzipped one, so it costs CI minutes to make the download bigger.

**Accept:** the public URL loads with a clean console, the arm's slider
swings it, a dropped set of meshes opens, and the zip Export hands the
browser is byte-identical to `riggen --export all` over the same files.

## What not to spend agent time on

Physics, collision checking, parametric primitives, docking UI, a second
renderer, USD, or theming. Re-open any of these only through an ADR.
