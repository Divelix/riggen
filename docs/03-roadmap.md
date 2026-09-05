# 03 — Roadmap

Every milestone ends with something you can run and show, and each retires
the scariest remaining unknown first. A milestone's "out" list is as binding
as its "in" list. Calibration: RoboCAD went from empty to 58k lines in three
weeks; this roadmap is smaller than that.

Spine: M0 → M1 → M2 → M3 → M4, then v0.2 → v0.3 → v0.4.

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

*Goal: a robot you can build from ten lines of Python, and the mesh work
the window could not do.*

- `riggen-py` (PyO3 over core + export): `Robot`, `Link`, `Joint`, `fk`,
  `validate`, `export_mjcf`, `export_urdf`, `load_urdf`; `riggen.show()`.
- Convex decomposition (CoACD port or a bundled binary — decide with an ADR).
- Named frames / MJCF sites; mimic joints; actuator presets.
- MJCF import; SDF export.
- Web demo build if the wasm check has stayed green.

**Status: done 2026-09-02, tag `v0.2.1`.** The risk — that "sim-ready" was
a claim rather than a feature — was retired piece by piece: `model.nu`
stopped being zero (ADR-0014), the pose graph is checked by the spec's own
parser (ADR-0016), and a foreign MJCF opens (ADR-0015). Decisions:
ADR-0009 (one wheel: the abi3 extension plus the binary as data),
ADR-0010 (the gizmo shares the viewport pointer), ADR-0011 (V-HACD from
`parry3d-f64`; the merge step is ours), ADR-0012 (frames as sites and
dummy links), ADR-0013 (mimics through one `fk::resolve_q`), ADR-0014
(three actuator presets, amending ADR-0004 §4), ADR-0015 (the MJCF import
subset and one import vocabulary), ADR-0016 (SDF 1.11 conventions),
ADR-0017 (web IO: one `FileSource` in, downloads out).

Two guesses this section made that the work overturned, kept because the
next cycle will make the same kind: convex decomposition needed **no**
CoACD port or bundled binary — `parry3d-f64` has V-HACD in pure Rust at
f64 — and the web demo was **not** merely "a build if the wasm check has
stayed green", but the seam that made `riggen-core` and `riggen-export`
runnable with no filesystem under them.

A measurement this file is the only record of. **The wasm bundle** at the
first deploy: 10.40 MB raw, **3.35 MB gzipped**, which is what a visitor
downloads — confirmed at 3.42 MB over the wire, so GitHub Pages does
compress `application/wasm`. The `web` profile (`opt-level = "s"`, fat
LTO) is worth 0.32 MB gzipped over `--release`; `wasm-opt` is *not* used,
because `-O2`, `-Os` and `-Oz` each take ~1 MB off the raw file and put
~0.12 MB **back on** the gzipped one.

**Out:** physics, a second renderer, and anything in §What not to spend
agent time on; a web worker for `jobs`, a WebGL2 fallback and a touch
layout, all backlog lines the demo's non-goals left behind.

**Accept:** `pip install riggen` gives both the app and `import riggen`;
MuJoCo and libsdformat load what the exporters write and agree with `fk`;
an MJCF round-trips; and the public demo loads with a clean console, its
slider swings the arm, and its Export is byte-identical to the CLI's.

---

## v0.3 — the hand-feel debt

*Goal: the window stops costing the user a puzzled minute. Nothing new is
exported; what is already there answers the mouse and the keyboard the way
a CAD tool does.*

**Status: done 2026-09-04, tag `v0.3.0`.** The risk — that three exit gates
(M2, M3, M4) had each ended with a list of small frictions and none had been
paid down since M2, while the public demo was putting that UI in front of
people who have read no docs — was retired plan by plan: `panels-and-numbers`,
`orbit-left-drag`, `viewport-answers-the-mouse`, `overlay-tells-the-truth`.
The cycle expected one ADR and took three, all of them about who owns the
pointer or what the user is allowed to believe: ADR-0018 (left = orbit; a
gizmo handle claims the primary drag), ADR-0019 (the wheel is claimable and
a drag keeps its hover pick) — both amendments to the switch table ADR-0010
published, which is now five switches and `set_pick_excluded` — and ADR-0020
(the overlay reads the scene's depth back). Two decisions were taken as
paragraphs in 01 and 02 rather than ADRs: history gestures, and `Reparent`
at the current `q`.

- **The overlay tells the truth.** A depth-tested overlay, so a glyph behind
  a part reads as behind it; a badge or tint on a joint glyph that is driven
  (ADR-0013) or actuated (ADR-0014), which before this looked like free
  joints.
- **Numbers are editable.** Properties fields as drag/scroll scrubbers
  (Blender-style, wheel to step); the inertial tensor readable — 2.86e-5
  must not render as a clipped `0.000029`.
- **The panels stop hiding things.** The Joints window opens itself when a
  document has a movable joint; a tool that wants the other kind of
  selection says so instead of doing nothing; clicking empty space with a
  joint selected clears it; per-geom collision editing; a material can be
  renamed.
- **The tree says what a drag will do.** A ghost row at the cursor and a
  grab cursor while reparenting; `Reparent { keep_world_pose }` at the
  current `q` rather than the zero configuration.
- **The viewport answers the mouse.** Left-drag orbits (shift+left and right
  pan, the middle pair kept); the five tools have keys `V` `G` `R` `J` `B`;
  the wheel over a rotate ring steps that ring by 5°, or 1° with shift; a
  translate drag runs the snap ladder under the cursor.

**Out:** any new format, importer or writer; distribution (crates.io, the
screencast, notarization); the demo's four gaps (web worker, WebGL2, touch,
directory drop) — all still backlog lines. §What not to spend agent time on
stands.

**Accept:** the M2 arm build, run by hand again end to end, produces no new
entry for this list — and the agent's own snapshot suite covers every
visible change (ADR-0003).

---

## v0.4 — the round trip keeps what it read, and the window has two modes

*Goal: a foreign MJCF survives import → edit → export with nothing silently
lost — what riggen has no field for, it gains a field for — and the window
that opens it opens in the mode a researcher wants first, the one for
looking and posing.*

Two halves, one cycle. The first is the file's; the second is the window's,
and it came out of the human's notes after v0.3 closed rather than an exit
gate: the window still has one mode, the editing one, and a floating slider
window over it is how a robot gets posed.

### The file: nothing silently lost

SEED §3 says the common case is editing, not building, and §4's fourth
differentiator is "import existing URDF, edit, export MJCF". ADR-0015 bought
the first half: a Menagerie-style file opens. The second half is what this
cycle is for — the import warns and drops, so a round trip through riggen
still costs the user their hand-edited XML. Every line below is one of those
drops, and each is a backlog line this section now owns.

- **Actuators become a model-level table.** `Joint::actuator` promoted to a
  top-level `Robot::actuators` keyed by what it drives — the shape MJCF has
  (ADR-0014's option F) — so an actuator on a tendon or a site has somewhere
  to land instead of an `ActuatorDropped` warning. A schema bump whose
  `upgrade_` step moves each `Some(spec)` into the map.
- **The escape hatch beside the three presets.** `<general>` — and
  `<adhesion>`, `<muscle>` — carried through with its `dyntype` / `gaintype`
  / `biastype`, so a user who needs one is not hand-editing after every
  export. Actuator gains in a `<default class>` rather than on every
  element, and explicit `ctrllimited` / `forcelimited` beside the
  `autolimits="true"` we write.
- **Couplings the document cannot hold.** Mimic chains — a follower whose
  leader also follows — turning `fk::resolve_q`'s one pass into a
  topological one, no schema change; `<joint ref>`, which moves a joint's
  zero and is warned and ignored today; `<tendon><fixed>` for a coupling
  that really is a cable, beside the `<equality>` a mimic writes.
- **Geometry the import refuses.** `.msh` meshes and an inline `<mesh vertex
  face>`, a `GeomDropped` warning and no geometry today.
- **Composition.** `<include>`, `<attach>`, `<replicate>` and MuJoCo 3's
  `<frame>` wrapper — every one an `ImportError::UnsupportedElement`
  (ADR-0015 §5) — behind one resolver, with `<frame>`'s transform folded
  into the bodies inside it.

⚠ OPEN: whether a `<body>` with several `<joint>`s synthesises massless
intermediate links, so MuJoCo's ball and planar DoFs import instead of being
refused as `ImportError::CompositeJoint`. ADR-0015 §5 turned it down because
a synthesised link is a link the user did not draw; a cycle about losing
nothing has to ask again. Needs an ADR before it is planned.

### The window: View and Edit

What a researcher does with a robot most of the time is look at it and
pose it; building and fixing it is the rarer, more dangerous thing, and the
window makes no difference between the two. Every line below is a backlog
line this section now owns.

- **Two modes, Tab between them.** A document opens in **View**. Tab goes
  through `consume_key` like the tool keys (the RoboCAD lesson), and the
  status bar names the mode. The switch table ADR-0010 published and
  ADR-0018/0019 amended gains a mode dimension — which picks answer, who
  owns the wheel — so this half's pointer rules want an ADR as its first
  line, the way v0.3's did.
- **View: the joint tree, with the sliders.** The left panel shows the
  movable joints as a tree in kinematic order, each row a scrubber aligned
  right — drag or wheel, the value and both limits readable on it, the
  scrubber idiom v0.3 built. It replaces the floating Joints window, which
  is deleted with its open-itself rule (01 §Panels, plans/panels-and-numbers
  OPEN 3). A follower's row stays what its slider was: read-only, at the
  resolved `q` (ADR-0013).
- **View: joints are the only thing under the cursor.** A mesh is neither
  hover-tinted nor selectable; the glyph is what answers, and the wheel
  over a hovered joint **drives it** instead of zooming — ADR-0019's
  `set_wheel_claimed`, claimed by a glyph the way a rotate ring claims it,
  with the ring's 5° / 1° steps. Today a hovered glyph leaves the wheel to
  the camera (ADR-0010).
- **The glyph shows the range and the value.** *Landed.* The limit arc is
  a **band** (01 §Joint glyphs): an annulus of three translucent sectors
  resulting in the full circle at 0.2, the limits over it at 0.5 and the
  run from zero to `q` at 0.9, the white spoke kept, a slide getting the
  same two as bars beside its axis. Sized from the part it belongs to —
  and now from the part's *own* frame, so the size no longer breathes as
  the joint turns — and depth-tested quad by quad (ADR-0020). Every state
  of it is in the snapshot suite (ADR-0003) and `glyph_driven_joint`'s
  amber survived.
- **Edit: the tree as it is, `q` locked.** The link tree, properties,
  gizmos and tools stay what v0.3 left; joints are highlighted in the tree
  but not posable there — posing is View's — and the gizmo moves a joint
  relative to its parent as it does now.
- **Zen mode on `Z`.** Every panel — menu bar, tree, properties, status
  bar, toolbar, the visibility row — hidden, the viewport filling the window
  with the robot alone; `Z` again brings them back. Bare `Z` is free (undo
  is Ctrl+Z, through `consume_key` in the same order), and the mode is the
  same in View and Edit.
- **A visibility row, top-right of the viewport.** One row of icons,
  Blender's overlay toggles for riggen's things: joints, joint names,
  links, frames, collision geometry — View › Collision geometry moves out
  of the menu into it, and the corner is the one the Joints window vacates.
  The toolbar keeps the top-left.

**Out:** SDF import — the reading direction stays URDF and MJCF, and
`libsdformat` stays a CI test dependency (ADR-0016 §6); a Gazebo model
package; the demo's four gaps and distribution, both still backlog lines;
§What not to spend agent time on stands. No new *writer*: the file half is
about what survives the way in and back out, not a fourth format. On the
window side: no rework of Edit beyond locking `q` — whether the gizmo on a
*link* (which moves its parent joint's origin with the subtree, ADR-0007)
survives a mode whose gesture is "move the joint relative to its parent"
is a backlog line, not this cycle's; no docking, no theming.

**Accept:** `menagerie_style.xml`, grown to carry the elements above,
imported and re-exported loads in MuJoCo with zero warnings, agrees with
`fk` to 1e-6, and its `<actuator>`, `<equality>` and `<tendon>` blocks name
what the original's did — no `ElementDropped`, `ActuatorDropped` or
`GeomDropped` warning left for anything this section lists. The `mujoco`
job's round-trip model stays held to the *original* document's `fk.json`.
And the sample arm, opened cold, is posed with nothing but the wheel over a
glyph and the joint tree's scrubbers, without a menu or a floating window
in the way; `Z`, and the robot is all there is; Tab, and it is the v0.3
editor again — every visible state of both modes in the snapshot suite
(ADR-0003).

## What not to spend agent time on

Physics, collision checking, parametric primitives, docking UI, a second
renderer, USD, or theming. Re-open any of these only through an ADR.
