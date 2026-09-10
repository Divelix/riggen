# Roadmap

Every milestone ends with something you can run and show, and each retires
the scariest remaining unknown first. A milestone's "out" list is as binding
as its "in" list. Calibration: RoboCAD went from empty to 58k lines in three
weeks; this roadmap is smaller than that.

Spine: M0 → M1 → M2 → M3 → M4, then v0.2 → v0.3 → v0.4 → v0.5.

---

## M0 — Skeleton and the ported viewport

*Goal: `riggen part.stl` opens a window with the part in it.*

**Status: done 2026-08-29, tag `m0`.**

- Workspace of Architecture; CI (fmt, clippy, test, wasm build check).
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

- `riggen-core`: types of Data Model, `validate`, `fk`, snapshot
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
cheaper than feared; the method is Data Model §Mesh features. Decisions:
ADR-0007 (the gizmo from `transform-gizmo-egui`, bridged through `mint`),
amended by ADR-0010 (its egui glue is ours, the pointer shared per handle).
The by-hand exit gate came back "generally fine" with nine backlog lines;
the last two — the ViewCube and the fly camera — are v0.5's.

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
layout is Architecture §Python distribution.

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

**Status: done 2026-09-10, tag `v0.4.0`.** Two halves, one cycle. The
file's risk — that a round trip through riggen still cost the user their
hand-edited XML — was retired field by field, schema 3 → 6, until
`ROUND_TRIP_DROPPED` was empty: ADR-0023 (actuators are a model-level
table in their own namespace), ADR-0024 (the `<general>` escape hatch
beside the three presets, and an actuator keeps the ranges its file said),
ADR-0025 (couplings: mimic chains, `<joint ref>` as `Joint::qpos_ref`,
`<tendon><fixed>` as `Robot::tendons`), and ADR-0026 (composition is
resolved at import and never stored — every `<include>` spliced and every
`<frame>` folded away before the reader looks, taking Menagerie's
importing files from 93 to 172 of 261). ADR-0022 asked the composite-joint
question again with the corpus behind it and refused again; `<replicate>`
and `<attach>` stay refused too, now saying what each *is*. The window's
risk came from the human's notes rather than an exit gate — the window had
one mode, the editing one, and a floating slider window over it was how a
robot got posed — and ADR-0021 with its three amendments retired it.

- **The file: nothing silently lost.** `Robot::actuators` with its own
  names and an `ActuatorTarget`; `<general>` as a fourth `ActuatorSpec`;
  mimic chains through one `fk::resolve_q`, `qpos_ref` as an MJCF-only
  offset, fixed tendons as a document table; `.msh` and inline
  `<mesh vertex face>` geometry materialised as ordinary assets;
  `mjcf_compose` ahead of the reader.
- **The window: View and Edit.** `Tab` between them, the joint tree with
  its scrubbers in place of the Joints window, joints the only thing under
  the cursor in View with the wheel driving a hovered one, the limit arc as
  a range-and-value band, Edit locked to the zero configuration, zen on
  `Z`, and a visibility row in the corner the Joints window vacated.

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

---

## v0.5 — the viewport and the camera

*Goal: the viewport stops being M0's. You always know which way you are
looking, you can get inside an assembly instead of only orbiting round it,
a part reads as standing somewhere rather than floating in a gradient, and
an edge is an edge.*

The oldest untouched cluster in the backlog, most of it from the M2 exit
gate. v0.3 paid down the hand-feel debt in the *panels* and the pointer;
this is the same debt in the camera and the pass under it, and the
viewport is what the demo puts in front of people who have read no docs.
Every line below is a backlog line this section now owns.

- **A ViewCube in the corner.** RoboCAD has one; M0 ships the axes triad
  and a text `persp` / `ortho` label. Clicking a face snaps the view the
  way `Num1/3/7` already do, and the projection toggle lives on the cube
  rather than beside it. It is corner chrome like the mode control and the
  visibility row (`chrome_rects`, 01 §Panels and menus), so the camera
  holds still and the picks are off under it.
- **A fly camera on `W A S D E Q`.** The keys are already reserved off the
  tool shortcuts (`tool.rs`); M0 ships the turntable orbit alone, and a
  joint buried inside an assembly is something you can orbit round but not
  get to. rerun's viewer is the reference, including **drawing the orbit
  pivot while the camera moves** — which the turntable wants too, and which
  is the half of this line that is not a new camera at all.
- **A ground grid at z = 0.** RoboCAD never had one either; M0 ships the
  gradient background alone, so a part at the origin and a part a metre up
  read the same. Z-up and meters are the document's (02 §Conventions), so
  the grid is simply where the floor is.
- **MSAA on the offscreen colour pass.** RoboCAD had none. The scene
  renders into an offscreen colour + `Depth32Float` pair and is blitted in
  `paint()` (01 §Frame loop), so this is that pair and the blit. The
  **pick pass stays single-sampled** — an `R32Uint` ID buffer cannot be
  resolved, and averaging two instance ids would invent a third.
- **Snapping during a rotate gizmo drag.** ADR-0019 §5 left the drag snap
  to translation because a rotation about a named axis has nothing in the
  ladder to land on. The answer is aligning the dragged frame's axis to a
  snapped feature's (a circle's, a face normal), which needs a rule for
  *which* of the three axes aligns and a second overlay idiom — an ADR if
  the rule turns out to be contested.
- **View's joint glyph loses its legacy pieces and gains an opaque band.**
  `glyph_overlay()` (`app/glyphs.rs`) draws the axis segment, the pivot
  dot, the origin triad and the actuator ring in View exactly as it does
  in Edit; View is meant to show nothing but the scrubbable ring (the
  band, or a prismatic joint's bars) and the current-`q` tick. Reported by
  the human alongside a rendering bug: the band's three-layer alpha stack
  (`RANGE_ALPHA`/`LIMIT_ALPHA`/`VALUE_ALPHA`, `layered()`) reads wrong at
  a grazing camera angle, where the annulus sector's triangle strip
  foreshortens and overlaps itself in screen space — alpha blending
  double-covers the overlap, opaque colour would not. The fix is three
  distinct opaque colours (full range / limits / value) instead of one
  hue at three alphas, plus a replacement cue for an actuated joint now
  that the ring marking it is gone, plus hit-testing for a prismatic
  joint's bars in View (today it's picked by the axis line alone, which
  is one of the pieces going). Edit keeps the full legacy glyph; only its
  band/bar colours change along with View's, since `push_arc`/
  `push_slide` are one code path for both modes. Absorbs two backlog
  lines (from the glyph band, and from plans/joint-glyph-range-and-value).

**Out:** any new format, importer or writer, and the import gap's last
mile — the 31 Menagerie files that import and then refuse to *export*,
`<attach>`, a `PackageMap` UI — all still backlog lines. Distribution
(crates.io, the screencast, notarization) and the demo's four gaps (web
worker, WebGL2, touch, directory drop) stay backlog lines too; a WebGL2
fallback in particular is a *second picking mechanism*, not a camera
change. A snap quantum for a translate drag stays out beside them: the
wheel's 5° step is the rotation half of it and the document has nowhere to
keep a general one. §What not to spend agent time on stands — no second
renderer, no docking, no theming.

**Accept:** the M2 arm build, run by hand once more, is navigated with the
ViewCube and the fly keys and never with the numpad; a part dropped at the
origin reads as standing on the grid rather than hanging in the gradient;
an edge at 1440×900 is smooth while a pick on the pixel beside it still
names the right instance and triangle; and a rotate drag lands the dragged
frame's axis on a bore's. Every visible state in the snapshot suite
(ADR-0003) — and because MSAA moves every golden at once, that refresh is
one `snapshots:` commit that says so and nothing else.

---

## What not to spend agent time on

Physics, collision checking, parametric primitives, docking UI, a second
renderer, USD, or theming. Re-open any of these only through an ADR.
