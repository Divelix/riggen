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
ADR-0007 (the gizmo, bridged through `mint`), amended by ADR-0010. The
exit gate's findings are backlog lines.

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
  tree drag it was in M1.
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

**Status: done 2026-08-29, tag `m3`.** The risk — MuJoCo loading our MJCF
with zero compiler warnings and agreeing with `fk` — was retired at step 5
and held through the URDF import; the `mujoco` CI job runs both arms.
Decisions: ADR-0008. The `mujoco.viewer` look is still the human's; the
exit gate's findings are an M3 heading in the backlog.

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

**Status: done 2026-08-30, tag `m4`.** The risk — a wheel from this
workspace that installs and runs on a clean venv — was retired at step 1
and held through the container matrix. Decisions: ADR-0002, amended by
ADR-0009; the layout and the measured sizes are Architecture §Python
distribution. The clean-VM window run is still the human's; the exit
gate's findings are an M4 heading in the backlog.

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

*Goal: a robot you can build from ten lines of Python, and the mesh work
the window could not do.*

**Status: done 2026-09-02, tag `v0.2.1`.** The risk — that "sim-ready" was
a claim rather than a feature — was retired piece by piece: `model.nu`
stopped being zero, the pose graph is checked by the spec's own parser,
and a foreign MJCF opens. Decisions: ADR-0009 to ADR-0017; the bundle the
demo ships is measured in Architecture §The web build.

- `riggen-py` (PyO3 over core + export): `Robot`, `Link`, `Joint`, `fk`,
  `validate`, `export_mjcf`, `export_urdf`, `load_urdf`; `riggen.show()`.
- Convex decomposition (CoACD port or a bundled binary — decide with an ADR).
- Named frames / MJCF sites; mimic joints; actuator presets.
- MJCF import; SDF export.
- Web demo build if the wasm check has stayed green.

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

**Status: done 2026-09-04, tag `v0.3.0`.** The risk — that three exit
gates (M2, M3, M4) had each ended with a list of small frictions, none
paid down since M2, while the public demo put that UI in front of people
who have read no docs — was retired plan by plan. Decisions: ADR-0018 and
ADR-0019, both amending ADR-0010's switch table, and ADR-0020.

- **The overlay tells the truth** — depth-tested, so a glyph behind a part
  reads as behind it, and a driven or actuated joint stops looking free.
- **Numbers are editable** — properties fields as drag/scroll scrubbers,
  and an inertial tensor readable at 2.86e-5.
- **The panels stop hiding things** — the Joints window opens itself, a
  tool that wants the other selection says so, per-geom collision editing,
  a renamable material.
- **The tree says what a drag will do** — a ghost row and a grab cursor
  while reparenting, at the current `q` rather than the zero configuration.
- **The viewport answers the mouse** — left-drag orbits, the five tools
  have keys, the wheel steps a rotate ring, a translate drag snaps.

**Out:** any new format, importer or writer; distribution (crates.io, the
screencast, notarization); the demo's four gaps (web worker, WebGL2, touch,
directory drop) — all still backlog lines. §What not to spend agent time on
stands.

**Accept:** the M2 arm build, run by hand again end to end, produces no new
entry for this list — and the agent's own snapshot suite covers every
visible change (ADR-0003).

---

## v0.4 — the round trip keeps what it read, and the window has two modes

*Goal: a foreign MJCF survives import → edit → export with nothing
silently lost — what riggen has no field for, it gains a field for — and
the window opens in the mode a researcher wants first, for looking and
posing.*

**Status: done 2026-09-10, tag `v0.4.0`.** Two halves. The file's risk —
that a round trip still cost the user their hand-edited XML — was retired
field by field, schema 3 → 6, until `ROUND_TRIP_DROPPED` was empty; the
window's, that it had one mode and a floating window was how a robot got
posed. Decisions: ADR-0021 to ADR-0026, ADR-0022 refusing again.

- **The file: nothing silently lost** — actuators as a model-level table
  with a `<general>` escape hatch, couplings (mimic chains, `qpos_ref`,
  fixed tendons), `.msh` and inline mesh geometry, and composition
  resolved ahead of the reader.
- **The window: View and Edit** — `Tab` between them, the joint tree with
  its scrubbers in place of the Joints window, joints the only pick in
  View with the wheel driving a hovered one, the limit arc as a
  range-and-value band, Edit at the zero configuration, zen on `Z`.

**Out:** SDF import — the reading direction stays URDF and MJCF, and
`libsdformat` stays a CI test dependency (ADR-0016 §6); a Gazebo model
package; the demo's four gaps and distribution, both still backlog lines.
No new *writer*: the file half is about what survives the way in and back
out, not a fourth format. No rework of Edit beyond locking `q` (a backlog
line); no docking, no theming; §What not to spend agent time on stands.

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

- **A ViewCube in the corner.** *Landed (plans/viewcube-and-fly-camera,
  ADR-0028).* RoboCAD's, ported into `riggen-app` as an egui painter
  widget over the 26 `ViewOrientation` variants `orientation.rs` had held
  unused since M0. Bottom-right, its rect the third in `chrome_rects`
  beside the mode control's and the visibility row's, and gone in zen for
  the same reason they are. A facet click animates to that view, a drag on
  the cube orbits, the house icon re-frames, and the projection button
  **is** the readout — the viewport's `persp` / `ortho` text is removed
  rather than duplicated, so zen has no readout at all (`P`, `Num5` and
  `debug_state().camera` still answer). The bottom-left axes triad stays:
  it names the axes, the cube names the faces.
- **A fly camera on `W A S D E Q`.** *Landed (plans/viewcube-and-fly-camera,
  ADR-0028).* Not a second camera: there is one turntable and the keys walk
  its **pivot**. `OrbitCamera::fly` adds to `target` along the camera's own
  basis in all six directions — `E` / `Q` follow the view's up, not world
  Z, so all six are one rule and the pole heuristic needs no case of its
  own — at `FLY_SPEED · distance · boost · dt`, leaving yaw, pitch and
  distance alone. The eye follows rigidly, so the pivot arrives inside the
  assembly with you and the same left-drag then orbits *locally*, which is
  the gesture that was missing. Nothing downstream of the camera learned a
  new question. The other half landed with it: the **orbit pivot is drawn**
  while a camera gesture is live — a cross at `target` in the viewport's
  own overlay, `Occlusion::Always`, and **no fade**, because a fade is a
  clock in a golden (ADR-0003).
- **A ground grid at z = 0.** *Landed (plans/ground-grid-and-msaa).*
  RoboCAD never had one either; M0 shipped the gradient background alone,
  so a part at the origin and a part a metre up read the same. Z-up and
  meters are the document's (02 §Conventions), so the grid is simply where
  the floor is: `grid.wgsl` unprojects each pixel and intersects the ray
  with the plane — no mesh, since an infinite plane has none without an
  edge the camera can reach — and shades metre and ten-metre lattices, each
  a constant pixel thickness against its own screen-space derivative and
  faded out by that same derivative toward the horizon. Depth-tested from
  the intersection and writing no depth of its own, so a part in front
  hides it and no glyph below z = 0 is lost. Furniture like the axes triad,
  drawn in zen — with one thing the other furniture has not: the bullet as
  written did not ask for a switch, and it has one, the visibility row's
  sixth toggle (**ground**, leftmost), because a floor this large is worth
  being able to turn off and the row that turns off everything else riggen
  draws was already in the corner above it.
- **MSAA on the offscreen colour pass.** *Landed
  (plans/ground-grid-and-msaa).* RoboCAD had none. The scene renders into
  an offscreen colour + `Depth32Float` pair and is blitted in `paint()`
  (01 §Frame loop), so this is that pair and the blit. Both attachments
  now carry the sample count the adapter admits to for both formats — 4 or
  1, asked once through `get_texture_format_features` rather than assumed,
  and reported by `debug_state().sample_count`. The **pick pass stays
  single-sampled** — an `R32Uint` ID buffer cannot be resolved, and
  averaging two instance ids would invent a third. Colour resolves in
  hardware; depth cannot, since WebGPU has no depth resolve attachment and
  no multisampled copy source, so `depth_resolve.wgsl` writes sample 0 into
  a single-sampled texture for the overlay readback to copy — sample 0 and
  not an average, for the reason the pick pass is single-sampled. ADR-0020
  is untouched: the overlay still reads one `Depth32Float` texel per
  pixel.
- **Snapping during a rotate gizmo drag.** *Landed
  (plans/rotate-drag-snapping, ADR-0029).* ADR-0019 §5 left the drag snap
  to translation for want of a rule saying which of the three axes aligns.
  The rule is one the user never answers: a ring drag has one degree of
  freedom, so only the two frame axes perpendicular to the ring can move,
  and the one that lands is whichever of their four signed directions the
  drag has already brought nearest. The ladder runs **direction-only**
  while a rotate drag is in flight — a fitted circle's axis or a face
  normal, a vertex and a box corner saying nothing about direction — and
  the snap is unconditional, so over geometry a rotate drag reaches four
  orientations per ring and free rotation is over the background. The
  second overlay idiom is a cyan spoke from the gizmo's pivot at the
  ring's own radius, with the ladder's readout at its tip and the axis in
  front of it. One drag, one command, as before.
- **View's joint glyph loses its legacy pieces and gains an opaque band.**
  *Landed (plans/joint-glyph, ADR-0027).* In View a glyph is the band or
  the bars, the tick at `q`, and a filled bore for a driven joint —
  nothing else; the axis segment, the pivot dot, the origin triad and the
  actuator ring answer *where a joint frame is*, which is Edit's question,
  and Edit still draws all four. The band's three shades are **opaque**
  (`shade` scales the RGB at full alpha) rather than one hue at three
  alphas, so the grazing-angle seam a foreshortened sector drew over
  itself is gone by construction. What View draws is what View answers: a
  slide is picked by its bars, and a hinge by its band, where the axis
  line used to take the pick. Absorbed two backlog lines (from the glyph
  band, and from plans/joint-glyph-range-and-value).

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
