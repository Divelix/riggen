# Plan: m2-placement-ux

- Started: 2026-08-29
- Milestone: M2
- Idea (verbatim from the human): "plan m2"

## Goal

A 3-DoF arm is assembled from a folder of STLs with the mouse alone, in
under five minutes, without typing a coordinate: parts are dropped under
the selected link (ADR-0006) so the chain builds itself; a joint's axis and
origin come from one click on a bore or shaft (circle fit with a visible
confidence readout); a part exported out of place is put in place by two
clicks (align: point→point, circle→circle concentric); a transform gizmo
moves a link or a joint pivot with drag = preview, release = one command;
every joint is visible as a glyph (axis, origin triad, limit arc, current
`q`) that highlights from the tree and back. All of it is pinned by
snapshot scenarios and the scripted `five_minute_arm` acceptance.

## Non-goals

- Inertials, collision, export, URDF import (M3).
- Snapping *during* a gizmo drag (the align tool is the mouse-only route;
  backlog line added by step 9).
- A geom-level gizmo: a geom's pose stays properties-panel-only; the gizmo
  edits joint origins (OPEN 2).
- Frame-rewriting commands at `q ≠ 0` (`Reparent`, `MoveJointFrame` work in
  the zero configuration; OPEN 1 and the existing backlog line).
- Tree drag feedback, multi-selection, measuring, a depth-tested overlay.
- Writing our own gizmo unless the step-4 spike fails (OPEN 3).

## Design deltas

- **`riggen-mesh`** — `TriMesh::cylinder` / `tube` generators (fixtures and
  tests), `stl::write_binary` (the fixture generator), and a `feature`
  module: welded edge adjacency (positions hashed exactly — STL repeats
  coordinates verbatim; partially delivers the "vertex welding" backlog
  line), smooth-region growth from a triangle across shared edges by
  dihedral angle, and `fit_circle(mesh, triangle) -> Option<CircleFit {
  center, axis, radius, residual, segments }>`: for a curved region the
  axis is the normalised sum of adjacent-normal cross products and the
  circle a 2-D least-squares fit of the region's vertices in the plane ⟂
  axis (center at the region's mean height); for a planar region the axis
  is the face normal and the circle is fitted to the boundary loop (a
  shaft's end face). No eigen solver, no B-Rep. `02-data-model.md` gets a
  "Mesh features" section beside "Inertials".
- **`riggen-core`** — `Command::MoveJointFrame { joint, origin, axis }`:
  rewrites the child frame and axis while re-expressing the child's geom
  poses and its child joints' origins so no world pose at `q = 0` changes
  (the pivot move; what "click the bore" and the joint gizmo commit).
  `fk::origin_for_world(robot, link, world) -> Pose`: the joint origin
  that puts `link` at `world` in the zero configuration (what the link
  gizmo and the align tool commit through one `SetJoint`). `02 §Commands`,
  `§Kinematics`.
- **`riggen-viewport`** — owns overlays, as `01 §Layer map` already says:
  an `Overlay` list of world-space primitives (segment, polyline, arc,
  point, label) drawn with egui's painter after the paint callback, plus
  `Viewport::project(DVec3) -> Option<Pos2>` and `cursor_ray(Pos2) -> Ray`
  (f64, from the inverse view-projection). Not depth-tested. The viewport
  never sees a `Joint`; the app builds the glyphs.
- **`riggen-app`** — `Tool { Select, Move, Rotate, PlaceJoint, Align }`
  with a toolbar floating in the viewport's top-left; `gizmo.rs`
  (transform-gizmo-egui behind a thin adapter, `mint` bridging glam 0.30 ↔
  the crate's glam), a `preview_world: Option<(LinkId, Pose)>` override
  that `sync_scene` applies during a drag; `glyphs.rs` (document → overlay
  items, hover hit-test in screen space); `snap.rs` (`SnapCandidate` from
  the hovered pick + `cursor_ray` + `ray_triangle`: hit point, vertex,
  AABB corner / face center, circle center with its fit, face normal;
  priority vertex > AABB > circle > point); `debug_state()` gains `tool`,
  `gizmo`, `glyphs`, `snap`. `01 §The document is the only state`,
  `§Panels and menus`, `§Frame loop`, `§Picking and snapping` (rewritten
  to the algorithm above), `§Testing`.
- **ADR-0007** (step 4): gizmo crate vs own, with the consequence that
  `transform-gizmo` pins `glam ^0.32` beside our 0.30 (two glam versions,
  bridged through `mint`; the wasm build check must stay green).

## Steps

- [x] Step 1 — `riggen-mesh`: `TriMesh::cylinder` / `tube`,
  `stl::write_binary`, `feature::{adjacency, grow_region, fit_circle}`.
  Tests: a cylinder wall at random poses fits its axis within 1e-6 and
  its center on the axis; a tube's inner wall too (inward normals); a
  cap loop gives axis = normal and center on the cap; a cube face fits
  nothing; jittered vertices report a matching `residual`; `segments`
  equals the generator's segment count.
- [x] Step 2 — `riggen-core`: `Command::MoveJointFrame` and
  `fk::origin_for_world`. Tests: `fk` at `q = 0` is identical before and
  after a frame move (geoms and grandchildren included); a zero axis is
  refused through `validate`; a no-op move adds no history entry;
  `origin_for_world` round-trips through `fk` for a 3-joint chain.
- [x] Step 3 — `riggen-app`: `Tool` enum, the floating toolbar (buttons by
  label: Select / Move / Rotate / Place joint / Align, `Esc` returns to
  Select), the zero-configuration rule (entering an editing tool with
  `q ≠ 0` resets the sliders with a status message, OPEN 1),
  `debug_state.tool`. Snapshot `toolbar`; every golden refreshed
  (`snapshots:` — the toolbar overlays the viewport corner).
- [ ] Step 4 — Gizmo spike and ADR-0007: `transform-gizmo-egui` 0.11 with
  glam's `mint` feature; link selected + Move/Rotate → the gizmo at the
  link frame, drag previews through `preview_world`, release commits one
  `SetJoint` via `origin_for_world`; joint selected → the gizmo at the
  joint frame commits one `MoveJointFrame` (OPEN 2). The gizmo wins
  the pointer while hovered (no select pick, no orbit). Snapshots
  `gizmo_move_link`, `gizmo_rotate_joint`; a `with_app` synthetic drag
  (press, `PointerMoved` steps, release, rendered between) asserts the
  part moved, exactly one history entry, undo restores; wasm check green.
  If the crate fights the ID buffer or the bridging is worse than a
  gizmo of our own, the ADR says so and the step builds the own one
  (OPEN 3).
- [ ] Step 5 — `riggen-viewport::Overlay`, `project`, `cursor_ray`; app
  joint glyphs: axis segment sized from the child's AABB (fallback: scene
  radius), origin triad, revolute limit arc with a tick at the current
  `q`, prismatic limit segment; drawn for movable joints and the selected
  joint (OPEN 4). `debug_state.glyphs` with screen positions. Snapshots
  `glyph_revolute`, `glyph_prismatic`; `pendulum`, `tree_pendulum`,
  `properties_joint`, `pendulum_swing` refreshed and shown.
- [ ] Step 6 — Hover both ways: a hovered tree row highlights its joint's
  glyph; a hovered glyph (screen distance to the axis segment) highlights
  the tree row and the status bar names the joint; clicking a glyph
  selects the joint. Snapshot `glyph_hover`.
- [ ] Step 7 — `snap.rs`: candidates from the hovered pick, memoised per
  `(instance, triangle)`; marker and readout in the overlay
  (`circle r 12.0 mm · 24 seg · res 0.01 mm` — a bad fit is obvious);
  `debug_state.snap`. Pure-function tests for the priority ladder and the
  pixel radius; snapshots `snap_vertex`, `snap_circle`.
- [ ] Step 8 — Place joint tool: joint selected + click a candidate →
  `MoveJointFrame` (circle: origin = center, axis = fit axis; face: axis
  = normal, origin = hit point; vertex / AABB / point: origin only).
  `with_app` test on a generated boss: axis within 0.5°, origin on the
  axis within 1 mm, one history entry. Snapshot `place_joint_bore`.
- [ ] Step 9 — Align tool: first click a feature on the selected link,
  second click a feature anywhere; point→point translates, circle→circle
  makes concentric (minimal rotation axis→axis, then center→center); one
  `SetJoint` on the link's parent joint via `origin_for_world`. Test on a
  tube offset from a boss; snapshot `align_concentric`. Backlog line:
  snapping during gizmo drags.
- [ ] Step 10 — Fixtures and the acceptance: `assets/fixtures/arm/{base,
  shoulder, upper, fore}.stl` in mm, written by an `#[ignore]` generator
  test (base with a Z boss, shoulder with a Y boss, upper bar with a Y
  boss, fore bar with a tube — exported offset by a known vector);
  `RiggenApp::project_world` for aiming clicks; the `five_minute_arm`
  scenario: import scale mm, drop-with-selection chain, kind combo ×3,
  Place joint ×3, Align ×1, a slider swing; asserts every joint axis
  within 0.5° and origin within 1 mm of the design line, history length
  = gesture count; PNG + JSON goldens. `visual-debug` skill updated
  (scenarios, `project_world`, synthetic drag).
- [ ] Step 11 — Exit gate: the human builds the arm by hand from the
  fixture folder, timed; every annoyance becomes a `docs/BACKLOG.md`
  line; roadmap status line; then `/retire-plan`.

## Acceptance

`cargo test -p riggen-app --test visual five_minute_arm` passes: the
scripted arm ends with three revolute joints whose axes are within 0.5° and
whose origins are within 1 mm (point-to-line) of the fixture's design
values, one history entry per gesture, and the pinned PNG/JSON. Plus the
by-hand run of step 11 reported and its list in the backlog.

## Docs to update on completion

- `docs/01-architecture.md` §Layer map — overlay primitives in the
  viewport; §The document is the only state — `Tool`, `preview_world`,
  the zero-configuration rule; §Panels and menus — toolbar, tools, hover
  both ways; §Frame loop — gizmo/snap/overlay order; §Picking and snapping
  — the implemented candidates and circle fit; §Testing — new scenarios
  and helpers (`project_world`, synthetic drag).
- `docs/02-data-model.md` §Commands — `MoveJointFrame`; §Kinematics —
  `origin_for_world`; new §Mesh features — `feature` module, `CircleFit`.
- `docs/03-roadmap.md` — M2 status line (date, tag `m2`, decisions).
- `docs/adr/0007-*.md` — written in step 4; listed in `adr/README.md`.
- `.agents/skills/visual-debug/SKILL.md` — scenario list, helper table.
- `docs/BACKLOG.md` — lines added by steps 9 and 11; the "vertex welding"
  line narrowed to `is_closed`.
- `AGENTS.md` current state — M2 done, next M3.

## Open questions

All four decided by the human on 2026-08-29, the recommendation in each
case; kept here so the steps that cite them read on their own.

- OPEN 1 (decided): editing tools work in the zero configuration —
  entering Move / Rotate / Place joint / Align with `q ≠ 0` resets the
  sliders with a status message. Commands do not carry `JointState`; the
  backlog line "`Reparent { keep_world_pose }` at the current `q`" covers
  the upgrade.
- OPEN 2 (decided): gizmo on a *link* moves the link (its parent joint
  origin; the subtree follows; one `SetJoint`); gizmo on a *joint* moves
  the pivot while the geometry stays (`MoveJointFrame`). Geom poses stay
  in the properties panel.
- OPEN 3 (decided in principle, agent confirms by step 4's spike):
  `transform-gizmo-egui` first; an own gizmo only if it fails pointer
  coexistence with the ID buffer, `mint` bridging, the wasm build, or
  snapshot determinism. ADR-0007 either way.
- OPEN 4 (decided): glyphs for movable joints plus the selected joint,
  not for unselected fixed joints.
