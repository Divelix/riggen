# Idea: viewcube-and-fly-camera

- Status: Open
- Raised: 2026-09-11
- Prompt (verbatim from the human): "next thing on roadmap"

The next thing on the roadmap is v0.5 §the viewport and the camera, whose
glyph line landed yesterday (ADR-0027). Four lines are left, and they are
two clusters, not one: **the camera** (ViewCube, fly keys, the orbit
pivot) and **the pass under it** (ground grid, MSAA — which share the
offscreen colour/depth pair and collide with ADR-0020's depth readback).
The rotate-drag snap is a third, and belongs to the gizmo, not the camera.
This idea is the camera cluster only; the pass is its own idea, and the
camera does not depend on it.

## Problem

The camera is M0's, and M0's is a turntable: yaw, pitch, distance, a
target (`camera/orbit.rs`). Two things follow.

**You do not always know which way you are looking.** The bottom-left
axes triad names the world axes and nothing else; the only way to reach a
named view is the numpad (`Num1/3/7/0`, `Ctrl` for the opposite face,
`viewport/mod.rs::handle_input`), which is folklore on a laptop with no
numpad and invisible on the web demo — the first riggen most people meet
(ADR-0017). `ViewOrientation` has carried all 26 canonical orientations
since M0 with a comment saying "kept whole for the ViewCube port later";
nothing calls the other 20.

**You can orbit round an assembly but not get into it.** Every gesture is
about one pivot: orbit turns round it, pan slides it, zoom approaches it
(`zoom_to_cursor` moves it under the cursor). A joint inside a closed
shell — a gripper between fingers, a hip inside a shroud — cannot be put
in front of the camera at all, which in View is exactly where the wheel
and the band want it (ADR-0021 §1, ADR-0027). `W A S D E Q` have been
reserved off the tool keys since M2 (01 §The document is the only state)
against this day.

## Constraints it runs into

- **`SEED.md` §non-goals** says nothing about cameras; the web build "stays
  a CI build check" as a *product*, but it is still the demo, so anything
  keyboard-only is half-delivered there.
- **ADR-0018** fixed the drag table: bare left orbits, right pans, shift
  makes either a pan, and a gizmo handle may claim the primary drag. Any
  camera work that wants a drag is amending it.
- **ADR-0021** makes the viewport's corners chrome: `View | Edit` and the
  toolbar top-left, the visibility row top-right, both recorded in
  `chrome_rects` so the camera holds still and the picks are off under
  them — and in zen the list is *cleared*. A new corner widget joins that
  list or it steals drags from the camera.
- **ADR-0003**: every visible change gets a golden, and ADR-0021's zen
  amendment refused a timed cue ("never a timed toast") because a golden
  would carry a clock.
- **The layer rule**: the viewport owns the camera and the projection, the
  app owns selection, chrome and document overlays. A ViewCube is
  interactive chrome that *writes* the camera — app side. The orbit pivot
  is camera feedback that no document knows about — viewport side.
- **Bare letter keys yield to a focused `TextEdit`** (`shortcuts.rs`), and
  `W`, `A`, `S`, `D` are letters: an inline link rename must not fly the
  camera.
- RoboCAD (the named ancestor) has a ViewCube: `robocad-ui/src/viewcube/`,
  973 lines plus 575 of tests, painted with egui over `ViewOrientation`
  and `Projection` — the two enums riggen already has, with the same
  variants.

## Options

The two lines split into three decisions: what the camera *is*, what the
cube *is*, and whether the pivot is drawn. The camera one is first because
the other two lean on it.

### A — Keep the turntable; flying translates its pivot *(the camera)*

`W A S D E Q` move `target` along the camera basis each frame at a speed
proportional to `distance`, with `Shift` fast and `Ctrl` slow; `yaw`,
`pitch` and `distance` are untouched, so the eye follows rigidly and the
pivot arrives inside the assembly with you — after which the same
left-drag orbits *locally*, which is the thing that was missing. This is
rerun's own answer (`re_view_spatial/src/eye.rs::handle_keyboard_navigation`
moves `pos` and `look_target` by the same delta, with an exponential
velocity smooth).

Nothing downstream changes: `view_proj`, `cursor_ray`, picking, the
overlay's projection, `frame_bounds`, the animations, `debug_state().camera`
and `camera/tests.rs`'s 520 lines all speak yaw/pitch/distance/target and
keep doing so. New state is a velocity and a speed factor.

Cost: **1 step**, most of it the input gating (over the viewport, not
while a text field has focus, live in both modes and in zen, `key_down`
not `key_pressed`, `request_repaint` while moving).

Forecloses: roll, and mouse-look-in-place. Neither is asked for, and roll
is wrong for a Z-up document.

### B — A second camera kind, `Orbit | FirstPerson` *(the camera)*

rerun's `Eye3DKind`. In first person the wheel dollies instead of zooming,
the drag turns the head, and the pivot is one unit ahead. A truer "fly
camera".

Cost: **2–3 steps** and an ADR that amends ADR-0018's drag table. Every
consumer must ask which kind is live; `StandardView`, `Home` and
`frame_bounds` each need an answer for first person; the mode needs to be
visible somewhere, which is a fourth thing in a corner. Forecloses
nothing, but it buys a second mental model for a window that just got its
*first* two modes (ADR-0021) — and the thing the researcher wants is to
put a joint in front of the camera, which A already does.

### C — A free camera, eye plus quaternion *(the camera)*

Replaces the turntable outright. Rejected on sight: `MAX_PITCH`,
`StandardView`, `ViewOrientation::from_direction`, `animate_to(yaw, pitch)`
and every camera test are yaw/pitch, the ViewCube's hit test wants yaw and
pitch too, and roll is a non-feature here.

### D — Port RoboCAD's ViewCube *(the cube)*

`facets.rs` (chamfered cube → 26 facets, 311 lines), `projection.rs`
(project, depth-sort, backface-cull, hit-test, 278) and `widget.rs` (the
egui widget returning `Select(ViewOrientation)` / `Orbit{dy,dp}` / `Home`
/ `ToggleProjection`, 384), with its 575 lines of tests. The port is
cgmath → glam and `robocad_viewport::{Projection, ViewOrientation}` →
`riggen_viewport::{…}`, which are the same enums. It lands in
`riggen-app` as `app/viewcube/` — painter-drawn, so no wgpu pass and no
shader — and registers in `chrome_rects` like every other corner widget.
Its facets are exactly the 26 orientations `ViewOrientation` already
holds, `animate_to_orientation` already exists to fly to one, and its
`ToggleProjection` button subsumes the `persp` / `ortho` text in the
bottom-right corner, which is what the roadmap asked for.

Cost: **2 steps** (the math with its tests; the widget wired to the
camera, with goldens).

### E — A flat six-ball axis gizmo instead *(the cube)*

Blender's: six labelled balls on the world axes, click one to snap.
~150 lines, no port, reads at a smaller size. But it answers "which way is
+X", which the triad already answers, rather than "which face am I on";
it reaches 6 orientations where the cube reaches 26; and it throws away a
tested port from the codebase AGENTS.md names as the ancestor.

### F — Draw the cube as a second wgpu pass *(the cube)*

The axes triad already is one, so the machinery exists. Rejected: picking
a facet needs a screen-space hit test regardless, the overlay is egui's,
the widget must live where `chrome_rects` and the pointer are (app), and a
third pass is one more thing MSAA would have to be taught later.

### G — Draw the orbit pivot while the camera moves *(the pivot)*

A small cross at `target`, sized as a fraction of `distance`, pushed by
the **viewport** into its own overlay (it is camera feedback; no document
knows about it) and drawn unconditionally on top, the class ADR-0020 §The
overlay gives cursor feedback. Visible exactly while a camera gesture is
live — an orbit, a pan, a fly key held, a camera animation — and gone the
frame it ends, with **no fade**, so no golden carries a clock (ADR-0021's
amendment). rerun fades its one; the fade is what would cost us the test.

This is the half of the fly line that is not a new camera at all: the
turntable has always had an invisible pivot, and "why is it turning round
*there*" is a question the window cannot answer today.

Cost: **1 step**.

### Do nothing

The numpad stays the only way to a named view on a machine that may not
have a numpad, the demo stays orbit-only, the 20 unused `ViewOrientation`
variants stay unused, and a joint inside a shell stays unreachable in the
mode built for posing it. v0.5's goal sentence is exactly these two
complaints, and its acceptance test ("navigated with the ViewCube and the
fly keys and never with the numpad") cannot be run.

## Recommendation

**A + D + G**, as one plan of five steps: fly keys, pivot cue, cube math,
cube widget, docs.

A wins because the whole of the fly line's *value* — a joint in front of
the camera, then orbit it locally — falls out of moving one `Vec3`, and B
buys a second camera model with a mode indicator for a window that just
learned its first two modes. What would change my mind: a by-hand run
where flying with the pivot locked to the eye feels like pushing the scene
rather than walking into it — the fix would then be B, and A is not wasted
work, because B's first-person kind is A's translation with the drag
rebound.

D wins because the port is 970 tested lines against ~150 written ones for
a widget that answers a narrower question, and because the 26 orientations
it needs have been sitting in `orientation.rs` since M0 waiting for it.

G is separable from both and is the cheapest visible thing in v0.5.

Two sub-decisions inside D, both cheap to reverse:

- **Which corner.** Top-right is the convention (Fusion, SolidWorks,
  RoboCAD) and is taken by the visibility row; bottom-right holds only the
  `persp` / `ortho` text the cube's own button replaces, and is where
  Onshape puts its cube. Preferred: **bottom-right**, nothing moves.
- **Does the axes triad stay?** RoboCAD keeps both — the triad names the
  axes in the gizmo's own colours, the cube names the faces and is
  clickable. Preferred: **keep it**, at the bottom-left where it is, which
  also keeps the change inside the app crate.

## Decision for the human

1. **The camera stays one turntable, and `W A S D E Q` translate its
   pivot** (A) — rather than a second first-person camera kind (B)?
   *Preferred: yes.*
2. **The ViewCube is RoboCAD's, ported into `riggen-app` as an egui
   painter widget** (D) — rather than a smaller axis gizmo (E)?
   *Preferred: yes.*
3. **Bottom-right corner, axes triad kept**, the cube's button taking over
   the `persp` / `ortho` label? *Preferred: yes; top-right with the
   visibility row pushed down is the alternative.*
4. **The orbit pivot is drawn while the camera moves, without a fade**
   (G)? *Preferred: yes.*
5. Is this **one plan** (five steps: fly, pivot, cube math, cube widget,
   docs) or two (`fly-camera` and `viewcube`)? *Preferred: one — the
   acceptance test names both, and `docs/plans/` is empty.*

**An ADR is needed** if 1 is answered yes: "navigation is one turntable —
the cube aims it, the keys move its pivot", which settles the recurring
first-person question, records that the cube is app-side chrome and the
pivot is viewport-side feedback, and states the no-fade rule against
ADR-0003. It amends nothing: ADR-0018's drag table is untouched, because
the cube's drag is on the cube's own rect.
