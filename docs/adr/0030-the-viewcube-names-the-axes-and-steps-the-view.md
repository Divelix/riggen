# ADR-0030: The ViewCube names the axes and steps the view; the triad goes

- Status: Accepted
- Date: 2026-09-12
- Amends: [ADR-0028](0028-navigation-is-one-turntable-the-cube-aims-it-the-keys-move-its-pivot.md)
  §3 — "the bottom-left axes triad stays" is reversed, and the cube gains
  one output, `Step`.
- Leaves intact: ADR-0028 §1 (one turntable, no roll — which is why the
  roll arrows stay out), §2, §4 and §5;
  [ADR-0021](0021-two-modes-and-edit-is-the-zero-configuration.md) (the
  cube's rect still goes in `chrome_rects`, and zen still draws no chrome);
  [ADR-0003](0003-headless-visual-snapshots.md)

## Context

ADR-0028 §3 kept the bottom-left axes triad beside the new cube on the
ground that they answer different questions: the triad names the axes, the
cube names the faces. What it did not weigh is that **the triad carries no
letters**. Three unlabelled lines in red, green and blue name the axes
only to a user who already knows which colour is which — and a user who
knows that does not need the triad. It named nothing to anyone else.

The human, looking at Onshape (the reference named here, as ADR-0028 named
it for the corner): "XYZ axes are attached to view cube corner — do the
same for our ViewCube and remove bottom-left gizmo." Then, over an Onshape
screenshot: "this is how it looks in onshape — do the same. Also these
arrows around it we need to add too." And on the screenshot: "you can see
full green axis that is behind the cube so it is transparent."

The human's answers, verbatim (2026-09-12):

1. "Make axes opaque with letters on the ends, but cube itself 50%
   transparent, while letters on cube opaque."
2. "Remain cube the same size. Axes must just stick to cube corners and
   rotate with cube like it is part of it."
3. "hide all in zen: viewcube and axes."

And on the letters: "you turn off axis letter visibility completely when
it goes behind cube — onshape on screenshot does exactly that."

Onshape draws six arrows round its cube: four triangles and two curved
ones. The curved pair spins the view about the screen's centre, which is
roll.

## Decision

**The world axes ride on the cube's corner, with letters; the cube's fill
goes translucent so a hidden arm shows through it; four arrows step the
view 15°; the viewport's triad and its wgpu surface are removed.**

1. **Three arms on the (−X, −Y, −Z) corner.** They start `AXES_GAP` =
   0.12 cube units outside the bounding cube's corner along the
   (−1, −1, −1) diagonal, run along +X, +Y and +Z for `ARM_LENGTH` = 2.4
   cube units — 1.2 edges, so each clears the far corner — and are drawn
   1.5 pt wide, opaque, in `AXIS_COLORS`. They are projected through the
   cube's own `camera_basis` and `cube_scale`, so they turn with the cube
   as a part of it and cannot drift from the facets. At the home view the
   corner is the silhouette's lower-left vertex: `X` runs along the bottom,
   `Z` up the left side, `Y` into the screen through the cube. The arms
   are paint only: not clickable, and they change nothing about what a
   click on the cube selects.

2. **The cube is filled at 50 % alpha; its labels, the arms and the
   letters are opaque.** An arm is split into the runs the cube hides and
   the runs nothing hides — a point is hidden iff its projection falls in
   a front-facing facet and it lies on the cube's side of that facet's
   plane, which is exact because the cube is convex — and the hidden runs
   are painted *before* the facets, the open ones after. The same opaque
   colour either way: the fill is what dims a hidden run.

   Showing a hidden arm through the fill, rather than dimming it (ADR-0020's
   rule for the scene's overlay) or cutting it, is the human's answer 1 and
   what the screenshot shows: the arm that runs behind the cube is the one
   that says which way is into the screen, and a cut arm would say nothing.
   The back facets are still culled, so a mirrored `BACK` never shows
   through `FRONT`; what shows through is the scene and the arms.

3. **A letter is drawn past each arm's end, or not at all.** `X`, `Y`,
   `Z`, 11 pt, in the arm's colour, 3 pt past the tip along the arm's
   screen direction. **A letter whose arm end is behind the cube is not
   drawn** — the arm still is, through the fill. When an arm points at the
   eye and shortens below 12 pt its letter turns by angle towards the
   corner's direction from the cube's centre, so it never lands on the
   corner and never jumps. And **a letter covers a letter**: at six edge
   views two arms land on one screen line outside the silhouette, and the
   nearer letter hides the farther, as the cube hides one behind it.

4. **Four step arrows, 15°, yaw or pitch only.** Faint triangles on a ring
   28 pt outside the cube rect's inscribed circle, at 12, 3, 6 and 9
   o'clock — past the farthest any letter reaches, so no arm or letter
   touches one at any orientation. A click is a new output,
   `ViewCubeAction::Step`, and flies the camera with the `animate_to` it
   already has, from wherever it is. An arrow turns the view the way
   dragging the cube towards it does — `Right` is −15° of yaw, `Up` −15° of
   pitch — so arrow and drag cannot disagree. Pitch stops at the face
   views' ±90°: `Down` at the Top view is a no-op, not a flip. Arrows are
   hit before the home icon, the button and the facets.

5. **The roll arrows stay out.** Onshape's curved pair is roll, and §1 of
   ADR-0028 refuses roll for a Z-up document whose every named view is yaw
   and pitch. It is a backlog line.

6. **The block grows round a cube of unchanged size.** The cube is still
   92 pt with the same fit. The widget's clip and the rect it registers in
   `chrome_rects` become the union of the cube, the arms' reach at any
   orientation, the arrows and the projection button, which moves below
   the down arrow. The home icon moves to the top-left corner of the arms'
   reach: on the cube's own corner it sat on the `Z` arm. The block is
   placed so none of it is clipped by the viewport's edge: the cube moves
   inward; it does not shrink.

7. **The triad is removed, surface and all.** `AxesTriadMesh`,
   `ColorVertex`, `axes.wgsl`, the axes pipeline, its uniform, the corner
   viewport and `OrbitCamera::axes_gizmo_view_proj` go. The three colours
   get one home in the app, `AXIS_COLORS`, which the cube, the joint and
   frame glyphs and the gizmo's handles share — so red, green and blue go
   on meaning X, Y and Z everywhere, now named by the thing that labels
   them.

8. **Zen has no axes indicator.** The cube, its arms and its arrows are
   chrome and go together (human's answer 3). Before, the triad was drawn
   in zen; now nothing is.

## Consequences

- The axes are named in words, next to the faces, in the one widget a user
  already looks at to know which way they face. The bottom-left corner is
  empty.
- The viewport loses a pipeline, a shader, a uniform buffer and a
  sub-viewport draw, and the render pass has one thing fewer to order.
- The bottom-right block is larger — about 160 pt square with the button
  below — and every golden shows it, so this costs one whole-suite snapshot
  refresh, taken once, with the images shown to the human (ADR-0003).
- The translucent cube is fainter, most of all in the light theme, where a
  pale fill over a pale background is 50 % of little. Its labels stay
  opaque and are what read.
- **Zen loses its only axes indicator**, by the human's call.
- **At the exact Top and Bottom views the side arrows change yaw and the
  picture does not turn**: there `OrbitCamera::basis` stands Y in for Z as
  the up hint and ignores yaw. Onshape spins the top view there. Making it
  do so is a change to ADR-0028's camera, not to the cube, and is left
  open.
- A hidden letter pops out when its arm's end clears the cube, and a
  covered letter pops back when two arms separate by a few degrees. Both
  are binary, as the pivot cue is (ADR-0028 §4), so a golden asserts them
  by value.

## Alternatives considered

- **Keep the triad and add letters to it.** Names the axes, but in a
  different corner from the thing that names the faces, in a wgpu pass
  that cannot draw text — a second, painter-drawn layer in that corner for
  three letters. The cube already projects through the orientation and
  already paints text.
- **Dim a hidden arm run, or cut it at the cube.** Dimming is ADR-0020's
  rule for the scene overlay, and a translucent fill over an opaque arm is
  that dimming already, without a second colour. Cutting hides the one arm
  that names the direction into the screen. The screenshot shows neither.
- **Fade a hidden letter instead of dropping it.** Rejected by the human
  ("turn off axis letter visibility completely"), and a partial letter
  behind a translucent fill reads as a smudge on the face label under it.
- **Arms from the cube's centre**, a triad inside the cube. Every arm then
  starts behind the fill, the letters crowd the face labels, and it is not
  what the reference does.
- **A tighter arrow ring, accepting a letter under an arrow now and then.**
  The first tuning (18 pt) put `Z`'s letter on the `Up` arrow at the
  FrontTopRight iso — a corner a facet click lands on. A letter under an
  arrow is a label the user cannot read and a click target the user
  cannot see; the ring moved out and the arms came in instead.
- **Roll arrows with a roll-less fallback** (spin by stepping yaw at the
  poles only). A curved arrow that turns the view about the screen centre
  at one view and about world Z at every other is two gestures behind one
  icon.
