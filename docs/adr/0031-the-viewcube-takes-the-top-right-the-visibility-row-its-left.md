# ADR-0031: The ViewCube takes the top-right; the visibility row moves to its left

- Status: Accepted
- Date: 2026-09-12
- Amends: [ADR-0028](0028-navigation-is-one-turntable-the-cube-aims-it-the-keys-move-its-pivot.md)
  §3 ("It sits **bottom-right**") and its rejected alternative "Top-right
  for the cube"; [ADR-0030](0030-the-viewcube-names-the-axes-and-steps-the-view.md)
  §6, only where it places the block.
- Leaves intact: everything else in both — the cube, its arms, letters,
  arrows, button and home icon, the block's layout and its registration in
  `chrome_rects`; [ADR-0021](0021-two-modes-and-edit-is-the-zero-configuration.md)
  (the row is still corner chrome, drawn in both modes and gone in zen)

## Context

ADR-0028 put the cube bottom-right. It weighed top-right — "the convention
in Fusion and SolidWorks" — and turned it down because the visibility row
held that corner (the one the Joints window vacated, ADR-0021) and moving
the row was a change to a settled corner. Bottom-right held only the
projection text the cube's button replaced, and is where Onshape puts its
cube.

With ADR-0030 landed and looked at, the human: "yes, it looks perfect. Now
move it to top-right (like in all CADs) corner and just shift visibility
buttons to the left of viewcube."

The objection ADR-0028 recorded was the row's claim on the corner, not a
reason the cube is better at the bottom. The human has now made the call
it deferred.

## Decision

1. **The ViewCube's block takes the top-right corner.** The block —
   the cube at its 92 pt, its arms and letters at any orientation, the
   arrows, and the projection button hanging below — is placed with its
   top-right 8 pt in from the viewport's top and right edges, the same
   margin every corner widget uses. Its internal layout is ADR-0030's,
   unchanged.

2. **The visibility row ends 8 pt short of the block**, top-aligned with
   it, laid out right to left as before. The row keeps its buttons, order,
   tooltips and behaviour; only its right edge moves.

3. **The bottom-right corner is empty**, as the bottom-left has been since
   ADR-0030.

## Consequences

- The cube is where a CAD user looks for it first, and both bottom corners
  are clear of chrome, so a robot framed low in the viewport is not hidden
  under a widget.
- The top edge now carries the most chrome: the mode control and, in Edit,
  the toolbar at the left; the row and the cube's block at the right. At
  the snapshot size both fit with room between them. A viewport narrow
  enough that the two sides meet — both side panels open on a small window
  — overlaps them, with no reflow; nothing handled that before either,
  but the right side is now about a block's width wider.
- The row's tooltips and the cube's arrows are now neighbours; the 8 pt
  gap keeps their hit rects apart.
- One whole-suite snapshot refresh: every golden shows the viewport's
  corners (ADR-0003).

## Alternatives considered

- **Keep the cube bottom-right** (ADR-0028 §3). The human reversed it.
- **The row below the cube's block**, in the right-hand column. The row is
  horizontal and wider than the block, so it would stick out past the
  block's left edge anyway, and it would sit halfway down the viewport's
  right side, in the scene.
- **The row above the cube**, pushing the block down. Puts the cube back
  off the corner the human asked for.
