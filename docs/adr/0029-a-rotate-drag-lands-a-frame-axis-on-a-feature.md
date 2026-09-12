# ADR-0029: A rotate drag lands one of the dragged frame's own axes on the feature under the cursor

- Status: Accepted
- Date: 2026-09-12
- Amends:
  [ADR-0019](0019-the-wheel-is-claimable-and-a-drag-keeps-the-hover-pick.md)
  §5 ("only a translate drag snaps"), which deferred exactly this to "a
  later plan".
- Leaves intact: ADR-0019 §§1–4 (the five switches, the wheel's quantum,
  the view ring's exclusion, `set_camera_blocked`),
  [ADR-0012](0012-frames-as-mjcf-sites-and-urdf-dummy-links.md) (a click
  under Rotate still *places* a selected frame), ADR-0003 (the new marker
  gets a golden).

## Context

ADR-0019 §5 let a **translate** gizmo drag run the snap ladder: the
previewed pose's translation becomes the snapped point, the rotation is
left alone, and the release commits it. A **rotate** drag was left out with
a reason that was honest about being unfinished — "a rotation about a named
axis has nothing in the ladder to land on" — and the alternative it
rejected named the two missing pieces: a rule for *which* of the three axes
aligns, and a second overlay idiom to show it.

The gap is real work, not a nicety. Everything downstream of a part's
orientation is decided by a drag: a joint frame's axis is expressed in the
child link frame (AGENTS.md), so getting a bore's axis to lie along a frame
axis is the difference between a joint that hinges about the hole and one
that hinges about nothing in particular. The routes that exist are **Align**
(two clicks, two circles, but it moves the *link* and wants a second feature
to aim at) and the wheel's 5° notches (a quantum, not a target). Neither is
"turn this until it lines up with that hole".

What makes the rule findable is that a ring drag has **one** degree of
freedom. The drag is solved in the ring's own plane, about the ring's own
axis; only the two frame axes perpendicular to that ring can move at all,
and their four signed directions sweep that plane 90° apart. So the
question "which axis?" has an answer the user never types: the one the drag
has already brought nearest.

## Decision

**A rotate gizmo drag snaps, direction-only, by correcting the previewed
rotation until one frame axis lies exactly along the feature's.**

1. **The plane is the ring's.** The ring latched at drag start — not
   `hovered_ring`, which is gated on a handle being under the cursor and
   goes `None` the moment the drag leaves the band — gives a world axis
   `n = r · ring.local()`. Every part of the rule lives in the plane
   normal to `n`, which is the only plane the drag can move anything in.

2. **The candidates are the four signed directions of the two frame axes
   perpendicular to the ring.** An X-ring drag offers ±Y and ±Z of the
   frame being dragged; the ring's own axis is not a candidate, because a
   rotation about it cannot move it.

3. **The target is the feature axis projected into that plane**, normalised.
   A feature axis parallel to the ring axis projects to nothing and there is
   **no snap** — the drag is free, as it is over the background. That is the
   honest answer: no rotation about this ring brings any axis onto that
   direction.

4. **The winner is the nearest by angle**, and the correction is the signed
   rotation about `n` that takes it exactly onto the target. Four candidates
   90° apart mean the correction is never more than 45°, so the snap never
   spins the part somewhere the drag was not already heading.

5. **The ladder runs direction-only.** A vertex and a box corner say nothing
   about direction — the same reason Place joint leaves the axis alone for
   them (§Picking and snapping) — so while a rotate drag is in flight those
   two rungs are skipped and the ladder is **circle > point**: a fitted
   circle's axis, else the hit triangle's face normal. Both count. A flat
   face is the only feature a part with no bore offers, and the readout
   names which one was used, so a cardinal normal on a box that produces a
   no-op is legible rather than mysterious.

6. **Unconditional, not within a tolerance band.** While a feature with an
   axis is under the cursor the drag lands on it; free rotation means
   dragging over the background, over a vertex, or about a ring the feature
   is parallel to. This is the bargain a translate drag already makes — it
   snaps whenever the ladder finds anything at all, and the ladder's last
   rung always exists — and repeating it costs no new constant and no rule
   the marker has to explain.

7. **The view ring is still not claimed** (ADR-0019 §3's reason, unchanged):
   the document has no name for the camera's forward axis, so a drag about
   it has no frame axis worth reporting. A drag there behaves as it does
   today.

8. **The marker is a spoke plus an axis-prefixed readout.** The snap's own
   cyan, a segment from the gizmo's pivot along the target direction at the
   ring's own world radius, and the ladder's readout prefixed with the axis
   that is landing: `+z » circle r 12.0 mm · 24 seg · res 0.01 mm` — `»`
   and not an arrow, for the reason `glyphs.rs::driven_marks` already
   found: egui's bundled fonts have none, and a tofu box says nothing. The
   circle overlay and the point marker the ladder already draws stay where
   they are, on the feature; the spoke and the words are on the gizmo,
   which is where the user is looking while dragging one.

9. **Nothing else changes.** One drag is still one command, committed on
   release exactly as previewed. No document field, no schema change, no
   export. The pointer policy's five switches are untouched: a rotate drag
   simply joins `RiggenApp::snapping`, which is what sets
   `set_select_suppressed`, and `set_pick_excluded` keys off a **link** drag
   rather than a translate one, so a rotate drag also looks through the
   subtree it is turning.

## Consequences

- Over geometry a rotate drag reaches only four orientations per ring. That
  is the point, and it is the translate drag's bargain; the background is a
  centimetre away and the wheel's notches are unaffected.
- `Tool::snaps()` is still false for Rotate: the snap remains a *gesture*
  affordance, not a tool one, so merely hovering with Rotate active draws
  nothing.
- A rotate drag now runs the ID-buffer readback and a circle fit every frame
  it is in flight — the same cost ADR-0019 accepted for the translate drag,
  and the fit is memoised per `(instance, triangle)`.
- `debug_state().snap.align` reports the landed axis, the target direction
  and the correction in degrees, so a headless scenario asserts the rule
  rather than the pixels. It is omitted when absent, so every existing JSON
  golden is byte-identical.
- A `GizmoTarget::Joint` drag aligns the *frame*, not the joint's `axis`:
  the axis is expressed in that frame and rides along, so a joint whose axis
  is not a frame axis is not landed by this gesture. Place joint does that
  in one click and keeps doing it.
- Undo is unchanged: one drag, one history entry, the committed pose the one
  the preview showed.

## Alternatives considered

- **A fixed axis — always +Z.** Predictable, one line, and matches
  `place_frame`'s Rotate arm, which turns a frame's +Z onto the feature's.
  Rejected because on two of the three rings +Z cannot move at all: a Z-ring
  drag would snap to nothing, and the gesture would be dead on the ring the
  user is most likely to grab. Nearest-of-four is only predictable *because*
  the drag has already shown which one is coming.
- **A tolerance band — snap only within ~10° of an alignment.** Keeps free
  rotation everywhere, which is the real argument for it. Rejected as §6: it
  buys back a freedom the background already gives, at the price of a second
  constant, a marker that has to explain when it is and is not live, and a
  drag that feels different over two halves of the same part.
- **A fourth candidate: the joint's own `axis` for a
  `GizmoTarget::Joint`.** Tempting, since that axis is what the export
  cares about. Rejected because it is usually not a frame axis, the rule
  would then depend on what the gizmo is attached to, and landing the axis
  itself is what Place joint already does in one click.
- **Claim the view ring too.** Rejected for ADR-0019 §3's reason: the same
  gesture twice would give two different alignments, and the axis named in
  the readout would be one the document cannot express.
- **Colour the spoke by the axis that lands** (red/green/blue, the triad's).
  Rejected: cyan is the snap's colour everywhere else, and an axis-coloured
  spoke at the gizmo's radius is indistinguishable from the gizmo's own
  handles at a glance. The axis is named in words instead.
- **Quantise the angle instead — 5°/15° under a modifier.** A different
  feature, and the wheel over a ring is already riggen's rotation quantum
  (ADR-0019 §2). It stays the backlog line it is.
