# ADR-0019: The wheel can be claimed by a rotate ring, and a gizmo drag keeps the hover pick

- Status: Accepted
- Date: 2026-09-03
- Amends: [ADR-0010](0010-gizmo-egui-glue-is-ours.md) §Decision 3,
  [ADR-0018](0018-left-drag-orbits-gizmo-claims-the-primary-drag.md)
  §Decision 3

## Context

Two of the three gestures v0.3 still owes the viewport
(`docs/ROADMAP.md` §v0.3, "The viewport answers the mouse") need the
pointer split more finely than the four switches can express.

**The wheel.** The rotate gizmo answers only a drag. A drag is the wrong
instrument for "turn this joint 30° about its own axis": it is solved
against the projection it started in, it has no quantum, and on a ring seen
nearly edge-on there is barely a handle to grab. The wheel has a quantum by
construction — one notch, one step — and the ring under the cursor already
names the axis. The Properties panel took the same shape one plan earlier:
Ctrl+wheel steps a number by a per-unit floor. But the viewport's wheel is
zoom, and zoom is the one camera gesture that works with no button at all.

**The drag snap.** The Align and Place joint tools snap the *click* to a
vertex, a box corner or a fitted circle's centre; a gizmo drag snaps to
nothing, so the one gesture that moves a part in the viewport is also the
one that cannot be told where to put it. The snap ladder is computed from
`Viewport::hovered()` — the ID buffer's hit under the cursor — and during a
gizmo drag there is no such hit, because ADR-0010 §3 blocks the pointer
outright while a drag is in flight.

That blocking rule was right for its reason and wrong in its reach. Its
reason is the camera: a drag is solved against the projection it started
in, so a wheel event mid-drag would make the part jump. Its reach is the
whole pointer, picks included — and the picks are exactly what the snap
needs. One flag was made to mean two things again, which is the mistake
ADR-0010 was itself written to correct.

## Decision

**The switch table grows a fifth row, and `pointer_blocked` narrows to the
camera.**

1. **`set_wheel_claimed(bool)`.** Turns off **zoom only**: the camera's
   drags, both picks, the standard-view keys and every other viewport
   shortcut stay live.

   | Switch | Turns off | Set while |
   |---|---|---|
   | `set_pick_suppressed` | both picks; the camera stays live | a gizmo handle or a joint glyph is under the cursor |
   | `set_select_suppressed` | the select pick; the hover keeps running | a placement tool is active, or a translate gizmo drag is in flight |
   | `set_camera_blocked` | camera input; the picks are not its business | the pointer is over the toolbar, or a gizmo drag is in flight |
   | `set_primary_drag_claimed` | `dragged_by(Primary)` only | a gizmo handle is under the cursor, or a drag is in flight |
   | `set_wheel_claimed` | zoom only | a **rotate ring** is under the cursor |

2. **The wheel over a rotate ring steps that ring.** 5° per notch, 1° with
   shift held, about the ring's own local axis, committed through the same
   path a drag commits. Notches within 0.4 s coalesce into one history
   entry, on the `WHEEL_BURST` rule the Properties scrubbers already use, so
   one burst is one undo (AGENTS.md: one gesture = one history entry).

   Bare wheel, not Ctrl+wheel. Ctrl is egui's zoom modifier and
   `raw_wheel_delta_y` drops events carrying it before the viewport sees
   them, so Ctrl+wheel over the viewport is not free — it is egui's UI
   scale. The cursor is on a ~15 px band the user aimed at; that is claim
   enough.

3. **Only the three local rings.** The crate draws a fourth ring around
   them, turning about the camera's forward axis; the wheel keeps zooming
   there. A step about an axis that changes whenever the camera moves is
   not one the user can predict or repeat, and the document has no name for
   it. Which ring the cursor is on is `app/gizmo.rs::ring_under_cursor`,
   which mirrors the crate's private geometry and is gated on the crate's
   own `pick_preview`, so it only ever says *which*, never *whether*.

4. **`set_pointer_blocked` becomes `set_camera_blocked`.** It stops the
   camera and nothing else. The toolbar, which needs both, sets it
   *and* `set_pick_suppressed` — no behaviour changes there. A gizmo drag
   sets it alone, so the hover pick keeps resolving under the cursor and
   the snap ladder can run against it. The select pick is off during the
   drag through `set_select_suppressed`, because the release is a drop, not
   a selection.

5. **Only a translate drag snaps.** A rotate drag is a rotation about a
   named axis; there is nothing in the snap ladder for it to land on, and
   inventing an alignment from a picked triangle would be a guess the user
   cannot see. The dragged link's own subtree is excluded from the hit, so
   a part cannot snap to itself.

## Consequences

- Over a ~15 px band while the Rotate tool has a target, the wheel no
  longer zooms. That is the price, and it is paid in the one place the user
  is deliberately pointing at a handle; moving the cursor a centimetre gets
  zoom back. The three other tools, and Rotate with nothing selected, are
  untouched.
- A gizmo drag now runs the ID-buffer readback every frame it is in flight,
  where before it ran none. That is one pick pass per frame, the same one
  hovering has always paid.
- `DebugState.input` reports the fifth switch beside the four, and stays
  absent from the JSON while all five are off, so goldens that suppress
  nothing are unchanged. `gizmo.hovered_ring` names the ring.
- The wheel is now a *committing* gesture, not only a viewing one. Undo
  after a burst rewinds the whole burst, which is the same bargain the
  Properties scrubbers made and the same one a drag makes.
- Touch is unaffected: there is no wheel, and pinch-zoom remains the
  backlog line ADR-0018 left it.

## Alternatives considered

- **Ctrl+wheel for the ring, keeping the bare wheel for zoom always.**
  Consistent with the Properties scrubbers, and free of the dead zoom band.
  Rejected because Ctrl+wheel over the viewport is not actually free: it is
  egui's zoom modifier, deliberately dropped by `raw_wheel_delta_y`, and
  claiming it back means intercepting a gesture the browser also spends on
  page zoom. The bare wheel over a handle the user is already pointing at
  is the more direct reading of "the viewport answers the mouse".
- **A sixth switch for the picks instead of narrowing `pointer_blocked`.**
  Would leave two flags whose meanings overlap — "the pointer is blocked"
  and "but not the picks" — which is how `set_input_suppressed` became the
  M2 exit gate's dead camera. Narrowing the existing one keeps every switch
  a single channel.
- **Let the snap run during a rotate drag too, aligning an axis to a
  feature.** A real gesture, and a bigger one: it needs a rule for which of
  the three axes aligns, and a second overlay idiom to show it. Left to a
  later plan rather than smuggled in beside the translate case.
- **Snap the drag to a grid or an angle increment as well.** The CAD
  idiom, and it wants a quantum the document has nowhere to keep. The
  wheel's own step (§2) is that quantum for rotation; a translation
  equivalent can be argued when someone asks for it.
- **Step the view ring too, for consistency under the cursor.** Rejected in
  §3: the same gesture twice would give two different rotations.
