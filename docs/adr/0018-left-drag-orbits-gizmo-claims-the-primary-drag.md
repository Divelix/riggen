# ADR-0018: The bare left-drag belongs to the camera; the gizmo claims the primary drag

- Status: Accepted
- Date: 2026-09-03
- Amends: [ADR-0010](0010-gizmo-egui-glue-is-ours.md) §Decision 3

## Context

The camera orbits on the **middle** button only. A left-drag over the
viewport does nothing at all, and three groups walk into that:

- **Trackpads have no middle button.** On the web demo — since ADR-0017 the
  first riggen most people meet — a visitor can zoom the sample arm and
  select its parts but cannot turn it. `README.md` says "orbit with the
  middle mouse button", and the demo page is not the README.
- **MuJoCo's `simulate` orbits on left-drag and pans on right-drag**, as do
  rerun, three.js `OrbitControls`, Sketchfab and every browser viewer this
  audience has met. Riggen's layout — a tree beside a viewer — reads as one
  of those, so the first gesture is a left-drag.
- Every by-hand run since M2 listed it; `docs/ROADMAP.md` opens v0.3
  with it.

It was never a one-liner because the left button is already spoken for
three times over: it *selects* (`response.clicked()` → the select pick), it
*places* (a snapping tool's click), and it *grabs a handle* (ADR-0010). The
rule has to say who owns a left press before anyone knows whether that press
will become a drag.

Two facts decide most of it. First, **egui already arbitrates click versus
drag**: `clicked()` fires on release only if the pointer stayed within
`InputOptions::max_click_dist` and under `max_click_duration`, and
`dragged_by(Primary)` fires once either is exceeded. So click-to-select
survives a left-drag orbit with no code of ours — a still press selects, a
moved press orbits — and the two constants are worth pinning in a test
rather than trusting.

Second, **ADR-0010 §Decision 2 deliberately let drags fall through to the
viewport**: the gizmo's widget senses clicks only, so egui's hit test
reports `click: gizmo, drag: viewport` and a drag from a handle orbits.
That was right while "drag" meant *middle*-drag. Once the primary drag
orbits, the same fall-through would orbit the camera *and* move the part,
because the gizmo reads the raw pointer state and never looks at its
widget. None of ADR-0010's three switches says what is needed here:
`set_pick_suppressed` and `set_select_suppressed` speak about picks, and
`set_pointer_blocked` takes the whole pointer — including the middle drag
and the wheel, which the gizmo has no claim on while it is merely hovered.

## Decision

**Left-drag orbits, and a gizmo handle claims that drag and nothing else.**

1. **The bindings.** Added to today's, not replacing them:

   | Gesture | Does |
   |---|---|
   | left-drag | orbit |
   | shift+left-drag | pan |
   | right-drag | pan |
   | middle-drag | orbit (unchanged) |
   | shift+middle-drag | pan (unchanged) |
   | wheel | zoom to cursor (unchanged) |

   `shift+left` pans because it is the only pan a one-button trackpad has,
   and shift already means pan on the middle button. Right-click has no
   meaning in the viewport today, so right-drag pan conflicts with nothing;
   a future viewport context menu would have to be click-only, which is how
   egui context menus already behave.

2. **Click-to-select is egui's click threshold, not a rule of ours.** No
   code decides between select and orbit; `max_click_dist` and
   `max_click_duration` do, and a test reads both off the context, asserts
   their values and drives one gesture under the threshold and one well
   past it. A press that jitters seven points orbits without selecting —
   the same trade every web viewer makes.

3. **A fourth switch: `set_primary_drag_claimed`.** ADR-0010's table gains
   a row:

   | Switch | Turns off | Set while |
   |---|---|---|
   | `set_primary_drag_claimed` | `dragged_by(Primary)` **only** — the middle and right drags, the wheel and both picks stay live | a gizmo handle is under the cursor, or a gizmo drag is in flight (`gizmo_captured()`) |

   `riggen-app` sets it from the same `over_handle || drag in flight` the
   gizmo already computes for `set_pick_suppressed`, next to the three it
   sets today. The layer rule of ADR-0010 holds unchanged: the viewport
   decides camera input from its `Response` and a switch, and still knows
   nothing about gizmos.

4. **Only the gizmo may withhold the left drag.** Pick suppression must not:
   a joint or frame glyph sets `set_pick_suppressed` so the click reaches
   the glyph rather than the geometry behind it, and a drag from a glyph
   orbits like a drag from anywhere else. Nothing under the cursor withholds
   the middle or right drag.

ADR-0010 §Decision 2 is **not** disturbed. The gizmo's widget still senses
clicks only; `orbit_works_from_a_gizmo_handle` still passes as written,
because it drags with the middle button, which no switch here touches.

## Consequences

- The trackpad visitor to the demo can turn the sample arm with the only
  button they have, and everyone arriving from MuJoCo, rerun or three.js
  finds the gesture they reached for. Nobody who orbits with the middle
  button loses anything.
- **Box select is foreclosed.** Left-drag on empty space is the CAD idiom
  for it (Onshape, Fusion, Blender), and the camera now has it; `shift+left`
  is spent on pan, so a later box select needs a third modifier or must
  overturn this ADR. Riggen has single selection only and neither `SEED.md`
  nor the roadmap asks for more, which is what makes the trade payable.
- **Touch orbit falls out for free.** A one-finger drag reaches egui as a
  primary drag, so the sample arm turns under a finger. Pinch-zoom and
  two-finger pan remain backlog lines and nothing here is tested on touch.
- The one-frame lag ADR-0010 already carries now also gates a drag rather
  than only a pick. It is benign for the same reason: the switch is set from
  the previous frame's `pick_preview`, and a human's cursor is resting on
  the handle before the press. A scripted test that teleports the cursor and
  presses in the same frame would see the lag; the harness drives drags in
  rendered steps, so it does not.
- A right-drag over the viewport must not raise the browser's context menu
  on the web. eframe's web backend `preventDefault`s `contextmenu` on the
  canvas, so it should not — confirmed by hand on the built demo. If it
  ever does, right-drag pan is dropped and `shift+left` carries the pan
  alone; the desktop is unaffected either way.
- `DebugState.input` reports the fourth switch beside the three, and stays
  absent from the JSON while all four are off, so goldens that suppress
  nothing are unchanged.

## Alternatives considered

- **Left-drag orbits only over empty space; a drag that starts on geometry
  is reserved (for box select later).** The decision would depend on the
  hover pick, which is an asynchronous ID-buffer readback and one frame
  late, so the same gesture would orbit or not depending on when the
  readback landed — exactly the inconsistency v0.3 exists to remove. And the
  demo visitor's first drag is *on* the arm, not beside it.
- **Modifier orbit: alt+left-drag (Maya, Fusion's "emulate 3-button
  mouse").** Zero conflicts with select, place or the gizmo, and one step of
  work — but undiscoverable, which is the problem restated: the trackpad
  visitor still cannot turn the arm until they read something. Alt is also
  the window-manager or browser menu key on some platforms. This is the
  answer to revisit if multi-select is ever adopted.
- **A preference, "left-drag orbits", default on (Blender's "emulate 3
  button mouse").** Riggen persists no preferences at all; a settings
  surface for one boolean is a new kind of thing, and the web build has
  nowhere to keep it. Loses until a second preference exists.
- **Let the gizmo's widget sense drags again, so the handle takes the left
  drag through egui's hit test.** This is the fix ADR-0010 §Decision 2
  explicitly removed: egui fills `hits.drag` from widgets that sense a drag
  and sets `potential_drag_id` from it on a press of **any** button, so
  sensing the primary drag takes the middle one with it and the handle
  freezes the orbit again. The claim has to be a switch the viewport reads,
  not a widget that swallows every button.
- **Reuse `set_pointer_blocked` for a hovered handle.** It turns off the
  wheel and both picks as well, so zoom and the hover tint would die
  wherever a handle happens to sit over the geometry — the M2 regression
  ADR-0010 was written to end.
