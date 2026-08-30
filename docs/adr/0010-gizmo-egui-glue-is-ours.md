# ADR-0010: The gizmo's egui glue is ours; the pointer is shared per handle

- Status: Accepted
- Date: 2026-08-30
- Amends: [ADR-0007](0007-transform-gizmo-crate-over-our-own.md) §Decision 1

## Context

ADR-0007 took `transform-gizmo-egui` and, for the pointer, took the crate's
`GizmoExt::interact` with it: register the gizmo's widget after the
viewport's rect in the same layer, and let `Viewport::set_input_suppressed`
turn the viewport off while `Gizmo::is_focused()`. Both halves were wrong in
the same direction, and the M2 exit gate collected the result three ways —
a dead camera, a zoom that felt laggy, clicks that only flickered the hover
tint. With Move or Rotate active the viewport stopped being a viewport.

Two causes, independent of each other:

1. `GizmoExt::interact` registers a 1×1 click-and-drag widget **at the
   cursor, every frame a gizmo is on screen**, whether or not a handle is
   anywhere near. egui's hit test prefers the widget registered last, so
   that widget took the hover, the click and — because the viewport's
   camera input keyed on `Response::hovered()` — the wheel, from a viewport
   whose geometry was in plain sight underneath.
2. `set_input_suppressed` was one switch for two different situations. "A
   handle is under the cursor, so do not pick the part behind it" and "the
   pointer belongs to something else entirely" are not the same policy, and
   collapsing them meant a hovered handle — or a hovered joint glyph, which
   used the same switch — froze the camera.

The crate itself was never the problem: `Gizmo::update`, `Gizmo::draw` and
`Gizmo::pick_preview` are all public, and they are the whole gizmo. Only its
egui adapter, forty lines of `ui.interact` and mesh conversion, decides who
gets the pointer.

## Decision

**ADR-0007 stands; its §Decision 1 does not.** The crate stays, at the same
version, behind the same one file. `app/gizmo.rs` provides its own
`interact` in place of `GizmoExt::interact`, and the viewport's one
suppression switch becomes three.

1. **Our own hit test, before any widget exists.** `Gizmo::pick_preview`
   asks the subgizmos directly whether the cursor is on a handle. It needs
   no widget and no `Response`, which is what makes it usable *before*
   deciding whether to register anything. `over_handle = viewport
   contains_pointer && cursor outside the toolbar && pick_preview(cursor)`,
   and `GizmoState::captured` is `over_handle || a drag is in flight` —
   never `is_focused()` after the fact.

2. **The widget is registered only on the frames the gizmo wants the
   pointer**, and it senses **clicks only**. `over_handle` (a handle is
   under the cursor) or a drag already in flight (the cursor by then
   anywhere); every other frame the viewport keeps the pointer it always
   had. `Sense::click()` rather than `click_and_drag()` because the widget
   exists for exactly one purpose — deny the viewport the *click* under a
   handle — and sensing drags as well takes every button's drag with it:
   `hit_test` fills `hits.drag` from the widgets that sense a drag, and
   `interaction.rs` sets `potential_drag_id` from it on a press of **any**
   button, so a middle-drag from a handle landed on a widget that does not
   orbit. Click-only, the hit test reports `click: gizmo, drag: viewport`,
   which is the split this needs. The gizmo reads the raw pointer state and
   never looks at this response at all.

3. **Three switches on the viewport**, because "the pointer is busy" has
   three meanings:

   | Switch | Turns off | Set while |
   |---|---|---|
   | `set_pick_suppressed` | both picks; the camera stays live | a gizmo handle or a joint glyph is under the cursor |
   | `set_select_suppressed` | the select pick; the hover keeps running | a placement tool is active (the click means "put it here") |
   | `set_pointer_blocked` | camera **and** picks | the pointer is over the toolbar, or a gizmo drag is in flight |

   A gizmo drag blocks the pointer outright because the drag is solved
   against the projection it started in: a wheel event mid-drag would make
   the part jump. A handle merely *hidden* over the geometry does not.

4. **Camera input keys on `Response::contains_pointer()`, not
   `hovered()`.** `hovered` is false whenever any later widget in the same
   layer took the hit — which is precisely the gizmo's own widget.
   `contains_pointer` is a containment test over the hit test's `close`
   set, and that set is filtered by *layer*: `hit_test.rs` walks the hits
   top-down and keeps only the layers above the first one that covers the
   whole search area. So a floating window (Joints, Materials) still stops
   it, while a same-layer widget drawn on top does not — which is why the
   toolbar, which floats in the viewport's own layer, needs
   `set_pointer_blocked` from its remembered rect and a window needs
   nothing.

The registration order from ADR-0007 is unchanged: viewport < gizmo <
toolbar.

## Consequences

- With Move or Rotate active the viewport behaves exactly as it does with
  no tool — orbit, pan, zoom-to-cursor, hover tint, click-to-select —
  everywhere except on a handle, where the gizmo takes the drag and nothing
  else does.
- Two one-frame lags, both benign and both pre-existing in kind. The
  switches are set before the viewport runs and the gizmo runs after it, so
  they carry last frame's answer; and `config.update_for_targets` runs
  inside `Gizmo::update`, so a `pick_preview` at the top of a frame
  hit-tests against last frame's target transform. The frame after the
  selection moves, the gizmo hit test aims at where the gizmo was. This is
  the same lag egui's own interaction has, and is not perceptible.
- ~40 lines of egui glue are ours to keep working across
  `transform-gizmo-egui` upgrades. They are ordinary egui, they live in the
  one file that already names the crate, and they replace behaviour we
  could not otherwise change.
- `set_pick_suppressed` is a general facility, as `set_input_suppressed`
  was: the joint glyphs use it, and any future overlay drawn in front of
  the geometry will.
- `DebugState.input` reports the three switches, so a scenario asserts the
  policy rather than the tint it happens to produce. It is omitted from the
  JSON while all three are off, so the goldens that suppress nothing are
  unchanged.

## Alternatives considered

- **Keep `GizmoExt::interact` and widen the suppression rules around it.**
  There is nothing to widen: the crate's adapter registers its widget
  unconditionally, so every rule would have to undo it from outside, and
  the viewport can not tell "the gizmo wants this pointer" from "the gizmo
  happens to be on screen".
- **Have the gizmo decline to register its widget while a non-primary
  button is down.** The obvious fix for the middle-drag, and it does not
  work — the scenario proved it before the real fix went in. egui computes
  the hit test in `begin_pass` against **`prev_pass.widgets`**, so the
  press frame is tested against the frame before it: the frame on which the
  cursor was resting on the handle and the widget was very much registered.
  `potential_drag_id` is taken from that, and a widget missing from the
  current frame is explicitly tolerated as drag-and-drop "in the air", so
  the drag simply goes nowhere. The fix has to be in what the widget
  senses, not in whether it exists.
- **Fork or patch `transform-gizmo-egui` to fix its adapter upstream.** The
  right long-term answer if the crate agrees, but it is a dependency we
  chose precisely so as not to maintain it. Our `interact` is smaller than
  the fork's diff would be and needs no coordination.
- **Drop the widget entirely and route the click through the viewport.**
  The viewport would then have to know what a gizmo handle is, which is the
  coupling ADR-0007's one-file adapter exists to prevent.
