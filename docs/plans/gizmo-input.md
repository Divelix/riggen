# Plan: gizmo-input

- Started: 2026-08-30
- Milestone: v0.2 (the M2 exit gate's first line)
- Idea (verbatim from the human): "The gizmo swallows **all** viewport
  pointer input, not just its own drag: with Move or Rotate active, zoom,
  pan, orbit and click-to-select stop working (two causes —
  `set_input_suppressed` is all-or-nothing, and
  `transform-gizmo-egui::interact` registers a click-sensing widget at the
  cursor *every* frame, which egui's hit test prefers over the viewport;
  reported three ways: dead camera, laggy-feeling zoom, clicks that only
  flicker the hover tint)"

## Goal

With Move or Rotate active, the viewport behaves exactly as it does with no
tool: orbit, pan and zoom-to-cursor work, hovering a part tints it, and a
click selects it — everywhere except on a gizmo handle, where the gizmo
takes the drag and nothing else does. The gizmo stops being an all-or-
nothing pointer sink: `app/gizmo.rs` owns the ~40 lines of egui glue that
`GizmoExt::interact` used to provide, hit-tests the handles itself with
`Gizmo::pick_preview` and registers an interaction widget only on the
frames it actually wants the pointer; the viewport's one
`input_suppressed` switch splits into *picking off* and *pointer taken by
something drawn over me*, so a hovered handle or joint glyph never freezes
the camera. `transform-gizmo-egui` stays (ADR-0007 holds); only its egui
adapter becomes ours, recorded as ADR-0010.

## Non-goals

- The gizmo crate itself. ADR-0007's decision — the crate over our own
  screen-space geometry, bridged through `mint` — is not re-opened; only
  its §Decision 1 (pointer coexistence) changes.
- A joint gizmo drag previewing nothing (`preview_world` covers a link
  drag only) — a separate backlog line, separate plan.
- Orbit on left-drag, keyboard shortcuts for the tools, the wheel turning
  a rotate gizmo, a depth-tested overlay, and the "this tool wants the
  other kind of selection" message. All stay in `docs/BACKLOG.md`.
- Snapping during a gizmo drag (the align/place split stays as M2 shipped
  it).

## Design deltas

- **`riggen-viewport::Viewport`** — `input_suppressed` (all-or-nothing)
  becomes two independent switches, with `select_suppressed` unchanged
  beside them:
  - `set_pick_suppressed(bool)` — no hover pick, no select pick; camera
    input still live. Set while a gizmo handle is under the cursor or a
    joint glyph is hovered.
  - `set_pointer_blocked(bool)` — the pointer belongs to something drawn
    over the viewport in the *same* egui layer (the toolbar): camera and
    picking both off.
  - Camera input keys on `Response::contains_pointer()` instead of
    `hovered()`. `contains_pointer` is a plain containment test over the
    hit-test's `close` set (egui `hit_test.rs`: layers covering the search
    area are filtered, widgets in the same layer are not), so a floating
    window still takes the wheel and a same-layer widget on top no longer
    does. `dragged_by(Middle)` stays as it is — step 3 makes the gizmo
    decline the drag instead.
- **`riggen-app::app::gizmo`** — no `GizmoExt`; a local `interact` over
  the core crate's `Gizmo::update` / `Gizmo::draw` / `Gizmo::pick_preview`.
  `GizmoState::captured` comes from our own hit test (`pick_preview ||
  drag.is_some()`), not from `is_focused()` after the fact.
- **`riggen-app::debug`** — a small `InputDebug { pick_suppressed,
  select_suppressed, pointer_blocked }` on `DebugState`, so a scenario can
  assert the policy and not only its visible effect. `GizmoDebug.captured`
  keeps its name and gains the new meaning.
- **`docs/01-architecture.md`** — §Frame loop diagram line
  (`set_input_suppressed` → the two switches), the §Gizmos paragraph
  ("turns the viewport's camera input and picking off wholesale" is no
  longer true), the joint-glyph sentence at §Panels, and the placement-tool
  paragraph at §Snapping.
- **ADR-0010** — *The gizmo's egui glue is ours; the pointer is shared per
  handle*, amending ADR-0007 (the README table gets "Accepted, amended by
  0010", as ADR-0002 already does for 0009).

## Steps

- [x] Step 1 — Own the egui glue. `app/gizmo.rs` stops calling
  `GizmoExt::interact` and does it itself: `update_config`, then
  `pick_preview(cursor)` gated on the viewport response containing the
  pointer and the cursor being outside `toolbar_rect`, then `ui.interact`
  on the 1×1 rect **only when** a handle is under the cursor or a drag is
  in flight, then `Gizmo::update { hovered: over_handle, drag_started,
  dragging }` and `Gizmo::draw` into the painter. `captured` becomes
  `over_handle || drag.is_some()`. New scenario
  `gizmo_leaves_the_pointer_alone`: Move active on a selected link, hover a
  part away from the handles → `selection.hovered` resolves and
  `gizmo.captured` is false; click there → the selection changes. The
  existing `gizmo_drag_*` scenarios still commit one command.
- [x] Step 2 — Split the viewport's switch. `set_input_suppressed` →
  `set_pick_suppressed` + `set_pointer_blocked`; camera input on
  `contains_pointer()`; the app sets `pick_suppressed` from
  `gizmo_state.captured || glyph_hover.is_some()` and `pointer_blocked`
  from the toolbar rect. `InputDebug` added. Harness gets `scroll_at`.
  Scenarios: `camera_works_while_the_gizmo_is_up` (wheel over the gizmo's
  own origin changes `camera.distance`) and
  `the_toolbar_does_not_zoom_the_camera`.
- [x] Step 3 — Non-primary buttons pass through. **The mechanism this step
  proposed does not work; see OPEN 5.** What landed instead: the gizmo's
  widget is registered with `Sense::click()` rather than
  `click_and_drag()`, so `hit_test` reports `click: gizmo, drag: viewport`
  and every button's drag lands on the viewport. Harness gets `middle_drag`
  (a real modifier through `ModifiersChanged`). Scenario
  `orbit_works_from_a_gizmo_handle`: middle-drag starting on the gizmo
  origin changes `camera.yaw_deg`/`pitch_deg` and commits no command;
  shift+middle-drag pans and does not orbit.
- [ ] Step 4 — A hovered glyph stops freezing the camera. `glyph_hover`
  feeds `pick_suppressed` only; the §Panels sentence in
  `docs/01-architecture.md` is corrected in the same commit. Extend
  `glyph_hover`: with the glyph hot, the wheel still zooms and no part is
  tinted.
- [ ] Step 5 — ADR-0010 and the docs sync. Written last, recording what
  step 1 proved rather than what we hoped: the pointer arrangement, why
  `pick_preview` is the hit test, the one-frame lag on the target
  transform (`config.update_for_targets` runs inside `update`, so
  `pick_preview` at the top of a frame reads last frame's target — the
  same lag `captured` always had), and the `close`-set reasoning behind
  `contains_pointer`. ADR-0007's row in `docs/adr/README.md` becomes
  "Accepted, amended by 0010".

## Acceptance

`cargo test -p riggen-app --test visual` green with the goldens unchanged,
plus one new scenario that is the complaint end to end:

`gizmo_shares_the_viewport` — arm loaded, a link selected, Move active; in
one run orbit (yaw changes), zoom-to-cursor (distance changes), click a
*different* part (the selection and therefore the gizmo target change),
then drag a handle (one command, the link moved). Every assertion in one
scenario, because the bug was that these stopped working *together*.

A `visual-debug` capture of the Move tool with the cursor on a part,
showing the hover tint under a drawn gizmo, goes to the human. Any golden
PNG that does change is shown before it is staged (AGENTS.md).

## Docs to update on completion

- `docs/01-architecture.md` §Frame loop, §Gizmos, §Panels (glyph hover),
  §Snapping — done inside steps 1–4, verified once at retirement.
- `docs/adr/README.md` — the 0007 row's status, the 0010 row (step 5).
- `docs/BACKLOG.md` — remove the "gizmo swallows all viewport pointer
  input" line from the M2 exit-gate section.
- `docs/03-roadmap.md` §M2 — the status paragraph's "the largest being that
  the gizmo swallows all viewport pointer input" gains "(fixed by
  plans/gizmo-input)".
- `AGENTS.md` current state — no change; this is not a milestone.

## Open questions

- ~~⚠ OPEN 1~~: **answered (human, 2026-08-30): the camera is frozen while
  a gizmo drag is in flight** — the drag is solved against the projection it
  started in, and a wheel event mid-drag would make the part jump.
  Everything else (a hovered handle, a hovered glyph) stays live. Step 2's
  policy must keep this: `pointer_blocked` (or an equivalent) while
  `gizmo_dragging()`.
- ~~⚠ OPEN 2~~: the handle dead zone needs no narrowing. `pick_preview`
  reports only the handles themselves, not the gizmo's bounding circle: at
  the pendulum's fitted view a point 140 px from the gizmo origin, with the
  translate handles 110 px long, is already clear of it (step 1's
  `gizmo_leaves_the_pointer_alone`). `GizmoMode::all_rotate()` /
  `all_translate()` stay as M2 set them.

- ⚠ OPEN 5 (new, agent, step 3 — **resolved in step 3**): "decline to
  register the widget while a non-primary button is down" cannot work, and
  the scenario proved it before the fix went in. egui computes the hit test
  in `begin_pass` against **`prev_pass.widgets`** (`context.rs`, the
  `interaction::interact` call), so the press frame is hit-tested against
  the frame *before* it — the frame on which the cursor was resting on the
  handle and the widget was very much registered. `potential_drag_id` is
  taken from that, and a widget missing from the current frame is
  explicitly tolerated ("this could be drag-and-drop … now in the air"), so
  the drag goes nowhere. The fix has to be in what the widget *senses*, not
  in whether it exists: `Sense::click()` keeps it out of `hits.drag`
  entirely. Step 5's ADR records this rather than the version above.

- ⚠ OPEN 4 (new, agent, step 5): step 1 cites `plans/gizmo-input` where
  ADR-0010 will be — `app/gizmo.rs` (module doc, `interact`), `app/mod.rs`,
  the new scenario, and `docs/01-architecture.md` §Gizmos. Step 5 swaps
  those four citations when the ADR exists.
- ~~⚠ OPEN 3~~: **answered (step 2): yes.** A floating window is a layer of
  its own and egui's hit test drops the layers under one that covers the
  search area, so `contains_pointer()` is already false over the Joints
  window and no rect check is needed for it — asserted in
  `the_toolbar_does_not_zoom_the_camera`, which checks that the camera stays
  put there while `pointer_blocked` is *false*. Only the toolbar, drawn in
  the viewport's own layer, needs `set_pointer_blocked`.
