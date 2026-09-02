# Plan: panels-and-numbers

- Started: 2026-09-02
- Milestone: v0.3 — the hand-feel debt (`docs/03-roadmap.md` §v0.3, bullets
  "Numbers are editable", "The panels stop hiding things", "The tree says
  what a drag will do")
- Idea: none — three roadmap bullets that need no decision beyond the open
  questions below. The viewport bullets ("answers the mouse", "the overlay
  tells the truth") are the sibling plan, written once
  `docs/ideas/orbit-left-drag.md` is decided.

## Goal

Every number in Properties is a scrubber: drag it, step it with the wheel,
or click and type; one drag is one history entry, and a small value such as
`2.86e-5` reads and edits as written instead of as `0.000029`. The panels
say what they are doing: the Joints window opens itself when a document has
a movable joint, a tool that needs a different selection says so in the
status bar instead of ignoring the click, clicking empty space clears a
joint or frame selection, a `Meshes` collision policy is editable geom by
geom, and a material can be renamed. A tree drag shows a ghost row and a
grab cursor, and dropping a link keeps it where the user sees it — at the
current `q`, not at the zero configuration.

## Non-goals

- Anything in the viewport: the orbit rule, tool shortcuts, the rotate
  gizmo on the wheel, snapping during a gizmo drag, the depth-tested
  overlay, driven/actuated badges — the sibling plan.
- Multi-select, a preferences system, touch, a narrow layout (backlog).
- Sliders in the Joints window: they already scrub.
- Validation of an Override tensor beyond what `SetInertial` refuses today.
- Any new format, importer, writer or SDK feature beyond the two commands
  the panels need (`RenameMaterial`, `Reparent` at `q`).

## Design deltas

- **`riggen-core::History`** gains a *gesture*: `apply_in_gesture(robot,
  command, gesture: GestureId)` applies the command now and coalesces every
  apply under the same id into one undo entry (the first apply records the
  before-state; later ones only advance the after-state). Release ends the
  gesture. This is how "drags preview; release commits" is kept for a
  scrubber whose preview *is* the document — one gesture, one history entry
  — without a per-field preview path (⚠ OPEN 1).
- **`riggen-core::Command`**: `RenameMaterial { from, to }` rewrites the
  key and every link's reference; refused for an unknown `from`
  (`UnknownMaterial`) and an existing `to` (new `EditError::MaterialExists`).
  `Reparent` gains `at: JointState` — the configuration whose world poses
  `keep_world_pose` preserves; `JointState::default()` is today's
  behaviour, so the SDK's `reparent(…, keep_world_pose=False)` is
  unchanged and gains `q: dict[int, float] | None = None` (⚠ OPEN 4).
  `docs/02-data-model.md` §Commands and history: the rule "frame-rewriting
  commands work in the zero configuration" gets the exception stated.
- **`riggen-viewport::Viewport`**: a resolved *select* pick is reported as
  an event (`take_select_result() -> Option<Option<PickHit>>`), not only as
  state, so a click that missed everything is distinguishable from no click.
- **`riggen-app` panels** (`docs/01-architecture.md` §Panels and menus):
  `number_field` becomes a scrubber (egui `DragValue` with our
  formatter/parser and the gesture commit); one `fmt_num` for fields and
  readouts (six significant figures, scientific below `1e-3`); the Joints
  window's `open` gets an "auto-opened for this document" rule; the tool
  status lines; the Materials window's inline rename; the tree's drag
  ghost. `debug_state()` reports the status line and the Joints window
  state already; it gains the drag in flight.
- No ADR expected (⚠ OPEN 1 and 4 could each become one if the human
  disagrees with the preferred answer).

## Steps

- [x] Step 1 — One number format. `fmt_num` renders six significant figures,
  scientific notation below `1e-3` (`2.86e-5`, `0.001`, `-3`, `1.25`),
  trailing zeros dropped, no `-0`; `fmt_readout` folds into it; the parser
  accepts both spellings. Fixes the bug that a small Override tensor edit
  is rejected as "no change" because both sides rounded to `0.000029`.
  Test: unit round-trips for the four cases; the `properties_inertial`
  Override tensor is visible in a snapshot and reads as written.
- [x] Step 2 — History gestures in `riggen-core`: `GestureId`,
  `apply_in_gesture`, and the coalescing rule above. Unit tests: five
  applies under one id → `undo_depth` grows by one and one `undo` restores
  the before-state; a different id starts a new entry; `is_dirty` and
  `mark_saved` behave as for a single apply.
- [x] Step 3 — Number fields scrub. `number_field` is a `DragValue`: a
  horizontal drag changes the value at a speed scaled to its magnitude
  (Blender's rule; Ctrl for fine), each frame's value goes through
  `apply_in_gesture`, `drag_stopped` ends the gesture; a click still opens
  the text editor with the Enter / Escape / lost-focus semantics of today.
  Scenario `properties_scrub`: drag the joint origin `x` field 40 points →
  the part moved in the viewport, exactly one undo returns it, the
  document is dirty once. Every existing Properties golden re-captured if
  the field look changes — shown to the human.
- [x] Step 4 — The wheel steps a hovered field (⚠ OPEN 2 decides the
  modifier): one increment per notch at the field's displayed precision,
  through the same gesture (one notch = one entry; a burst within a short
  window coalesces). Scenario: three notches over `mass` → three
  increments, undo count as decided.
- [x] Step 5 — The Joints window opens itself. Rule (⚠ OPEN 3): when a
  document replaces the current one and has a movable joint, the window
  opens; when the first movable joint is created by a command, it opens;
  the user closing it is respected until the next document. Scenarios:
  opening the sample arm shows the window without the menu; closing it and
  switching another joint to Revolute does not reopen it; a new empty
  document does not open it. Every golden that loads a movable document
  will change — shown to the human, staged with `snapshots:`.
- [x] Step 6 — A tool says what it needs. Public constants in the style of
  `ZERO_CONFIG_STATUS`, set on tool entry *and* on selection change while
  the tool is active: Move / Rotate with nothing or the root selected,
  Place joint without a joint selected, Align without a link selected.
  Cleared when the selection satisfies the tool. Scenario
  `tools_say_what_they_need` asserts the status text per (tool, selection)
  pair from `debug_state()`.
- [x] Step 7 — Clicking empty space clears a joint or frame selection. The
  viewport's select event (design delta) drives
  `sync_selection_from_viewport`: a miss clears any selection; a hit
  selects the link as today. A click under a snapping tool issues no
  select pick and therefore clears nothing. Scenario: select a joint by
  its glyph, click the background → `Selection::None`; repeat under Place
  joint → the joint stays selected.
- [x] Step 8 — Per-geom collision editing for `Meshes`: per geom the pose
  rows (scrubbing, from step 3), a remove button, and "Add file…" through
  the `FileSource` seam (ADR-0017) so it works in the web build; each
  commit is one `SetCollision`. Scenario `properties_collision_meshes`:
  edit a pose, remove a geom, add the cube; the export zip lists the
  remaining collision meshes.
- [x] Step 9 — Materials can be renamed. Core `RenameMaterial` (design
  delta) with unit tests: references follow, unknown `from` and taken `to`
  refused; SDK `rename_material(from, to)` with a pytest and the `.pyi`.
  The Materials window's name cell renames on double-click or F2, the
  tree's inline-rename idiom. Scenario `materials_rename`: rename `PLA`,
  the link's Properties shows the new name.
- [x] Step 10 — The tree says what a drag will do: a ghost row with the
  link's name follows the cursor, the cursor is `Grabbing`, the row under
  it highlights as the drop target, and a row that would be refused (the
  link's own subtree, the root as source) shows `NotAllowed` instead.
  Scenario `tree_drag_ghost` captured mid-drag; `debug_state()` reports
  the drag.
- [ ] Step 11 — `Reparent` at the current `q`. Core: `at: JointState`;
  origin = `world_q(new_parent)⁻¹ ∘ world_q(link) ∘ motion(kind, axis,
  q_link)⁻¹`, which reduces to today's formula at `q = 0`. Unit test: FK
  of every link at `q` is unchanged after a reparent at `q`, and at
  `q = 0` it is *not* (the test that proves the field matters). The tree
  drop passes `self.q`; the SDK gains `q=`; scenario `tree_reparent_posed`
  drags a link with the arm swung 45° and the part does not jump.

## Acceptance

`cargo test --workspace` green with the eight new scenarios, and the M2 arm
build run by hand from the sample STLs with the Joints window never opened
from the menu, every number set by scrubbing or the wheel, one link
reparented with the arm posed, one material renamed, one `Meshes` geom
removed — and none of the three roadmap bullets this plan owns produces a
new line for the v0.3 friction list. The SDK's pytest suite passes on the
built wheel with the two new calls.

## Docs to update on completion

- `docs/01-architecture.md` §Panels and menus — Properties number fields
  (scrub, wheel, text), the Joints window's auto-open rule, the tool status
  lines, Materials rename, the tree's drag ghost; §Picking and snapping —
  the select event and the empty-click rule; §Testing — the new goldens in
  the documented list; §Python SDK — `rename_material`, `reparent(q=)`.
- `docs/02-data-model.md` §Commands and history — `RenameMaterial`,
  `Reparent { at }`, history gestures, the zero-configuration exception;
  §Core types if `History`'s public surface is listed there.
- `README.md` — the first-run paragraph no longer says to open Joints from
  the menu; a line on scrubbing.
- `python/riggen/_riggen.pyi` — the two signatures (in the step commits;
  verified at retirement).
- `docs/BACKLOG.md` — confirm the `Meshes`-is-read-only line is gone
  (moved into the roadmap at the v0.2 close); add any overflow.
- `docs/03-roadmap.md` §v0.3 — status line for the three bullets.
- `AGENTS.md` current state — one line.

## Open questions

- ⚠ OPEN 1 (human, before step 2): **a scrub is one history entry, not
  one command.** The rule "one gesture = one command" exists so undo is
  per gesture and the SDK sees whole edits; a scrubber previews *through*
  the document, so the honest reading is one *history entry* per gesture,
  coalesced from per-frame `Set…` commands. The alternative — a preview
  override for every field type the way `preview_world` and
  `preview_material_color` do for two — is unbounded. Preferred: the
  gesture entry; AGENTS.md's line becomes "one gesture = one history
  entry". If the human wants an ADR for it, it is a short one.
  **Decided 2026-09-02:** the gesture entry, no ADR.
- ⚠ OPEN 2 (human, by step 4): **which wheel steps a field?** A plain wheel
  over Properties scrolls the panel today. Preferred: **Ctrl+wheel** steps
  (Blender's rule), plain wheel keeps scrolling. Alternative: plain wheel
  steps while a field is hovered and scrolls elsewhere — discoverable, but
  a panel that stops scrolling when the cursor crosses a field is the kind
  of surprise v0.3 is removing.
  **Decided 2026-09-02:** Ctrl+wheel steps, plain wheel scrolls.
- ⚠ OPEN 3 (human, by step 5): **when does the Joints window open
  itself?** Preferred: on a document replace with a movable joint *and* on
  the first movable joint created, with the user's close respected until
  the next document. Narrower alternative: document replace only.
  **Decided 2026-09-02:** document replace and first movable joint, the close respected until the next document.
- ⚠ OPEN 4 (human, by step 11): **`Reparent` at `q` — a paragraph or an
  ADR?** It is the one frame-rewriting command allowed off the zero
  configuration, because it is a tree edit made while posing, not a
  placement. Preferred: a paragraph in 02 §Commands and history; no ADR.
  Also the SDK shape: `q: dict[int, float] | None = None` (preferred) versus
  a `JointState`-like object.
  **Decided 2026-09-02:** a paragraph in 02, `q: dict[int, float] | None = None`.
