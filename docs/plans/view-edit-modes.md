# Plan: view-edit-modes

- Started: 2026-09-05
- Milestone: v0.4, the window half (03 §The window: "Two modes, Tab between
  them", "View: the joint tree", "View: joints are the only thing under the
  cursor", "Edit: the tree as it is, `q` locked")
- Idea: docs/ideas/view-edit-modes.md (absorbed; option A, all six
  decisions with the preferred answer)
- Idea (verbatim from the human): "2 modes: View and Edit, user can switch
  between them by pressing Tab; when user opens model - View mode activated
  … view/edit split with view as default - I think it is the first thing we
  need to implement before anything else"

## Goal

The window has two modes and `Tab` switches them. **View** is the posed
robot: the left panel is the joint tree — movable joints in kinematic
order, fixed joints collapsed through, a follower's row read-only at its
resolved `q` with the rule under it (ADR-0013) — each row a scrubber that
drags and steps by the wheel at the rotate ring's 5° / 1° quantum
(ADR-0019 §2) and shows the value and both limits; the properties panel and
the toolbar are hidden; only joint glyphs answer the cursor, a click selects
the joint, the wheel over it steps it, a mesh is neither hover-tinted nor
selectable; the tool keys do nothing but hint. **Edit** is the v0.3 editor
at the zero configuration: `Tab` into it stashes `q` and rewinds to zero
without a history entry, `Tab` back restores it, and the per-tool reset
with its status line is gone. A `View | Edit` control sits top-left with
`Tab` in its tooltip and the status bar names the mode. A document, an
import and the demo open in View; File › New and a mesh drop open in Edit.
The floating Joints window and its open-itself rule are deleted. The
contract is ADR-0021.

## Non-goals

- The glyph's shape (plans/joint-glyph-range-and-value) and the visibility
  row, zen mode, the rest of 03 §The window.
- Any change to the viewport crate: the mode is a policy on the five
  switches the app already sets (01 §Picking and snapping).
- Persisting the mode across runs: the open rule decides it every time.
- Edit's own behaviour beyond the zero configuration — tools, gizmos, tree
  drag, properties stay as v0.3 left them (03 §v0.4 Out).
- Undo for posing: `q` stays derived state, never a command.

## Design deltas

- **ADR-0021 — two modes, and Edit is the zero configuration.** What each
  mode owns (picks, wheel, panels, `q`, selection), `Tab`, the open rule,
  the tool keys in View; supersedes the per-tool `q` reset of 01 §Panels
  (plans/m2-placement-ux OPEN 1) and narrows 02 §Commands' "`Reparent` at
  the current `q`" to the SDK's `q=` (the GUI's tree drag is in Edit, at
  zero).
- **`riggen-app/src/app/mod.rs`** (the field) and **`mode.rs`** (the enum
  and `set_mode`, its own module like `tool.rs`): `Mode { View, Edit }` on
  `RiggenApp`, `stashed_q: Option<JointState>`; `set_mode` does the stash / rewind /
  restore and the selection carry-over (a joint selection survives, a link
  or frame selection clears). `handle_shortcuts` consumes bare `Tab`,
  yielding to a focused `TextEdit` as every bare key does. The per-frame
  switch block: in View, `set_pick_suppressed(true)` unconditionally and
  `set_wheel_claimed(glyph_hover.is_some())`; the wheel over a hovered
  glyph steps that joint's `q` by 5° (1° with shift; `STEP_M`'s floor for
  a prismatic) through the existing `set_joint_value` path.
- **`tool.rs`**: `set_tool` no longer resets `q`; `ZERO_CONFIG_STATUS` and
  `edits_frames()`'s reset role go; in View the tool keys set a status hint
  (`VIEW_TOOL_HINT`, public for tests) and change nothing.
- **`panels/joints.rs` → `panels/joint_tree.rs`**: the View left panel. A
  `JointRow` scrubber widget (name, a bar with the limits at its ends and
  the value as a fill and a number, drag and wheel), rows nested by the
  kinematic chain, "Reset all". `JointsWindow` and its open-itself state
  deleted; Window › Joints leaves the menu.
- **`panels/mod.rs` / `mod.rs` frame order**: the left panel draws the
  joint tree in View and the link tree in Edit; the properties panel and
  the toolbar draw in Edit only; the `View | Edit` segmented control is a
  new floating frame at the viewport's top-left in both modes, the toolbar
  to its right in Edit (both same-layer, both `set_camera_blocked` +
  `set_pick_suppressed` while hovered, as the toolbar is today).
- **`document.rs`**: `replace_document` applies the open rule — View when
  the new document has a movable joint and arrived as a document or an
  import, Edit for New and for a drop that is meshes alone (ADR-0006) — in
  place of `joints_window.document_replaced`; `document_changed`'s hook
  goes with the window.
- **`debug/mod.rs`**: `UiDebug::mode: &'static str`, `UiDebug::windows`
  loses `"joints"`; `GlyphDebug` unchanged.
- **`tests/visual/harness.rs`**: `open_for_editing(path)` = `open_path` +
  `Tab`, the helper every editing scenario switches to; `press_tab`.
- **`docs/01-architecture.md`**: §Panels and menus (the mode, the joint
  tree, the toolbar and properties per mode, the Joints window paragraph
  removed, the zero-configuration paragraph rewritten), §Picking and
  snapping (a "mode policy" paragraph under the switch table), §Testing
  (the helper); **`docs/02-data-model.md`** §Commands (`Reparent`'s `at`
  paragraph narrowed). Each in the step that changes the behaviour.

## Steps

- [x] Step 1 — ADR-0021 written and accepted: the mode contract as the
  Goal states it, its Context the idea's Problem, its Alternatives the
  idea's B and C. Docs only; the later steps cite it.
- [x] Step 2 — `Mode` on the app, default **Edit** for now: `set_mode`,
  bare `Tab` in `handle_shortcuts` (yielding to a focused text field),
  `UiDebug::mode`. No visible change yet — a harness test presses `Tab`
  and reads `debug_state().ui.mode` both ways, and a focused rename field
  keeps its `Tab`.
- [x] Step 3 — Edit is the zero configuration: `set_mode(Edit)` stashes `q`
  and rewinds, `set_mode(View)` restores; `set_tool`'s reset and
  `ZERO_CONFIG_STATUS` removed and the tests that asserted the status line
  rewritten to assert the stash. 01 §Panels' zero-configuration paragraph
  and 02 §Commands' `Reparent` paragraph updated in the same commit. Golden
  changes only where a scenario entered a tool posed.
- [ ] Step 4 — View's pointer policy (the riskiest interaction): in View the
  picks are off, a glyph click selects the joint, the wheel over a hovered
  glyph steps `q` at the ring's quantum, the tool keys hint. Harness tests
  through `debug_state`: no hover over a mesh in View; `q` after
  `scroll_at` a glyph; the hint after `G`. New golden `view_wheel_on_glyph`
  (a scenario that `Tab`s into View, since Edit is still the default).
- [ ] Step 5 — The joint tree panel, drawn in View only: `JointRow`, the
  kinematic nesting, follower rows, "Reset all", the wheel on a row at the
  same quantum. New goldens `view_joint_tree` (the arm: three rows, one a
  follower with its rule) and `view_joint_tree_scrub` (a row mid-drag). The
  Joints window still exists and is untouched, so no other golden moves.
- [ ] Step 6 — Panels per mode and the open rule, **the one full refresh**:
  the `View | Edit` control top-left, the status bar's mode, properties and
  toolbar hidden in View, the Joints window deleted with its rule and menu
  item, `replace_document`'s open rule (document / import / demo → View,
  New / mesh drop → Edit). The harness gains `open_for_editing` and every
  editing scenario switches to it; scenarios that showed the Joints window
  become View scenarios. Every golden refreshed once; `snapshots:` in the
  message and the images shown to the human. 01 §Panels and §Picking
  rewritten in the same commit.
- [ ] Step 7 — The View hover target grows to the glyph's band and its
  interior (the glyph plan's "hover" section), **after
  plans/joint-glyph-range-and-value step 2 has landed** and `band_points()`
  exists; in Edit the target stays the axis segment. Golden `glyph_hover`
  gains a View twin `view_glyph_hover_band`.

## Acceptance

`cargo test` green. The M2 five-minute-arm scenario passes with one `Tab`
added at its start and no other change. `--example arm` opened cold is in
View: the joint tree shows three rows with limits, the wheel over a glyph
turns that joint, a click on a mesh selects nothing; `Tab` shows the arm at
zero with the v0.3 editor around it; `Tab` again restores the pose. The
demo's first frame is View. `debug_state().ui.windows` never contains
`"joints"`.

## Docs to update on completion

- `docs/01-architecture.md` §Panels and menus, §Picking and snapping,
  §Testing — written in steps 3, 4 and 6; verify against the code.
- `docs/02-data-model.md` §Commands — the `Reparent` paragraph, step 3.
- `docs/03-roadmap.md` §v0.4 §The window — the four bullets this plan
  covers stay as the cycle's record; the glyph bullet's ⚠ OPEN loses its
  hover half once step 7 lands.
- `docs/BACKLOG.md` — the panels line about `Slider`'s clamping workaround
  (the Joints sliders) is obsolete once the window is gone: drop that
  half; the Materials-window anchoring line is re-read against the new
  top-left control.
- `README.md` — any mention of the Joints window or of opening into the
  editor; the hero image if the first frame changed enough.
- `AGENTS.md` current state — unchanged unless the cycle closes.

## Open questions

- ⚠ OPEN: the prismatic wheel quantum — 5° has no metre analogue; the
  proposal is properties' `STEP_M` floor scaled as `scrub_speed` does.
  Agent, step 4.
- ⚠ OPEN: what View shows for a document with **no** movable joint that
  arrived as a document (an all-fixed import): an empty joint tree with a
  "nothing to pose — Tab to edit" line, or Edit directly. Proposal: the
  line; the open rule stays one sentence. Human, before step 6.
- ⚠ OPEN: ordering with the glyph plan — step 7 here waits on that plan's
  step 2, and that plan's step 2 should run after step 6 here so its
  goldens are refreshed in View once. Agent sequences; the human is told
  when `/work` crosses plans.
