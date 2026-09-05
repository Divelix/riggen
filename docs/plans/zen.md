# Plan: zen

- Started: 2026-09-05
- Milestone: v0.4, the window half (03 §The window: "Zen mode on `Z`")
- Idea (verbatim from the human): "zen"

## Goal

`Z` empties the window of everything that is not the robot: the menu bar,
the status bar, the left panel (the joint tree in View, the link tree in
Edit), the properties panel, and the corner chrome — the `View | Edit`
control, the toolbar beside it, and the visibility row. The viewport fills
the window and the robot is all there is; `Z` again brings every one of
them back exactly as it was. Zen is **orthogonal to the mode** (ADR-0021):
it is the same key, the same effect and the same state in View and Edit,
and it changes nothing about which mode the window is in, what answers the
cursor, or what the visibility row's five toggles are set to. It is
transient app state — never persisted, never in the document —
`debug_state().ui.zen` reports it, and both modes' zen is in the snapshot
suite.

## Non-goals

- **Any change to picking, the wheel, or the switch table.** Zen removes
  chrome from the screen; the switches ADR-0010/0018/0019/0021 publish
  are set from the mode and the hover exactly as they are today. The only
  consequence is that `chrome_rects` is empty, so there is no rect
  blocking the camera and suppressing picks — which is what "no chrome"
  already means.
- **The visibility row's five toggles.** Zen hides the *row*; what the row
  says stays in force, and leaving zen shows the row still set the way the
  user left it. Zen is not a sixth toggle and does not join `Overlays`.
- **The axes triad, the gradient background and the `persp` label.** They
  are the viewport crate's, they are orientation rather than content, and
  the visibility-row plan already fenced them off.
- **Full-screen.** `Z` hides riggen's own chrome inside the window it has;
  the OS window keeps its decorations and its size. A separate key for
  `ViewportCommand::Fullscreen` is a backlog line, not this plan.
- **Modals.** New / Open / Quit's unsaved-changes modal and the Export
  dialog float over everything and are answers the user is owed; they are
  drawn in zen as they are anywhere else.
- **Any viewport-crate change** (ADR-0021 §6). Zen is which panels the app
  draws and nothing more.

## Design deltas

- **ADR-0021, a third amendment** (step 1): **zen is orthogonal to the
  mode.** The two-mode rule of §1–§4 is untouched; zen is one boolean over
  it, hiding every panel and both pieces of corner chrome in either mode,
  never persisted (§4's reasoning — the open rule answers the mode from
  what arrived — applies to zen the same way: nothing about the window's
  furniture survives a run). Two consequences the amendment settles:
  (a) with the chrome gone `chrome_rects` is empty, so nothing blocks the
  camera or suppresses picks by position, and the mode's own rules are the
  whole story; (b) the second amendment's guarantee that *the status bar
  names what is hidden* has no status bar in zen, so **`Esc` leaves zen**
  beside `Z` — a viewport that answers nothing is then never a window the
  user cannot get out of with the key they already reach for.
  `docs/adr/README.md` gets the amendment note on 0021's row.
- **`riggen-app/src/app/mode.rs`**: `zen: bool` beside `mode`, with
  `zen()` / `set_zen(bool)` / `toggle_zen()`. Zen lives here because it is
  the same policy — which panels are drawn — that `Mode` is, and it is
  read in the same three places.
- **`riggen-app/src/app/mod.rs`** `ui`: `menu_bar`, `status_bar`, the
  match on `self.mode` that draws the left panel and the properties panel,
  and `viewport_chrome` each run only when zen is off; when it is on,
  `chrome_rects` is **cleared** rather than left holding last frame's
  rects. `materials_window` is skipped while zen is on with
  `materials_window.open` untouched, so the window comes back with the
  chrome.
- **`riggen-app/src/app/shortcuts.rs`**: bare `Z` after the text-field
  guard and after the `Ctrl+Shift+Z` / `Ctrl+Y` / `Ctrl+Z` block — the
  RoboCAD `consume_key` ordering lesson, since a bare pattern swallows its
  modified variant — guarded on `self.pending.is_none()` the way `Tab` is.
  `Esc` leaves zen before it leaves a tool, so one press is one effect.
- **`riggen-app/src/debug/mod.rs`** `UiDebug`: `pub zen: bool`, skipped
  when `false` so no existing golden gains a line.
- **`docs/01-architecture.md` §Panels and menus**: a **Zen** bullet — the
  key, the list of what goes, that the toggles and the mode are untouched,
  that it is never remembered, and that `Esc` is the second way out.
  **§Picking and snapping**: the `chrome_rects` paragraph notes that the
  list is empty in zen. **§Shortcuts**: `Z` in the yield-to-`TextEdit`
  set, ordered after `Ctrl+Z`.
- **`docs/03-roadmap.md` §The window**: the zen bullet marked *landed*, as
  the glyph and visibility-row bullets are.

## Steps

- [x] Step 1 — **ADR-0021's third amendment: zen is orthogonal to the
  mode.** What it hides (every panel, both pieces of corner chrome), what
  it deliberately leaves alone (the mode, the five toggles, the switch
  table), that it is never persisted, and the two consequences above —
  empty `chrome_rects`, and `Esc` beside `Z` because there is no status
  bar left to read. `docs/adr/README.md` updated. Docs only, no code, no
  goldens.
- [ ] Step 2 — **`Z`, and the empty window.** `zen` on `RiggenApp` with
  its three methods in `mode.rs`; the five panel calls and
  `viewport_chrome` gated in `ui`, `chrome_rects` cleared; bare `Z` in
  `handle_shortcuts` in the ordering above; `debug_state().ui.zen`. A unit
  test that `Ctrl+Z` still undoes and does not toggle zen, and that `Z`
  in a focused `TextEdit` types a `z`. Golden: `zen_view` — the sample arm
  in View with nothing but the viewport. Look at it (`visual-debug`)
  before it is fixed. 01 §Panels, §Picking and §Shortcuts written here.
- [ ] Step 3 — **Zen in Edit, `Esc`, and coming back.** The same key in
  Edit takes the toolbar, the link tree and the properties panel with the
  rest; the Materials window is skipped while zen is on and returns with
  its `open` flag intact; `Esc` leaves zen before it leaves a tool. A
  harness assertion for the round trip: `Z` `Z` from View leaves
  `debug_state()` — mode, selection, tool, overlays, camera — as it was.
  Golden: `zen_edit`.

## Acceptance

`cargo test` green. On the sample arm opened cold: `Z` leaves the
viewport alone in the window with the robot in it and
`debug_state().ui.zen` true; the wheel over a glyph still poses the joint
and the camera still orbits, because zen changed no switch; `Z` again
renders the window **byte-for-byte** as `view_opens_with_the_document`'s
golden, which proves the round trip restores every panel and the camera
untouched. `Tab` inside zen still switches modes and `Esc` leaves zen.
The same in Edit for `zen_edit`.

## Docs to update on completion

- `docs/01-architecture.md` §Panels and menus (the Zen bullet),
  §Picking and snapping (`chrome_rects` is empty in zen), §Shortcuts (`Z`,
  and its order against `Ctrl+Z`) — written in steps 2–3; verify against
  the code at retirement.
- `docs/03-roadmap.md` §The window — the zen bullet marked landed.
- `docs/adr/README.md` — 0021's row notes the third amendment (step 1).
- `AGENTS.md` current state — zen lands; the window half is then complete
  and next is the composite-joint ADR before the file half.
- `README.md` — the hero image is `view_opens_with_the_document`'s golden,
  which the acceptance pins as unchanged; nothing to retake.

## Open questions

- ~~⚠ OPEN: **whether `Esc` leaves zen.**~~ **Answered by the human
  2026-09-05: yes**, the recommendation. With the status bar gone there is
  no line saying what is hidden, and `Esc` is the key a user presses when a
  window will not respond. The cost is accepted: zen is ordered before the
  tool, so a tool active in zen takes two presses. Written into ADR-0021's
  third amendment (step 1), implemented in step 3.
- ~~⚠ OPEN: **whether entering zen is silent.**~~ **Answered by the human
  2026-09-05: yes**, the recommendation. Nothing is painted and nothing
  fades, so no golden has a clock in it. If the by-hand run finds the empty
  window reads as a hang, the fallback is a hint drawn in a viewport corner
  for as long as zen is on — never a timed toast — and that is a new
  amendment, not this plan.
