# Plan: visibility-row

- Started: 2026-09-05
- Milestone: v0.4, the window half (03 §The window: "A visibility row,
  top-right of the viewport")
- Idea: docs/ideas/visibility-row.md (absorbed)
- Idea (verbatim from the human): "visibbility row"

## Goal

The viewport's **top-right** corner holds a row of five toggles —
**joints**, **joint names**, **links**, **frames**, **collision** — drawn
in both modes as small marks in riggen's own idiom, each remembered across
sessions the way `View › Collision geometry` already is, and that menu
item deleted now that the row owns it. A toggle turns its things off
**completely**: the drawing and the pointer target go dark together, so
there is never a target the user can hit but cannot see (the reason
ADR-0020 §5 dims a hidden run instead of dropping it, one layer up). With
joints off, View has nothing under the cursor at all and the status bar
says so; with links off, Edit's mesh-aiming tools have nothing to hit and
the status bar says that too. Visibility is app state — eframe storage,
never `.riggen` (schema 3 unchanged) — and `debug_state().ui.overlays`
reports it, so a scenario asserts the state and not only the pixels.

## Non-goals

- **Zen on `Z`** (03 §The window). Its panel list has to name this row, so
  it follows in its own plan; nothing here hides a panel.
- The **axes triad**, the `persp` label and the frame-rate HUD: the triad
  is orientation rather than content, and the HUD is the Debug menu's
  (`set_frame_hud_visible`). Neither joins the row.
- Per-object visibility — a single link or joint hidden from the tree
  (Blender's `H`). The row is five class toggles and nothing per-id.
- Any change to what the **viewport crate** knows: it already has
  `set_instance_visible`, and glyphs are built app-side (ADR-0021 §6, the
  layer rule).
- An **icon font**. The marks are drawn with `egui`'s painter or they are
  short text labels; no new asset, no licence, no wasm payload.
- The **Materials window's** anchor. It collides with the *top-left*
  chrome today (a `plans/panels-and-numbers` backlog line); this plan takes
  the top-right, so that line is rewritten at retirement rather than
  executed here.

## Design deltas

- **ADR-0021, a new amendment** (step 1): §1 and §6 gain the rule *a hidden
  thing answers nothing*. Today §1 says that in View **only joint glyphs
  answer the cursor**; with joints hidden that becomes **nothing answers**
  — `set_pick_suppressed` stays on unconditionally, so meshes do not come
  back as a fallback, and the wheel goes back to the camera because no
  glyph is there to claim it. The switch table gains the row's rect beside
  the toolbar's. `docs/adr/README.md` gets the amendment note on 0021's
  row.
- **`riggen-app/src/app/overlays.rs`** (new): `Overlays { joints,
  joint_names, links, frames, collision: bool }`, all `true` by default
  except `collision`, which keeps `show_collision`'s `false`; one eframe
  storage key per field under a `riggen.overlays.` prefix, replacing
  `SHOW_COLLISION_KEY`, with a missing key falling back to the default so
  an old profile loads. The corner widget lives here — the row, its marks,
  and the rect it returns.
- **`riggen-app/src/app/mod.rs`**: `show_collision: bool` becomes
  `overlays: Overlays`; `toolbar_rect: Option<Rect>` becomes
  `chrome_rects: Vec<Rect>` (the `View | Edit` control and toolbar at the
  left, the row at the right), and the switch block at `mod.rs:525` blocks
  the camera and suppresses picks under **any** of them. The View menu
  loses its one checkbox and, with it, the menu (`View` keeps nothing —
  the button goes).
- **`riggen-app/src/app/glyphs.rs`**: `joint_glyphs()` returns empty when
  `overlays.joints` is off and `frame_glyphs()` when `overlays.frames` is,
  which takes out the drawing *and* `glyph_at` / `frame_glyph_at` in one
  place — the seam that makes the ADR's rule one line rather than five.
  The label loop in `glyph_overlay` and the frame name in
  `push_frame_overlay` are gated on `overlays.joint_names`.
- **`riggen-app/src/app/document.rs`**: `sync_scene` sets each visual
  instance's visibility from `overlays.links` beside the collision
  instances' from `overlays.collision`; `set_show_collision` becomes
  `set_overlay(field, bool)`, which still calls `sync_scene`.
- **`riggen-app/src/debug/mod.rs`** `UiDebug`: `collision_view: bool`
  becomes `overlays: Vec<&'static str>` — the names of what is **off**,
  omitted when everything is on, so the common golden gains no line and a
  scenario asserts by name.
- **`docs/01-architecture.md` §Panels and menus**: the row, its corner,
  what each toggle turns off, that it is remembered and never in the
  document, and the View menu's deletion (step 2–3). **§Picking and
  snapping**: a hidden thing is not a target — the app-side glyph test
  goes dark with the drawing, and `set_pick_suppressed` is unchanged.
- **`docs/03-roadmap.md` §The window**: the visibility row bullet marked
  landed, as the glyph bullet is.

## Steps

- [x] Step 1 — **ADR-0021's amendment**: a hidden thing answers nothing —
  §1's "only joint glyphs answer the cursor" becomes "and nothing at all
  when joints are hidden", §6's switch table gains the row's rect, and the
  reasoning cites ADR-0020 §5. `docs/adr/README.md` updated. Docs only, no
  code, no goldens.
- [x] Step 2 — **The row, and the two instance-backed toggles.**
  `app/overlays.rs` with the `Overlays` struct, its per-field eframe
  persistence (old `SHOW_COLLISION_KEY` profiles read as default), and the
  corner widget at the viewport's top-right: five toggle buttons with
  drawn marks, tooltips naming each. `chrome_rects` replaces
  `toolbar_rect` and the camera/pick block covers both corners.
  **links** and **collision** wired through `sync_scene`; `View ›
  Collision geometry` and the `View` menu deleted.
  `debug_state().ui.overlays` lands. Goldens: `overlay_row` (the arm in
  View, everything on) and `overlay_row_links_off` (the glyphs alone over
  the background). Look at the marks before they are fixed
  (`visual-debug`) and settle the ⚠ OPEN below from the image.
- [x] Step 3 — **The three glyph toggles, and the empty-viewport line.**
  `overlays.joints` and `overlays.frames` empty `joint_glyphs()` /
  `frame_glyphs()`, so drawing and the hover test go together;
  `overlays.joint_names` gates the two label loops. The status bar names
  what is hidden (`joints hidden`), so a viewport that answers nothing is
  never a mystery. A harness assertion for the ADR's rule: with joints off
  in View, `glyph_at` is `None` over the arm, `debug_state().glyphs` is
  empty, and the wheel over the same point zooms instead of posing.
  Goldens: `overlay_row_joints_off` and `overlay_row_names_off`. 01
  §Panels and §Picking written in this step and step 2.

## Acceptance

`cargo test` green. On the sample arm in View: turning **joints** and
**frames** off leaves the posed robot alone in the viewport while the
joint tree still scrubs it; turning every toggle back on renders the
viewport **byte-for-byte** as `view_opens_with_the_document`'s golden, so
the row's default state is provably the state that golden was taken in.
With everything off, `debug_state().glyphs` is empty, `glyph_at` answers
`None` anywhere over the robot, and the wheel zooms.

## Docs to update on completion

- `docs/01-architecture.md` §Panels and menus (the row, the corner, the
  persistence, the View menu's deletion) and §Picking and snapping (a
  hidden thing is not a target) — written in steps 2–3; verify against the
  code at retirement.
- `docs/03-roadmap.md` §The window — the visibility row bullet marked
  landed, as the glyph bullet is. The bullet itself stays until the cycle
  closes.
- `docs/adr/README.md` — 0021's row notes the second amendment (step 1).
- `docs/BACKLOG.md` — the `plans/panels-and-numbers` line that wants the
  Materials window anchored *"at the top-right, the corner the Joints
  window vacated"* is rewritten: the top-right is the row's, so the
  window needs a corner that is neither.
- `AGENTS.md` current state — the row lands; next is zen on `Z`.
- `README.md` — the hero image is `view_opens_with_the_document`'s golden,
  which the acceptance pins as unchanged; nothing to retake unless it
  moves.

## Open questions

- Decided (agent, step 2, from the golden, on the human's "use recommended
  option"): the **drawn marks stay** — four of the five read at twelve
  points. The one that did not was the frames triad, which at that size
  reads as an arrow; it became the link tree's own `⌖` (ADR-0012), which
  is the mark the app already uses for a frame. No text fallback, no icon
  font.
- ⚠ OPEN: **what View picks when joints are hidden.** The plan takes the
  idea's recommendation — nothing at all, `set_pick_suppressed` unchanged
  — and step 1 writes it into ADR-0021. The alternative the human may
  prefer is that hiding joints in View falls back to mesh picking, which
  would make the row change a *mode's* rule rather than only what is
  drawn. Decided by the human at step 1; **recommended: nothing answers.**
