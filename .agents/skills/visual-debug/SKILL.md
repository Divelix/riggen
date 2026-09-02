---
name: visual-debug
description: See riggen's GUI without a human describing it — render the real app headlessly to a PNG you can read, and dump what the app thinks it drew as JSON. Use when working on anything visible and you need to check the result rather than reason about it: the link tree, properties, joints or materials panels, menus and modals, viewport framing, hover/selection tint, instance positions after FK, joint glyphs and gizmos (M2), the axes triad. Also use when a visual snapshot test fails, when asked what a screen looks like or whether a UI change worked, and before any UPDATE_SNAPSHOTS=1.
---

# Seeing the riggen GUI

You can look at this app. Do that instead of reasoning about pixels from
source — the whole wgpu paint callback renders headlessly (shaded instances,
ID-buffer picking, hover/selection tint, axes triad), and
`RiggenApp::debug_state()` reports what the app believes it drew. Never ask
the human to describe the screen.

Design and rationale: `docs/adr/0003-headless-visual-snapshots.md` and
`docs/01-architecture.md` §Testing. Read those before changing how the
harness works; this file is how to *use* it.

## Which of the two do you want?

**Looking at an arbitrary state** → the scratch target. Nothing is compared,
nothing is pinned.

```sh
cargo test -p riggen-app --test visual_scratch -- --nocapture
RIGGEN_SCRATCH_OPEN=assets/fixtures/pendulum.riggen \
  cargo test -p riggen-app --test visual_scratch -- --nocapture
```

Writes `target/visual-scratch/scratch.png` and `scratch.json` and prints both
paths. **Read the PNG with the Read tool** — it renders as an image; read the
JSON when the question is "which number is wrong". `RIGGEN_SCRATCH_OPEN`
opens a `.riggen` (or a mesh) and fits the view before the capture, so a
document is looked at without editing anything. For any other state, edit
the body of `crates/riggen-app/tests/visual_scratch.rs` — every helper the
real scenarios use is available — run, read, and **revert the edit before
committing**; the file is tracked and its default body is on purpose.

**Pinning a state against regressions** → a scenario in
`crates/riggen-app/tests/visual/main.rs`. Each captures a PNG *and* a
`debug_state()` JSON, both committed under `crates/riggen-app/tests/snapshots/`
(`startup`, `cube`, `hover_cube`, `select_cube`, `three_parts`, `pendulum`,
`mm_scale_part`, `tree_pendulum`, `tree_reparent`, `properties_link`,
`properties_joint`, `pendulum_swing`, `materials`, `dirty_title`,
`unsaved_confirm`, `debug_menu`; M2 adds `toolbar`, `gizmo_move_link`,
`gizmo_rotate_joint`, `glyph_revolute`, `glyph_prismatic`, `glyph_hover`,
`snap_vertex`, `snap_circle`, `place_joint_bore`, `align_concentric` and
the acceptance, `five_minute_arm`; v0.3 adds `properties_scrub`,
`properties_wheel`, `joints_window_opens_itself`,
`tools_say_what_they_need`, `click_empty_clears`,
`properties_collision_meshes`, `materials_rename`, `tree_drag_ghost` and
`tree_reparent_posed` — the full list is in 01 §Testing).

```sh
cargo test -p riggen-app --test visual
```

Keep this suite small and aimed at what no unit test can reach. Behaviour
that needs the app but no picture goes through `harness::with_app`, which
has no goldens at all.

## Reading the JSON

The viewport is one paint callback, so it emits no AccessKit nodes: a picture
can be taken of it but it cannot be queried. The JSON is that half.

- `camera` — eye/target/up, yaw/pitch/distance, near/far, projection, view
  and projection matrices, `animating`
- `document` — file, name, dirty, import scale, links (id, name, parent
  joint, geoms, material), joints (kind, parent, child, `q`), selection
- `ui` — rename in progress, open windows, modal, window title
- `instances` — per instance: link/geom key, visible, triangle count,
  bounds, world `position` at the current `q`, colour
- `selection` — the hovered and selected `{instance, triangle}` the ID
  buffer resolved
- `ui.tool` — the active tool, by its toolbar label
- `glyphs` — per joint glyph: pivot, world axis, size, `q`, its **screen**
  position, `hovered` / `active`
- `gizmo` — target (`"link l3"` / `"joint j7"`), mode, origin, screen
  position, `dragging`, `captured`
- `snap` — what the cursor is pointing at: kind, point, normal, the axis a
  joint would take, and for a circle its radius / segments / residual in
  millimetres plus the readout string
- `status` — the status bar's one-off message; `viewport_rect`

A misplaced glyph or a link that "did not move" is a wrong *number* here:
the picture says something is off, `instances[i].position` and `camera`
say which and by how much. Every float is rounded to six decimals.

At runtime the same JSON is under **Debug → Copy state (JSON)** / **Save
state (JSON)…**, next to egui's own `DebugOptions` toggles (`debug_on_hover`,
`show_widget_hits`, `show_interactive_widgets`, …). A capture taken with
those on is a picture of the layout skeleton including the parts that
never paint — useful for "why is this widget where it is".

## Writing a scenario or scratch body

Helpers in `crates/riggen-app/tests/visual/harness.rs`; app-side entry
points in `crates/riggen-app/src/debug/mod.rs` and the `pub` API of
`RiggenApp`.

| Need | Call |
| --- | --- |
| a reproducible frame (no animation, no pick in flight) | `settle(harness)` |
| any GPU-dependent state (picking!) | `pump_rendered(harness, n)` |
| a viewport click that actually selects | `click_at(harness, pos)` |
| frame the geometry, no animation | `harness.state_mut().fit_view_now()` |
| a point over the geometry | `harness.state().viewport_center()` |
| **where a world point lands on screen** | `harness.state().project_world(DVec3)` |
| a gizmo handle to aim at | `debug_state().gizmo.screen` (its view-plane handle) |
| a joint glyph to aim at | `joint_glyphs()[i]`, then `project_world` along the axis |
| a drag (gizmo, or anything) | `synthetic_drag(harness, from, to, steps)` |
| a widget floating **over** the viewport | `click_widget(harness, label)` |
| the tool | `harness.state_mut().set_tool(Tool::PlaceJoint)` |
| load a document or a mesh | `harness.state_mut().open_path(path)` |
| unit-cube fixtures as meters | `set_import_scale(1.0)` (the harness already does) |
| edit the document | `harness.state_mut().apply(Command::…)` |
| a menu or button | `harness.get_by_label("Window").click(); harness.step();` |
| a field | `get_all_by_label("x").nth(0)` (origin) / `.nth(1)` (axis) |
| a combo | `Role::ComboBox`; its items by label once open |
| a slider, exactly | AccessKit `SetValue` via `harness.event(AccessKitActionRequest { .. })` |
| the state dump | `harness.state().debug_state()` / `debug_state_json()` |

Things that will otherwise cost you an hour:

1. **`Harness::step` does no GPU work.** The paint callback runs in
   `render`. Anything depending on the GPU — the ID-buffer pick above all —
   needs `pump_rendered`, which does both.
2. **A viewport click is one event per rendered frame** — that is what
   `click_at` does. `step` drains every queued event and keeps only the
   last one's output, so `drag_at`/`drop_at` produce a click frame that is
   computed and thrown away, and that frame leaves a pick permanently in
   flight. egui widgets are not affected: `get_by_label(..).click()` +
   `step()` is enough.
3. **A new popup or window is laid out invisibly on its first frame.** The
   AccessKit nodes exist at once, the pixels one frame later: `settle` (or
   one more `step`) before the capture, or the menu is missing from the PNG
   while every query on it passes.
4. **Do not start a camera animation.** The clock is not injected, so an
   animated frame is not reproducible. `fit_view_now`, never Home;
   `debug_state().camera.animating` is the tell.
5. **kittest cannot drag a tree row onto another.** Reparent through the
   command API and draw the result; `synthetic_drag` works for a one-off
   check only.
6. **`Node::click()` on a widget over the viewport leaves a pick stuck.**
   It queues press and release together and `step` runs one *unrendered*
   logic pass per queued event, so the pick the pointer move issues is
   recorded by a frame nothing renders. Use `click_widget`, which drives
   the same events `click_at` does. (The viewport abandons such a request
   after `MAX_PICK_FRAMES` now, so this is slow rather than fatal.)
7. **Aim a placement click at a point you can see.** `project_world` gives
   the screen position of any world point; pick one on the *camera-facing*
   side of a shaft and off the mid-plane if a block is in the way, or the
   snap ladder answers with the vertex or box corner that happens to be
   nearer in screen space. `five_minute_arm`'s `aim_at_shaft` is the
   worked example.

Scenarios are serialised behind a mutex — concurrent lavapipe devices at
1440×900 segfault. Keep new scenarios going through `harness::scenario`,
which holds it, and keep the size: the goldens encode it.

## When a snapshot test fails

1. Read `tests/snapshots/<name>.diff.png` and `<name>.new.png` — both are
   written on failure and both are readable as images.
2. The JSON half panics with the **first differing line**, which is usually
   the whole story; the full output is at `<name>.json.new`.
3. Decide whether the change is intended. If it is:

```sh
UPDATE_SNAPSHOTS=1 cargo test -p riggen-app --test visual
```

One env var updates both halves. **Look at the diff image before you accept
an update**, and show the human the new image (AGENTS.md). A suite updated
reflexively is worse than no suite, because it looks like coverage; the
watch signal is `UPDATE_SNAPSHOTS=1` in a commit with no matching
intentional UI change. A commit that refreshes goldens says `snapshots:`
and why (`.agents/rules/git.md`).

## Limits

- Native only, and needs a wgpu adapter. Without `mesa-vulkan-drivers`
  (lavapipe) the scenarios print `SKIPPING` and pass — an environment
  failure, not coverage. CI installs it, so a skip there is a bug.
- lavapipe is the reference environment on purpose (`egui_kittest` sorts
  CPU adapters first), which is what makes goldens comparable across
  machines. Pixels may still shift slightly across mesa releases;
  `kittest.toml` carries the tolerance.
- No wasm coverage.
- Text queries (`get_by_label`, …) reach panels, menus and windows, which
  are real egui widgets. They do **not** reach the viewport — that is what
  `debug_state()` is for.
