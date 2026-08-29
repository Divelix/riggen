# Plan: visual-debugging

- Started: 2026-08-29
- Milestone: M2 (prerequisite)
- Idea (verbatim from the human): "before planning m2, implement visual
  debugging we did in robocad. Thus you would be able to check gui without
  need of human in the loop"

## Goal

The agent checks a UI change by rendering the real app headlessly and
reading the PNG and the `debug_state()` JSON itself, guided by a
`visual-debug` skill that says so and carries the recipes; the running app
exposes the same JSON and egui's layout overlays under a Debug menu. No
human describes a screen. This is RoboCAD's "visual debugging for agents"
(its ADR-0014) brought to parity: riggen carried the harness, the scratch
target and `debug_state()` over in M0 (ADR-0003) but not the two
agent-facing entry points — the skill and the Debug menu — which is what
still made a human necessary.

## Non-goals

- Changing the harness, the existing goldens' content, `kittest.toml`
  thresholds, or what `debug_state()` reports.
- Any M2 feature (gizmos, snapping, glyphs).
- wasm coverage of the snapshot suite; a `--dump-state` CLI flag (RoboCAD's
  ADR-0014 rejected it: reaching the state worth dumping needs a window and
  a GPU, and the harness produces the same JSON headlessly); `egui_mcp` /
  `eframe/inspection` (deferred there, not re-opened here).

## Design deltas

- `docs/01-architecture.md` §Panels and menus: a **Debug** menu bullet
  (egui `DebugOptions` overlays; Copy state (JSON) / Save state (JSON)…, the
  runtime route to `debug_state()`). §Testing: the runtime route and the
  `RIGGEN_SCRATCH_OPEN` variable the scratch target reads.
- `AGENTS.md` rules: one line — the agent looks at the UI itself through
  the `visual-debug` skill; never asks the human to describe the screen.
- `.agents/skills/work/SKILL.md` step 3 names the skill.
- New `.agents/skills/visual-debug/SKILL.md` (reaches `.claude/skills/`
  through the existing directory symlink).
- No ADR: ADR-0003 already decided this and only its entry points slipped.

## Steps

- [x] Step 1 — Debug menu. `crates/riggen-app/src/app/debug_menu.rs`
  (sibling of `file_menu.rs`), wired after Window in `menu_bar`. Seven
  `DebugOptions` checkboxes read from the active theme's style and written
  to both themes only when changed (RoboCAD's light/dark lesson); separator;
  **Copy state (JSON)** → `ctx.copy_text(debug_state_json())` and the
  status `debug state copied`; **Save state (JSON)…** → `rfd` save dialog
  and `std::fs::write` (wasm: the existing "no filesystem in the browser"
  status). Tests: scenario `debug_menu` (menu open) and a golden-less
  `with_app` test that clicks Copy and asserts the status. A fourth
  menu-bar entry shifts every existing PNG: refresh with
  `UPDATE_SNAPSHOTS=1`, confirm the JSON goldens are byte-identical, look
  at `startup.diff.png`, and say `snapshots:` in the commit body. Docs:
  01 §Panels and menus, §Testing.
- [ ] Step 2 — `visual-debug` skill and the scratch env var.
  `.agents/skills/visual-debug/SKILL.md`: the two paths (scratch vs.
  scenario), reading the JSON section by section, the helper table
  (`settle`, `pump_rendered`, `click_at`, `fit_view_now`,
  `viewport_center`, `open_path`, `set_import_scale`, egui-side queries,
  AccessKit `SetValue` for sliders), the gotchas, the failing-snapshot
  runbook, the runtime route, the limits. `tests/visual_scratch.rs` reads
  `RIGGEN_SCRATCH_OPEN` (a `.riggen` path): opens it, settles, fits, pumps,
  captures — so a document is looked at without editing a tracked file.
  Pointers in `AGENTS.md`, `/work` step 3, `docs/README.md`; 01 §Testing.

Each step is one commit-sized unit with its own test or snapshot.

## Acceptance

In a fresh session with no human:

1. `RIGGEN_SCRATCH_OPEN=assets/fixtures/pendulum.riggen cargo test -p
   riggen-app --test visual_scratch -- --nocapture`; the PNG read with the
   Read tool shows the pendulum, the JSON lists two links and one joint.
2. `cargo test -p riggen-app --test visual` is green, `debug_menu` included.
3. `cargo build -p riggen-app --target wasm32-unknown-unknown` is green.

## Docs to update on completion

- `docs/03-roadmap.md` §M2 — one status line: the visual-debugging
  prerequisite landed (skill, Debug menu, `RIGGEN_SCRATCH_OPEN`).
- `AGENTS.md` current state — mention the `visual-debug` skill in the M2
  lead-in; stay under ~15 lines.
- Drift check of `docs/01-architecture.md` §Testing against
  `crates/riggen-app/tests/visual/harness.rs` and the skill.

## Open questions

- Decided at step 1: no golden for a `show_widget_hits` frame. The image
  is egui's overlay under the resting cursor, would churn with every egui
  upgrade and shows nothing about riggen; the toggle is asserted golden-less
  (`debug_overlay_toggle_sets_both_themes`).
