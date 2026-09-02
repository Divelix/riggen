# Riggen

Native robot assembler for RL researchers: drop meshes, build the kinematic
tree, place joints, compute inertials and collision geometry, export MJCF and
URDF. Rust + egui/eframe + own wgpu viewport, shipped as a Python wheel.

Read in this order: `SEED.md` (charter, competition, stack), `README.md`
(what the user sees: install, first run, the CLI), `docs/01-architecture.md`, `docs/02-data-model.md`, `docs/03-roadmap.md`,
`docs/adr/`, then the rules in `.agents/rules/*.md` (git, docs lifecycle) —
Claude Code loads them automatically via `.claude/rules`; any other agent
reads them here. Skills for the idea → plan → work → retire → close-cycle pipeline live in
`.agents/skills/` (same symlink arrangement).

## Setup (once per clone)

```sh
git config core.hooksPath .githooks   # fmt, clippy -D warnings, test before every commit
```

## Current state

**v0.2 closed 2026-09-02, tag `v0.2.1`.** The cycle that made "sim-ready" a
feature rather than a claim: the Python SDK in the one wheel (ADR-0009),
V-HACD collision (ADR-0011), frames (ADR-0012), mimics (ADR-0013),
actuator presets (ADR-0014), MJCF import (ADR-0015), SDF export
(ADR-0016), and the web demo at
[divelix.github.io/riggen](https://divelix.github.io/riggen/) over one
`FileSource` seam (ADR-0017). `.riggen` is **schema 3**, with the upgrade
chain `load` walks; three writers and two importers share one
`ResolvedRobot` and one warning vocabulary.
**Before that:** M4 the wheel; M3 the writers, URDF import, inertials,
collision; M2 the mouse-only arm; M1 the document, commands, history,
`.riggen`; M0 the viewport.
**Now: v0.3 — the hand-feel debt** (`docs/03-roadmap.md`), no new format
and no new distribution. Landed 2026-09-03: numbers scrub and Ctrl+wheel
steps them (one gesture = one history entry), the panels stop hiding
things, the tree says what a drag will do and reparents at the current
`q`. Left: the viewport answers the mouse and keyboard, the overlay tells
the truth about depth and driven joints — decide
`docs/ideas/orbit-left-drag.md` first, then plan them.

## Rules that are not derivable from the code

- Lower crates never name upper crates' types; `riggen-core` and
  `riggen-export` never depend on egui or wgpu (`riggen-py` links them).
- One gesture = one history entry. A drag previews (a gizmo, off the
  document) or coalesces (a scrubber, through `History` gestures); release
  commits.
- Meters, radians, right-handed, Z-up, f64 in the document. Joint frame is
  the child link frame.
- Decisions go in `docs/adr/`; `⚠ OPEN:` in a doc marks a deferred one.
- Every UI change that can be seen gets a snapshot test (ADR-0003). When a
  snapshot changes, show the human the image.
- The agent looks at the UI itself — `visual-debug` skill (scratch capture,
  `debug_state()` JSON, Debug menu); for the web build, a headed-Chromium
  CDP driver (01 §Testing). Never ask the human to describe the screen.
- Update this file's "Current state" when a milestone lands; keep it under
  ~15 lines — the roadmap holds the detail.
- Backlog line → `/idea` (brainstorm, `docs/ideas/`) → `/plan` (todo,
  `docs/plans/`) → `/work` (one step, one commit) → `/retire-plan` (docs
  updated, plan deleted); `/close-cycle` at a roadmap boundary. Not every
  idea becomes a plan. Details: `.agents/rules/docs-lifecycle.md`.
- Trunk-based git, `main` always green, commit per plan step, never push
  unasked: `.agents/rules/git.md`.
- Crates.io and local checkouts: egui/rerun under `~/Documents/code/rust/`
  and RoboCAD at `~/Documents/code/pet/cad/robocad` (the ancestor: viewport,
  `mass.rs`, snapshot harness, the `consume_key` shortcut lesson in its
  `CLAUDE.md`) are reference reading, never `path =` deps.
