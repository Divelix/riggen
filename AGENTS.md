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

**Web demo done (2026-09-02, ADR-0017):** riggen runs in a browser at
[divelix.github.io/riggen](https://divelix.github.io/riggen/), on the same
readers and writers as the desktop — one `FileSource` seam (`Disk`, or a
drop gesture's files by *name*), `export_files` under `export`, downloads
instead of a filesystem. WebGPU only; V-HACD asks before freezing the tab.
**v0.2 before it:** SDF export (ADR-0016) and MJCF import (ADR-0015) — a
third writer and a second reader over the same `ResolvedRobot` and one
import vocabulary; actuator presets (ADR-0014), mimics through the one
`fk::resolve_q` (ADR-0013), named frames (ADR-0012), V-HACD on
`riggen-app::jobs` (ADR-0011), the `cp310-abi3` wheel that is both the app
and `import riggen` (ADR-0009). `.riggen` is **schema 3**, with the upgrade
chain `load` walks. **Before that:** M3 the writers from one
`ResolvedRobot`, URDF import, inertials, collision; M2 the mouse-only arm;
M1 the document, commands, history, `.riggen`.
**Next:** `/close-cycle` for v0.2 — nothing is left open in the roadmap.
Pages needs one by-hand switch: Settings › Pages › Source: GitHub Actions.

## Rules that are not derivable from the code

- Lower crates never name upper crates' types; `riggen-core` and
  `riggen-export` never depend on egui or wgpu (`riggen-py` links them).
- One gesture = one command. Drags preview; release commits.
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
