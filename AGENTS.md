# Riggen

Native robot assembler for RL researchers: drop meshes, build the kinematic
tree, place joints, compute inertials and collision geometry, export MJCF and
URDF. Rust + egui/eframe + own wgpu viewport, shipped as a Python wheel.

Read in this order: `SEED.md` (charter, competition, stack), `README.md`
(what the user sees: install, first run, the CLI), `docs/01-architecture.md`, `docs/02-data-model.md`, `docs/03-roadmap.md`,
`docs/adr/`, then the rules in `.agents/rules/*.md` (git, docs lifecycle) —
Claude Code loads them automatically via `.claude/rules`; any other agent
reads them here. Skills for the idea → plan → work → retire pipeline live in
`.agents/skills/` (same symlink arrangement).

## Setup (once per clone)

```sh
git config core.hooksPath .githooks   # fmt, clippy -D warnings, test before every commit
```

## Current state

**M4 done (2026-08-30, tag `m4`):** `uv tool install riggen && riggen
--example arm` (ADR-0002). Root `pyproject.toml`, maturin `bindings =
"bin"`: the `riggen` binary sits in the wheel's `scripts/`, no console
script; `python -m riggen` execs it. `--help` / `--version` (git hash via
`build.rs`) / `--timing` / `--example arm`. `ci.yml` builds and smokes the
linux wheel on every push; `release.yml` builds five targets + sdist,
smokes each OS, publishes to TestPyPI on dispatch and PyPI + GitHub
Release on a `v*` tag through trusted publishing. `README.md` is the user
page and the PyPI page. M3: MJCF + URDF export from one `ResolvedRobot`
(ADR-0004, ADR-0008), MuJoCo loads it clean and matches `fk`, URDF
import, inertials, collision. M2: the mouse-only arm (toolbar, gizmo,
glyphs, snapping). M1: the `Robot` document, commands, history, `.riggen`
v1, panels. M0: mesh, viewport, the `egui_kittest` suite (`visual-debug`
skill). **Next: v0.2** — the Python SDK (`riggen-py`, PyO3).

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
  `debug_state()` JSON, Debug menu). Never ask the human to describe the
  screen.
- Update this file's "Current state" when a milestone lands; keep it under
  ~15 lines — the roadmap holds the detail.
- Backlog line → `/idea` (brainstorm, `docs/ideas/`) → `/plan` (todo,
  `docs/plans/`) → `/work` (one step, one commit) → `/retire-plan` (docs
  updated, plan deleted). Not every idea becomes a plan. Details:
  `.agents/rules/docs-lifecycle.md`.
- Trunk-based git, `main` always green, commit per plan step, never push
  unasked: `.agents/rules/git.md`.
- Crates.io and local checkouts: egui/rerun under `~/Documents/code/rust/`
  and RoboCAD at `~/Documents/code/pet/cad/robocad` (the ancestor: viewport,
  `mass.rs`, snapshot harness, the `consume_key` shortcut lesson in its
  `CLAUDE.md`) are reference reading, never `path =` deps.
