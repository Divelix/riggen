# Riggen

Native robot assembler for RL researchers: drop meshes, build the kinematic
tree, place joints, compute inertials and collision geometry, export MJCF and
URDF. Rust + egui/eframe + own wgpu viewport, shipped as a Python wheel.

Read in this order: `SEED.md` (charter, competition, stack),
`docs/01-architecture.md`, `docs/02-data-model.md`, `docs/03-roadmap.md`,
`docs/adr/`, then the rules in `.agents/rules/*.md` (git, docs lifecycle) —
Claude Code loads them automatically via `.claude/rules`; any other agent
reads them here. Skills for the idea → plan → work → retire pipeline live in
`.agents/skills/` (same symlink arrangement).

## Setup (once per clone)

```sh
git config core.hooksPath .githooks   # fmt, clippy -D warnings, test before every commit
```

## Current state

**M3 done (2026-08-29, tag `m3`):** the arm exports as MJCF and URDF from
one `ResolvedRobot` (ADR-0004; ADR-0008: meshes baked to meters as STL,
`fullinertia`, headless `riggen --export`); MuJoCo loads it with zero
warnings and matches `fk` (`python/tests/test_mjcf_load.py`, the `mujoco`
CI job); `urdf-rs` round-trips it. Inertials: `riggen_mesh::mass_properties`
through `riggen_core::inertial` (three `InertialSpec` modes, `check`).
Collision: hull, fitted primitives or imported `Meshes`, drawn translucent.
Properties has Inertial and Collision blocks; File has Import URDF… and
Export… (a modal listing every `ExportError`). `arm.riggen` / `arm.urdf`
are the corpus. M2: the mouse-only arm (toolbar, gizmo, glyphs, snapping).
M1: the `Robot` document, commands, history, `.riggen` v1, panels. M0:
mesh, viewport, the `egui_kittest` suite (`visual-debug` skill).
**Next: M4** — README, screencast, the wheel (ADR-0002).

## Rules that are not derivable from the code

- Lower crates never name upper crates' types; `riggen-core` and
  `riggen-export` never depend on egui or wgpu (the v0.2 SDK links them).
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
