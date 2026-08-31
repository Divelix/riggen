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

**Mimic joints done (2026-08-31, ADR-0013):** `Joint::mimic` couples one
joint's `q` to another's; `fk::resolve_q` is the one implementation,
`validate` refuses chains and what MuJoCo would mis-simulate, and it writes
as a URDF `<mimic>` and an MJCF `<equality polycoef>` — checked against the
sampled `qpos` in CI — and imports back. **`.riggen` is schema 2**, with
the upgrade chain `load` walks.
**Named frames (ADR-0012):** `Robot::frames` live end to end — tree, glyph,
panel, gizmo, SDK — as an MJCF `<site>` and a URDF massless dummy link, one
namespace with the links. **Convex decomposition (ADR-0011):**
`parry3d-f64`'s V-HACD plus the merge step it omits, N geoms computed on
`riggen-app::jobs`. **v0.2 SDK (`v0.2.0`, ADR-0009):** one `cp310-abi3`
wheel gives the app *and* `import riggen`; `python/build_wheel.py` is the
recipe. **M4**: the wheel and `release.yml`. **M3**: MJCF + URDF from one
`ResolvedRobot`, MuJoCo-clean, URDF import, inertials, collision. **M2**:
the mouse-only arm. **M1**: the document, commands, history, `.riggen`.
**Next:** the remaining v0.2 lines of `docs/03-roadmap.md` (actuator
presets, MJCF import).

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
  updated, plan deleted); `/close-cycle` at a roadmap boundary. Not every
  idea becomes a plan. Details: `.agents/rules/docs-lifecycle.md`.
- Trunk-based git, `main` always green, commit per plan step, never push
  unasked: `.agents/rules/git.md`.
- Crates.io and local checkouts: egui/rerun under `~/Documents/code/rust/`
  and RoboCAD at `~/Documents/code/pet/cad/robocad` (the ancestor: viewport,
  `mass.rs`, snapshot harness, the `consume_key` shortcut lesson in its
  `CLAUDE.md`) are reference reading, never `path =` deps.
