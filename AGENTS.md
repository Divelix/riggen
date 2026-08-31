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

**SDF export done (2026-09-01, ADR-0016):** `sdf.rs` is a third writer
over the same `ResolvedRobot` and no new field — SDF 1.11, links posed
`relative_to` their parent, native `<capsule>`, `<frame>` and
`<axis><mimic>`, only the actuator still a comment. `Format` is a set
(`mjcf|urdf|sdf|both|all`); the `sdf` CI job holds the file to libsdformat
itself at 1e-9. **MJCF import before it (ADR-0015):** an `.xml` opens by
every route a `.urdf` does, over the reading half of `xml.rs`; one import
vocabulary for both formats. **v0.2 before that:** actuator presets
(ADR-0014), mimic joints through the one `fk::resolve_q` (ADR-0013), named
frames (ADR-0012), V-HACD decomposition on `riggen-app::jobs` (ADR-0011),
and the `cp310-abi3` wheel that is both the app and `import riggen`
(ADR-0009). `.riggen` is **schema 3**, with the upgrade chain `load` walks.
**Before that:** M3 the writers from one `ResolvedRobot`, URDF import,
inertials, collision; M2 the mouse-only arm; M1 the document, commands,
history, `.riggen`.
**Next:** `/close-cycle` for v0.2 — every plan is retired and only the
conditional web-demo line is left in `docs/03-roadmap.md`.

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
