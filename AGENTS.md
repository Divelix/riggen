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

**v0.4 — the round trip keeps what it read, and the window has two modes
— is open (2026-09-04).** The window half is done: **View** — joint tree
with scrubbers, joints the only pick, wheel drives a hovered joint, a
glyph for range and value, zen on `Z` — with Tab to **Edit** (ADR-0021).
A `<body>` with several `<joint>`s stays refused (**ADR-0022**). The file
half: `Robot::actuators` (ADR-0023), `.msh` / inline mesh geometry, the
escape hatch (ADR-0024) — `<general>` as a fourth `ActuatorSpec`, ranges
on the actuator at schema 5 — and couplings (**ADR-0025**, retired
2026-09-09) — mimic chains resolved by one topological pass, `<joint ref>`
as `Joint::qpos_ref`, `<tendon><fixed>` as `Robot::tendons` with its own
`ActuatorTarget` variant, schema 6; the corpus's `<equality>` and
`<tendon>` blocks now compared with the original's too, `grip` off the
drop list. **Next:** composition (`<include>` / `<attach>` / `<frame>`).
**Before it:** v0.3 (`v0.3.0`) paid down the hand-feel debt (ADR-0018/19/20);
v0.2 (`v0.2.1`) made "sim-ready" a feature — SDK wheel, V-HACD, frames /
mimics / actuators, MJCF import, SDF export, the web demo (ADR-0009 to
0017); M4 the wheel; M3 writers, URDF import, inertials, collision; M2
the mouse-only arm; M1 the document and `.riggen`; M0 the viewport.

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
