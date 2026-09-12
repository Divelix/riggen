# Riggen

Native robot assembler for RL researchers: drop meshes, build the kinematic
tree, place joints, compute inertials and collision geometry, export MJCF and
URDF. Rust + egui/eframe + own wgpu viewport, shipped as a Python wheel.

Read in this order: `SEED.md` (charter, competition, stack), `README.md`
(what the user sees: install, first run, the CLI), `docs/ARCHITECTURE.md`, `docs/DATA-MODEL.md`, `docs/ROADMAP.md`,
`docs/adr/`, then the rules in `.agents/rules/*.md` (git, docs lifecycle) —
Claude Code loads them automatically via `.claude/rules`; any other agent
reads them here. Skills for the idea → plan → work → retire → close-cycle pipeline live in
`.agents/skills/` (same symlink arrangement).

## Setup (once per clone)

```sh
git config core.hooksPath .githooks   # fmt, clippy -D warnings, test before every commit
```

## Current state

**v0.5 — the viewport and the camera — is closed (2026-09-12); the human
tags `v0.5.0`.** Still one turntable (ADR-0028): the ViewCube aims it and
owns the projection readout, `W A S D E Q` walk its pivot into an
assembly, and the pivot is drawn while a gesture is live. Beside it a
ground grid at z = 0, a multisampled scene pass, a rotate drag that lands
a frame axis on the feature under it (ADR-0029), and in View a glyph that
is the band or the bars alone, opaque (ADR-0027).
**Next:** v0.6, the import gap's last mile — the 31 Menagerie models that
import and then refuse to *export*, a default material for an imported
link, a `PackageMap` UI, the two numbers `validate` skips, collision poses
under `MoveJointFrame`. Nothing planned yet; `/idea` for the first line.
**Before:** v0.4 (`v0.4.0`) the lossless MJCF round trip and the two
modes, schema 6, Menagerie's importing files 93 → 172 of 261 (ADR-0021 to
0026; composite joints, `<replicate>` and `<attach>` refused); v0.3
(`v0.3.0`) the hand-feel debt (ADR-0018/19/20); v0.2
(`v0.2.1`) sim-ready — SDK wheel, V-HACD, frames / mimics / actuators,
MJCF import, SDF export, the web demo (ADR-0009 to 0017); M4 the wheel;
M3 writers, URDF import, inertials, collision; M2 the mouse-only arm;
M1 the document and `.riggen`; M0 the viewport.

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
- MuJoCo Menagerie is cloned at `~/Documents/code/sim/mujoco_menagerie`
  (261 models with their meshes). Scan it to *size* an import decision —
  run the built binary over every `.xml` and bucket the outcomes, which is
  how the composition idea got its numbers — but it is never a CI
  dependency and never a committed fixture: what a scan finds gets
  concentrated by hand into `assets/fixtures/menagerie_style.xml` and the
  `menagerie_style_arm.xml` it includes, which is the corpus the `mujoco`
  job actually runs.
