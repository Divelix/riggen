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

**v0.4 — the round trip keeps what it read, and the window has two modes
— is closed (2026-09-10); the human tags `v0.4.0`.** An MJCF import is
lossless now: actuators as a model-level table (ADR-0023), the `<general>`
escape hatch (ADR-0024), couplings (ADR-0025) — mimic chains, `<joint ref>`
as `Joint::qpos_ref`, `<tendon><fixed>` as `Robot::tendons` — `.msh` and
inline mesh geometry, and composition (ADR-0026): `mjcf_compose` splices
every `<include>` and folds every `<frame>` away before the reader looks,
never storing either, so Menagerie's importing files go 93 → 172 of 261.
Schema 6. Composite joints, `<replicate>` and `<attach>` stay refused
(ADR-0022, ADR-0026 §4). The window is two modes (ADR-0021): **View** —
joint tree with scrubbers, joints the only pick, the wheel driving a
hovered joint, a glyph for range and value — Tab to **Edit** at the zero
configuration, zen on `Z`, a visibility row top-right. **Next:** v0.5, the
viewport and the camera — ViewCube, fly camera, ground grid, MSAA,
rotate-drag snapping. **Before:** v0.3 (`v0.3.0`) the hand-feel debt
(ADR-0018/19/20); v0.2 (`v0.2.1`) "sim-ready" as a feature — SDK wheel,
V-HACD, frames / mimics / actuators, MJCF import, SDF export, the web demo
(ADR-0009 to 0017); M4 the wheel; M3 writers, URDF import, inertials,
collision; M2 the mouse-only arm; M1 the document and `.riggen`; M0 the
viewport.

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
