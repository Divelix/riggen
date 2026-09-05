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
— is open (2026-09-04).** Two halves: a foreign MJCF survives import →
edit → export with nothing lost (`Robot::actuators`, `<general>`, mimic
chains, `<joint ref>`, `<tendon>`, `.msh`, `<include>` / `<attach>` /
`<frame>`), and the window opens in a **View** mode — joint tree with
scrubbers, joints the only pick, wheel drives a hovered joint, a glyph
showing range and value, a visibility row, zen on `Z` — with Tab to
**Edit**. **Next:
`/idea` or `/plan` for a first line of either half;** each half's ⚠ OPEN
wants an ADR (composite joints) or an idea (the glyph) first. **Before it:** v0.3 (tag `v0.3.0`) paid down the
hand-feel debt — scrubbers, panels that stop hiding things, left-drag orbit
(ADR-0018), tool keys and a claimable wheel (ADR-0019), a depth-tested
overlay that marks a driven joint (ADR-0020). v0.2 (tag `v0.2.1`) made
"sim-ready" a feature — the SDK in the one wheel (ADR-0009), V-HACD
(ADR-0011), frames, mimics and actuators (ADR-0012/13/14), MJCF import
(ADR-0015), SDF export (ADR-0016), the web demo over one `FileSource` seam
(ADR-0017); `.riggen` is **schema 3**. Then M4 the wheel; M3 the writers,
URDF import, inertials, collision; M2 the mouse-only arm; M1 the document,
commands, history, `.riggen`; M0 the viewport.

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
