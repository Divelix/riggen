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

**M0 done (2026-08-29, tag `m0`):** five-crate workspace with green CI,
`riggen-mesh` (TriMesh, STL/OBJ loaders, AABB, ray/triangle), the viewport
ported from `robocad-viewport` to glam (orbit camera, instance scene,
ID-buffer picking, whole-instance hover/select), an eframe shell that opens
files from the CLI, drag-and-drop and File › Open, and the `egui_kittest`
snapshot suite with `debug_state()`. `riggen-core` / `riggen-export` are
placeholders. **Next: M1** — document, tree, joints, FK (03-roadmap §M1);
mass properties from `robocad-kernel/src/mass.rs` come with M3.

## Rules that are not derivable from the code

- Lower crates never name upper crates' types; `riggen-core` and
  `riggen-export` never depend on egui or wgpu (the v0.2 SDK links them).
- One gesture = one command. Drags preview; release commits.
- Meters, radians, right-handed, Z-up, f64 in the document. Joint frame is
  the child link frame.
- Decisions go in `docs/adr/`; `⚠ OPEN:` in a doc marks a deferred one.
- Every UI change that can be seen gets a snapshot test (ADR-0003). When a
  snapshot changes, show the human the image.
- Update this file's "Current state" when a milestone lands; keep it under
  ~15 lines — the roadmap holds the detail.
- Backlog line → `/idea` (brainstorm, `docs/ideas/`) → `/plan` (todo,
  `docs/plans/`) → `/work` (one step, one commit) → `/retire-plan` (docs
  updated, plan deleted). Not every idea becomes a plan. Details:
  `.agents/rules/docs-lifecycle.md`.
- Trunk-based git, `main` always green, commit per plan step, never push
  unasked: `.agents/rules/git.md`.
- Crates.io and local checkouts: egui/rerun under `~/Documents/code/rust/`
  are reference reading, never `path =` deps.
