# Plan: convex-decomposition

- Started: 2026-08-30
- Milestone: v0.2
- Idea: `docs/ideas/convex-decomposition.md` (absorbed, with one correction)
- Idea (verbatim from the human): "/idea convex decomposition"

**Correction to the idea, 2026-08-30.** The idea's central constraint —
"crates.io has no V-HACD, no 3D approximate convex decomposition at all" —
is wrong. `parry3d-f64` 0.30.2 (Dimforge, Apache-2.0, 2026-08-07) has
`transformation::vhacd::{VHACD, VHACDParameters}`: the same algorithm, in
pure Rust, at f64. Option A's vendored C++ header, its `cc` build, the
native-only crate, the wasm exclusion and the five-target build risk are
therefore all unnecessary, and this plan does none of them. What survives
of A is its shape: an export-time policy whose parameters live in the
document, pieces recomputed and cached, computed off the UI thread.

## Goal

A concave part — a gripper finger, a C-bracket, an L-base — gets collision
geometry that keeps its concavity, from the window and from the SDK, with
the same rules in both. `CollisionPolicy::ConvexDecomposition { max_hulls,
resolution, concavity }` stops being an `ExportError::Unsupported` and
becomes a real policy: `riggen_mesh::decompose` wraps `parry3d-f64`'s
pure-Rust V-HACD beside the quickhull we already have, `resolve` turns it into N
convex pieces written as `<stem>_hull_0.stl … _N.stl` and N collision geoms
per link in both MJCF and URDF, the document keeps the *parameters and never
the pieces* (ADR-0008's rule for hulls, extended), the pieces are computed
once per `(MeshId, params)` on the app's first job thread
(`riggen-app::jobs`) so the window never freezes, the properties panel
offers the policy with editable parameters and a piece count, View ›
Collision geometry draws every piece, and `link.collision =
riggen.ConvexDecomposition(max_hulls=8)` works from Python. The `mujoco` CI
job loads a decomposed model with zero warnings.

## Non-goals

- **CoACD** (idea option B) and **any C++ dependency at all** (option A's
  `cc` build). The function boundary is `decompose(&TriMesh, &DecompParams)
  -> Vec<TriMesh>`, so a better backend can replace parry later behind it.
- **An SDK-only `coacd` wheel** (option D).
- **Manual split planes** (option E) — a backlog line, written in step 2.
- **Async mesh loading and an async export dialog.** `jobs` arrives here for
  decomposition only; the backlog lines "Async mesh loading via `jobs`" and
  "the export dialog re-resolves on every option change" stay open.
- Per-geom collision editing of `Meshes` (M3 exit gate), oriented primitive
  fits, collision *checking*.
- Any schema version bump: the new parameters are `#[serde(default)]`
  additions to a variant that is already in v1.
- A **web UI** for it. The dependency is pure Rust so the wasm build check
  stays green and `decompose` compiles there, but nothing in this plan is
  exercised in a browser.

## Design deltas

- **`riggen-mesh::decomp`** (01 §Layer map), a module beside `hull`:
  `decompose(&TriMesh, &DecompParams) -> Result<Vec<TriMesh>, DecompError>`
  over `parry3d-f64`'s `VHACD`. No new crate and no target gate — the layer
  map already says geometry algorithms live in `riggen-mesh`, and a pure-Rust
  dependency needs neither.
- **`parry3d-f64` in `[workspace.dependencies]`** at its default features
  (`default-features = false` does not compile — the crate needs its own
  `dim3`/`f64` defaults; the default set pulls no `rayon` or `serde`
  anyway). Its `glamx` bridge pins **glam 0.33.6**,
  so the lock file holds a third glam beside our 0.30 and transform-gizmo's
  0.32. They never meet: `decompose` takes and returns our `TriMesh`, converts
  through plain `[f64; 3]` at the call, and no parry or glam-0.33 type appears
  in a signature — the ADR-0007 containment, applied again.
- **ADR-0011** (step 2): `parry3d-f64` for the algorithm — why a physics
  library is an acceptable source for one mesh module, the third glam and how
  it is contained, why not vendored V-HACD C++ or CoACD (the idea's option A
  and B, and the search that made A look necessary), and "the document stores
  the policy, never the pieces", extended from ADR-0008.
- **`riggen-core` (02 §Core types)**: the variant gains two fields,
  `ConvexDecomposition { max_hulls: u32, resolution: u32, concavity: f64 }`,
  each `#[serde(default = …)]`, so every existing `.riggen` v1 file reads
  unchanged. No `schema_version` change (02 §Schema).
- **`riggen-export::resolve` (02 §`ResolvedRobot`)**: a `DecompSource` trait
  beside `MeshLookup` with two implementations — `ComputeNow` (the CLI, the
  SDK, the tests: runs V-HACD inline) and the app's cache-only source, which
  reports `ExportError::DecompositionPending` for an entry the job thread
  has not delivered. Pieces are keyed `(MeshId, params)` and written
  `<stem>_hull_0.stl … _N.stl`; `ExportError::Unsupported` for the variant
  disappears and `DegenerateDecomposition` takes its place in the list.
- **MJCF and URDF writers**: no change — both already write several
  collision geoms per link (a URDF import's `Meshes`). The exported model
  gains one `<geom>` / `<collision>` per piece (02 §Export mapping table).
- **`riggen-app::jobs`** (01 §Jobs and threads, rewritten): RoboCAD's
  `EvalExecutor` shape as that section already prescribes — a `std::thread`,
  an `mpsc` request/result pair, `wake` bound to `ctx.request_repaint()`,
  results drained once per frame, inline on wasm. One job kind for now:
  `Decompose { mesh: MeshId, params }`.
- **`RiggenApp`** (01 §The document is the only state): `jobs: Jobs` and
  `decomp: HashMap<(MeshId, DecompParams), DecompState>` join the derived,
  never-saved state; `CollisionSource` gains a `Piece(MeshId, DecompParams,
  usize)` variant so `sync_collision` draws each piece translucently.
- **Properties › Collision**: `Decomposition` moves into
  `CollisionMode::OFFERED`, with `max_hulls` / `resolution` / `concavity`
  fields, a "pieces: N" readout and a spinner while the job runs. Every
  commit stays one `SetCollision`.
- **The SDK** (01 §Python SDK): `riggen.ConvexDecomposition` beside the
  string policies, through the same `{"ConvexDecomposition": {...}}` document
  JSON `_riggen.set_collision` already takes — no new `Command` method.
- **Fixtures**: `assets/fixtures/bracket.stl`, a U-shaped part with one
  concavity a hull fills, written by the `#[ignore]`d generator test beside
  the arm fixtures; the sample arm's `arm.riggen` gains nothing, but the
  `mujoco` CI job gets a second model built on it.

## Steps

- [x] Step 1 — `riggen_mesh::decompose` over `parry3d-f64`'s `VHACD`:
  `DecompParams` (`max_hulls`, `resolution`, `concavity`), the `TriMesh`
  conversion in and out, `DecompError`. `assets/fixtures/bracket.stl` and
  its `#[ignore]`d generator beside the arm's. Test: the bracket decomposes
  into >1 piece, every piece is convex, and a point in the notch is outside
  every piece while being inside the convex hull — the property the whole
  plan exists for. Also: `cargo build -p riggen-app --target
  wasm32-unknown-unknown` still green, and the built wheel's size before and
  after recorded (the one number OPEN 1 leaves open).
- [ ] Step 2 — **ADR-0011**, written knowing step 1's answer; 01 §Layer map
  and §Cargo workspace note the dependency and the third glam; a backlog
  line for the idea's option E (manual split planes).
- [ ] Step 3 — the parameters in `riggen-core`: `max_hulls`, `resolution`,
  `concavity` with serde defaults, mirroring `riggen_mesh::DecompParams`
  (core does not re-export it — the document type stays plain serde data).
  Test: `assets/fixtures/pendulum.riggen` and a hand-written v1 file
  carrying `{"ConvexDecomposition":{"max_hulls":4}}` both round-trip. 02
  §Core types updated in the same commit.
- [ ] Step 4 — `resolve`: the `DecompSource` trait, `ComputeNow`, the
  `(MeshId, params)` cache, `<stem>_hull_N.stl`, `DecompositionPending` and
  `DegenerateDecomposition`; `ExportError::Unsupported` for the variant is
  deleted. Test: a robot whose link is `ConvexDecomposition` resolves to
  N>1 `ResolvedGeom::Mesh`, the written directory holds
  `bracket_hull_0.stl…`, and the URDF round-trips through `urdf-rs`.
- [ ] Step 5 — `riggen-app::jobs`: the thread, the channel, the repaint
  wake, the once-per-frame drain, `Jobs::request` deduplicating in-flight
  keys. Test: a harness test requests a decomposition and pumps frames until
  the cache holds it, asserting on the job's own result rather than a sleep.
- [ ] Step 6 — the properties panel: the policy offered, its three fields,
  the piece count and the spinner; `CollisionSource::Piece` and
  `sync_collision` drawing every piece. Snapshots
  `properties_collision_decomposition` (the panel, pieces ready) and
  `collision_decomposition` (the bracket's pieces in the viewport). Defaults
  for the three parameters are chosen here from measurements on the bracket
  and the arm's `fore.stl` (OPEN 2).
- [ ] Step 7 — the SDK: `riggen.ConvexDecomposition` dataclass, `__init__`
  export, `_riggen.pyi`, `robot.py`'s `collision` getter/setter round-trip;
  a pytest in `python/tests/sdk/` asserting the policy survives
  `to_json`/`load` and that `export` writes N mesh files.
- [ ] Step 8 — the acceptance: a decomposed model in the `mujoco` CI job —
  the bracket as a link of a two-link robot, exported headlessly via
  `riggen --export mjcf`, loaded by `mujoco.MjModel.from_xml_path` with zero
  compiler warnings, `mj_forward` agreeing with `fk` to 1e-6, and the model
  reporting more than one `geom` on the decomposed body.

## Acceptance

`cargo test --workspace` green (including step 1's notch-point property and
step 6's two snapshots), the `wasm` job still green, and the `mujoco` CI job
loading the decomposed model with zero MuJoCo compiler warnings and
`ngeom > 1` on the bracket body with `mj_forward` matching `fk` to 1e-6.

## Docs to update on completion

- `docs/01-architecture.md` §Layer map — `decomp` in `riggen-mesh`'s box;
  §Cargo workspace — `parry3d-f64` in the dependency-policy paragraph,
  beside the ADR-0007 note about the two glam versions (now three).
- `docs/01-architecture.md` §The document is the only state — `jobs` and the
  decomposition cache in the `RiggenApp` listing; `sync_collision`'s
  paragraph gains the pieces.
- `docs/01-architecture.md` §Jobs and threads — rewritten: the thread now
  exists, what runs on it, what still does not.
- `docs/01-architecture.md` §Python SDK — `ConvexDecomposition` in the API list.
- `docs/02-data-model.md` §Core types — the variant's three fields, "post-MVP"
  gone; §Schema — the `#[serde(default)]` note, still v1; §`ResolvedRobot`
  and the export mapping table — `<stem>_hull_N.stl`, N geoms per link;
  the `ExportError` list — `DecompositionPending` and
  `DegenerateDecomposition` in, `Unsupported` out.
- `docs/adr/0011-*.md` — written as step 2, listed here so retirement checks it.
- `docs/03-roadmap.md` v0.2 — "Convex decomposition" moves from "Still open"
  to a dated done line citing ADR-0011.
- `docs/BACKLOG.md` — the manual-split-planes line added (step 2); nothing
  removed (decomposition is a roadmap line, not a backlog line).
- `AGENTS.md` current state — one clause: decomposition landed, the first
  job thread with it.
- `README.md` — the collision-policy sentence, if it enumerates them.

## Open questions

- **Finding, step 1: `parry3d-f64` has no merge step, so `max_convex_hulls`
  is a recursion depth and not a ceiling.** `do_compute_acd` splits a
  binary tree `2·2^ceil(log2(max_convex_hulls))` leaves deep and returns
  every leaf; its own comment describes the merge that would bring the
  count back to what was asked and parry does not implement it. Measured:
  a *convex* cube comes back as **nine** pieces, and `max_hulls: 1` gives
  four. Both are wrong for an exported model — the user asks for four
  collision geoms and gets sixteen — so `decomp::merge` is ours: repeatedly
  join the pair whose common hull adds the least volume, unconditionally
  while there are more pieces than `max_hulls` and after that only while it
  costs less than `concavity`. The cube is one piece again, the bracket
  keeps its notch, and `max_hulls` means what it says. **ADR-0011 (step 2)
  records this**: it is the one thing we implement that the dependency was
  supposed to bring, and it is why the boundary is `decompose`, not
  `VHACD` itself.

- `⚠ OPEN 1:` ~~what `parry3d-f64` costs to compile~~ — **measured
  2026-08-30, closed.** 15 new crates (`approx byteorder ena glamx hash32
  heapless num-complex num-derive parry3d-f64 robust rstar safe_arch simba
  spade wide`); the other 24 of its 39 deps are already in our 446-package
  lock. **9.0 s** for the whole stack from clean with deps at `opt-level =
  3`, and it builds for `wasm32-unknown-unknown`. No cargo feature gate:
  the dependency is unconditional. Its `glamx` 0.3 pins **glam 0.33.6**, so
  the lock holds three glam versions (0.30.10, 0.32.1, 0.33.6) — contained
  as ADR-0007 contains the second, and ADR-0011 records it. The **wheel** grows
  **10 532 556 → 10 541 521 bytes, +8 965 (+0.09 %)** (step 1,
  `python/build_wheel.py`, manylinux_2_34 x86_64) — negligible, and OPEN 1
  is closed. The caveat: nothing *calls* `decompose` yet, so thin LTO drops
  most of parry from that build; step 8 re-measures once `resolve` and the
  SDK reach it, and it changes nothing unless it is large.
- `⚠ OPEN 2:` **the default parameters** the combo starts with. V-HACD's own
  defaults are generous (tens of hulls at a high voxel resolution, seconds
  per part). *Agent decides* by step 6, measured on `bracket.stl` and
  `fore.stl`, favouring "a second, not a minute" over piece count.
- `⚠ OPEN 3:` ~~exporting while a job is in flight~~ — **decided by the
  human 2026-08-30: block the export.** `ExportError::DecompositionPending`
  is listed in the export dialog beside every other blocker, the way an
  unloadable mesh or a bad inertial already is; the line clears itself when
  the job lands and the user presses the button. No modal, no spinner over
  the dialog.
