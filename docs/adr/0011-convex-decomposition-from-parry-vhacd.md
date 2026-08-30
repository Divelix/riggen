# ADR-0011: Convex decomposition from `parry3d-f64`'s V-HACD; the merge step is ours; the document stores parameters, not pieces

- Status: Accepted
- Date: 2026-08-30

## Context

Every simulator riggen exports to treats a collision mesh as convex: MuJoCo
takes the convex hull of a `<geom type="mesh">` itself, and URDF consumers
do the same. So a gripper finger, a C-bracket or a U-channel collides as a
solid block, and the only way to keep a concavity is to hand the simulator
several convex pieces. That is approximate convex decomposition, and
`CollisionPolicy::ConvexDecomposition` has been a variant with no
implementation — `ExportError::Unsupported` — since M1.

The idea that opened this (`docs/ideas/convex-decomposition.md`, now
absorbed) surveyed the field and concluded that **crates.io has no V-HACD
and no 3D approximate convex decomposition at all**, so option A was to
vendor the original C++ V-HACD header, build it with `cc`, gate the module
to native targets and drop it from the wasm build; option B was CoACD (also
C++, and GPL-adjacent licensing to check); option D was to ship
decomposition only in the SDK, as a separate `coacd` wheel; option E was to
skip the algorithm entirely and let the user place split planes by hand.

That constraint was wrong. `parry3d-f64` 0.30.2 (Dimforge, Apache-2.0)
has `transformation::vhacd::{VHACD, VHACDParameters}`: V-HACD, ported to
Rust, at `f64` because that is the `-f64` crate. It builds for
`wasm32-unknown-unknown`. Three things had to be checked before depending
on a physics engine for one mesh module:

1. **Cost.** parry is a collision-detection library; we want one function
   from it.
2. **A third glam.** parry 0.30's `glamx` bridge pins `glam 0.33.6`. The
   workspace is on 0.30 and `transform-gizmo` on 0.32 (ADR-0007).
3. **Whether it actually does the job.**

## Decision

**`parry3d-f64` at its default features, in `[workspace.dependencies]`, as a
dependency of `riggen-mesh` only, behind one module: `riggen_mesh::decomp`.**
No vendored C++, no `cc` build, no target gate, no second wheel.

1. **Cost, measured.** 15 new crates (`approx byteorder ena glamx hash32
   heapless num-complex num-derive parry3d-f64 robust rstar safe_arch simba
   spade wide`); the other 24 of its 39 dependencies were already in our
   446-package lock. **9.0 s** to build the whole new stack from clean with
   `[profile.dev.package."*"] opt-level = 3`. The wheel grows **162 253
   bytes, +1.5 %** (10 532 556 → 10 694 809, manylinux_2_34 x86_64, thin
   LTO, measured once `resolve` and the SDK actually reach `decompose` —
   before that the linker dropped nearly all of parry and the figure was a
   misleading +0.09 %). A pure-Rust dependency at that price is cheaper than
   ~1500 lines of vendored C++ that only builds on four of our five targets.
   `default-features = false` is not an option: it drops the crate's own
   `dim3`/`f64` selection and does not compile. The default set pulls
   neither `rayon` nor `serde`.

2. **The third glam is contained exactly as the second is** (ADR-0007).
   `decompose(&TriMesh, &DecompParams) -> Result<Vec<TriMesh>, DecompError>`
   takes and returns our types; the conversion is component-wise through
   `f64` at the call, in `decomp.rs`, which is the only file in the
   workspace that names `parry3d_f64`. No parry type and no glam-0.33 type
   appears in a signature anywhere else, so the lock file's three glams
   never meet. `decomp` is a *module*, not a crate, because the layer map
   already says geometry algorithms live in `riggen-mesh` and a pure-Rust
   dependency needs no target gate.

3. **It does the job, with one gap we fill.** `parry3d-f64` implements the
   split half of V-HACD and not the merge half. `do_compute_acd` splits a
   binary tree `2·2^ceil(log2(max_convex_hulls))` leaves deep and returns
   every leaf; its own comment describes the merge that would bring the
   count back to what was asked, and the function does not do it. So
   `max_convex_hulls` is a **recursion depth, not a ceiling**: measured, a
   *convex* cube comes back as **nine** pieces, and `max_convex_hulls: 1`
   gives four. Both are wrong for an exported model — a user who asks for
   four collision geoms must not get sixteen.

   **`decomp::merge` is therefore ours**: repeatedly join the pair of
   pieces whose common hull adds the least volume relative to the part's
   own, unconditionally while there are more pieces than `max_hulls`, and
   after that only while the join costs less than `concavity` — the same
   threshold that decided the splits, so a split that bought nothing is
   undone and a split across a real concavity is kept. The cube is one
   piece again and `max_hulls` means what it says. This is also why the
   boundary is `decompose` and not `VHACD`: the contract riggen offers is
   ours to keep whichever backend is behind it.

**The document stores the parameters and never the pieces.** ADR-0008 fixed
this for hulls — `<stem>_hull.stl` is written at export and computed from
the source mesh, never stored — and it extends unchanged:
`ConvexDecomposition { max_hulls, resolution, concavity }` is what a
`.riggen` file holds, and `<stem>_hull_0.stl … _N.stl` are derived at
export from `(MeshId, params)`. A decomposition is a pure function of a
mesh and three numbers; storing its output would be storing a build
artefact in the source, and would go stale the moment the mesh file changed
under its content hash.

## Consequences

- Decomposition is available everywhere the document is: the window, the
  CLI export, the SDK, and — since the dependency is pure Rust — the wasm
  build, which still compiles even though nothing exercises it in a browser.
- It is **not** cheap enough for a frame: seconds on a real part, dominated
  by O(`resolution`³) voxelization. The app computes it on a job thread
  (docs/01-architecture.md §Jobs and threads) keyed by `(MeshId, params)`;
  the CLI and the SDK run it inline, where a blocking second is expected.
- Three glam versions are in the lock file. Nothing of ours names two of
  them; `cargo tree -d` showing three glams is expected, not a warning.
- The pieces are convex hulls of a voxel partition, so they overlap a
  little and bulge past the surface by about a voxel. That is what a
  collision proxy is, and it is why `resolution` is a user-visible
  parameter rather than a constant.
- A better backend (CoACD, or parry gaining a real merge) is a change
  inside `decomp.rs`. Nothing above it names V-HACD.
- We now maintain ~60 lines of merge logic that a complete V-HACD would
  have given us. If parry implements it, ours is deleted and the ADR
  superseded.

## Alternatives considered

- **Vendored V-HACD C++ built with `cc`** (the idea's option A) — the
  option the wrong survey made look necessary. It costs a C++ toolchain on
  five build targets, a native-only module, an exclusion from the wasm
  build, and a vendored copy nobody upstreams to. A pure-Rust port of the
  same algorithm at `f64` costs 9 seconds of build time.
- **CoACD** (option B) — better decompositions than V-HACD on hard inputs,
  and also C++, also vendored, with licensing to check. Behind the
  `decompose` boundary it stays available as a later swap if V-HACD's
  quality is ever the complaint; today nothing has complained.
- **Decomposition in the SDK only, as a separate wheel** (option D) — two
  code paths for one policy, and the window, which is the product
  (ADR-0002), would be the half that cannot do it.
- **Manual split planes** (option E) — real work for the user on every
  part, and a document that stores geometry rather than parameters. Kept as
  a backlog line for the cases where an automatic split is wrong, not as
  the answer.
- **Calling `parry3d_f64::VHACD` from `riggen-export` directly** — would
  put parry's types and glam 0.33 in a second crate and leave the
  `max_hulls` gap unfixed at the only place that could fix it.
