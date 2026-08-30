# Idea: convex-decomposition

- Status: Open
- Raised: 2026-08-30
- Prompt (verbatim from the human): "/idea convex decomposition"

## Problem

A concave part — a gripper finger, a C-bracket, an L-shaped base, a fore
arm with a cut-out — gets one of three collision shapes today: its own
triangles (`SameAsVisual`, which MuJoCo turns into one convex hull
anyway), `ConvexHull` (the same hull, written explicitly), or hand-fitted
primitives. All three fill the concavity: the gripper cannot close on
anything, the bracket collides with what it wraps. Every RL user with a
gripper hits this on day one, and the current answer is "split the mesh
in a CAD tool" — the thing riggen exists to avoid. SEED.md §3 names
"MuJoCo needs convex collision geometry" as the problem and §4.3 lists
"convex hulls / decomposition" as a differentiator; §6 defers
decomposition to post-MVP, and that is where we are.

What exists: `CollisionPolicy::ConvexDecomposition { max_hulls }` in the
document (02 §Core types, schema v1), the properties panel shows it
read-only ("not supported yet"), `resolve` reports it as
`ExportError::Unsupported` (`riggen-export/src/resolve.rs:309`), and
`riggen-mesh::hull` (quickhull, 416 lines) is the only geometry algorithm
of that kind. The writers already handle *several* collision geoms per
link (a URDF import's `Meshes`), and ADR-0008 fixed how derived meshes
are written (`<stem>_hull.stl` beside the source) — so the export side is
plumbing, not design.

## Constraints it runs into

- **No Rust implementation exists.** crates.io has no V-HACD, no CoACD,
  no 3D approximate convex decomposition at all (searched 2026-08-30).
  Whatever we choose is either C++ behind a binding, a port, or a Python
  package. ADR-0001's dependency policy is "from crates.io"; a vendored
  C++ header is new ground and needs an ADR.
- **The wasm build check** (`ci.yml` `wasm`, ADR-0001) compiles
  `riggen-app` for `wasm32-unknown-unknown`. C++ through `cc` does not
  build there, so decomposition cannot live unconditionally in
  `riggen-mesh`: a separate crate or a cargo feature, native only.
- **The layer map** (01): geometry algorithms live in `riggen-mesh`;
  `riggen-export::resolve` consumes them; the app and the SDK both go
  through `resolve`. Anything that runs only in the SDK breaks "the same
  rules in the window and the script" (SEED §4.1) unless it is stored
  in the document as plain meshes.
- **No job thread** (01 §Jobs and threads; BACKLOG: "the export dialog
  re-resolves on every option change"). Hulls are milliseconds;
  decomposition is seconds to tens of seconds per part. The first
  decomposition is the first job that cannot run on the UI thread.
- **Five wheel targets** (ADR-0009): manylinux 2_28 (gcc-toolset), the
  aarch64 cross container, macOS clang, MSVC. A C++ dependency must
  compile on all five inside `release.yml`; the containers and runners
  do have the compilers, nothing in the tree uses them yet.
- **Schema v1** (02 §Schema): the variant exists with one field. More
  parameters (resolution, concavity threshold, hull vertex cap) are
  `#[serde(default)]` additions, no schema bump.
- Non-goals untouched: this is collision *geometry*, not collision
  checking (SEED §4 non-goals).

## Options

### A — V-HACD 4 through `cc`, as an export-time policy

V-HACD 4 is one header (`VHACD.h`, BSD-3, no dependencies, ~6k lines,
optional `std::thread`), the decomposition MuJoCo Menagerie and most RL
pipelines were built with before CoACD. A new crate `riggen-decomp`
(beside `riggen-mesh`, native only, excluded from the wasm build)
compiles it with `cc` behind a 40-line C shim: `decompose(&TriMesh,
&Params) -> Vec<TriMesh>`. `resolve` treats `ConvexDecomposition` like
`ConvexHull`: computed once per `MeshId`, cached, written as
`<stem>_hull_0.stl … _N.stl`, N collision geoms per link in MJCF and
URDF. The document keeps the *policy and its parameters*, never the
pieces — the same rule as hulls (ADR-0008). The window gains the policy
in the combo, a piece count and a "decompose" job with a spinner (the
first `riggen-app::jobs`), pieces drawn as translucent overlays like
hulls are today. The SDK gets `link.collision =
riggen.ConvexDecomposition(max_hulls=8)` for free through the schema.

Trade-offs: a C++ compiler joins the build on every target (present
everywhere already, but a new failure class in `release.yml`); V-HACD's
quality is a notch below CoACD on thin features (it voxelises). Cost:
~8 plan steps (crate + shim, params + schema defaults, resolve/export,
the job thread, the panel and overlay, the SDK spec, the five-target
build, ADR + docs). Forecloses nothing: CoACD could replace the shim
later behind the same `decompose`.

### B — CoACD through `cc`

State of the art (collision-aware, far better on thin and articulated
parts). But CoACD is a CMake project with `spdlog`, `cdt`, and OpenVDB
for its preprocessing path — OpenVDB drags in TBB and Boost. Vendoring
that into a `cc` build for five targets is a project of its own, and
the maintenance surface is the opposite of "lightweight". Cost: A's
steps plus 3–5 for the build alone, with real risk of never being green
on Windows. Only worth it if V-HACD's output is demonstrably not good
enough for users.

### C — a pure-Rust port of V-HACD

Keeps ADR-0001 clean (no C++), works in wasm, no shim. A ~6k-line
algorithm port with a correctness harness against the reference
implementation's output; 5–7 steps for the port alone before any of A's
integration steps, and a long tail of numerical differences to chase.
The payoff — no C++ compiler — buys little: the compilers are already
on every target. Not now; revisit if the shim becomes a maintenance
problem.

### D — SDK-only, CoACD from PyPI, pieces baked into the document

`pip install "riggen[decomp]"` pulls the `coacd` wheel (prebuilt for
the same five platforms); `link.decompose(out_dir, **params)` runs it in
Python, writes the pieces as STL assets and sets
`CollisionPolicy::Meshes(pieces)` — which already imports, exports and
draws (read-only) today. Zero C++ in our build, best-quality output,
2–3 steps. Trade-offs: the window cannot do it (it shows the result),
which contradicts SEED §4.1 and §4.3 — the sim-ready pipeline would
have a Python-only stage; the document references generated files the
user must keep beside the source; MuJoCo's own convex-hull step then
runs on our pieces, fine. Honest as a stopgap, wrong as the design.

### E — manual split planes in the window

No algorithm at all: the user clicks a plane on the part (the snapping
already finds faces and edges), the mesh is cut there, each side gets a
hull; a handful of cuts fixes a bracket or a finger. Fits "place it by
clicking" (SEED §5) and needs only a plane-cut-with-cap in
`riggen-mesh` (~2 steps) plus a tool and a `Split { planes }` policy.
Trade-offs: does not scale to a gripper's fifty concavities, and it is a
fourth policy to explain. A good *complement* to A for the two-cut case,
not a substitute.

### Do nothing

`ConvexHull` and primitives stay the answer; users split meshes
elsewhere or hand-fit primitives. The SEED §4.3 differentiator keeps
its hole, and every gripper user meets it. The variant, the panel line
and the `Unsupported` error keep advertising a feature that is not
there.

## Recommendation

**A.** It is the only option that keeps the window and the SDK equal,
keeps the document free of derived files, and lands with the tools we
have (a `cc` build, a job thread we owe anyway). V-HACD is what the
field shipped on for a decade; "good enough for MuJoCo" is a low bar it
clears. B loses on build weight, C on cost for no user-visible gain, D
on the window, E on scope. If, after A, users show parts where V-HACD's
voxelisation fails, B slots in behind the same `decompose` signature —
that is the point of the crate boundary.

What would change my mind: a Rust decomposition crate appearing (then
C for free), or `release.yml` failing to compile the header on one
target after an honest try (then D as the stopgap, with the variant
renamed to say what it is).

## Decision for the human

1. **Run it in Rust for both the window and the SDK (A), or SDK-only
   through the `coacd` wheel (D)?** Preferred: A.
2. **Export-time policy with parameters in the document, pieces
   recomputed and cached like hulls (A), or pieces baked as mesh assets
   (D's model)?** Preferred: policy — the document stays a description.
3. **Take the job thread now, as part of this?** Preferred: yes; the
   first decomposition is the first thing that must not freeze the
   window, and BACKLOG already asks for it.
4. **An ADR?** Yes — ADR-0010: a vendored C++ header behind `cc` (the
   first non-crates.io dependency, bending ADR-0001), native-only
   crate off the wasm build, V-HACD chosen over CoACD and why, and the
   "policy, not pieces" rule extended from ADR-0008.
5. **E (manual split planes) as a follow-up idea, or dropped?**
   Preferred: a backlog line, not part of this.
