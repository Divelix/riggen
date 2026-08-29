# ADR-0008: Export conventions — meshes baked to meters as STL, `fullinertia`, a headless CLI export

- Status: Accepted
- Date: 2026-08-29

## Context

M3 makes `riggen-export` real (ADR-0004 fixed the shape: one `resolve`,
dumb writers). Three conventions had to be chosen that the plan
(plans/m3-sim-ready OPEN 4, OPEN 5 and the `fullinertia` delta) argued out:

1. A document references mesh files in whatever unit and up-axis they came
   in, with `MeshAsset::scale` and `fix_up` turning them into document
   meters at load. An exported model has to reproduce that transform:
   either the writers copy the source file and write `scale` (URDF) /
   `scale` on `<mesh>` (MJCF) plus a rotation on every geom, or the
   exporter writes the transformed geometry.
2. MJCF inertials are usually `pos quat mass diaginertia` — the principal
   frame — which needs an eigen-decomposition in the writer and a
   quaternion of the principal axes, degenerate when two moments are
   equal. MuJoCo also accepts `fullinertia="Ixx Iyy Izz Ixy Ixz Iyz"` and
   does the decomposition itself.
3. CI must load the exported MJCF in MuJoCo (ADR-0004 §2). Something has
   to produce the files on a machine with no display: the app binary, or
   a second binary in `riggen-export`.

## Decision

1. **Meshes are written in meters as binary STL**, `scale` and `fix_up`
   baked into the vertices, one file per referenced `MeshId` under
   `meshes/<stem>.stl` (`<stem>_hull.stl` beside it for hulls). No `scale`
   attribute is ever written, in either format, and an OBJ source becomes
   an STL. The two writers see one `Arc<TriMesh>` per file, already in
   meters, and the mass properties the `<inertial>` was computed from are
   the same numbers the simulator's mesh has.
2. **`<inertial pos mass fullinertia>`** in MJCF. The document holds the
   tensor about the CoM in link axes; that is exactly what `fullinertia`
   takes, and MuJoCo's own decomposition is the one that will be used
   anyway. `riggen-core::inertial::check` still solves for the principal
   moments (triangle inequality), but the writer ships no eigenvectors.
3. **`riggen --export mjcf|urdf|both --out DIR INPUT`** on the app binary,
   returning before eframe starts. `riggen-app::cli` loads the document,
   builds the headless `MeshStore`, resolves, and writes; it needs no
   display, and CI's `mujoco` job runs it with `rust-cache`. A separate
   `riggen-export` binary is deferred until the measured job time says
   the app's link step is worth avoiding.

Every file — model and meshes — is written through a `.tmp` sibling and a
rename, like `file::save`.

## Consequences

- The export directory is self-contained: `<name>.xml`, `<name>.urdf`,
  `meshes/`. Moving it moves the robot; nothing points back at the
  document's source meshes.
- Mesh identity across a round trip is by stem, not by content: a URDF
  import registers the written STLs as new assets with their own hashes.
- Users who need the visual mesh in its original unit or format keep the
  source file; the export is a derived artefact, like a build output.
- The MJCF `<inertial>` is not human-checkable against a principal frame
  at a glance; the properties panel shows the principal moments instead.

## Alternatives considered

- **Copy sources and write `scale`** — MJCF's `<mesh scale>` cannot express
  `fix_up` (it has no rotation), so every geom would need a composed
  `quat` and the hull would be computed on the unscaled file; two paths
  for one transform.
- **`diaginertia` + principal `quat`** — needs eigenvectors, has a
  degenerate case, and buys nothing MuJoCo does not do itself.
- **A `riggen-export` binary** — saves CI the app's link time but adds a
  second CLI to keep in step; measured later (OPEN 5).
