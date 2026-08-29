# Plan: m3-sim-ready

- Started: 2026-08-29
- Milestone: M3
- Idea (verbatim from the human): "plan M3. If you need some example MJCF,
  probably you can find it in ~/Documents/code/py/mjlab-pg"

## Goal

The M2 arm (`assets/fixtures/arm/`) exports as MJCF and URDF from one
`ResolvedRobot`; `mujoco.MjModel.from_xml_path` loads the MJCF with zero
compiler warnings and `mj_forward` body poses match `riggen_core::fk` to
1e-6 at several joint configurations; the URDF round-trips through `urdf-rs`
with an independent FK agreeing with ours. Every link's inertial is computed
from its closed meshes at the material density (or overridden / hybrid), the
sanity checks that MuJoCo fails silently on (mass > 0, positive-definite,
triangle inequality, closed mesh) block export in a dialog that says why,
and collision geometry — same as visual, one convex hull per geom, or fitted
box / cylinder / sphere / capsule primitives — is chosen per link and shown
translucent in the viewport. A Menagerie-style URDF opens from the command
line (`riggen robot.urdf`) or File › Import, and re-exports as an MJCF that
loads too. `riggen --export` does all of it headlessly, which is what CI's
Python job runs.

## Non-goals

- Convex decomposition, actuators (effort / velocity are written as an XML
  comment, ADR-0004 §4), MJCF import, named frames / sites, mimic joints,
  SDF. All stay on the backlog / roadmap.
- Async jobs: hulls and fits run synchronously on the UI thread and are
  cached per `MeshId` beside the loaded mesh. `riggen-app::jobs` moves to
  the backlog with the first mesh that makes it hurt (01 §Jobs and threads
  changes accordingly).
- PCA / oriented bounding boxes: primitive fits start from the AABB in the
  link frame and the user moves them; an OBB fit is a backlog line.
- The README, screencast and wheel — M4. M3 only provides the sample robot
  they will use.
- Physics, contact tuning (`friction`, `solref`, `condim`): the exported
  MJCF writes MuJoCo's defaults for everything the document does not hold.
- The Python SDK (v0.2). The Python in this plan is one test file.

## Design deltas

**`riggen-mesh`** gains three modules and two generators:

- `mass::mass_properties(mesh, density) -> MassProps { volume, mass, com,
  inertia: DMat3 }` — the signed-tetrahedra port of RoboCAD's
  `mass.rs`, tensor about `com` in mesh axes. `MassProps::is_closed` is
  answered by `feature::adjacency(mesh).is_closed()` (RoboCAD's
  independent-volume cross-check had a second volume from truck; we have
  the topology instead). A negative signed volume means inward winding and
  is folded (`abs`) with a flag, not treated as an error.
- `hull::convex_hull(points) -> TriMesh` — quickhull, own implementation
  (no new dependency), outward-wound, degenerate input (coplanar,
  collinear) reported as `MeshError::DegenerateHull`.
- `fit::{box_fit, cylinder_fit, sphere_fit, capsule_fit}(points) ->
  Primitive`-shaped results in the point cloud's own frame: box = AABB;
  sphere = AABB centre + max distance; cylinder / capsule axis along the
  longest AABB extent, radius from the max radial distance about it.
- `TriMesh::sphere(radius, segments)` and `TriMesh::capsule(radius,
  length, segments)` beside `cube` / `cylinder` / `tube`, for the
  collision view and the fit tests.

**`riggen-core`**:

- `inertial::compose_inertial(link, &impl MeshLookup, &materials) ->
  Result<Inertial, InertialError>` where `MeshLookup` is a trait (`fn
  mesh(&self, MeshId) -> Option<&TriMesh>`) the app's mesh store and the
  export CLI both implement — core still stores no geometry. Per geom:
  `mass_properties` at the link's density (material, else
  `density_override`, else error), tensor rotated into the link frame and
  parallel-axis shifted by the geom pose, summed; then the `InertialSpec`
  mode. `Inertial { mass, com, inertia }` is the value every consumer
  reads; `computed` is kept beside it for the comparison readout.
- `inertial::check(&Inertial) -> Vec<InertialError>`: mass > 0, symmetric,
  positive-definite, triangle inequality on the principal moments (a Jacobi
  eigen-solve for a symmetric 3×3, ~40 lines, tested against diagonal and
  rotated-diagonal tensors). `InertialError::OpenMesh { geom }` when
  `Computed` / `Hybrid` meets an open mesh.
- `CollisionPolicy::Meshes(Vec<Geom>)` — collision meshes that are not the
  visuals, in link frame, so a URDF `<collision><mesh>` imports losslessly
  (02 §URDF import's `⚠ OPEN`, resolved by OPEN 1). Adding an enum variant does not
  bump the schema: v1 files without it still read.
- `SetRoot` stays refused across a movable joint (OPEN 2).

**`riggen-export`** stops being a placeholder:

- `resolve(&Robot, &impl MeshLookup) -> Result<ResolvedRobot,
  Vec<ExportError>>` per 02 §`ResolvedRobot`; `ResolvedGeom` is `Mesh {
  name, mesh: Arc<TriMesh>, pose }` or `Primitive(Primitive)`. Hulls are
  computed here, per referenced `MeshId`, once. `ExportError` wraps
  `ValidationError`, `InertialError { link, .. }`, `ZeroMassMovableLink`
  (MuJoCo refuses a moving body with no mass; an empty static body is
  fine and gets no `<inertial>`), `UnloadableMesh`.
- **Meshes are written in meters as binary STL**, scale and `fix_up` baked,
  one file per referenced `MeshId` (`<stem>.stl`) plus `<stem>_hull.stl`
  for hulls; no `scale` attribute is ever written and OBJ sources become
  STL. Rationale and the two other export conventions below go in
  **ADR-0008** (step 4).
- `mjcf::write(&ResolvedRobot, &ExportOptions) -> String`: `<compiler
  angle="radian" meshdir="meshes" autolimits="true"/>`, `<default
  class="visual">` / `<default class="collision">` as the unirobot example
  does (visual `contype=0 conaffinity=0 group=2`; collision `group=3`,
  translucent rgba), one `<asset><mesh>` per file, nested `<body pos
  quat>`, `<joint type="hinge|slide" axis range>`, **`<inertial pos mass
  fullinertia>`** — MuJoCo does the principal-axes decomposition itself,
  which is one eigen-solver we do not have to ship in the writer (02
  §Format mapping's "diaginertia + quat" row changes). Optional
  `<freejoint/>` on the root (OPEN 3). Effort / velocity per ADR-0004 §4 as
  a comment. Quaternion `w x y z` through one tested helper. No XML
  crate: a 30-line escaping writer, since the output is fixed-shape.
- `urdf::write(&ResolvedRobot, &ExportOptions) -> String`; `<mesh
  filename>` in the chosen `MeshPathStyle { Relative, Package(String),
  Absolute }`. MJCF ignores the style (it has `meshdir`).
- `export(&ResolvedRobot, &ExportOptions, dir) -> Result<Vec<PathBuf>>`
  writes `<name>.xml` / `<name>.urdf` and `meshes/`, through `.tmp` +
  rename per file like `file::save`.
- `urdf_in::load(path, &PackageMap) -> Result<(Robot, Vec<ImportWarning>),
  ImportError>` over `urdf-rs` 0.9: links / joints direct, `<inertial>` →
  `Override`, `<mesh scale>` → `MeshAsset::scale` (uniform only; a
  non-uniform scale is a warning and the largest component), `<collision>`
  mesh → `CollisionPolicy::Meshes`, primitives → `Primitives`, `<mimic>`
  and `<safety_controller>` dropped with a warning, `package://` resolved
  through the map else the file's directory.
- `riggen-export` depends on `riggen-mesh`, `riggen-core`, `urdf-rs`; a
  `Cargo.toml` comment says why each.

**`riggen-viewport`**: a per-instance `RenderGroup { Opaque, Translucent
}`; translucent instances draw after every opaque one, alpha-blended,
depth-tested without depth write, and are skipped by the pick pass.

**`riggen-app`**: `--export mjcf|urdf|both --out DIR INPUT` on the CLI
returns before eframe starts (`INPUT` is `.riggen` or `.urdf`; `--fk-samples`
also writes `<name>.fk.json`, the poses the Python test compares against);
`load_files` accepts `.urdf`; File › Import URDF… and File › Export…;
View › Collision geometry; the properties panel's link section grows an
Inertial and a Collision block; `sync_scene` keeps a second instance map
`collision_instances: BTreeMap<(LinkId, usize), InstanceId>`.

**CI**: a `mujoco` job (`python/tests/test_mjcf_load.py`, `pip install
mujoco numpy`, then `cargo run -p riggen-app -- --export mjcf
--fk-samples --out target/sample assets/fixtures/arm/arm.riggen` and the
same for `arm.urdf`).

**Docs**: 01 §Layer map, §Cargo workspace, §The document is the only
state, §Panels and menus, §Frame loop, §Jobs and threads, §File format,
§Testing; 02 §Inertials, §`ResolvedRobot`, §Format mapping, §URDF import;
03 M3 status line; ADR-0008; AGENTS.md; BACKLOG (the `is_closed` line is
absorbed here; the `SetRoot` line is decided by OPEN 2).

## Steps

Ordered so the milestone's risk — "MuJoCo loads it with zero warnings and
agrees with our FK" — is retired by step 5, before any UI exists.

- [x] Step 1 — `riggen-mesh::mass`: port `mass.rs` to `TriMesh` / glam,
  `MassProps`, inward-winding fold, closedness from `Adjacency`. Tests:
  cube and cylinder against the analytic tensors, a translated cube's CoM
  moves and its tensor about the CoM does not, `TriMesh::cube` minus a face
  reports open. Remove the `is_closed` backlog line.
- [x] Step 2 — `riggen-core::inertial`: `MeshLookup`, `compose_inertial`
  with the three `InertialSpec` modes, `check` with the Jacobi
  eigen-solve. Tests: two cubes in one link equal one box's tensor via the
  parallel-axis theorem, a rotated geom's tensor is the rotated tensor,
  `Hybrid` scales mass and tensor together, `Override` passes through, an
  open mesh under `Computed` is `OpenMesh`, a flat plate's tensor fails
  the triangle inequality only after a bad `Override`.
- [ ] Step 3 — `riggen-export::resolve`: `ResolvedRobot`, `ExportError`,
  topological link order, `None` / `SameAsVisual` / `Primitives` /
  `Meshes` collision resolution (hull deferred to step 7), the zero-mass
  movable link rule. Tests on the pendulum fixture and hand-built robots:
  order, each error, an empty static root resolves with no inertial.
- [ ] Step 4 — `riggen-export::mjcf` + `export` (meshes baked to meters as
  STL) + the quaternion helper + **ADR-0008** (meters-STL meshes,
  `fullinertia`, headless CLI export) + `riggen --export` on the CLI.
  Tests: quaternion order, XML escaping, a golden MJCF string for a
  two-link robot with every joint kind, the written STL's AABB is the
  scaled one, `--export` on `pendulum.riggen` produces the files.
- [ ] Step 5 — Sample robot + MuJoCo load test: an `#[ignore]`d generator
  test (beside `write_arm_fixtures`) builds the arm from `ARM_DESIGN`
  with materials and saves `assets/fixtures/arm/arm.riggen`;
  `--fk-samples` writes poses at five joint configurations;
  `python/tests/test_mjcf_load.py` loads the MJCF with a
  `set_mju_user_warning` hook that fails on any warning and compares
  `data.xpos` / `data.xquat` against the samples to 1e-6; `.github/
  workflows/ci.yml` gains the `mujoco` job. Locally: `uv run --with mujoco
  --with numpy python python/tests/test_mjcf_load.py`. Also a
  `sample_arm_opens` corpus test in core. **This is the milestone's risk;
  stop and report if MuJoCo warns.**
- [ ] Step 6 — `riggen-export::urdf` writer + the round-trip FK test:
  export the sample arm as URDF, parse with `urdf-rs` (dev-dependency
  here, real dependency from step 13), compute FK **independently** from
  the parsed `xyz rpy axis` with glam alone, compare end-effector poses on
  a 5³ joint grid against `riggen_core::fk` to 1e-9. Golden URDF string
  for the two-link robot; every `MeshPathStyle`.
- [ ] Step 7 — `riggen-mesh::hull` (quickhull) + `CollisionPolicy::
  ConvexHull` in `resolve` (one `<stem>_hull.stl` per referenced mesh, the
  hull cached per `MeshId` in the resolver). Tests: hull of a cube with
  interior points has 8 vertices and 12 triangles, every input point is
  inside or on the hull within 1e-9, hull volume ≥ mesh volume for the
  arm parts, outward winding via positive signed volume, degenerate input
  errors.
- [ ] Step 8 — `riggen-mesh::fit` + `TriMesh::sphere` / `capsule` +
  `Primitives` written by both writers (MJCF `type="box|cylinder|sphere|
  capsule" size pos quat`, URDF `<box size>` / `<cylinder radius length>`
  / `<sphere radius>`; a capsule in URDF becomes a cylinder plus a
  warning). Tests: each fit of its own generator mesh returns the
  generator's numbers; MJCF `size` is half-extents / (radius,
  half-length) — pinned in a test because it is the classic mistake.
- [ ] Step 9 — Viewport `RenderGroup::Translucent` + app collision
  instances from the policy (`SameAsVisual` draws nothing extra; hulls and
  primitives draw as translucent instances) + View › Collision geometry
  toggle, off by default, remembered through eframe storage. Snapshots:
  `collision_hull`, `collision_primitives`; a pick over a translucent
  instance still hits the visual behind it.
- [ ] Step 10 — Properties › Inertial: mode combo (Computed / Override /
  Hybrid), density override, mass / CoM / tensor fields for Override, mass
  for Hybrid, and the computed readout beside it (mass, CoM, principal
  moments, "open mesh" in warning colour naming the geom). One
  `SetInertial` per commit. The status bar warns when a dropped mesh is
  open. Snapshots: `properties_inertial`, `properties_inertial_open_mesh`.
- [ ] Step 11 — Properties › Collision: policy combo, primitives list with
  add (each kind, fitted to the link's geoms on creation) / remove / "Fit
  to mesh" / pose + size fields, `Meshes` shown read-only with its file
  names. One `SetCollision` per commit. Snapshot: `properties_collision`.
- [ ] Step 12 — File › Export…: a modal with format (MJCF / URDF / both),
  directory (`rfd`), mesh path style (URDF only), floating base (MJCF
  only, OPEN 3), the `resolve` errors listed with the link they name and
  the Export button disabled while any exist; success in the status bar
  with the path. Snapshots: `export_dialog`, `export_blocked`.
- [ ] Step 13 — `riggen-export::urdf_in` + File › Import URDF… + `.urdf`
  in `load_files` and on the CLI + `assets/fixtures/arm/arm.urdf`, a
  hand-written Menagerie-style URDF of the arm (`package://` paths,
  `<inertial>`, a separate `<collision>` mesh, one `continuous` joint, a
  `<mimic>` to warn on). Tests: import validates and matches the sample's
  FK, warnings are the expected ones, import → MJCF export produces a
  file; the `mujoco` CI job loads that file too. Snapshot: `import_urdf`.
- [ ] Step 14 — Acceptance and drift: the Acceptance block below green
  locally and in CI; the design docs read against the code (01, 02) with
  the discrepancy list emptied; the M2 exit-gate style by-hand run:
  export the arm, open it in `mujoco.viewer`, report what was annoying to
  `docs/BACKLOG.md` under an M3 heading.

## Acceptance

```sh
cargo test --workspace                                   # incl. round-trip FK, resolve, writers, hull, fit, snapshots
cargo run -p riggen-app -- --export mjcf --fk-samples --out target/sample assets/fixtures/arm/arm.riggen
cargo run -p riggen-app -- --export mjcf --out target/sample-urdf assets/fixtures/arm/arm.urdf
uv run --with mujoco --with numpy python python/tests/test_mjcf_load.py target/sample target/sample-urdf
```

The Python test passes when both MJCFs load with zero compiler warnings and
every sampled body pose matches to 1e-6. The same four lines are the CI
`mujoco` job. Tag `m3` on the retirement commit.

## Docs to update on completion

- `docs/01-architecture.md` §Layer map — `riggen-mesh` row (mass, hull,
  fit), `riggen-export` row is real; §Cargo workspace — the new modules,
  `urdf-rs`, `python/tests`; §The document is the only state —
  `collision_instances`, the hull / fit caches in `LoadedMesh`; §Panels
  and menus — Inertial / Collision blocks, Export and Import modals, View
  menu; §Frame loop — the translucent instance set; §Jobs and threads —
  rewritten: no jobs, hulls synchronous and cached, `jobs` on the backlog;
  §File format — the export directory as it is; §Testing — round-trip and
  MuJoCo tests as they are, the new scenarios listed.
- `docs/02-data-model.md` §Inertials — `mass_properties` signature,
  closedness from adjacency, `MeshLookup`, `compose_inertial`, `check`;
  `CollisionPolicy::Meshes`; §`ResolvedRobot` — `ResolvedGeom` kinds,
  `ExportError`, `ExportOptions`; §Format mapping — `fullinertia` row,
  meshes in meters with no `scale`, primitives row, `<freejoint/>`; §URDF
  import — the `⚠ OPEN` resolved, warnings listed; §Commands — the
  `SetRoot` sentence per OPEN 2.
- `docs/03-roadmap.md` §M3 — status line with the decisions and what the
  by-hand run said.
- `docs/adr/0008-export-conventions.md` — written in step 4; `adr/README.md`
  index line.
- `docs/BACKLOG.md` — `is_closed` line removed (step 1), `SetRoot` line
  resolved per OPEN 2, new lines: `riggen-app::jobs` for hulls, OBB /
  PCA primitive fits, per-geom collision editing for `Meshes`, and the M3
  by-hand run's findings.
- `AGENTS.md` current state — M3 done, tag `m3`, next M4.

## Open questions

All five decided by the human on 2026-08-29, taking the recommendation
each time:

- `OPEN 1` — **decided:** `CollisionPolicy::Meshes(Vec<Geom>)` is a
  first-class variant; imported URDF collision meshes are kept, exported,
  and shown read-only in the properties panel in M3.
- `OPEN 2` — **decided:** `SetRoot` stays refused across a movable joint;
  the backlog line moves under Rejected at retirement ("a URDF always has
  a root; the reversed-pivot convention is a design question nothing in
  M3 needs").
- `OPEN 3` — **decided:** floating base is an `ExportOptions` checkbox in
  the export dialog, not a document field.
- `OPEN 4` — **decided:** meshes are baked to meters as binary STL on
  export, OBJ sources included, no `scale` attribute; first line of
  ADR-0008.
- `OPEN 5` — **decided:** the `mujoco` CI job runs the app's `--export`
  with `rust-cache`; the agent moves the CLI to a small `riggen-export`
  binary only if the measured job time makes it worth it (step 5).
