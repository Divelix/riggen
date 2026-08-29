# 02 — Data Model & Conventions

Everything in this document is `riggen-core` or `riggen-export`, and none of
it knows egui exists.

## Conventions (binding on every crate)

- **Units:** meters, kilograms, radians, seconds. Always. STL files carry no
  unit; an app-wide import-units setting (`File › Import units`, default mm
  → ×0.001, ADR-0006) is copied onto each dropped mesh's `MeshAsset::scale`,
  editable per asset afterwards and never baked into the file.
- **Handedness / up:** right-handed, Z-up, matching URDF and MuJoCo's
  default. Y-up meshes get a fixed rotation on the asset, not a convention
  switch.
- **Poses:** `Pose { t: DVec3, r: DQuat }`, always "this frame expressed in
  the parent frame". Composition is `parent ∘ child`; a matrix is derived,
  never stored.
- **RPY:** the URDF convention, `R = Rz(yaw) · Ry(pitch) · Rx(roll)`,
  radians, `rpy = (roll, pitch, yaw)`. `Pose::from_xyz_rpy` / `to_xyz_rpy`
  live in core because the properties panel (degrees, converted at the
  edge) and the URDF writer need the same pair; extraction pins pitch to
  `[-π/2, π/2]` and folds roll into yaw at gimbal lock.
- **Joint frame = child link frame.** The joint's origin *is* the child
  link's frame, expressed in the parent link's frame. The axis is expressed
  in that frame. This is the URDF rule and it maps one-to-one onto MJCF
  (below), so `ResolvedRobot` uses it and the exporters need no re-rooting.
- **Inertial frames:** the stored tensor is about the link's CoM, in link
  axes. URDF writes it that way; MJCF gets a principal-axes decomposition.
- **Numbers are f64** in the document and kinematics. f32 exists only past
  the GPU boundary in `riggen-viewport`.

## Core types (`riggen-core`)

```rust
pub struct Robot {
    pub name: String,
    pub links:  BTreeMap<LinkId, Link>,
    pub joints: BTreeMap<JointId, Joint>,
    pub frames: BTreeMap<FrameId, Frame>,      // post-MVP; present in schema from day one, always empty
    pub assets: BTreeMap<MeshId, MeshAsset>,   // file references, not geometry
    pub root:   LinkId,
    pub materials: BTreeMap<String, Material>, // name → density (kg/m³), colour
    pub next_id: IdGen,                        // hands out every id (ADR-0005)
}

pub struct MeshAsset {
    pub path: PathBuf,          // absolute + normalized in memory; relative to the .riggen on disk
    pub content_hash: u64,      // FNV-1a 64 of the file bytes, taken at registration
    pub scale: f64,             // unit conversion applied on load (0.001 for mm)
    pub fix_up: Option<DQuat>,  // Y-up → Z-up etc., applied after scale
}

pub struct Material { pub density: f64, pub color: [f32; 4] }  // kg/m³, linear RGBA

pub struct Link {
    pub name: String,
    pub visuals: Vec<Geom>,
    pub collision: CollisionPolicy,   // default SameAsVisual
    pub inertial: InertialSpec,       // default Computed { density_override: None }
    pub material: Option<String>,
}

pub struct Geom {
    pub id: GeomId,             // stable per-link id; the viewport's instance key is (LinkId, GeomId)
    pub mesh: MeshId,
    pub pose: Pose,             // geom in link frame
    pub color: Option<[f32; 4]>, // overrides the link material's colour
}

pub struct Joint {
    pub name: String,
    pub kind: JointKind,
    pub parent: LinkId,
    pub child: LinkId,
    pub origin: Pose,           // child link frame in parent link frame
    pub axis: DVec3,            // unit, in child frame; ignored for Fixed
    pub limits: Option<Limits>, // required for Revolute/Prismatic, absent for Continuous
    pub dynamics: Dynamics,     // damping, friction, armature (MJCF); defaults zero
}

pub enum JointKind { Fixed, Revolute, Continuous, Prismatic }

pub struct Limits { pub lower: f64, pub upper: f64, pub effort: f64, pub velocity: f64 }

pub enum InertialSpec {
    Computed { density_override: Option<f64> },        // from meshes at material/override density
    Override { mass: f64, com: DVec3, inertia: DMat3 }, // measured values win; computed stays visible
    Hybrid   { mass: f64 },                             // computed tensor & CoM, scaled to a weighed mass
}

pub enum CollisionPolicy {
    None,
    SameAsVisual,
    ConvexHull,                         // one hull per visual geom
    Primitives(Vec<Primitive>),         // boxes/cylinders/spheres/capsules in link frame
    ConvexDecomposition { max_hulls: u32 }, // post-MVP
}

pub struct Frame { pub name: String, pub parent: LinkId, pub pose: Pose }  // TCP, sensors; post-MVP
```

`Robot::new(name)` is a root link `base_link` plus `Robot::default_materials()`
(aluminium 2700, steel 7850, PLA 1240, ABS 1040, nylon 1150, rubber 1100).
`Robot::add_asset(MeshAsset) -> MeshId` registers a mesh file; it is not a
command, so undoing the link that used it leaves the asset registered for
the session and redo never reloads the file. An asset no geom references is
dropped on save.

**Ids** (ADR-0005) are `u32` newtypes — `LinkId`, `JointId`, `GeomId`,
`MeshId`, `FrameId` — handed out by one per-document counter
(`Robot::next_id`, so an id is unique across kinds too), stored in
`BTreeMap`s (iteration is id order, which is creation order), serialised as
`"l3"` / `"j7"` / `"g2"` / `"m1"` strings, and never reused within a
document's life. A geom id inside a new link comes from the caller
(`robot.next_id.alloc()`); link and joint ids are allocated by `AddLink`.

Invariants, enforced by `validate()` (first error) / `validation_errors()`
(all of them) and by the command layer never producing a violating state:

- The joint graph is a **tree** rooted at `root`; every non-root link has
  exactly one parent joint and the root has none. A loop is reported by
  `ValidationError::Cycle` with the links in parent order.
- Every id a joint, geom, frame or link material names exists.
- Link, joint and material names are unique (per kind) and are valid XML
  names / MJCF identifiers: `[A-Za-z_][A-Za-z0-9_.-]*`.
- A movable joint's `axis` is finite and non-zero; the properties panel
  normalises it on commit.
- A `Revolute`/`Prismatic` joint has `limits` with `lower <= upper`; every
  pose, limit and density is finite; densities are non-negative.

## Commands and history

```rust
pub enum Command {
    AddLink { link: Box<Link>, parent: LinkId, joint: Joint }, // allocates the link and joint ids, sets joint.parent/child
    RemoveLink(LinkId),                                        // the whole subtree; root refused
    RenameLink(LinkId, String), RenameJoint(JointId, String),
    AddGeom(LinkId, Geom), RemoveGeom(LinkId, GeomId), SetGeomPose(LinkId, GeomId, Pose),
    SetJoint(JointId, Joint),                                  // one gesture = one SetJoint; parent/child in the value are ignored
    Reparent { link: LinkId, new_parent: LinkId, keep_world_pose: bool },
    SetLinkMaterial(LinkId, Option<String>), UpsertMaterial(String, Material), RemoveMaterial(String),
    SetAsset(MeshId, MeshAsset),                               // scale / fix-up edits
    SetInertial(LinkId, InertialSpec), SetCollision(LinkId, CollisionPolicy), SetRoot(LinkId),
}
```

Joints are the edges of the tree (ADR-0005): a link arrives with its parent
joint and leaves with its subtree, and "connect two links" *is* `Reparent`.
There is no `AddJoint` / `RemoveJoint`. `Reparent` refuses the root and any
`new_parent` inside the link's own subtree (`EditError::WouldCreateCycle`);
with `keep_world_pose` it rewrites the joint origin from `fk` in the
**zero configuration** so every world pose at `q = 0` is unchanged — the
single most common assembly operation and the reason FK lives in core.
`RemoveMaterial` is refused while a link uses the material
(`MaterialInUse`). `SetRoot` reverses the fixed joints on the path to the
old root and refuses a movable one (a reversed revolute pivot has no home in
the swapped child frame); M3 decides whether to relax that.

`Command::apply(self, &mut Robot) -> Result<Option<LinkId>, EditError>`
mutates and then validates, so on `Err` the robot may be half-edited;
`History::apply` therefore runs it on a clone:

```rust
pub struct History { undo: Vec<Robot>, redo: Vec<Robot>, saved_depth: Option<usize> }

impl History {
    pub fn apply(&mut self, robot: &mut Robot, cmd: Command) -> Result<Option<LinkId>, EditError>;
    pub fn undo(&mut self, robot: &mut Robot) -> bool;   // false when there is nothing to undo
    pub fn redo(&mut self, robot: &mut Robot) -> bool;
    pub fn mark_saved(&mut self);                        // the current depth is what is on disk
    pub fn is_dirty(&self) -> bool;                      // by history position, not by content
}
```

`apply` runs the command on a clone, validates, then pushes the pre-state
and commits; a refused command leaves robot and history untouched (the id
counter included), and a command whose result equals the document is
dropped without an entry — the properties panel can re-commit what it shows
without growing the history. `saved_depth` becomes `None` when an edit
branches past it, so "undo below the save, edit" stays dirty until the next
save. `EditError` is `Invalid(ValidationError)`, `UnknownId { kind, id }`,
`UnknownMaterial`, `WouldCreateCycle { link, new_parent }`,
`CannotRemoveRoot`, `CannotReparentRoot`, `MaterialInUse { material, link }`,
`MovableJointOnRootPath(JointId)`.

## Kinematics

```rust
pub struct JointState(pub BTreeMap<JointId, f64>);   // q per movable joint; derived, never saved; absent reads as 0

/// World pose of every link reachable from the root for the given joint values.
pub fn fk(robot: &Robot, q: &JointState) -> BTreeMap<LinkId, Pose>;
/// The child frame's displacement for one joint value.
pub fn motion(kind: JointKind, axis: DVec3, q: f64) -> Pose;
```

`world(child) = world(parent) ∘ joint.origin ∘ motion(kind, axis, q)` where
`motion` is `rotation(axis, q)` for `Revolute`/`Continuous`, `translation
(axis · q)` for `Prismatic`, identity for `Fixed` (a zero axis, which
`validate` rejects, yields identity rather than NaN). Computed by one
depth-first pass from the root; the tree invariant makes the order trivial
and independent of id order. This function is the oracle the export
round-trip test compares against and what `Reparent { keep_world_pose }`
reads.

## Inertials (`riggen-mesh` → `riggen-core`)

Per `Geom`, `riggen-mesh::mass_properties(mesh, density)` returns volume,
mass, CoM and the inertia tensor about the CoM in mesh axes, via the signed
tetrahedra decomposition ported from RoboCAD (with its independent volume
cross-check as the "is this mesh closed?" signal — an open STL gives a
nonsense tensor, and the UI must say so). `riggen-core::compose_inertial`
transforms each geom's result into the link frame (rotate the tensor,
parallel-axis shift), sums, then applies the `InertialSpec` mode.

Export-time checks (block export, explain why): mass > 0; tensor symmetric
and positive-definite; principal moments satisfy the triangle inequality
(`I1 + I2 >= I3` and permutations). MuJoCo refuses the last two silently
enough that this check alone justifies the tool.

## `ResolvedRobot` (`riggen-export`)

The exporters never see `Robot`. `resolve(&Robot, &MeshStore) ->
Result<ResolvedRobot, Vec<ValidationError>>` produces a pure-numeric,
convention-fixed intermediate:

```rust
pub struct ResolvedRobot {
    pub name: String,
    pub links: Vec<ResolvedLink>,     // topological order, root first
    pub joints: Vec<ResolvedJoint>,   // same order as links[1..], joint i parents link i
}
pub struct ResolvedLink {
    pub name: String,
    pub visuals: Vec<ResolvedGeom>,        // mesh file name + pose in link frame
    pub collisions: Vec<ResolvedGeom>,     // hull/primitives already computed
    pub inertial: ResolvedInertial,        // mass, com, tensor about com in link axes
}
pub struct ResolvedJoint { name, kind, parent: usize, child: usize, origin: Pose, axis: DVec3, limits, dynamics }
```

Each writer is then a dumb serialiser. Adding SDF later is a new writer,
not a new resolve.

## Format mapping

| Concept | URDF | MJCF |
|---|---|---|
| Link | `<link name>` | `<body name>` nested under its parent body |
| Joint origin (child frame in parent frame) | `<joint><origin xyz rpy/>` | `<body pos quat>` of the child body |
| Joint axis (child frame) | `<joint><axis xyz/>` | `<joint axis>` inside the child body, `pos="0 0 0"` |
| Fixed | `type="fixed"` | no `<joint>` element |
| Revolute | `type="revolute"` + `<limit lower upper effort velocity/>` | `type="hinge" range="lo hi" limited="true"` |
| Continuous | `type="continuous"` | `type="hinge"` without `range` |
| Prismatic | `type="prismatic"` + `<limit/>` | `type="slide" range="lo hi"` |
| Visual geom | `<visual><origin/><geometry><mesh filename scale/></geometry></visual>` | `<geom type="mesh" mesh=… pos quat contype="0" conaffinity="0" group="2"/>` |
| Collision geom | `<collision>…` | `<geom … group="3"/>` (mesh → MuJoCo takes the convex hull itself; primitives map directly) |
| Inertial | `<inertial><origin xyz(com) rpy="0 0 0"/><mass/><inertia ixx ixy ixz iyy iyz izz/></inertial>` | `<inertial pos(com) quat(principal axes) mass diaginertia/>` — eigendecomposition, or `fullinertia` when a principal frame is ill-defined |
| Mesh assets | file path per geom | `<asset><mesh name file scale/></asset>`, one per `MeshId` |
| Root | first `<link>` | `<worldbody>` child; a free-floating base gets `<freejoint/>` (setting) |
| Effort / velocity | `<limit effort velocity/>` | `<actuator>` `forcerange`/`ctrlrange` — post-MVP, not silently dropped: a comment in the file says so |
| Angles | radians | **`<compiler angle="radian" meshdir="meshes"/>` is always written** — MJCF's default is degrees |

Quaternion order: MJCF is `w x y z`; `glam::DQuat` is `x y z w`. One helper,
one place, tested.

## URDF import (`riggen-export::urdf_in`)

`urdf-rs` → `Robot`: links and joints map directly (it is the native
convention); `<inertial>` becomes `InertialSpec::Override`; mesh filenames
resolve `package://` via a user-supplied map or the file's directory; a
`<collision>` mesh that differs from the visual becomes a
`CollisionPolicy::SameAsVisual` with a warning (v1 stores one collision
policy per link, not per-geom collision meshes). ⚠ OPEN: decide at M3 whether
imported collision meshes should be kept as opaque extra geoms rather than
downgraded.

## Schema

`{ "schema_version": 1, "robot": Robot }`. `Robot` derives
`serde::{Serialize, Deserialize}` with `#[serde(deny_unknown_fields)]` on
every struct (the envelope too) so a typo in a hand-edited file fails loudly
with the field's name, and `#[serde(default)]` only on fields added in a
later version, alongside its `upgrade_` step and corpus fixture. `load`
reads the version first, tolerant of everything else, so a newer file is
reported as `FileError::UnsupportedVersion` rather than as an unknown
field, and validates the document after resolving paths — a hand-edited
file that breaks an invariant is `FileError::Invalid`, not a half-open
document. `assets/fixtures/pendulum.riggen` (base + arm from the cube
fixtures, one revolute hinge, produced by `save` itself) is the first corpus
file; `file::tests::corpus_pendulum_opens` keeps it opening and re-saving
byte-for-byte forever.
