# 02 — Data Model & Conventions

Everything in this document is `riggen-core` or `riggen-export`, and none of
it knows egui exists.

## Conventions (binding on every crate)

- **Units:** meters, kilograms, radians, seconds. Always. STL files carry no
  unit; the import dialog asks (default mm → ×0.001) and the scale is stored
  on the `MeshAsset`, never baked into the file.
- **Handedness / up:** right-handed, Z-up, matching URDF and MuJoCo's
  default. Y-up meshes get a fixed rotation on the asset, not a convention
  switch.
- **Poses:** `Pose { t: DVec3, r: DQuat }`, always "this frame expressed in
  the parent frame". Composition is `parent ∘ child`; a matrix is derived,
  never stored.
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
    pub links:  SlotMap<LinkId, Link>,
    pub joints: SlotMap<JointId, Joint>,
    pub frames: SlotMap<FrameId, Frame>,      // post-MVP; present in schema from day one
    pub assets: SlotMap<MeshId, MeshAsset>,   // file references, not geometry
    pub root:   LinkId,
    pub materials: BTreeMap<String, Material>, // name → density (kg/m³), colour
}

pub struct MeshAsset {
    pub path: RelativePath,     // relative to the .riggen file
    pub content_hash: u64,
    pub scale: f64,             // unit conversion applied on load (0.001 for mm)
    pub fix_up: Option<DQuat>,  // Y-up → Z-up etc.
}

pub struct Link {
    pub name: String,
    pub visuals: Vec<Geom>,
    pub collision: CollisionPolicy,
    pub inertial: InertialSpec,
    pub material: Option<String>,
}

pub struct Geom {
    pub id: GeomId,             // stable per-link id; the viewport's instance key is (LinkId, GeomId)
    pub mesh: MeshId,
    pub pose: Pose,             // geom in link frame
    pub color: Option<[f32; 4]>,
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
    Computed { density_override: Option<f64> },       // from meshes at material/override density
    Override { mass: f64, com: DVec3, inertia: Mat3 }, // measured values win; computed stays visible
    Hybrid   { mass: f64 },                            // computed tensor & CoM, scaled to a weighed mass
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

Invariants, enforced by `Robot::validate()` and by the command layer never
producing a violating state:

- The joint graph is a **tree** rooted at `root`; every non-root link has
  exactly one parent joint. Closed loops are rejected at edit time with the
  message that names the loop (v1 non-goal, but the error must be good).
- Link and joint names are unique and are valid XML names / MJCF identifiers.
- `axis` is normalised on write; a zero axis is rejected.
- A `Revolute`/`Prismatic` joint has `limits` with `lower <= upper`.

Ids are `slotmap` keys: stable across edits, never reused within a session,
serialised as `"l3"`/`"j7"` style strings so a `.riggen` diff is readable.

## Commands and history

```rust
pub enum Command {
    AddLink(Link), RemoveLink(LinkId), RenameLink(LinkId, String),
    AddGeom(LinkId, Geom), RemoveGeom(LinkId, GeomId), SetGeomPose(LinkId, GeomId, Pose),
    AddJoint(Joint), RemoveJoint(JointId), SetJoint(JointId, Joint),   // one gesture = one SetJoint
    Reparent { link: LinkId, new_parent: LinkId, keep_world_pose: bool },
    SetInertial(LinkId, InertialSpec), SetCollision(LinkId, CollisionPolicy),
    SetMaterial(..), SetAsset(..), SetRoot(LinkId),
}
```

`History::apply(&mut Robot, Command) -> Result<(), EditError>` validates,
pushes the pre-state snapshot, mutates. `Reparent` with `keep_world_pose`
rewrites the joint origin from the current FK so the part does not jump —
the single most common assembly operation and the reason FK lives in core.

## Kinematics

```rust
pub struct JointState(pub Vec<(JointId, f64)>);   // q per non-fixed joint; derived, never saved

/// World pose of every link for the given joint values.
pub fn fk(robot: &Robot, q: &JointState) -> HashMap<LinkId, Pose>;
```

`world(child) = world(parent) ∘ joint.origin ∘ motion(kind, axis, q)` where
`motion` is `rotation(axis, q)` for `Revolute`/`Continuous`, `translation
(axis · q)` for `Prismatic`, identity for `Fixed`. Computed by one
depth-first pass from the root; the tree invariant makes the order trivial.
This function is the oracle the round-trip test compares against.

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

`schema_version: 1`. `Robot` derives `serde::{Serialize, Deserialize}` with
`#[serde(deny_unknown_fields)]` on every struct so a typo in a hand-edited
file fails loudly, and `#[serde(default)]` only on fields added in a later
version, alongside its `upgrade_` step and corpus fixture.
