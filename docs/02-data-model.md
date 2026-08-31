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
- **Numbers are f64** in every quantity that has units — poses, masses,
  densities, limits — and in the kinematics. The exceptions are colours
  (`Material::color`, `Geom::color`), which are `[f32; 4]` because they go
  to the GPU and nothing computes with them.

## Core types (`riggen-core`)

```rust
pub struct Robot {
    pub name: String,
    pub links:  BTreeMap<LinkId, Link>,
    pub joints: BTreeMap<JointId, Joint>,
    pub frames: BTreeMap<FrameId, Frame>,      // named frames on links: TCP, sensor mounts
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
    pub mimic: Option<Mimic>,   // this joint follows another one (ADR-0013); schema 2
    pub actuator: Option<ActuatorSpec>, // what drives it in MJCF (ADR-0014); schema 3
}

/// q(this) = multiplier * q(joint) + offset — URDF's <mimic> (ADR-0013).
pub struct Mimic { pub joint: JointId, pub multiplier: f64, pub offset: f64 }

/// One <actuator> in the MJCF, named after its joint (ADR-0014). MJCF-only:
/// URDF keeps <limit effort velocity/> and a comment naming what it lost.
pub enum ActuatorSpec {
    Position { kp: f64, kv: f64 },   // <position kp kv ctrlrange forcerange>
    Velocity { kv: f64 },            // <velocity kv ctrlrange forcerange>
    Motor    { gear: f64 },          // <motor gear ctrlrange forcerange>
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
    Meshes(Vec<Geom>),                  // collision meshes that are not the visuals (a URDF import)
    ConvexDecomposition { max_hulls: u32, resolution: u32, concavity: f64 }, // V-HACD, ADR-0011
}

pub struct Frame { pub name: String, pub parent: LinkId, pub pose: Pose }  // TCP, sensor mount, grasp pose
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
`"l3"` / `"j7"` / `"g2"` / `"m1"` / `"f0"` strings, and never reused within a
document's life. A geom id inside a new link comes from the caller
(`robot.next_id.alloc()`); link and joint ids are allocated by `AddLink`.

Invariants, enforced by `validate()` (first error) / `validation_errors()`
(all of them) and by the command layer never producing a violating state:

- The joint graph is a **tree** rooted at `root`; every non-root link has
  exactly one parent joint and the root has none. A loop is reported by
  `ValidationError::Cycle` with the links in parent order.
- Every id a joint, geom, frame or link material names exists, and no two
  geoms of one link share a `GeomId` (`DuplicateGeomId`).
- Link, joint, frame and material names are valid XML names / MJCF
  identifiers: `[A-Za-z_][A-Za-z0-9_.-]*`. Link and joint names are unique
  per kind; **frames share the links' namespace** — a frame name is unique
  among frames *and* different from every link name, because the URDF
  writer turns each frame into a `<link>` (ADR-0012). The fixed joint it
  exports to, `<frame>_fixed`, must not be an existing joint's name either
  (`DuplicateFrameName`, `FrameJointNameCollision`).
- A movable joint's `axis` is finite and non-zero; the properties panel
  normalises it on commit.
- A `Revolute`/`Prismatic` joint has `limits` with `lower <= upper`. Joint
  origins, joint limits, frame poses and material densities are finite, and
  densities are non-negative. Geom poses and an `Override` inertial's
  numbers are **not** checked — a backlog line, not a rule.
- A `Mimic`'s leader exists, is movable, is not the follower itself and does
  not itself mimic — **chains are rejected** (ADR-0013), as is a mimic on a
  `Fixed` joint. `multiplier` is finite and non-zero, `offset` is finite,
  and the leader's range mapped through `(multiplier, offset)` fits inside
  the follower's own limits, so MJCF's `range` and the equality constraint
  cannot fight (`DanglingMimicJoint`, `SelfMimic`, `MimicOnFixedJoint`,
  `MimicLeaderFixed`, `MimicChain`, `ZeroMimicMultiplier`,
  `MimicExceedsLimits`). A `Continuous` follower has no range to leave, so
  the last check is vacuous; a `Continuous` leader has an unbounded one,
  which no bounded follower can hold.

## Commands and history

```rust
pub enum Command {
    AddLink { link: Box<Link>, parent: LinkId, joint: Joint }, // allocates the link and joint ids, sets joint.parent/child
    RemoveLink(LinkId),                                        // the whole subtree, its frames, and any mimic that followed it; root refused
    RenameLink(LinkId, String), RenameJoint(JointId, String),
    AddGeom(LinkId, Geom), RemoveGeom(LinkId, GeomId), SetGeomPose(LinkId, GeomId, Pose),
    SetJoint(JointId, Joint),                                  // one gesture = one SetJoint; parent/child in the value are ignored
    MoveJointFrame { joint: JointId, origin: Pose, axis: DVec3 },  // moves the pivot, not the geometry
    Reparent { link: LinkId, new_parent: LinkId, keep_world_pose: bool },
    SetLinkMaterial(LinkId, Option<String>), UpsertMaterial(String, Material), RemoveMaterial(String),
    SetAsset(MeshId, MeshAsset),                               // scale / fix-up edits
    SetInertial(LinkId, InertialSpec), SetCollision(LinkId, CollisionPolicy), SetRoot(LinkId),
    AddFrame(Frame),                                           // allocates the FrameId, returns it
    RemoveFrame(FrameId), SetFrame(FrameId, Frame), RenameFrame(FrameId, String),
}

/// What a command created, for the caller that selects it afterwards.
pub enum Created { Link(LinkId), Frame(FrameId) }
```

Joints are the edges of the tree (ADR-0005): a link arrives with its parent
joint and leaves with its subtree, and "connect two links" *is* `Reparent`.
There is no `AddJoint` / `RemoveJoint`. `Reparent` refuses the root and any
`new_parent` inside the link's own subtree (`EditError::WouldCreateCycle`);
with `keep_world_pose` it rewrites the joint origin from `fk` in the
**zero configuration** so every world pose at `q = 0` is unchanged — the
single most common assembly operation and the reason FK lives in core.
`MoveJointFrame` is the other half of that pair, and the one the placement
tools commit: it writes a new `origin` (the child link frame in the parent
frame) and `axis` (in the **new** child frame — the joint frame *is* the
child link frame) and re-expresses the child's **visual** geom poses, its
own child joints' origins, its frames and an `Override` inertial through
`origin_new⁻¹ ∘ origin_old`, so no world pose at `q = 0` changes and only
the pivot moves. `CollisionPolicy::Meshes` and `Primitives` poses are not
re-expressed and do move — a backlog line. `Reparent` moves a link between parents; `MoveJointFrame`
moves where a link's joint turns. Both work in the zero configuration
(plans/m2-placement-ux OPEN 1); the app resets `q` before entering an
editing tool. `RemoveMaterial` is refused while a link uses the material
(`MaterialInUse`). `SetRoot` reverses the fixed joints on the path to the
old root and refuses a movable one (a reversed revolute pivot has no home in
the swapped child frame). That stays so: a URDF always has a root, and a
reversed-pivot convention is a design question nothing needed
(plans/m3-sim-ready OPEN 2, rejected).

The four frame commands mirror the link ones: `AddFrame` allocates the
`FrameId` and hands it back as `Created::Frame`, `RenameFrame` is the tree's
inline rename, and `SetFrame` replaces name, parent link and pose in one
value — the properties panel's single commit. `SetFrame` may move a frame to
another link; like `SetJoint` it writes what it is given, so a caller that
wants the world pose kept computes the new pose through `fk` first. A frame
needs no removal command of its own for `RemoveLink`, which already takes
the frames of the subtree it removes, and `MoveJointFrame` re-expresses the
frames of the link whose joint frame moved.

`RemoveLink` also **clears the `mimic` of any joint that followed one of
the joints it removes** (ADR-0013): deleting a subtree must not fail
because of a coupling elsewhere in the tree, so the follower is freed
rather than the deletion refused. Turning a leader `Fixed` through
`SetJoint` *is* refused, by `validate`, naming the follower — that edit is
about the coupling's own leader.

The Python SDK's edit methods are these commands, one call each, applied
the same way but with no history (`riggen._riggen.Robot`, 01 §Python SDK).

`Command::apply(self, &mut Robot) -> Result<Option<Created>, EditError>`
mutates and then validates, so on `Err` the robot may be half-edited;
`History::apply` therefore runs it on a clone:

```rust
pub struct History { undo: Vec<Robot>, redo: Vec<Robot>, saved_depth: Option<usize> }

impl History {
    pub fn new() -> Self;                                // a document that counts as saved
    pub fn apply(&mut self, robot: &mut Robot, cmd: Command) -> Result<Option<Created>, EditError>;
    pub fn undo(&mut self, robot: &mut Robot) -> bool;   // false when there is nothing to undo
    pub fn redo(&mut self, robot: &mut Robot) -> bool;
    pub fn can_undo(&self) -> bool;  pub fn can_redo(&self) -> bool;
    pub fn undo_depth(&self) -> usize;                   // edits past the initial state
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

/// `q` with every mimic joint's value replaced by the one its leader implies.
pub fn resolve_q(robot: &Robot, q: &JointState) -> JointState;
/// World pose of every link reachable from the root for the given joint values.
pub fn fk(robot: &Robot, q: &JointState) -> BTreeMap<LinkId, Pose>;
/// World pose of every named frame: `world(frame.parent) ∘ frame.pose`.
pub fn frames(robot: &Robot, q: &JointState) -> BTreeMap<FrameId, Pose>;
/// The child frame's displacement for one joint value.
pub fn motion(kind: JointKind, axis: DVec3, q: f64) -> Pose;
/// The joint origin that puts `link` at `world` at q = 0; None for the root.
pub fn origin_for_world(robot: &Robot, link: LinkId, world: Pose) -> Option<Pose>;
```

`world(child) = world(parent) ∘ joint.origin ∘ motion(kind, axis, q)` where
`motion` is `rotation(axis, q)` for `Revolute`/`Continuous`, `translation
(axis · q)` for `Prismatic`, identity for `Fixed` (a zero axis, which
`validate` rejects, yields identity rather than NaN). Computed by one
depth-first pass from the root; the tree invariant makes the order trivial
and independent of id order. This function is the oracle the export
round-trip test compares against and what `Reparent { keep_world_pose }`
reads.

`fk` keeps returning **links only**: its `BTreeMap<LinkId, Pose>` is the
export oracle and the round-trip tests' contract, and a frame is not a body.
`frames` is the separate one pass over the same result, and it is what
`--fk-samples` writes as `sites` and the SDK's `frame.world(q)` returns.
`--fk-samples` writes every movable joint's `q`, a follower's at its
**derived** value, so the `qpos` it hands MuJoCo already satisfies the
equality the MJCF carries. It also writes an `actuators` block — name,
kind, driven joint, gains and the two ranges per `<actuator>` (ADR-0014),
derived from the document beside the writer's own derivation from
`ResolvedJoint`, so the MuJoCo acceptance compares two statements of one
rule rather than the writer with itself.

`fk` resolves mimic joints first, through `resolve_q`: a follower's `q` is
`multiplier · q(leader) + offset` (ADR-0013) and whatever the caller put in
the follower's own slot is ignored, not an error — it is derived state.
`resolve_q` is the **single implementation** of that rule; the Joints window
and `--fk-samples` read it too, so the number the viewport draws and the
number the export writes cannot drift apart. One pass suffices, not a fixed
point, because `validate` rejects a leader that itself mimics.

`origin_for_world` is the inverse of one step of it: `world(link) =
world(parent) ∘ origin` at `q = 0`, so the origin wanted is
`world(parent)⁻¹ ∘ world`. The link gizmo and the align tool know where a
part should end up in the world and need the number the document stores;
they commit it as one `SetJoint`.

## Mesh features (`riggen-mesh::feature`)

There is no B-Rep. An STL is a triangle soup that repeats the same
coordinates verbatim per facet, so `feature::adjacency` recovers topology by
welding positions **exactly** — bit-for-bit, `-0.0` folded to `0.0`, no
tolerance (a tolerance would merge two genuinely distinct vertices a micron
apart, and no exporter writes a shared corner as two different floats). It
yields the welded index per vertex, the neighbour across each triangle edge
(manifold edges only: a boundary or a three-way seam has none) and
`is_closed()`, which M3's mass properties need.

```rust
pub fn adjacency(mesh: &TriMesh) -> Adjacency;
/// Triangles reachable from `seed` without turning more than `max_dihedral` in one step.
pub fn grow_region(mesh: &TriMesh, adjacency: &Adjacency, seed: usize, max_dihedral: f64) -> Vec<usize>;
pub fn fit_circle(mesh: &TriMesh, triangle: usize) -> Option<CircleFit>;
/// The same, over an `Adjacency` the caller already has (the app memoises one per mesh).
pub fn fit_circle_with(mesh: &TriMesh, adjacency: &Adjacency, triangle: usize) -> Option<CircleFit>;

pub struct CircleFit { center: DVec3, axis: DVec3, radius: f64, residual: f64, segments: usize }
```

`grow_region` compares the dihedral angle **locally**, between a triangle
and the neighbour it is entered from, which is what lets a cylinder wall
grow all the way round while a 90° corner stops it. `DEFAULT_MAX_DIHEDRAL`
is 70°: the coarsest cylinder `MIN_SEGMENTS` accepts turns by 60° per step.

`fit_circle` is "click the bore, get the joint axis" (01 §Picking and
snapping). For a **curved** region the axis is the normalised sum of the
adjacent normals' cross products (each neighbouring pair turns about the
cylinder's axis; the signs are made consistent against the first one, and
the result is flipped to give its largest component a positive sign, since
a joint axis has no preferred direction) and the circle is a Kåsa
least-squares fit of every region vertex projected into the plane ⟂ axis,
centred at the region's mean height along it. For a **planar** region the
axis is the face normal and the fit runs on the region's boundary loop —
a shaft's end face is exactly its rim. No eigen solver, no B-Rep.

`residual` is the RMS distance of the fitted points from the circle, in
document meters, and is shown in the viewport so a bad fit is obvious
rather than silent. `segments` is the number of distinct angular positions
around the axis — the generator's segment count for a machine-made
cylinder. A fit with fewer than `MIN_SEGMENTS` = 6 is refused: four
coplanar corners of a square are exactly concyclic, and nothing in the
residual tells a cube face from a very coarse bore, so the segment count
has to.

`TriMesh::cylinder` / `TriMesh::tube` generate the test and fixture
geometry (ring vertices are computed once and copied, so a quad's two
triangles share bit-identical positions), and `stl::write_binary` writes
it out — the M2 arm fixtures are produced by an `#[ignore]`d generator
test rather than checked in as opaque bytes.

## Inertials (`riggen-mesh` → `riggen-core`)

Per `Geom`, `riggen_mesh::mass_properties(&mesh, density) -> MassProps {
volume, mass, com, inertia: DMat3, is_closed, inward_winding }` returns
volume, mass, CoM and the inertia tensor about the CoM in mesh axes, via
the signed tetrahedra decomposition ported from RoboCAD. RoboCAD's
independent-volume cross-check is replaced by topology: `is_closed` is
`feature::adjacency(mesh).is_closed()` (every edge shared by exactly two
triangles), exact rather than a tolerance — an open STL gives a nonsense
tensor, and the UI must say so. A negative signed volume means the mesh is
wound inward; it is folded (`abs`) and flagged, not treated as an error.
`riggen_core::inertial::compose_inertial(&link, &impl MeshLookup,
&materials) -> Result<LinkInertial, InertialError>` transforms each geom's
result into the link frame (rotate the tensor by the geom pose, move the
CoM), sums them (mass-weighted CoM, parallel-axis shift of every tensor to
it), then applies the `InertialSpec` mode: `Computed` is the sum at the
material density (or `density_override`; neither is `NoDensity`),
`Override` passes the stored values through, `Hybrid` scales the sum's mass
and tensor together to the weighed mass. `LinkInertial { inertial,
computed }` — `inertial: Inertial { mass, com, inertia }` is what every
consumer reads; `computed` is the mesh sum kept beside it for the
properties panel's comparison readout (`None` under `Override` when the
meshes cannot be measured). `MeshLookup` is a trait (`fn mesh(&self,
MeshId) -> Option<&TriMesh>`) the app's mesh store and the export CLI
implement — core still stores no geometry. `Computed` / `Hybrid` meeting
an open mesh is `InertialError::OpenMesh { geom }`; a link with no geoms
is a zero inertial, fine for a static body.

Export-time checks (`inertial::check(&Inertial) -> Vec<InertialError>`,
block export, explain why): mass > 0; every value finite; tensor symmetric
and positive-definite; principal moments satisfy the triangle inequality
(`I1 + I2 >= I3` and permutations). The moments come from
`principal_moments`, a cyclic Jacobi eigen-solve for the symmetric 3×3;
the axes are not needed because the MJCF writer hands MuJoCo the full
tensor (ADR-0008). MuJoCo refuses the last two silently enough that this
check alone justifies the tool.

## `ResolvedRobot` (`riggen-export`)

The exporters never see `Robot`. `resolve` produces a pure-numeric,
convention-fixed intermediate:

```rust
pub fn resolve(&Robot, &impl MeshLookup, &impl DecompSource, &ExportOptions)
    -> Result<ResolvedRobot, Vec<ExportError>>;

pub struct ResolvedRobot {
    pub name: String,
    pub links: Vec<ResolvedLink>,     // topological order, root first
    pub joints: Vec<ResolvedJoint>,   // joints[i] is the parent joint of links[i + 1]
    pub meshes: BTreeMap<String, Arc<TriMesh>>, // every file to write, by stem, in meters
    pub floating_base: bool,
}
pub struct ResolvedLink {
    pub name: String,
    pub visuals: Vec<ResolvedGeom>,
    pub collisions: Vec<ResolvedGeom>,     // SameAsVisual copies visuals; hulls, decomposition
                                           // pieces and primitives computed
    pub inertial: Option<Inertial>,        // None for an empty static body: no <inertial>
    pub sites: Vec<ResolvedSite>,          // the link's frames, FrameId order
}
pub struct ResolvedSite { pub name: String, pub pose: Pose }  // frame in the link frame
pub enum ResolvedGeom { Mesh { name, mesh: Arc<TriMesh>, pose }, Primitive(Primitive) }
pub struct ResolvedJoint { name, kind, parent: usize, child: usize, origin: Pose, axis: DVec3, limits, dynamics,
                           mimic: Option<ResolvedMimic>, actuator: Option<ActuatorSpec> }
pub struct ResolvedMimic { pub joint: usize, pub multiplier: f64, pub offset: f64 }  // joint indexes ResolvedRobot::joints
pub struct ExportOptions { format: Format, mesh_paths: MeshPathStyle, floating_base: bool }
```

`resolve` returns **every** problem it finds, so the export dialog lists
them all at once: `ExportError::{Invalid(ValidationError), Inertial { link,
name, error }, ZeroMassMovableLink { link, name }, UnloadableMesh { mesh,
path, reason }, DegenerateHull { … }, DegenerateDecomposition { … },
DecompositionPending { mesh, path }}` — each carrying what the dialog needs
to name the thing that failed. A link whose
parent joint is movable — or the root when `floating_base` is set — must
have mass, because MuJoCo refuses a moving body without it; an empty static
body is fine and gets no `<inertial>`. Mesh file stems are the assets' own
stems made into identifiers, `_2`, `_3`, … when two collide;
`CollisionPolicy::ConvexHull` adds `<stem>_hull` — `riggen_mesh::convex_hull`
(quickhull) of the visual mesh, computed once per `MeshId` however many
links share it, at the visual's pose; a mesh that spans no volume is
`ExportError::DegenerateHull`. `CollisionPolicy::ConvexDecomposition` adds
**N** geoms per visual, `<stem>_hull_0` … `<stem>_hull_<N-1>` at the
visual's pose — V-HACD (ADR-0011), computed once per `(MeshId,
DecompParams)`; a mesh decomposed at two different parameter sets gets a
second family `<stem>_hull2_0 …` so the files never collide. A mesh V-HACD
finds nothing solid in is `ExportError::DegenerateDecomposition`.

Where those pieces come from is the `DecompSource` trait, `resolve`'s third
argument: `ComputeNow` runs V-HACD inline (the CLI, the SDK, the tests,
where a blocking second is what the caller asked for), while the app hands
over a cache its job thread fills and reports `DecompMiss::Pending` for an
entry that has not landed — which becomes `ExportError::DecompositionPending`
and blocks the export until the job lands, listed beside every other
blocker (no modal, no spinner over the dialog).

A `Joint::mimic` becomes a `ResolvedMimic` whose `joint` is an **index into
`ResolvedRobot::joints`**, not a `JointId`, so both writers stay dumb
serialisers of the vector they already have (ADR-0004 §1, ADR-0013).

Every `Frame` becomes a `ResolvedSite` on its parent link, in `FrameId`
order, carrying its link-frame pose unchanged; a frame on a link the tree
does not reach cannot get this far, because `validate` rejected the
document first. No frame adds an `ExportError` of its own — the name rules
under §Invariants are validation errors.

`MeshStore`
is the headless `MeshLookup` (files read and brought to meters as the
viewport does); the app implements the trait on its own store.

Each writer is then a dumb serialiser. Adding SDF later is a new writer,
not a new resolve.

## Format mapping

| Concept | URDF | MJCF |
|---|---|---|
| Link | `<link name>` | `<body name>` nested under its parent body |
| Joint origin (child frame in parent frame) | `<joint><origin xyz rpy/>` | `<body pos quat>` of the child body |
| Joint axis (child frame) | `<joint><axis xyz/>` | `<joint axis>` inside the child body; no `pos` is written, and MuJoCo's default is the body origin, which is the joint frame |
| Fixed | `type="fixed"` | no `<joint>` element |
| Revolute | `type="revolute"` + `<limit lower upper effort velocity/>` | `type="hinge" range="lo hi" limited="true"` |
| Continuous | `type="continuous"` | `type="hinge"` without `range` |
| Prismatic | `type="prismatic"` + `<limit/>` | `type="slide" range="lo hi"` |
| Visual geom | `<visual><origin/><geometry><mesh filename/></geometry></visual>` | `<geom class="visual" mesh=… pos quat/>` with `<default class="visual">` = `type="mesh" contype="0" conaffinity="0" group="2"` |
| Collision geom (one per resolved collision — N of them for a decomposition) | `<collision>…` | `<geom class="collision" type="mesh" mesh=… />` (mesh → MuJoCo takes the convex hull itself; primitives map directly), `<default class="collision">` = `group="3"`, translucent rgba |
| Primitive | `<box size>` (full extents), `<cylinder radius length>`, `<sphere radius>`; a capsule becomes a cylinder plus a warning | `type="box|cylinder|sphere|capsule" size pos quat` — **`size` is half-extents / (radius, half-length)**, pinned by a test |
| Inertial | `<inertial><origin xyz(com) rpy="0 0 0"/><mass/><inertia ixx ixy ixz iyy iyz izz/></inertial>` | `<inertial pos(com) mass fullinertia="Ixx Iyy Izz Ixy Ixz Iyz"/>` — MuJoCo does the principal-axes decomposition itself (ADR-0008) |
| Mesh assets | `meshes/<stem>.stl`, path style per `MeshPathStyle` | `<asset><mesh name file/></asset>`, one per written **file** — a referenced mesh, plus each hull and decomposition piece; **meshes are written in meters as binary STL, no `scale`** (ADR-0008) |
| Root | first `<link>` | `<worldbody>` child; `floating_base` in `ExportOptions` adds `<freejoint name="root"/>` |
| Frame (`Frame`, a `ResolvedSite`) | a massless `<link name="tcp"/>` — no visual, collision or inertial — plus `<joint name="tcp_fixed" type="fixed">` with the frame pose as its `<origin xyz rpy/>`; the dummy links after every real link and the fixed joints after every real joint, so the file still reads root-first (ADR-0012) | `<site name pos quat/>` inside its body after the geoms, bare: no `size`, `group` or `rgba`, so MuJoCo's default 0.005 m sphere marks it (ADR-0012) |
| Mimic (`ResolvedMimic`) | `<mimic joint multiplier offset/>` inside the follower's `<joint>`, after `<dynamics>` | `<equality><joint joint1="follower" joint2="leader" polycoef="offset multiplier 0 0 0"/></equality>` after `</worldbody>` — a **soft** solver constraint, not a reduction (ADR-0013) |
| Actuator (`ActuatorSpec`) | nothing — `<transmission>` is a `ros_control` relic; a comment after the `<joint>` names the preset and its gains, like the `armature` one (ADR-0014) | one `<actuator>` block after `</equality>`: `<position kp kv>` / `<velocity kv>` / `<motor gear>`, `name` and `joint` both the **joint's own name** |
| Effort / velocity | `<limit effort velocity/>` | the actuator's `forcerange="-effort effort"`, and its `ctrlrange` — `lower upper` for a position servo, `±velocity` for a velocity one, the normalised `-1 1` for a motor. A zero `effort` / `velocity` is the *unfilled* value, so the attribute is **omitted** and MuJoCo's unbounded default stands, never `0 0`. A joint with no actuator keeps the comment naming what was dropped (ADR-0004 §4 as amended by ADR-0014) |
| Dynamics | `<dynamics damping friction/>` | `damping`, `frictionloss`, `armature` on the `<joint>`, written only when non-zero |
| Angles | radians | **`<compiler angle="radian" meshdir="meshes" autolimits="true"/>` is always written** — MJCF's default is degrees |

MJCF's `polycoef` is `a0 a1 a2 a3 a4` in `y − y0 = a0 + a1(x − x0) + …`,
where `x` and `y` are the two joints' deviations from their `qpos0`. We
never write `ref`, so both references are zero and `(offset, multiplier, 0,
0, 0)` is exactly URDF's `q_y = k·q_x + o`; the last three slots are always
zero, because non-linear coupling is not modelled.

Quaternion order: MJCF is `w x y z`; `glam::DQuat` is `x y z w`. One helper,
one place, tested (`xml::quat_wxyz`). Numbers are written with twelve
decimals, trailing zeros trimmed, `-0` folded, and `pos` / `quat` are
omitted at their defaults, so the files read like hand-written ones and the
golden tests stay legible. No XML crate: `xml.rs` is a 30-line escaping
writer, since the output is fixed-shape.

## URDF import (`riggen-export::urdf_in`)

`urdf_in::load(path, &PackageMap) -> Result<(Robot, Vec<ImportWarning>),
ImportError>` over `urdf-rs`. Links and joints map directly (URDF's
joint-frame convention is ours, ADR-0004); `<inertial>` becomes
`InertialSpec::Override` (the tensor rotated from the inertial frame into
link axes); a `<mesh scale>` becomes `MeshAsset::scale` (uniform only — a
non-uniform one is a warning and the largest component); `<collision>`
meshes that repeat the visuals are `SameAsVisual`, any other set is
`CollisionPolicy::Meshes` kept losslessly (OPEN 1, decided: no
downgrade), collision primitives are `Primitives`; `<mimic>` becomes a `Joint::mimic`
(ADR-0013), resolved in a second pass so it may name a joint further down
the file, with URDF's own defaults (multiplier 1, offset 0) filled in.
`package://name/rest`
resolves through the map, else `rest` beside the file, else `name/rest`
under an ancestor of the file's directory — `urdf-rs`'s own resolution
shells out to `rospack`. Nothing is dropped silently: `ImportWarning::{
MimicDropped, SafetyControllerDropped, NonUniformScale,
PrimitiveVisualDropped, MixedCollisionDropped, NoInertial,
PackageUnresolved, MeshNotFound }` reach the status bar (File › Import
URDF…, a dropped `.urdf`, or `riggen --export … robot.urdf` on stderr).
`ImportWarning::MimicDropped` now carries a `reason`, and only for a
coupling the document cannot hold: a leader that is not a joint in the file
or is `fixed`, a joint following itself, a `<mimic>` on a `fixed` joint, a
chain, a zero multiplier, a multiplier or offset that is not a number, and
a reach outside the follower's own limits. `validate` owns those rules — the import runs it and phrases
its verdict — so a refused coupling is dropped and the file still opens; it
never turns into an `ImportError`.

A massless childless link is **not** turned back into a `Frame`: nothing
distinguishes our exported dummy from a real unweighed link, and guessing
would silently delete links, so the asymmetry with the URDF writer is
deliberate and round-tripping our own file gains one link per frame
(ADR-0012). `floating` / `planar` / `spherical` joints, a missing link, no
or several roots and a result that fails `validate` are `ImportError`s,
beside `Io` and `Parse` for a file that cannot be read or understood. The imported
document is untitled until saved. `assets/fixtures/arm/arm.urdf` is the
corpus file: the arm with every one of the above in it, whose FK matches
`arm.riggen`'s and whose MJCF export the `mujoco` CI job loads too.

## Schema

`{ "schema_version": 3, "robot": Robot }`. `Robot` derives
`serde::{Serialize, Deserialize}` with `#[serde(deny_unknown_fields)]` on
every struct (the envelope too) so a typo in a hand-edited file fails loudly
with the field's name, and `#[serde(default)]` only on fields added in a
later version, alongside its `upgrade_` step and corpus fixture. `load`
reads the version first, tolerant of everything else, so a file outside
`OLDEST_SCHEMA_VERSION..=SCHEMA_VERSION` is reported as
`FileError::UnsupportedVersion` rather than as an unknown field; it then
walks the `upgrade_vN_to_vN+1` chain from the version it found up to
`SCHEMA_VERSION`, and validates the document after resolving paths — a hand-edited
file that breaks an invariant is `FileError::Invalid`, not a half-open
document. `assets/fixtures/pendulum.riggen` (base + arm from the cube
fixtures, one revolute hinge, produced by `save` itself) is the first corpus
file and is frozen at **schema 1**: it is what the upgrade chain reads, and
`file::tests::corpus_pendulum_opens` keeps it opening forever and re-saving
as a v3 document that round-trips. The byte-for-byte fixtures are the v3
ones, `bracket.riggen` and `arm/arm.riggen`.

**Schema 2** adds `Joint::mimic` (ADR-0013) and **schema 3** adds
`Joint::actuator` (ADR-0014). Both `upgrade_` steps are empty for the same
reason — an older file simply has no such key and `#[serde(default)]` fills
in the `None` it meant — and they are the chain `load` walks;
`file::tests::a_v2_file_opens_as_v3_with_no_actuators` pins the second, from
a v2 document made by stripping the key back out of the committed fixture.

`CollisionPolicy::ConvexDecomposition`'s `resolution` and `concavity` are so
far the only fields added after their variant existed, and they are the
worked example of the rule: all three fields carry `#[serde(default = …)]`
taken from `riggen_mesh::DecompParams::default()`, a v1 file carrying only
`{"ConvexDecomposition": {"max_hulls": 4}}` opens with the algorithm's
defaults filled in (`file::tests::a_v1_file_with_only_max_hulls_reads_with_the_defaults`),
and `schema_version` did not move for them — nothing that reads an old file
needed to change, so there was no upgrade step. That is still the rule for
a field a *variant* gains; `Joint::mimic` bumped the version because it is
a field on a struct every document has. Such a file remains readable: it
declares schema 1 and comes in through the chain above.
