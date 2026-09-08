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
  edge) and the URDF and SDF writers need the same pair — SDF's `<pose>`
  angles are URDF's `rpy`; extraction pins pitch to `[-π/2, π/2]` and folds
  roll into yaw at gimbal lock.
- **Joint frame = child link frame.** The joint's origin *is* the child
  link's frame, expressed in the parent link's frame. The axis is expressed
  in that frame. This is the URDF rule, writing it as MJCF is one-to-one
  (below), and it is also SDF's own default for a joint's pose and its
  axis frame — so `ResolvedRobot` uses it and no writer re-roots anything. Reading MJCF is not: `<joint pos>` anchors a joint anywhere
  in the body frame, so the import moves the link frame onto the anchor
  and re-expresses the body's contents against it (§MJCF import). Our own
  writer emits no `pos`, so the round trip moves nothing.
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
    pub tendons: BTreeMap<TendonId, Tendon>,   // fixed tendons (ADR-0025 §4); schema 6
    pub actuators: BTreeMap<ActuatorId, Actuator>, // what drives a joint or a tendon (ADR-0023); schema 4
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
    pub qpos_ref: f64,          // MJCF <joint ref>: qpos = q + qpos_ref (ADR-0025); schema 6
}

/// q(this) = multiplier * q(joint) + offset — URDF's <mimic> (ADR-0013).
pub struct Mimic { pub joint: JointId, pub multiplier: f64, pub offset: f64 }

/// MJCF's <tendon><fixed>: a named linear combination of joint values,
/// length = Σ coef · qpos, with a range and its own passive dynamics
/// (ADR-0025 §4). Unlike a Mimic it is a constraint MuJoCo enforces with
/// forces, not a rule that resolves a value: `fk` never reads one and every
/// joint on a tendon stays free. `range` is in **MJCF's own absolute
/// terms** — it is never shifted by a `qpos_ref` on either side, because
/// MuJoCo evaluates a fixed tendon over `qpos`, not over deviations from
/// `qpos0`. `limited` is MuJoCo's auto | true | false, as ActuatorRanges
/// holds it. Its two consumers are the MJCF writer and the SDK.
pub struct Tendon {
    pub name: String,               // unique among tendons
    pub joints: Vec<TendonJoint>,   // >= 1, each joint once, none Fixed
    pub range: Option<[f64; 2]>,
    pub limited: Option<bool>,
    pub stiffness: f64,
    pub damping: f64,
    pub frictionloss: f64,
}

pub struct TendonJoint { pub joint: JointId, pub coef: f64 }  // coef != 0

/// One <actuator> element (ADR-0023, amending ADR-0014): its own name — the
/// target joint's by default, unique among actuators, MJCF's namespaces
/// being per element type — what it drives, the spec saying how (a preset
/// or a `General`), and the ranges its file said.
/// Several may target one joint: MuJoCo sums them, and foreign files ship
/// them.
pub struct Actuator {
    pub name: String,
    pub target: ActuatorTarget,
    pub spec: ActuatorSpec,
    pub ranges: ActuatorRanges,      // what the file said (ADR-0024); schema 5
}

/// The ranges the <actuator> element said itself, preferred by the writer
/// over the ones it derives from the joint (ADR-0024 §3). A None range is
/// "the writer derives it" — a riggen-authored actuator; a None flag is
/// MJCF's own `auto` under the autolimits="true" every export writes.
/// Some(false) beside no range is an imported actuator that named none:
/// unlimited, not clamped to the joint.
pub struct ActuatorRanges {
    pub ctrl: Option<[f64; 2]>,
    pub force: Option<[f64; 2]>,
    pub ctrl_limited: Option<bool>,
    pub force_limited: Option<bool>,
}

/// What an actuator drives: a joint, or a fixed tendon — the second variant
/// ADR-0023 shaped the enum for and ADR-0025 §4 filled in. A site or body
/// target is still dropped on import. A tendon actuator is **not "on"** any
/// of the tendon's joints: `target.joint()` is None for it, so no joint's
/// panel offers it, `SetActuators` neither replaces nor counts it, and the
/// joint-specific refusals below say nothing about it.
pub enum ActuatorTarget { Joint(JointId), Tendon(TendonId) }

/// How it drives it (ADR-0014): a preset, or MJCF's own general actuator
/// model, the escape hatch for a file that wrote one (ADR-0024). MJCF-only:
/// URDF keeps <limit effort velocity/> and a comment naming what it lost.
/// Not `Copy` since `General` — three Vecs — so a by-value site clones.
pub enum ActuatorSpec {
    Position { kp: f64, kv: f64 },   // <position kp kv ctrlrange forcerange>
    Velocity { kv: f64 },            // <velocity kv ctrlrange forcerange>
    Motor    { gear: f64 },          // <motor gear ctrlrange forcerange>
    General(General),                // <general dyntype gaintype biastype dynprm gainprm biasprm gear>; schema 5
}

/// What a <general> carries (ADR-0024 §4): the three types, their prm
/// vectors as the file wrote them (at most General::MAX_PRM = 10 each, not
/// the ten MuJoCo zero-fills to), and the first of the six `gear`s. Each
/// type enum spells itself as MJCF does: `mjcf_name()` / `from_mjcf()`.
/// `Default` is MuJoCo's bare <general>: None / Fixed / None, empty vectors,
/// gear 1.
pub struct General {
    pub dyntype: DynType,     // None | Integrator | Filter | FilterExact | Muscle | User
    pub gaintype: GainType,   // Fixed | Affine | Muscle | User
    pub biastype: BiasType,   // None | Affine | Muscle | User
    pub dynprm: Vec<f64>,
    pub gainprm: Vec<f64>,
    pub biasprm: Vec<f64>,
    pub gear: f64,
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
`MeshId`, `FrameId`, `ActuatorId`, `TendonId` — handed out by one
per-document counter
(`Robot::next_id`, so an id is unique across kinds too), stored in
`BTreeMap`s (iteration is id order, which is creation order), serialised as
`"l3"` / `"j7"` / `"g2"` / `"m1"` / `"f0"` / `"a4"` / `"t6"` strings, and never reused
within a document's life. A geom id inside a new link comes from the caller
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
  origins, joint limits, `qpos_ref`, frame poses and material densities are
  finite, and densities are non-negative. Geom poses and an `Override` inertial's
  numbers are **not** checked — a backlog line, not a rule.
- A `Mimic`'s leader exists, is movable and is not the follower itself. It
  **may itself follow** — a chain resolves (ADR-0025) — but a ring of
  followers with no free leader at its head does not (`MimicCycle`, whose
  `joints` are in follow order from the lowest id in the ring, the way
  `Cycle` names a link loop). A mimic on a `Fixed` joint is refused.
  `multiplier` is finite and non-zero, `offset` is finite, and the range of
  the **free** joint at the head of the chain, mapped through the composed
  map, fits inside the follower's own limits, so MJCF's `range` and the
  equality constraint cannot fight (`DanglingMimicJoint`, `SelfMimic`,
  `MimicOnFixedJoint`, `MimicLeaderFixed`, `MimicCycle`,
  `ZeroMimicMultiplier`, `MimicExceedsLimits`). A `Continuous` follower has
  no range to leave, so the last check is vacuous; a `Continuous` free
  leader has an unbounded one, which no bounded follower can hold.
- A `Tendon` runs over at least one joint (`EmptyTendon`), each of which is
  in the document (`DanglingTendonJoint`), movable (`TendonOnFixedJoint` —
  which is what refuses demoting one of a tendon's joints, the way
  `MimicLeaderFixed` refuses demoting a leader), named once
  (`DuplicateTendonJoint`) and weighed by a non-zero finite `coef`
  (`ZeroTendonCoef`, `NonFinite`). Its `stiffness`, `damping` and
  `frictionloss` are finite, and a `range` — held in MJCF's own absolute
  terms, so the check needs no coordinates — has `lower < upper`
  (`InvalidTendonRange`). Its name is unique **among tendons**
  (`DuplicateTendonName`) and nowhere else, MJCF's namespaces being per
  element type.
- An `Actuator`'s `target` names a joint **or a tendon** of the document
  (`DanglingActuatorTarget`). A joint target is movable and does not follow
  another one (`ActuatorOnFixedJoint`, `ActuatorOnMimicFollower`: a fixed
  joint has no `<joint>` for MJCF to drive, and a follower is already driven
  by its `<equality>`); a **tendon** target only has to exist, because a
  tendon's joints are free and movable by construction (ADR-0025 §4). Its
  gains are finite (`NonFinite`) and usable —
  `kp` / `kv` may be zero but never negative, a `gear` may be negative but
  never zero (`InvalidActuatorGain`). Those are statements about the
  presets: a `General` (ADR-0024) is refused for exactly two things — a
  `prm` vector longer than MuJoCo's ten (`ActuatorPrmTooLong`, naming the
  vector and its length) and a non-finite entry or `gear` (`NonFinite`,
  naming the slot) — and its gains are otherwise MuJoCo's to interpret,
  not riggen's to bound. Its `ranges` are finite numbers like
  every other in the document (`NonFinite`) and nothing more: whether an
  inverted or empty range limits anything is MuJoCo's to decide under the
  flags written beside it (ADR-0024). Every refusal names the **actuator**,
  which has a name of its own: unique among actuators
  (`DuplicateActuatorName`) and nowhere else, so it may be the driven
  joint's — which is the default. **Several actuators on one joint is
  legal** and deliberately so: MuJoCo sums their controls, and refusing it
  here would reject models MuJoCo accepts and make the MJCF import drop what
  a file said (ADR-0023 §3).

## Commands and history

```rust
pub enum Command {
    AddLink { link: Box<Link>, parent: LinkId, joint: Joint }, // allocates the link and joint ids, sets joint.parent/child
    RemoveLink(LinkId),                                        // the whole subtree, its frames, any mimic that followed it, and its joints' terms in every tendon; root refused
    RenameLink(LinkId, String), RenameJoint(JointId, String),
    AddGeom(LinkId, Geom), RemoveGeom(LinkId, GeomId), SetGeomPose(LinkId, GeomId, Pose),
    SetJoint(JointId, Joint),                                  // one gesture = one SetJoint; parent/child in the value are ignored
    MoveJointFrame { joint: JointId, origin: Pose, axis: DVec3 },  // moves the pivot, not the geometry
    Reparent { link: LinkId, new_parent: LinkId, keep_world_pose: bool, at: JointState },  // `at`: the configuration kept
    SetLinkMaterial(LinkId, Option<String>), UpsertMaterial(String, Material), RemoveMaterial(String),
    RenameMaterial { from: String, to: String },               // every link's reference follows
    SetAsset(MeshId, MeshAsset),                               // scale / fix-up edits
    SetInertial(LinkId, InertialSpec), SetCollision(LinkId, CollisionPolicy), SetRoot(LinkId),
    AddFrame(Frame),                                           // allocates the FrameId, returns it
    RemoveFrame(FrameId), SetFrame(FrameId, Frame), RenameFrame(FrameId, String),
    AddTendon(Tendon),                                         // allocates the TendonId, returns it
    RemoveTendon(TendonId),                                    // and the actuators driving it
    SetTendon(TendonId, Tendon), RenameTendon(TendonId, String),
    AddActuator(Actuator),                                     // allocates the ActuatorId, returns it
    RemoveActuator(ActuatorId), SetActuator(ActuatorId, Actuator), RenameActuator(ActuatorId, String),
    SetActuators(Option<ActuatorSpec>),                        // every movable joint at once; mimic followers skipped
}

/// What a command created, for the caller that selects it afterwards.
pub enum Created { Link(LinkId), Frame(FrameId), Tendon(TendonId), Actuator(ActuatorId) }
```

Joints are the edges of the tree (ADR-0005): a link arrives with its parent
joint and leaves with its subtree, and "connect two links" *is* `Reparent`.
There is no `AddJoint` / `RemoveJoint`. `Reparent` refuses the root and any
`new_parent` inside the link's own subtree (`EditError::WouldCreateCycle`);
with `keep_world_pose` it rewrites the joint origin from `fk` so every
world pose **at the configuration `at`** is unchanged — the single most
common assembly operation and the reason FK lives in core.
`MoveJointFrame` is the other half of that pair, and the one the placement
tools commit: it writes a new `origin` (the child link frame in the parent
frame) and `axis` (in the **new** child frame — the joint frame *is* the
child link frame) and re-expresses the child's **visual** geom poses, its
own child joints' origins, its frames and an `Override` inertial through
`origin_new⁻¹ ∘ origin_old`, so no world pose at `q = 0` changes and only
the pivot moves. `CollisionPolicy::Meshes` and `Primitives` poses are not
re-expressed and do move — a backlog line. `Reparent` moves a link between parents; `MoveJointFrame`
moves where a link's joint turns. `MoveJointFrame` works in the zero
configuration, which is what the app's Edit mode is for the whole of its
stay (ADR-0021 §2). **`Reparent` is the one frame-rewriting command
allowed off it**: `at: JointState` is the configuration whose world poses
are kept — the zero configuration by default and from the GUI's tree
drop, which happens in Edit; a posed `q=` in the SDK, whose caller may be
mid-pose — and the origin written is
`world_at(new_parent)⁻¹ ∘ world_at(parent) ∘ origin`: the origin
re-expressed from the old parent's frame to the new one's, both at `at`
(the link's own joint value cancels; the joints *above* it are what move
it), which reduces to the zero-configuration rewrite when `at` is zero. The
off-zero form was first decided for the tree drop (plans/panels-and-numbers
OPEN 4, 2026-09-02), when a tree edit could be made while posing and the
part had to stay where the user saw it; ADR-0021 §5 narrows it to the SDK. `SetActuators` is the whole-model apply (ADR-0014): every movable joint that
does not follow another one gets the same actuator, in one command and one
undo, because the uniform case is the common one and clicking seven joints is
the tedium the app exists to remove. A follower is *skipped*, not refused —
its `<equality>` already drives it — the same way `RemoveLink` frees a
follower rather than failing. Re-expressed over the table (ADR-0023) it
first clears every actuator that targets a joint, then adds one per free
movable joint, named after it. The per-actuator edit is the quartet beside
it, `AddActuator` / `RemoveActuator` / `SetActuator` / `RenameActuator`,
which follow the frame commands exactly: `AddActuator` allocates the
`ActuatorId` and hands it back as `Created::Actuator`, `RenameActuator` is
the inline rename, and `SetActuator` replaces name, target, spec and ranges in one
value. An actuator naming a joint or a tendon the document does not have is
refused (`UnknownId`). Three things take an actuator away without being
asked. Two are because `validate` would otherwise refuse the edit that
caused them: `RemoveLink` drops the actuators of the joints it removes, and
`SetJoint` drops the joint's own when it retypes it to `Fixed` — a fixed
joint has no degree of freedom to drive, exactly as it has no value to
mimic. The third is `RemoveTendon`, below.

The tendon commands are the same quartet once more (ADR-0025 §4):
`AddTendon` allocates the `TendonId` and hands it back as
`Created::Tendon`, `RenameTendon` is the inline rename, and `SetTendon`
replaces name, joints, coefficients, range and dynamics in one value. A
tendon naming a joint the document does not have is refused (`UnknownId`)
before anything changes. **`RemoveTendon` takes the actuators that drive
it**: a removal is a gesture about the tendon, its actuators are an edit of
the tendon rather than of something elsewhere in the tree, and the whole
thing undoes in one keystroke. `RemoveLink` does the same by degrees — it
drops the removed joints' terms from every tendon, and a tendon left with
no terms goes with the subtree, taking its actuators — while *demoting* one
of a tendon's joints to `Fixed` through `SetJoint` is refused by `validate`
naming the tendon, exactly as demoting a mimic leader is refused naming its
follower. `SetActuators` leaves a tendon's actuators alone: a tendon
actuator is not "on" any joint.
`RemoveMaterial` is refused while a link uses the material
(`MaterialInUse`); `RenameMaterial` rewrites the key and every link's
reference in one step, refused for an unknown `from` (`UnknownMaterial`)
and a `to` the document already has (`MaterialExists`). `SetRoot` reverses the fixed joints on the path to the
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
pub struct GestureId(pub u64);                           // a drag, press to release; the caller's value
pub struct History { undo: Vec<Robot>, redo: Vec<Robot>, saved_depth: Option<usize>, gesture: Option<GestureId> }

impl History {
    pub fn new() -> Self;                                // a document that counts as saved
    pub fn apply(&mut self, robot: &mut Robot, cmd: Command) -> Result<Option<Created>, EditError>;
    pub fn apply_in_gesture(&mut self, robot: &mut Robot, cmd: Command, gesture: GestureId) -> Result<Option<Created>, EditError>;
    pub fn end_gesture(&mut self);                       // release
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
save.

**One gesture = one history entry.** A scrubbed number field previews
*through* the document — one `Set…` per frame — and the user dragged once,
so `apply_in_gesture` coalesces: the first changing apply under a
`GestureId` pushes the pre-state and opens the gesture, every later one
under the same id only advances the document, and `end_gesture` (release)
closes it. A plain `apply`, `undo`, `redo` or `mark_saved` closes it too,
so a popped entry is never advanced and what a save wrote is a whole
entry; a refused or no-op command opens nothing. The id is the caller's
(the field's widget id), and `undo_depth` grows by one per drag however
many frames it previewed through (plans/panels-and-numbers OPEN 1, decided
2026-09-02 without an ADR). `EditError` is `Invalid(ValidationError)`, `UnknownId { kind, id }`,
`UnknownMaterial`, `WouldCreateCycle { link, new_parent }`,
`CannotRemoveRoot`, `CannotReparentRoot`, `MaterialInUse { material, link }`,
`MaterialExists(String)`, `MovableJointOnRootPath(JointId)`.

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
equality the MJCF carries. It also writes an `actuators` block — the
actuator's own name, its kind, the driven joint, the gains, the two ranges
and the `ctrllimited` / `forcelimited` MuJoCo must end up with per
`<actuator>` (ADR-0014, ADR-0023, ADR-0024), and for a `general` its
three type names and three `prm` vectors as the document holds them, in
`ActuatorId` order,
derived from the document beside the writer's own derivation from
`ResolvedActuator`, so the MuJoCo acceptance compares two statements of one
rule rather than the writer with itself — both now beginning "the
actuator's own range, else the joint's". Name and joint are separate
fields there because they are separate fields in the document: assuming one
from the other is exactly the loss ADR-0023 closes.

`fk` resolves mimic joints first, through `resolve_q`: a follower's `q` is
`multiplier · q(leader) + offset` (ADR-0013) and whatever the caller put in
the follower's own slot is ignored, not an error — it is derived state.
`resolve_q` is the **single implementation** of that rule; the joint tree
and `--fk-samples` read it too, so the number the viewport draws and the
number the export writes cannot drift apart. A leader may itself follow, so
it is a **topological** pass with a memo (ADR-0025): a follower reads its
leader's resolved value, however long the chain. On a document `validate`
would refuse with `MimicCycle` it terminates rather than looping — every
joint in the ring keeps its raw `q`, the way `fk` terminates on a link
loop, and a chain hanging off the ring resolves against those raw values.

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
    pub actuators: Vec<ResolvedActuator>, // ActuatorId order (ADR-0023)
    pub tendons: Vec<ResolvedTendon>, // TendonId order (ADR-0025 §4); MJCF only
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
                           mimic: Option<ResolvedMimic>, qpos_ref: f64 }  // qpos_ref: MJCF's ref, read by the MJCF writer alone
pub struct ResolvedActuator { pub name: String, pub target: ResolvedTarget, pub spec: ActuatorSpec,
                              pub ranges: ActuatorRanges }  // ranges as the document keeps them (ADR-0024)
pub enum ResolvedTarget { Joint(usize), Tendon(usize) }  // indexes ResolvedRobot::joints / ::tendons (ADR-0025 §4)
pub struct ResolvedTendon { pub name: String, pub joints: Vec<ResolvedTendonJoint>, pub range: Option<[f64; 2]>,
                            pub limited: Option<bool>, pub stiffness: f64, pub damping: f64, pub frictionloss: f64 }
pub struct ResolvedTendonJoint { pub joint: usize, pub coef: f64 }  // joint indexes ResolvedRobot::joints
pub struct ResolvedMimic { pub joint: usize, pub multiplier: f64, pub offset: f64 }  // joint indexes ResolvedRobot::joints
pub struct ExportOptions { format: Format, mesh_paths: MeshPathStyle, floating_base: bool }
pub struct Format { pub mjcf: bool, pub urdf: bool, pub sdf: bool }  // a set, not a choice; Default is all three
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
`ResolvedRobot::joints`**, not a `JointId`, so every writer stays a dumb
serialiser of the vector it already has (ADR-0004 §1, ADR-0013).

Every `Tendon` becomes a `ResolvedTendon` in `TendonId` order, its joints
the same indices (ADR-0025 §4), and an actuator's `ResolvedTarget` names a
joint or a tendon by index for the same reason. A tendon target has no
`Limits` behind it, so the MJCF writer derives neither range for such an
actuator and writes only what the file said; the URDF and SDF writers
ignore `tendons`, `qpos_ref` and a tendon-targeted actuator entirely.

Every `Frame` becomes a `ResolvedSite` on its parent link, in `FrameId`
order, carrying its link-frame pose unchanged; a frame on a link the tree
does not reach cannot get this far, because `validate` rejected the
document first. No frame adds an `ExportError` of its own — the name rules
under §Invariants are validation errors.

`MeshStore`
is the headless `MeshLookup` (files read and brought to meters as the
viewport does); the app implements the trait on its own store.

Each writer is then a dumb serialiser, and there are three of them —
`mjcf.rs`, `urdf.rs`, `sdf.rs` — over this one resolve. SDF cost a writer
and no new field (ADR-0016).

`ExportOptions::format` is a **set**, not a choice: `Format { mjcf, urdf,
sdf }`, defaulting to all three, written in the export dialog as three
checkboxes and spelled on the command line and in the SDK as `mjcf`,
`urdf`, `sdf`, `both` (the first two, kept from when there were only two)
or `all`. An empty set is expressible and is not a ready export — it would
write a `meshes/` folder and no file that reads it — so the Export button
refuses it. `MeshPathStyle` is read by the URDF and SDF writers; MJCF
ignores it, because it has `meshdir`.

## Format mapping

| Concept | URDF | MJCF | SDF (ADR-0016) |
|---|---|---|---|
| The file | `<robot name>` | `<mujoco model>` | `<sdf version="1.11"><model name>` — 1.11 is the first spec with `<axis><mimic>`, and `libsdformat14` (Gazebo Harmonic) reads it |
| Link | `<link name>` | `<body name>` nested under its parent body | `<link name>`, flat |
| Joint origin (child frame in parent frame) | `<joint><origin xyz rpy/>` | `<body pos quat>` of the child body | the **child link's** `<pose relative_to="«parent link»">`; the root link has no `<pose>`, and no `<pose>` is written on the joint at all — SDF's default for it is the child link frame, which is our joint frame |
| Joint axis (child frame) | `<joint><axis xyz/>` | `<joint axis>` inside the child body; no `pos` is written, and MuJoCo's default is the body origin, which is the joint frame | `<axis><xyz>`, with **no `expressed_in`**: its default frame is the joint frame, the same child link frame |
| Fixed | `type="fixed"` | no `<joint>` element | `type="fixed"` |
| Revolute | `type="revolute"` + `<limit lower upper effort velocity/>` | `type="hinge" range="lo hi"` — no `limited` attribute is written, because `autolimits="true"` on the `<compiler>` makes a written range a limiting one | `type="revolute"` + `<axis><limit><lower><upper>` |
| Continuous | `type="continuous"` | `type="hinge"` without `range` | `type="continuous"`, and **no `<limit>`**: SDF's ±inf default is what unlimited means, where `0 0` would be a locked joint |
| Prismatic | `type="prismatic"` + `<limit/>` | `type="slide" range="lo hi"` | `type="prismatic"` + `<axis><limit/>` |
| Visual geom | `<visual><origin/><geometry><mesh filename/></geometry></visual>` | `<geom class="visual" mesh=… pos quat/>` with `<default class="visual">` = `type="mesh" contype="0" conaffinity="0" group="2"` | `<visual name="«link»_visual_«i»"><pose/><geometry><mesh><uri/>` — SDF requires a name on every visual and collision and requires it unique within the link |
| Collision geom (one per resolved collision — N of them for a decomposition) | `<collision>…` | `<geom class="collision" type="mesh" mesh=… />` (mesh → MuJoCo takes the convex hull itself; primitives map directly), `<default class="collision">` = `group="3"`, translucent rgba | `<collision name="«link»_collision_«i»">…`, one per piece; SDF has no class system and none is invented |
| Primitive | `<box size>` (full extents), `<cylinder radius length>`, `<sphere radius>`; a capsule becomes a cylinder plus a warning | `type="box\|cylinder\|sphere\|capsule" size pos quat` — **`size` is half-extents / (radius, half-length)**, pinned by a test | `<box><size>` (full extents), `<cylinder><radius><length>`, `<sphere><radius>` and a **native `<capsule><radius><length>`**, its `length` the cylindrical part — the one place SDF beats URDF, so nothing is apologised for |
| Inertial | `<inertial><origin xyz(com) rpy="0 0 0"/><mass/><inertia ixx ixy ixz iyy iyz izz/></inertial>` | `<inertial pos(com) mass fullinertia="Ixx Iyy Izz Ixy Ixz Iyz"/>` — MuJoCo does the principal-axes decomposition itself (ADR-0008) | `<inertial><pose>`(com)`</pose><mass/><inertia><ixx>…<izz/>` — numbers in element bodies, not attributes |
| Mesh assets | `meshes/<stem>.stl`, path style per `MeshPathStyle` | `<asset><mesh name file/></asset>`, one per written **file** — a referenced mesh, plus each hull and decomposition piece; **meshes are written in meters as binary STL, no `scale`** (ADR-0008) | `<geometry><mesh><uri>`, the same `MeshPathStyle` with no new variant: `meshes/<stem>.stl`, `model://<name>/meshes/…` (SDF's own scheme, what `package://` is to URDF), or `file:///…` |
| Root | first `<link>` | `<worldbody>` child; `floating_base` in `ExportOptions` adds `<freejoint name="root"/>` | first `<link>`; a **fixed** base is `<joint name="world_joint" type="fixed"><parent>world</parent>`, and `floating_base` is that joint left out |
| Frame (`Frame`, a `ResolvedSite`) | a massless `<link name="tcp"/>` — no visual, collision or inertial — plus `<joint name="tcp_fixed" type="fixed">` with the frame pose as its `<origin xyz rpy/>`; the dummy links after every real link and the fixed joints after every real joint, so the file still reads root-first (ADR-0012) | `<site name pos quat/>` inside its body after the geoms, bare: no `size`, `group` or `rgba`, so MuJoCo's default 0.005 m sphere marks it (ADR-0012) | `<frame name attached_to="«link»"><pose/>` after the joints — `<pose>`'s default `relative_to` *is* `attached_to`, so the link-frame pose goes out unchanged, and no dummy link is needed |
| Mimic (`ResolvedMimic`) | `<mimic joint multiplier offset/>` inside the follower's `<joint>`, after `<dynamics>` | `<equality><joint joint1="follower" joint2="leader" polycoef="offset multiplier 0 0 0"/></equality>` after `</worldbody>` — a **soft** solver constraint, not a reduction (ADR-0013) | `<axis><mimic joint="«leader»"><multiplier><offset><reference>0` — SDF 1.11's own element. Its rule is `follower = multiplier·(leader − reference) + offset`, which at `reference = 0` is URDF's exactly |
| Tendon (`ResolvedTendon`) | nothing — URDF has no tendon | one `<tendon>` block after `</equality>`, one `<fixed name>` per table entry in `TendonId` order with a `<joint joint coef/>` child per joint: `limited` when the tendon says it, `range` when it has one — in MJCF's own absolute `Σ coef · qpos` terms, so **never shifted by a `qpos_ref`** (ADR-0025 §4) — and `stiffness`, `damping`, `frictionloss` only when non-zero, MuJoCo's own zeros standing otherwise | nothing, for URDF's reason |
| `Joint::qpos_ref` | nothing — URDF's `<limit>` is in deviation terms, which `Limits` already is | `ref="…"` on the `<joint>` when non-zero, **`range` shifted by it** (MuJoCo keeps both in `qpos` terms; the document's `q` is the deviation from the authored pose, `qpos = q + qpos_ref`, ADR-0025 §3), and a `ctrlrange` *derived* from the joint's range for a `<position>` shifted the same way; a `ctrlrange` the actuator says itself is written as said, being in `qpos` terms already. The `polycoef` of a mimic over such a joint is **not** shifted: MuJoCo's deviations are from `qpos0 = ref` | nothing, for URDF's reason |
| Actuator (`ActuatorSpec`) | nothing — `<transmission>` is a `ros_control` relic; a comment after the `<joint>` names the preset and its gains (a `General`'s three type names), like the `armature` one (ADR-0014) | one `<actuator>` block after `</equality>`, one element per table entry in `ActuatorId` order: `<position kp kv>` / `<velocity kv>` / `<motor gear>`, or `<general dyntype gaintype biastype dynprm gainprm biasprm gear>` for a `General` (ADR-0024) — the three type names always, each `prm` vector with the trailing zeros MuJoCo would fill in trimmed off but never emptied (`dynprm="0"` is zero; an absent `dynprm` is MuJoCo's one), `gear` like a motor's, and no derived `ctrlrange` since there is no preset to derive one from. `name` is the **actuator's own** (the joint's by default, ADR-0023) and `joint` — or `tendon`, for a tendon-targeted one (ADR-0025 §4) — the driven target's. Several may drive one target; MuJoCo sums them | nothing, and the same comment. Gazebo drives a joint through a `<plugin>` naming a C++ class, a shared library and a version of Gazebo — a simulator configuration, not a robot description (ADR-0016 §5) |
| Effort / velocity | `<limit effort velocity/>` | **the actuator's own ranges, else the joint's** (ADR-0024): a `ctrlrange` / `forcerange` in `Actuator::ranges` is written as it was said, with an explicit `ctrllimited` / `forcelimited` whenever its flag is `Some`; only where the actuator says nothing is `forcerange="-effort effort"` derived, and `ctrlrange` — `lower upper` for a position servo, `±velocity` for a velocity one, the normalised `-1 1` for a motor. A zero `effort` / `velocity` is the *unfilled* value, so the attribute is **omitted** and MuJoCo's unbounded default stands, never `0 0`; a flag of `false` beside no range derives nothing either. A joint **no** actuator targets keeps the comment naming what was dropped (ADR-0004 §4 as amended by ADR-0014, re-keyed by ADR-0023) | `<axis><limit><effort><velocity>`, **omitted when zero** for MJCF's reason: SDF's default is infinity and a literal `0` is a joint that can exert nothing |
| Dynamics | `<dynamics damping friction/>` | `damping`, `frictionloss`, `armature` on the `<joint>`, written only when non-zero | `<axis><dynamics><damping><friction>`, written only when either is non-zero; `armature` is a comment, as in URDF |
| Angles | radians | **`<compiler angle="radian" meshdir="meshes" autolimits="true"/>` is always written** — MJCF's default is degrees | radians; `<pose>` is `x y z roll pitch yaw` with URDF's own `Rz·Ry·Rx` convention, through the one `Pose::to_xyz_rpy` the URDF writer uses too |

MJCF's `polycoef` is `a0 a1 a2 a3 a4` in `y − y0 = a0 + a1(x − x0) + …`,
where `x` and `y` are the two joints' deviations from their `qpos0`, and
`qpos0` is each joint's `ref`. The document's `q` is exactly that deviation
(ADR-0025 §3), so `(offset, multiplier, 0, 0, 0)` is URDF's `q_y = k·q_x +
o` whatever `qpos_ref` either joint carries; the last three slots are
always zero, because non-linear coupling is not modelled.

**`pybullet` reads the SDF wrong**, by our choice and not by accident. It
ignores `//pose/@relative_to` — it will place a child link at its parent's
pose and report success — and reads every number as f32. The SDF is written
for `libsdformat`, which is Gazebo, Drake and the spec itself; the answer
for a pybullet user is the `.urdf` the same export writes (ADR-0016 §2).

Quaternion order: MJCF is `w x y z`; `glam::DQuat` is `x y z w`. One helper,
one place, tested (`xml::quat_wxyz`). Numbers are written with twelve
decimals, trailing zeros trimmed, `-0` folded, and `pos` / `quat` are
omitted at their defaults, so the files read like hand-written ones and the
golden tests stay legible.

`xml.rs` holds both halves. **Writing** is forty lines of escaping, since
the output is fixed-shape and an XML crate would only add a way to emit
something MuJoCo cannot read. It escapes an attribute value and, because
SDF puts its numbers in element bodies, a body too: `Xml::text` writes
`<tag>text</tag>` on one line, and `xml::pose6` is SDF's six-number
`<pose>` over the same `Pose::to_xyz_rpy` that `urdf::origin_attrs` goes
through — SDF's roll-pitch-yaw *is* URDF's, so there is one helper and not
two. **Reading** is a `quick-xml` DOM —
`xml::parse(&str) -> Result<Node, ParseError>`, tag plus attributes plus
children — because a file we did not write decides its own escaping, CDATA
and self-closing tags (ADR-0015 §2). Beside it, `Node::orientation`
collapses MJCF's five spellings of one rotation (`quat`, `euler`,
`axisangle`, `xyaxes`, `zaxis`) under an `AngleConvention` that defaults to
MJCF's own **degrees** and intrinsic `xyz`; that is the mirror of
`quat_wxyz` and obeys the same rule, one place and tested.

**Read back differently.** The table above is the writing direction. Only
two of the three columns are also read: **there is no SDF import**, and
`libsdformat` is a test dependency of the CI job that validates the writer
(ADR-0016 §6), never a runtime one. The MJCF import (§MJCF import)
reverses its own column except where MJCF has no room:

- `Limits::effort` and `velocity` return only from an `<actuator>`'s
  `forcerange` (any preset) and a **velocity** servo's `ctrlrange`. A joint
  with no actuator carries them only in the apologetic comment, which is
  text; a position servo's `ctrlrange` is the joint's position range and
  says nothing about rate. The actuator itself keeps both ranges as
  written (`Actuator::ranges`, ADR-0024), so what the writer derives for
  the joint and what it writes for the actuator can differ, and the
  actuator's wins.
- `ExportOptions::floating_base` is not a document field, so a
  `<freejoint>` comes back as a warning and the robot imports fixed to the
  world.
- A `<default>` class tree is resolved at import and dropped (ADR-0015 §3),
  so re-exporting a foreign file writes explicit attributes and our own two
  classes rather than the twenty it had.
- `CollisionPolicy::ConvexHull` and `ConvexDecomposition` are *parameters*
  and were never geometry (ADR-0011), so the hulls they produced come back
  as `CollisionPolicy::Meshes` of the same N files.
- A URDF frame is **not** read back as a `Frame` (ADR-0012) while an MJCF
  `<site>` is: nothing tells our exported dummy link from a real unweighed
  one, and a `<site>` is unambiguous.

## URDF import (`riggen-export::urdf_in`)

`urdf_in::load(path, &PackageMap, &dyn FileSource) -> Result<(Robot,
Vec<ImportWarning>), ImportError>` over `urdf-rs`. The source is where the
file's own bytes and every mesh's come from: `riggen_core::Disk` natively,
the files of one browser drop otherwise (ADR-0017). Links and joints map directly (URDF's
joint-frame convention is ours, ADR-0004); `<inertial>` becomes
`InertialSpec::Override` (the tensor rotated from the inertial frame into
link axes); a `<mesh scale>` becomes `MeshAsset::scale` (uniform only — a
non-uniform one is a warning and the largest component); `<collision>`
meshes that repeat the visuals are `SameAsVisual`, any other set is
`CollisionPolicy::Meshes` kept losslessly (OPEN 1, decided: no
downgrade), collision primitives are `Primitives`; `<mimic>` becomes a `Joint::mimic`
(ADR-0013), resolved in a second pass so it may name a joint further down
the file, with URDF's own defaults (multiplier 1, offset 0) filled in.
`Robot::actuators` is always empty on import: URDF has no actuator element
to read, which is also why nothing is dropped and no warning appears
(ADR-0014).
`package://name/rest`
resolves through the map, else `rest` beside the file, else `name/rest`
under an ancestor of the file's directory — `urdf-rs`'s own resolution
shells out to `rospack`. Those candidates are probed through the same
source, so in a browser "beside the file" means "in the same drop", where
paths are matched by file name and directories are ignored (ADR-0017 §3);
a mesh the set does not carry is the `MeshNotFound` a moved file already
was. Nothing is dropped silently: `ImportWarning::{
MimicDropped, SafetyControllerDropped, NonUniformScale,
PrimitiveVisualDropped, MixedCollisionDropped, NoInertial,
PackageUnresolved, MeshNotFound }` reach the status bar (File › Import
URDF…, a dropped `.urdf`, or `riggen --export … robot.urdf` on stderr).
`ImportWarning::MimicDropped` now carries a `reason`, and only for a
coupling the document cannot hold: a leader that is not a joint in the file
or is `fixed`, a joint following itself, a `<mimic>` on a `fixed` joint, a
ring of followers, a zero multiplier, a multiplier or offset that is not a
number, and a reach outside the follower's own limits. A **chain** is not
among them any more (ADR-0025): it is read, kept and written back. `validate` owns those rules — the import runs it and phrases
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
`ImportWarning` and `ImportError` are not this module's: they live in
`riggen-export::import` and MJCF speaks them too (§MJCF import,
ADR-0015 §4).

## MJCF import (`riggen-export::mjcf_in`)

`mjcf_in::load(path, &dyn FileSource) -> Result<(Robot,
Vec<ImportWarning>), ImportError>` over `xml::parse`. The source reads the
model file and hashes every mesh — the filesystem natively, one browser
drop otherwise, where `<compiler meshdir>` still composes a path and the
lookup still ends at a file name (ADR-0017 §3). MJCF is a MuJoCo *scene* rather than a robot
description, so the import reads the subset the document has fields for,
names everything else, and refuses the handful of shapes the document
cannot represent at all — ADR-0015 fixes which is which.

**Read before any body is**, because they change what every number after
them means: `<compiler angle eulerseq meshdir assetdir autolimits>` into a
`Compiler`, and the `<default>` class tree into a `Defaults` — flattened,
so each class already carries its ancestors and applying it to an element
is one lookup and a merge. An element's class is the one it names, else the
`childclass` in force, else `main`; an element that spells its own rotation
drops the class's, whichever of the five spellings each used. Neither
survives the read: the document holds resolved numbers, exactly as
`resolve` hands the writers resolved numbers (ADR-0004 §1).

**The tree.** `<worldbody>`'s single `<body>` is the root link and the
nesting is the tree. One `<joint>` becomes the edge above its body —
`hinge` with a range is `Revolute` and without it `Continuous`, `slide` is
`Prismatic`, no element at all is `Fixed` with an invented
`<link>_joint` name — and `range` / `damping` / `frictionloss` /
`armature` fill `Limits` and `Dynamics`. A range is a limit when `limited`
says so, else when `autolimits` is on and the range is not `0 0`; hinge
ranges are converted out of `<compiler angle>`, slide ranges are lengths.
`ref` is `Joint::qpos_ref`, converted like the range on a hinge and a
length on a slide, and **the range is shifted by it** on the way in
(ADR-0025 §3): MuJoCo keeps `range` in `qpos` terms, `Limits` bounds the
document's `q` — the deviation from the authored pose — and the writer
shifts it back, so a `ref` joint's range round-trips and every check
`validate` makes keeps its meaning. An invented ±1 m on an unranged slide
is already a deviation and is not shifted.
MJCF anchors a joint at `<joint pos>` in the **body** frame while the
document's joint frame *is* the child link frame (§Conventions), so the
link frame is moved onto the anchor: the parent joint's `origin` carries
the move and everything inside the body — the inertial CoM, the geom and
site poses, the child bodies' own poses — is re-expressed by subtracting
it. `<inertial>` becomes an `InertialSpec::Override`: `fullinertia` is
already in body axes about the CoM and is read straight back, `diaginertia`
is rotated by the element's own orientation.

**Geometry.** `<asset><mesh name file scale>` becomes a `MeshAsset` under
`meshdir` (else `assetdir`), registered once per file and scale; an unnamed
mesh is known by its file's stem, as in MuJoCo. `<geom type="mesh">`
becomes a `Geom`, the primitives come back with MJCF's half-extents undone
— and with `fromto`, which names a cylinder's or capsule's two ends and
replaces its pose. Which side a geom falls on is decided once per link:
our own `class="visual"` / `"collision"` if any geom uses them, else
`contype == 0 && conaffinity == 0` is a visual, else every geom is a
visual. A link whose file made that distinction and has nothing colliding
is `CollisionPolicy::None`; one whose file never made it is `SameAsVisual`,
so geometry is never silently lost. Collision meshes that repeat the
visuals are `SameAsVisual`, any other set is `Meshes`, primitives are
`Primitives`. Every `<site>` becomes a `Frame` on its body (ADR-0012's
promised symmetry), placed after the whole tree is read because frames and
links are one namespace and a link further down may take the name.

An inline `<mesh vertex face>` (MJCF's own syntax for a mesh with no
`file` attribute) has no backing file, but `.riggen` only ever stores
paths, never mesh bytes (`file::to_json` rebases `asset.path` and never
writes geometry into the JSON — §Nothing is dropped silently), so import
materializes one: the parsed mesh is written out as an ordinary `.stl`
next to the source MJCF, named after the `<mesh name>` (`inline_N` for an
unnamed one, suffixed to avoid colliding with a file already at that
path), then registered exactly like any other imported mesh — no new
`MeshAsset` variant, no schema bump. On the web build, where there is
nowhere to write, the bytes go into the app's `DroppedSet` (ADR-0017)
under a synthesized name instead. Either way the mesh becomes an
ordinary, disk- or drop-backed asset from the moment it is imported, so a
saved-and-reopened document never depends on the source MJCF still being
there.

**The two blocks after `</worldbody>`.** `<equality><joint polycoef>` is
`y − y0 = a0 + a1(x − x0) + …` over deviations from `qpos0 = ref`, which is
what the document's `q` is a deviation from, so it is a `Joint::mimic`
exactly when the last three coefficients are zero and the constraint is
active — a `<joint ref>` on either joint changes nothing (ADR-0013 as
amended by ADR-0025 §3).
`<tendon><fixed>` is a `Robot::tendons` entry (ADR-0025 §4): its
`<joint joint coef>` children in the file's order, its `range`, `limited`
and its passive dynamics. A fixed tendon's length is `Σ coef · qpos`,
**absolute** — not a deviation from `qpos0` the way an `<equality>`'s
`polycoef` is — so nothing about it is shifted by a `qpos_ref`, in either
direction. Its defaults come from `<default><tendon>`, which is where
MuJoCo files a `<fixed>`'s class.
`<position kp kv>` / `<velocity kv>` / `<motor gear>` / `<general>` driving
a joint **or a tendon** become **one `Robot::actuators` entry each**
(ADR-0014, ADR-0023, ADR-0024, ADR-0025 §4), under the `name` the file gave
— its target's only when the file said so — and a second element on an
already-driven target is a second entry, because MuJoCo sums their
controls. An element is read as a preset
**iff every attribute it carries is one the preset can express**
(ADR-0024 §2); a `<position gear>` or `<position timeconst>` is what
MuJoCo makes of it, a `General` with a fixed gain, an affine bias and the
filter — and a `<general>` is read whole: the three types by their MJCF
spellings (an unknown one is a parse error naming the choices), the three
`prm` vectors as written, `gear`'s first number, and the class tree
resolved through `<default class>` like any other element's attributes. A
`<default><general>` applies to `<general>` elements only: MuJoCo also
lets a preset inherit one, which riggen does not read. Each keeps its own `forcerange` and
`ctrlrange` as written, with their `ctrllimited` / `forcelimited`
(`Actuator::ranges`, ADR-0024): a written flag as written, an absent one
left `auto` beside a range — which the writer's own `autolimits="true"`
reproduces — and recorded as `false` where the file named no range (or
had `autolimits` off), because MuJoCo makes such an actuator unlimited
and the writer, told nothing, would hand it the joint's range instead.
The same two attributes are also where `Limits::effort` and
`Limits::velocity` come back from; the joint has one of each, so the
**first** actuator to drive it fills them and a later one leaves them
alone — and an actuator on a *tendon* fills neither, there being no joint
behind it.
`ActuatorDropped` is left for what the document still has no room for
(ADR-0024: **the target, not the tag**): an actuator driving a site or a
body, or a joint through `jointinparent`; a `<muscle>`, which needs a
`lengthrange` riggen does not compute; a tag outside the three presets and
`<general>` (`<intvelocity>`, `<damper>`, `<cylinder>`); a `joint` or a
`tendon` naming nothing in the file — a tendon riggen itself dropped
included — and whatever `validate` refuses,
dropped with its reason rather than failing the import (a `prm` vector
past ten entries included), naming the **actuator** since it has a name
of its own. Couplings go first, so an actuator is not taken down by an
`<equality>` that is itself about to go. What an element carries that no
variant holds — `actdim`, `actearly`, `actrange`, `lengthrange`,
`cranklength`, a preset's `dampratio` / `inheritrange`, which MuJoCo
computes from the model — is counted once per attribute name (`<general
actdim> × 1`) and the element is read without it; a `<fixed>`'s
`springlength`, `margin`, `armature` and `sol*` pairs are counted the same
way (ADR-0025 §4), and its `group` / `rgba` / `width` are decorative and
silent.
`TendonDropped` names a tendon the document cannot hold, and a tendon
goes **whole** — half a linear combination is a different tendon: a
`<spatial>`, which routes over sites, wrapping geoms and pulleys; a
`<fixed>` over a joint that is not in the file or is `Fixed`, over one
joint twice, with a zero coefficient, with no joints at all, with a range
that bounds nothing, or under a name already taken.
`<equality><tendon>`, a different element of the same name, is an
`ElementDropped` like any other unread one.

**Nothing is dropped silently.** Beside the shared `MimicDropped`,
`NonUniformScale`, `PrimitiveVisualDropped`, `MixedCollisionDropped`,
`NoInertial` and `MeshNotFound`, the MJCF variants are `ImportWarning::{
ElementDropped, GeomDropped, FreeJointDropped, ActuatorDropped,
TendonDropped, FrameDropped, MassFromGeomIgnored, LimitsInvented }`. Every
element the import does not read is counted and named once per tag —
`<camera> × 3`, not three warnings — and so is a robot-changing attribute like `<general
actdim>`; the decorating ones (`solref`, `friction`, `rgba`, `group`, …) are
not warned about, because one line each would bury the ones that matter.
`LimitsInvented` is the document's own gap: it has no unlimited
`Prismatic`, so an unranged `slide` gets ±1 m and is told so.

**The shapes it refuses** are `ImportError::{ CompositeJoint, JointOnRoot,
UnsupportedElement }` beside the URDF import's `UnsupportedJoint`,
`MultipleRoots`, `NoRoot`, `Io`, `Parse` and `Invalid`: several `<joint>`s
in one `<body>` (MuJoCo's ball or planar DoF against a tree whose joints
are its edges), a joint on the root body (whose link has no parent joint),
`type="ball"` and a `type="free"` anywhere but the root, and `<include>` /
`<replicate>` / `<attach>` / `<frame>` / `<compiler
coordinate="global">`. Each of those would change the robot if imported
anyway, which is the line ADR-0015 §5 draws. The composite one was asked
again in v0.4 and refused again with the corpus behind it (ADR-0022): its
message says what the shape means and that splitting the body into nested
bodies with one joint each is the same model, imported. A `<freejoint>` on the root
is a *warning*, not a refusal: it costs an `ExportOptions` field, not a
document one.

The imported document is untitled until saved.
`assets/fixtures/menagerie_style.xml` is the corpus file — degrees, a
`<default>` tree with a `childclass` and class names that are not ours, all
five orientation spellings, a `<joint ref>` on the pan with the chain's
first equality over it (ADR-0025 §3), a `fromto` capsule, a non-uniform mesh scale, a
`<general>` whose gains come through the class tree two levels up, a
`<tendon><fixed>` over two joints with a range, passive dynamics and a
`springlength` the document counts rather than keeps, driven by a `<motor>`
(ADR-0025 §4), an `<actuator>` named something other than its joint plus
a second one on that same joint (both kept, ADR-0023) carrying an explicit
`ctrllimited="false"` beside a range (ADR-0024), and eight elements the
document has no field for — and its test pins the result warning by warning. The round trip
itself is the `mujoco` CI job's fourth model: the arm exported, imported
and exported again, held to the *original* document's `fk.json`. The
corpus is its fifth (ADR-0024, ADR-0025): imported and re-exported, it
must load with zero warnings, agree with `fk`, and carry the original's
`<actuator>`, `<equality>` and `<tendon>` blocks element for element — the
model MuJoCo builds from each file compared per actuator, per coupling and
per tendon, with what riggen still drops a named allowlist in the script,
now empty (01 §Testing). Its `arm/thing.msh` is a
tetrahedron for that reason: MuJoCo's own `.msh` reader refuses fewer than
four vertices.

## Schema

`{ "schema_version": 6, "robot": Robot }`. `Robot` derives
`serde::{Serialize, Deserialize}` with `#[serde(deny_unknown_fields)]` on
every struct (the envelope too) so a typo in a hand-edited file fails loudly
with the field's name, and `#[serde(default)]` only on fields added in a
later version, alongside its `upgrade_` step and corpus fixture. `load`
reads the version first, tolerant of everything else, so a file outside
`OLDEST_SCHEMA_VERSION..=SCHEMA_VERSION` is reported as
`FileError::UnsupportedVersion` rather than as an unknown field; it then
walks the `upgrade_vN_to_vN+1` chain from the version it found up to
`SCHEMA_VERSION` **on the JSON** — each step takes `&mut serde_json::Value`
and rewrites the document into the next version's shape — and only then
parses it into `Robot`, and validates after resolving paths — a hand-edited
file that breaks an invariant is `FileError::Invalid`, not a half-open
document. The chain runs before the parse because `Robot` is *today's*
shape with `deny_unknown_fields`: a key an older version wrote and today's
struct no longer has would be refused before any step could move it. The
promise survives the ordering — a typo in a v1 file walks both steps and
still fails naming the field (`file::tests::an_unknown_key_in_an_old_file_still_fails_naming_it`);
what a `Value` cannot carry is a line/column, so only JSON that does not
parse at all reports one. `assets/fixtures/pendulum.riggen` (base + arm from the cube
fixtures, one revolute hinge, produced by `save` itself) is the first corpus
file and is frozen at **schema 1**: it is what the upgrade chain reads, and
`file::tests::corpus_pendulum_opens` keeps it opening forever and re-saving
as a v6 document that round-trips. `assets/fixtures/driven.riggen` is the
second, frozen at **schema 3**: small, mesh-less and hand-written, it is
what the first *non-empty* step moves, and
`file::tests::corpus_driven_upgrades_its_actuators_into_the_table` pins that
migration entry by entry. The byte-for-byte fixtures are the v6 ones,
`bracket.riggen` and `arm/arm.riggen`.

**Schema 2** adds `Joint::mimic` (ADR-0013) and **schema 3** adds
`Joint::actuator` (ADR-0014). Both `upgrade_` steps are empty for the same
reason — an older file simply has no such key and `#[serde(default)]` fills
in the `None` it meant — and they are the first two links of the chain
`load` walks; `file::tests::a_v2_file_opens_as_v6_with_no_actuators` pins
the second, from a v2 document made by dropping the actuators back out of
the committed fixture.

**Schema 4** moves the actuator off the joint into `Robot::actuators`
(ADR-0023), and `upgrade_v3_to_v4` is the first step that is not empty:
each `Some(spec)` becomes one table entry, named after its joint and
allocated from `next_id` **in `JointId` order** — numeric, not the
lexicographic order the JSON object's keys sit in — so one v3 file always
upgrades to the same ids. It is also why the chain moved onto the JSON at
all: `Joint` has no `actuator` key any more, and `deny_unknown_fields`
would refuse a v3 file before any step could move it.

**Schema 5** adds `Actuator::ranges` (ADR-0024), and `upgrade_v4_to_v5` is
empty for schema 2's and 3's reason: a v4 actuator has no `ranges` key,
and the `#[serde(default)]` — every range and flag `None`, the writer
deriving all four from the joint — is what a v4 document meant.
`file::tests::a_v4_file_opens_as_v6_with_the_writer_deriving_every_range`
pins it, from a v4 document made by dropping the `ranges` keys back out
of the committed fixture.

**Schema 6** adds `Joint::qpos_ref` (ADR-0025 §3) and `Robot::tendons`
(ADR-0025 §4) — one bump for the two, both landing in the same release —
and `upgrade_v5_to_v6` is empty for the same reason again: a v5 joint has
no `qpos_ref` key and the default of `0.0` is what it meant (the document's
`q` was already the deviation from the authored pose, and nothing riggen
wrote ever moved MuJoCo's zero), while a v5 document has no `tendons` key
and coupled nothing.
`file::tests::a_v5_file_opens_as_v6_with_every_joint_at_its_authored_zero`
pins both, from a v5 document made the same way.

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
