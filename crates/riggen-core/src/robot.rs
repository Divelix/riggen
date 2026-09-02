//! The `Robot` document and its parts (docs/02-data-model.md §Core types).
//! Plain data with serde derives; every struct is `deny_unknown_fields` so a
//! typo in a hand-edited `.riggen` fails loudly (§Schema).

use std::collections::BTreeMap;
use std::path::PathBuf;

use riggen_mesh::glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};

use crate::ids::{FrameId, GeomId, IdGen, JointId, LinkId, MeshId};
use crate::pose::Pose;

/// The whole document. `frames` holds the named frames on links (TCP,
/// sensor mounts — ADR-0012); `assets` holds file references, never
/// geometry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Robot {
    pub name: String,
    pub links: BTreeMap<LinkId, Link>,
    pub joints: BTreeMap<JointId, Joint>,
    pub frames: BTreeMap<FrameId, Frame>,
    pub assets: BTreeMap<MeshId, MeshAsset>,
    pub root: LinkId,
    /// name → density (kg/m³), colour.
    pub materials: BTreeMap<String, Material>,
    /// Hands out every id in the document (ADR-0005).
    pub next_id: IdGen,
}

/// A mesh file the document references. `path` is absolute in memory and
/// rebased relative to the `.riggen` file on disk (docs/01-architecture.md
/// §File format).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshAsset {
    pub path: PathBuf,
    /// FNV-1a 64 over the file bytes, computed at registration.
    pub content_hash: u64,
    /// Unit conversion applied on load (0.001 for a millimetre file).
    pub scale: f64,
    /// Y-up → Z-up and the like; applied after `scale`.
    pub fix_up: Option<DQuat>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Link {
    pub name: String,
    pub visuals: Vec<Geom>,
    pub collision: CollisionPolicy,
    pub inertial: InertialSpec,
    /// Key into `Robot::materials`.
    pub material: Option<String>,
}

impl Link {
    /// An empty link with the M1 defaults: no geoms, `SameAsVisual`
    /// collision, computed inertial at the material density, no material.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            visuals: Vec::new(),
            collision: CollisionPolicy::SameAsVisual,
            inertial: InertialSpec::Computed {
                density_override: None,
            },
            material: None,
        }
    }
}

/// A visual geom; `(LinkId, GeomId)` is the viewport's instance key.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Geom {
    pub id: GeomId,
    pub mesh: MeshId,
    /// Geom in link frame.
    pub pose: Pose,
    pub color: Option<[f32; 4]>,
}

/// The edge from `parent` to `child`. `origin` is the child link frame in
/// the parent link frame; the axis is in the child frame (§Conventions).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Joint {
    pub name: String,
    pub kind: JointKind,
    pub parent: LinkId,
    pub child: LinkId,
    pub origin: Pose,
    /// Unit, in child frame; ignored for `Fixed`.
    pub axis: DVec3,
    /// Required for `Revolute` / `Prismatic`, absent for `Continuous`.
    pub limits: Option<Limits>,
    pub dynamics: Dynamics,
    /// This joint follows another one instead of moving freely (ADR-0013).
    /// Added in schema 2, hence the `default`: a v1 file has no such key.
    #[serde(default)]
    pub mimic: Option<Mimic>,
    /// What drives this joint in MJCF (ADR-0014). Added in schema 3, hence
    /// the `default`: a v2 file has no such key.
    #[serde(default)]
    pub actuator: Option<ActuatorSpec>,
}

/// A coupled degree of freedom: `q(this) = multiplier * q(joint) + offset`
/// — URDF's `<mimic>`, MJCF's `<equality><joint polycoef>` (ADR-0013).
///
/// `joint` is the **leader**: a movable joint, not this one, that does not
/// itself mimic. Chains are rejected by `validate`, so resolving a
/// follower's `q` is one pass and never recursive.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Mimic {
    pub joint: JointId,
    pub multiplier: f64,
    pub offset: f64,
}

/// The actuator a movable joint carries, as one of the three presets an RL
/// user reaches for (ADR-0014). MJCF-only: it is written as an `<actuator>`
/// element named after its joint, with `ctrlrange` from the joint's limits
/// and `forcerange` from `Limits::effort`. URDF has no actuator and says so
/// in a comment.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ActuatorSpec {
    /// `<position kp kv>`: a servo tracking a target angle / offset.
    Position { kp: f64, kv: f64 },
    /// `<velocity kv>`: a servo tracking a target rate.
    Velocity { kv: f64 },
    /// `<motor gear>`: direct force / torque, `ctrl` normalised to `-1 1`.
    Motor { gear: f64 },
}

impl ActuatorSpec {
    /// The MJCF element name, and what the panel's combo labels it.
    pub fn kind_name(self) -> &'static str {
        match self {
            Self::Position { .. } => "position",
            Self::Velocity { .. } => "velocity",
            Self::Motor { .. } => "motor",
        }
    }
}

impl Joint {
    /// A `Fixed` joint at identity from `parent` to `child`.
    pub fn fixed(name: impl Into<String>, parent: LinkId, child: LinkId) -> Self {
        Self {
            name: name.into(),
            kind: JointKind::Fixed,
            parent,
            child,
            origin: Pose::IDENTITY,
            axis: DVec3::Z,
            limits: None,
            dynamics: Dynamics::default(),
            mimic: None,
            actuator: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum JointKind {
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
}

impl JointKind {
    /// Whether the joint has a degree of freedom (a `q` and a slider).
    pub fn is_movable(self) -> bool {
        !matches!(self, Self::Fixed)
    }

    /// Whether `Joint::limits` must be present.
    pub fn requires_limits(self) -> bool {
        matches!(self, Self::Revolute | Self::Prismatic)
    }
}

/// Radians or meters depending on the joint kind.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Limits {
    pub lower: f64,
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64,
}

/// MJCF-side joint parameters; all zero by default.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Dynamics {
    pub damping: f64,
    pub friction: f64,
    pub armature: f64,
}

/// Density is stored only until M3's mass properties consume it.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Material {
    /// kg/m³.
    pub density: f64,
    /// Linear RGBA.
    pub color: [f32; 4],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum InertialSpec {
    /// From meshes at the material density, or `density_override`.
    Computed { density_override: Option<f64> },
    /// Measured values win; the computed ones stay visible for comparison.
    /// `inertia` is about `com`, in link axes.
    Override {
        mass: f64,
        com: DVec3,
        inertia: DMat3,
    },
    /// Computed tensor and CoM, scaled to a weighed mass.
    Hybrid { mass: f64 },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CollisionPolicy {
    None,
    SameAsVisual,
    /// One hull per visual geom.
    ConvexHull,
    /// Hand-placed primitives in link frame.
    Primitives(Vec<Primitive>),
    /// Collision meshes that are not the visuals, in link frame — what a
    /// URDF `<collision><mesh>` imports to, losslessly (M3, OPEN 1).
    /// Editable geom by geom in the properties panel (pose, remove, add a
    /// file); a v1 file without the variant still reads.
    Meshes(Vec<Geom>),
    /// Approximate convex decomposition of every visual mesh: N convex
    /// pieces that keep the part's concavity, where one hull would fill it
    /// (ADR-0011). The three fields mirror `riggen_mesh::DecompParams` and
    /// are the *parameters* — the pieces are derived at export from the
    /// mesh and these numbers, never stored (ADR-0008, extended). Each has
    /// a serde default, so a v1 file written before they existed — one
    /// carrying only `max_hulls` — still reads.
    ConvexDecomposition {
        #[serde(default = "default_max_hulls")]
        max_hulls: u32,
        #[serde(default = "default_resolution")]
        resolution: u32,
        #[serde(default = "default_concavity")]
        concavity: f64,
    },
}

// One source of truth for the three defaults: the algorithm's own, in
// `riggen-mesh`. Core does not re-export `DecompParams` — the document type
// stays plain serde data that `riggen-export` maps across.
fn default_max_hulls() -> u32 {
    riggen_mesh::DecompParams::default().max_hulls
}

fn default_resolution() -> u32 {
    riggen_mesh::DecompParams::default().resolution
}

fn default_concavity() -> f64 {
    riggen_mesh::DecompParams::default().concavity
}

impl CollisionPolicy {
    /// The geoms this policy holds itself (`Meshes`), so callers that walk
    /// every mesh reference in a link do not match on the variant.
    pub fn geoms(&self) -> &[Geom] {
        match self {
            Self::Meshes(geoms) => geoms,
            _ => &[],
        }
    }
}

/// A collision primitive in link frame (`pose` is the primitive's centre
/// frame; cylinders and capsules extend along its Z axis).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Primitive {
    Box {
        pose: Pose,
        size: DVec3,
    },
    Cylinder {
        pose: Pose,
        radius: f64,
        length: f64,
    },
    Sphere {
        pose: Pose,
        radius: f64,
    },
    Capsule {
        pose: Pose,
        radius: f64,
        length: f64,
    },
}

/// A named frame on a link: a TCP, a sensor mount, a grasp pose. `pose` is
/// in the parent link frame. Exported as an MJCF `<site>` and a URDF
/// massless dummy link on a fixed joint (ADR-0012); its name shares the
/// links' namespace.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Frame {
    pub name: String,
    pub parent: LinkId,
    pub pose: Pose,
}

impl Robot {
    /// An empty document: one root link `base_link` and the default
    /// materials.
    pub fn new(name: impl Into<String>) -> Self {
        let mut next_id = IdGen::new();
        let root: LinkId = next_id.alloc();
        let mut links = BTreeMap::new();
        links.insert(root, Link::new("base_link"));
        Self {
            name: name.into(),
            links,
            joints: BTreeMap::new(),
            frames: BTreeMap::new(),
            assets: BTreeMap::new(),
            root,
            materials: Self::default_materials(),
            next_id,
        }
    }

    /// Seeded into every new document. Densities in kg/m³, colours linear
    /// RGBA chosen to be told apart in the viewport.
    pub fn default_materials() -> BTreeMap<String, Material> {
        fn m(density: f64, color: [f32; 4]) -> Material {
            Material { density, color }
        }
        BTreeMap::from([
            ("aluminium".to_owned(), m(2700.0, [0.77, 0.78, 0.80, 1.0])),
            ("steel".to_owned(), m(7850.0, [0.45, 0.47, 0.50, 1.0])),
            ("PLA".to_owned(), m(1240.0, [0.90, 0.55, 0.20, 1.0])),
            ("ABS".to_owned(), m(1040.0, [0.20, 0.45, 0.85, 1.0])),
            ("nylon".to_owned(), m(1150.0, [0.92, 0.92, 0.85, 1.0])),
            ("rubber".to_owned(), m(1100.0, [0.15, 0.15, 0.15, 1.0])),
        ])
    }

    /// Registers a mesh file. Not a command: undoing "drop a mesh" undoes
    /// the `AddLink` / `AddGeom`, the asset stays for the session (so redo
    /// does not reload the file) and an unreferenced asset is pruned on save.
    pub fn add_asset(&mut self, asset: MeshAsset) -> MeshId {
        let id = self.next_id.alloc();
        self.assets.insert(id, asset);
        id
    }

    /// The joint whose child is `link`; `None` for the root (and for an
    /// orphan in an invalid document).
    pub fn parent_joint(&self, link: LinkId) -> Option<JointId> {
        self.joints
            .iter()
            .find(|(_, j)| j.child == link)
            .map(|(id, _)| *id)
    }

    /// Joints whose parent is `link`, in id order.
    pub fn child_joints(&self, link: LinkId) -> impl Iterator<Item = JointId> + '_ {
        self.joints
            .iter()
            .filter(move |(_, j)| j.parent == link)
            .map(|(id, _)| *id)
    }

    /// `link` and every descendant, depth-first in parent-before-child
    /// order; just `[link]` for a leaf. A loop (rejected by `validate`) is
    /// visited once.
    pub fn subtree(&self, link: LinkId) -> Vec<LinkId> {
        let mut out = Vec::new();
        let mut stack = vec![link];
        while let Some(l) = stack.pop() {
            if out.contains(&l) {
                continue;
            }
            out.push(l);
            // Reverse so the lowest child id is popped (visited) first.
            let children: Vec<LinkId> = self
                .child_joints(l)
                .map(|j| self.joints[&j].child)
                .collect();
            stack.extend(children.into_iter().rev());
        }
        out
    }

    /// Whether `link` is `ancestor` itself or hangs somewhere below it.
    pub fn is_in_subtree(&self, link: LinkId, ancestor: LinkId) -> bool {
        self.subtree(ancestor).contains(&link)
    }

    /// Mesh ids referenced by at least one geom, visual or collision.
    pub fn referenced_assets(&self) -> std::collections::BTreeSet<MeshId> {
        self.links
            .values()
            .flat_map(|l| l.visuals.iter().chain(l.collision.geoms()))
            .map(|g| g.mesh)
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::Id;

    #[test]
    fn new_robot_has_a_root_and_the_default_materials() {
        let robot = Robot::new("r");
        assert_eq!(robot.links.len(), 1);
        assert_eq!(robot.links[&robot.root].name, "base_link");
        assert!(robot.joints.is_empty());
        assert_eq!(robot.materials.len(), 6);
        assert_eq!(robot.materials["aluminium"].density, 2700.0);
        assert_eq!(robot.parent_joint(robot.root), None);
        assert_eq!(robot.child_joints(robot.root).count(), 0);
    }

    #[test]
    fn add_asset_hands_out_fresh_ids() {
        let mut robot = Robot::new("r");
        let asset = MeshAsset {
            path: PathBuf::from("/tmp/a.stl"),
            content_hash: 1,
            scale: 0.001,
            fix_up: None,
        };
        let a = robot.add_asset(asset.clone());
        let b = robot.add_asset(asset);
        assert_ne!(a, b);
        assert_ne!(a.raw(), robot.root.raw(), "one counter across id kinds");
        assert_eq!(robot.assets.len(), 2);
        assert!(robot.referenced_assets().is_empty());
    }

    #[test]
    fn subtree_is_parent_before_child() {
        let mut robot = Robot::new("r");
        let root = robot.root;
        let add = |robot: &mut Robot, parent: LinkId, name: &str| -> LinkId {
            let link: LinkId = robot.next_id.alloc();
            robot.links.insert(link, Link::new(name));
            let joint: JointId = robot.next_id.alloc();
            robot
                .joints
                .insert(joint, Joint::fixed(format!("{name}_j"), parent, link));
            link
        };
        let a = add(&mut robot, root, "a");
        let b = add(&mut robot, a, "b");
        let c = add(&mut robot, root, "c");
        assert_eq!(robot.subtree(root), vec![root, a, b, c]);
        assert_eq!(robot.subtree(a), vec![a, b]);
        assert_eq!(robot.subtree(c), vec![c]);
        assert!(robot.is_in_subtree(b, a));
        assert!(robot.is_in_subtree(a, a));
        assert!(!robot.is_in_subtree(a, b));
        assert!(!robot.is_in_subtree(c, a));
    }

    #[test]
    fn serde_round_trip_and_unknown_field_rejected() {
        let mut robot = Robot::new("r");
        let mesh = robot.add_asset(MeshAsset {
            path: PathBuf::from("/tmp/a.stl"),
            content_hash: 7,
            scale: 1.0,
            fix_up: Some(DQuat::from_rotation_x(1.0)),
        });
        let child: LinkId = robot.next_id.alloc();
        let geom_id: GeomId = robot.next_id.alloc();
        let mut link = Link::new("arm");
        link.visuals.push(Geom {
            id: geom_id,
            mesh,
            pose: Pose::IDENTITY,
            color: Some([1.0, 0.0, 0.0, 1.0]),
        });
        link.material = Some("steel".into());
        robot.links.insert(child, link);
        let joint: JointId = robot.next_id.alloc();
        robot
            .joints
            .insert(joint, Joint::fixed("base_to_arm", robot.root, child));

        let json = serde_json::to_string_pretty(&robot).unwrap();
        assert!(json.contains("\"root\": \"l0\""), "{json}");
        // root, mesh, child link, geom, joint → five ids handed out.
        assert!(json.contains("\"next_id\": 5"), "{json}");
        let back: Robot = serde_json::from_str(&json).unwrap();
        assert_eq!(back, robot);

        let typo = json.replace("\"materials\"", "\"materialz\"");
        let err = serde_json::from_str::<Robot>(&typo)
            .unwrap_err()
            .to_string();
        assert!(err.contains("materialz"), "{err}");
    }
}
