//! Every edit of the document is a [`Command`] (docs/02-data-model.md
//! §Commands and history). A command is applied to a clone, the result is
//! validated, and only then does it replace the document — so a refused
//! command leaves nothing behind. Joints are tree edges (ADR-0005): a link
//! arrives with its parent joint and leaves with its subtree; "connect two
//! links" is [`Command::Reparent`].

use std::fmt;

use crate::fk::{JointState, fk};
use crate::ids::{FrameId, GeomId, Id, JointId, LinkId, MeshId};
use crate::pose::Pose;
use riggen_mesh::glam::{DMat3, DVec3};

use crate::robot::{
    ActuatorSpec, CollisionPolicy, Frame, Geom, InertialSpec, Joint, Link, Material, MeshAsset,
    Robot,
};
use crate::validate::{ValidationError, validate};

#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Adds `link` under `parent` with `joint` as the edge. The command
    /// allocates the link and joint ids and overwrites `joint.parent` /
    /// `joint.child`; geom ids inside `link.visuals` are the caller's
    /// (`robot.next_id.alloc()`).
    AddLink {
        /// Boxed only to keep the enum small (clippy `large_enum_variant`).
        link: Box<Link>,
        parent: LinkId,
        joint: Joint,
    },
    /// Removes the link, its parent joint and its whole subtree (and any
    /// frame attached to it). The root is refused.
    RemoveLink(LinkId),
    RenameLink(LinkId, String),
    RenameJoint(JointId, String),
    AddGeom(LinkId, Geom),
    RemoveGeom(LinkId, GeomId),
    SetGeomPose(LinkId, GeomId, Pose),
    /// Replaces everything about a joint except its endpoints: `parent` /
    /// `child` in the value are ignored, the edge is what [`Reparent`]
    /// changes. One gesture = one `SetJoint`.
    ///
    /// [`Reparent`]: Command::Reparent
    SetJoint(JointId, Joint),
    /// Moves a joint's frame **without moving anything in the world**: the
    /// new `origin` (the child link frame in the parent frame) and `axis`
    /// (in the *new* child frame, since the joint frame is the child link
    /// frame) are written, and the child's geom poses, its own child joints'
    /// origins, its frames and an `Override` inertial are all re-expressed
    /// so no world pose in the zero configuration changes. Only the pivot
    /// the joint turns about moves.
    ///
    /// This is what "click the bore" and a gizmo on a *joint* commit
    /// (plans/m2-placement-ux OPEN 2). Like every frame-rewriting command it
    /// works in the zero configuration; a document at `q != 0` is reset
    /// first by the app (OPEN 1).
    MoveJointFrame {
        joint: JointId,
        origin: Pose,
        axis: DVec3,
    },
    /// Moves `link` (with its subtree) under `new_parent` by rewriting its
    /// parent joint's `parent`. Refused for the root and for a `new_parent`
    /// inside `link`'s own subtree. With `keep_world_pose` the joint origin
    /// is rewritten from FK so every world pose in the **zero configuration**
    /// (`q = 0`) is unchanged; without it the origin is kept and the part
    /// jumps.
    Reparent {
        link: LinkId,
        new_parent: LinkId,
        keep_world_pose: bool,
    },
    SetLinkMaterial(LinkId, Option<String>),
    /// Adds or replaces a material by name.
    UpsertMaterial(String, Material),
    /// Refused while a link uses the material.
    RemoveMaterial(String),
    /// Scale / fix-up edits. Registering an asset is `Robot::add_asset`.
    SetAsset(MeshId, MeshAsset),
    SetInertial(LinkId, InertialSpec),
    SetCollision(LinkId, CollisionPolicy),
    /// Makes `link` the root by reversing the joints on the path from it to
    /// the old root (parent and child swapped, origin inverted). Exact for
    /// `Fixed` joints; a movable joint on the path is refused, because its
    /// pivot cannot be expressed in the swapped child frame. No UI in M1.
    SetRoot(LinkId),
    /// Adds a named frame on `frame.parent`. The command allocates the
    /// `FrameId` and returns it as [`Created::Frame`], the way `AddLink`
    /// returns its link.
    AddFrame(Frame),
    RemoveFrame(FrameId),
    /// Replaces a frame whole — name, parent link and pose in one value,
    /// which is what the properties panel commits after an edit. Changing
    /// `parent` moves the frame to another link; the *caller* decides
    /// whether the world pose is kept (the panel does, through `fk`), so
    /// the command writes what it is given, like [`SetJoint`].
    ///
    /// [`SetJoint`]: Command::SetJoint
    SetFrame(FrameId, Frame),
    /// The tree's inline rename, beside `RenameLink` / `RenameJoint`.
    RenameFrame(FrameId, String),
    /// Gives **every movable joint** the same actuator, or takes it away
    /// (ADR-0014): the uniform case is the common one, and clicking seven
    /// joints is the tedium we exist to remove. One gesture, one command,
    /// one undo.
    ///
    /// A mimic follower is skipped rather than refused — it is already
    /// driven by its `<equality>`, and "apply to the whole model" must not
    /// fail because of a coupling elsewhere in the tree, the same way
    /// `RemoveLink` does not (ADR-0013). The per-joint edit is
    /// [`SetJoint`], which carries `actuator` like `mimic`.
    ///
    /// [`SetJoint`]: Command::SetJoint
    SetActuators(Option<ActuatorSpec>),
}

/// What a command created, for the caller that selects it afterwards.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Created {
    Link(LinkId),
    Frame(FrameId),
}

impl Created {
    /// The link, if that is what was created.
    pub fn link(self) -> Option<LinkId> {
        match self {
            Self::Link(l) => Some(l),
            Self::Frame(_) => None,
        }
    }

    /// The frame, if that is what was created.
    pub fn frame(self) -> Option<FrameId> {
        match self {
            Self::Frame(f) => Some(f),
            Self::Link(_) => None,
        }
    }
}

/// Why a command was refused. The document is untouched in every case.
#[derive(Debug, Clone, PartialEq)]
pub enum EditError {
    /// The result would violate an invariant (`validate`).
    Invalid(ValidationError),
    /// An id the command names is not in the document; `kind` is
    /// "link", "joint", "geom" or "mesh".
    UnknownId {
        kind: &'static str,
        id: String,
    },
    UnknownMaterial(String),
    /// `Reparent`: `new_parent` is `link` itself or one of its descendants.
    WouldCreateCycle {
        link: LinkId,
        new_parent: LinkId,
    },
    CannotRemoveRoot,
    CannotReparentRoot,
    /// `RemoveMaterial` while `link` (the lowest such id) uses it.
    MaterialInUse {
        material: String,
        link: LinkId,
    },
    /// `SetRoot` across a `Revolute` / `Continuous` / `Prismatic` joint.
    MovableJointOnRootPath(JointId),
}

impl fmt::Display for EditError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Invalid(e) => write!(f, "{e}"),
            Self::UnknownId { kind, id } => write!(f, "no {kind} {id} in the document"),
            Self::UnknownMaterial(m) => write!(f, "no material \"{m}\" in the document"),
            Self::WouldCreateCycle { link, new_parent } => write!(
                f,
                "cannot hang link {link} under {new_parent}: {new_parent} is inside its subtree"
            ),
            Self::CannotRemoveRoot => write!(f, "the root link cannot be removed"),
            Self::CannotReparentRoot => write!(f, "the root link cannot be reparented"),
            Self::MaterialInUse { material, link } => {
                write!(f, "material \"{material}\" is used by link {link}")
            }
            Self::MovableJointOnRootPath(j) => write!(
                f,
                "cannot change the root across movable joint {j}; only fixed joints can be reversed"
            ),
        }
    }
}

impl std::error::Error for EditError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Invalid(e) => Some(e),
            _ => None,
        }
    }
}

impl From<ValidationError> for EditError {
    fn from(e: ValidationError) -> Self {
        Self::Invalid(e)
    }
}

fn unknown<I: Id>(id: I) -> EditError {
    EditError::UnknownId {
        kind: I::KIND,
        id: id.to_string(),
    }
}

fn link_mut(robot: &mut Robot, id: LinkId) -> Result<&mut Link, EditError> {
    robot.links.get_mut(&id).ok_or_else(|| unknown(id))
}

fn joint_mut(robot: &mut Robot, id: JointId) -> Result<&mut Joint, EditError> {
    robot.joints.get_mut(&id).ok_or_else(|| unknown(id))
}

fn geom_mut(robot: &mut Robot, link: LinkId, geom: GeomId) -> Result<&mut Geom, EditError> {
    link_mut(robot, link)?
        .visuals
        .iter_mut()
        .find(|g| g.id == geom)
        .ok_or_else(|| unknown(geom))
}

fn require_link(robot: &Robot, id: LinkId) -> Result<(), EditError> {
    if robot.links.contains_key(&id) {
        Ok(())
    } else {
        Err(unknown(id))
    }
}

impl Command {
    /// Applies the command to `robot` and validates the result. On `Err`
    /// the document may be half-edited (a validation failure is found after
    /// the mutation) — [`History::apply`] works on a clone for that reason.
    /// Returns what `AddLink` / `AddFrame` created.
    ///
    /// [`History::apply`]: crate::history::History::apply
    pub fn apply(self, robot: &mut Robot) -> Result<Option<Created>, EditError> {
        let created = self.mutate(robot)?;
        validate(robot)?;
        Ok(created)
    }

    fn mutate(self, robot: &mut Robot) -> Result<Option<Created>, EditError> {
        match self {
            Command::AddLink {
                link,
                parent,
                mut joint,
            } => {
                require_link(robot, parent)?;
                let link_id: LinkId = robot.next_id.alloc();
                let joint_id: JointId = robot.next_id.alloc();
                joint.parent = parent;
                joint.child = link_id;
                robot.links.insert(link_id, *link);
                robot.joints.insert(joint_id, joint);
                return Ok(Some(Created::Link(link_id)));
            }
            Command::RemoveLink(link) => {
                require_link(robot, link)?;
                if link == robot.root {
                    return Err(EditError::CannotRemoveRoot);
                }
                let doomed = robot.subtree(link);
                robot
                    .joints
                    .retain(|_, j| !doomed.contains(&j.child) && !doomed.contains(&j.parent));
                robot.frames.retain(|_, f| !doomed.contains(&f.parent));
                for l in doomed {
                    robot.links.remove(&l);
                }
                // A survivor that followed one of the removed joints keeps
                // moving, freely: deleting a link is not the moment to
                // refuse an edit somewhere else in the tree (ADR-0013).
                let survivors: Vec<JointId> = robot.joints.keys().copied().collect();
                for joint in robot.joints.values_mut() {
                    if joint.mimic.is_some_and(|m| !survivors.contains(&m.joint)) {
                        joint.mimic = None;
                    }
                }
            }
            Command::RenameLink(link, name) => link_mut(robot, link)?.name = name,
            Command::RenameJoint(joint, name) => joint_mut(robot, joint)?.name = name,
            Command::AddGeom(link, geom) => link_mut(robot, link)?.visuals.push(geom),
            Command::RemoveGeom(link, geom) => {
                let visuals = &mut link_mut(robot, link)?.visuals;
                let at = visuals
                    .iter()
                    .position(|g| g.id == geom)
                    .ok_or_else(|| unknown(geom))?;
                visuals.remove(at);
            }
            Command::SetGeomPose(link, geom, pose) => geom_mut(robot, link, geom)?.pose = pose,
            Command::SetJoint(id, joint) => {
                let slot = joint_mut(robot, id)?;
                *slot = Joint {
                    parent: slot.parent,
                    child: slot.child,
                    ..joint
                };
            }
            Command::MoveJointFrame {
                joint,
                origin,
                axis,
            } => {
                let (child, delta) = {
                    let j = robot.joints.get(&joint).ok_or_else(|| unknown(joint))?;
                    // Old child coordinates → new child coordinates: a point
                    // sits at `origin_old ∘ p_old` in the parent either way,
                    // so `p_new = origin_new⁻¹ ∘ origin_old ∘ p_old`.
                    (j.child, origin.inverse().compose(&j.origin))
                };
                let j = joint_mut(robot, joint)?;
                j.origin = origin;
                j.axis = axis;

                let rotation = DMat3::from_quat(delta.r.normalize());
                if let Some(link) = robot.links.get_mut(&child) {
                    for geom in &mut link.visuals {
                        geom.pose = delta.compose(&geom.pose);
                    }
                    // A measured inertial is in link axes about `com`, and
                    // the link frame just moved under it (M3 has the UI).
                    if let InertialSpec::Override { com, inertia, .. } = &mut link.inertial {
                        *com = delta.transform_point(*com);
                        *inertia = rotation * *inertia * rotation.transpose();
                    }
                }
                for other in robot.joints.values_mut() {
                    if other.parent == child {
                        other.origin = delta.compose(&other.origin);
                    }
                }
                for frame in robot.frames.values_mut() {
                    if frame.parent == child {
                        frame.pose = delta.compose(&frame.pose);
                    }
                }
            }
            Command::Reparent {
                link,
                new_parent,
                keep_world_pose,
            } => {
                require_link(robot, link)?;
                require_link(robot, new_parent)?;
                if link == robot.root {
                    return Err(EditError::CannotReparentRoot);
                }
                if robot.is_in_subtree(new_parent, link) {
                    return Err(EditError::WouldCreateCycle { link, new_parent });
                }
                // A valid non-root link always has one; an orphan in a
                // document that somehow bypassed validation is reported as
                // unknown rather than silently given a joint.
                let joint_id = robot.parent_joint(link).ok_or_else(|| unknown(link))?;
                let origin = keep_world_pose.then(|| {
                    let world = fk(robot, &JointState::default());
                    world[&new_parent].inverse().compose(&world[&link])
                });
                let joint = joint_mut(robot, joint_id)?;
                joint.parent = new_parent;
                if let Some(origin) = origin {
                    joint.origin = origin;
                }
            }
            Command::SetLinkMaterial(link, material) => {
                link_mut(robot, link)?.material = material;
            }
            Command::UpsertMaterial(name, material) => {
                robot.materials.insert(name, material);
            }
            Command::RemoveMaterial(name) => {
                if !robot.materials.contains_key(&name) {
                    return Err(EditError::UnknownMaterial(name));
                }
                if let Some((&link, _)) = robot
                    .links
                    .iter()
                    .find(|(_, l)| l.material.as_deref() == Some(name.as_str()))
                {
                    return Err(EditError::MaterialInUse {
                        material: name,
                        link,
                    });
                }
                robot.materials.remove(&name);
            }
            Command::SetAsset(mesh, asset) => {
                let slot = robot.assets.get_mut(&mesh).ok_or_else(|| unknown(mesh))?;
                *slot = asset;
            }
            Command::SetInertial(link, spec) => link_mut(robot, link)?.inertial = spec,
            Command::SetCollision(link, policy) => link_mut(robot, link)?.collision = policy,
            Command::SetRoot(new_root) => {
                require_link(robot, new_root)?;
                // The path up from the new root, checked before anything moves.
                let mut path = Vec::new();
                let mut cursor = new_root;
                while cursor != robot.root {
                    let joint_id = robot.parent_joint(cursor).ok_or_else(|| unknown(cursor))?;
                    let joint = &robot.joints[&joint_id];
                    if joint.kind.is_movable() {
                        return Err(EditError::MovableJointOnRootPath(joint_id));
                    }
                    path.push(joint_id);
                    cursor = joint.parent;
                }
                for joint_id in path {
                    let joint = joint_mut(robot, joint_id)?;
                    std::mem::swap(&mut joint.parent, &mut joint.child);
                    joint.origin = joint.origin.inverse();
                }
                robot.root = new_root;
            }
            Command::AddFrame(frame) => {
                require_link(robot, frame.parent)?;
                let id: FrameId = robot.next_id.alloc();
                robot.frames.insert(id, frame);
                return Ok(Some(Created::Frame(id)));
            }
            Command::RemoveFrame(id) => {
                robot.frames.remove(&id).ok_or_else(|| unknown(id))?;
            }
            Command::SetFrame(id, frame) => {
                require_link(robot, frame.parent)?;
                let slot = robot.frames.get_mut(&id).ok_or_else(|| unknown(id))?;
                *slot = frame;
            }
            Command::RenameFrame(id, name) => {
                robot.frames.get_mut(&id).ok_or_else(|| unknown(id))?.name = name;
            }
            Command::SetActuators(actuator) => {
                for joint in robot.joints.values_mut() {
                    if joint.kind.is_movable() && joint.mimic.is_none() {
                        joint.actuator = actuator;
                    } else {
                        joint.actuator = None;
                    }
                }
            }
        }
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fk::{JointState, fk, frames};
    use crate::ids::FrameId;
    use crate::robot::{ActuatorSpec, Frame, JointKind, Limits, Mimic};
    use riggen_mesh::glam::{DQuat, DVec3};
    use std::collections::BTreeMap;
    use std::f64::consts::FRAC_PI_2;
    use std::path::PathBuf;

    const EPS: f64 = 1e-9;

    fn assert_pose_eq(a: &Pose, b: &Pose) {
        assert!((a.t - b.t).length() < EPS, "{a:?} != {b:?}");
        assert!(
            a.r.abs_diff_eq(b.r, EPS) || a.r.abs_diff_eq(-b.r, EPS),
            "{a:?} != {b:?}"
        );
    }

    /// Atomic like `History::apply`: a refused command leaves `robot` as it
    /// was, so one test can chain several refusals.
    fn apply(robot: &mut Robot, command: Command) -> Result<Option<Created>, EditError> {
        let mut next = robot.clone();
        let created = command.apply(&mut next)?;
        *robot = next;
        Ok(created)
    }

    fn add(robot: &mut Robot, parent: LinkId, name: &str, joint: Joint) -> LinkId {
        apply(
            robot,
            Command::AddLink {
                link: Box::new(Link::new(name)),
                parent,
                joint,
            },
        )
        .unwrap()
        .and_then(Created::link)
        .unwrap()
    }

    fn fixed(name: &str, origin: Pose) -> Joint {
        Joint {
            origin,
            ..Joint::fixed(name, LinkId::from_raw(0), LinkId::from_raw(0))
        }
    }

    fn revolute(name: &str, origin: Pose, axis: DVec3) -> Joint {
        Joint {
            kind: JointKind::Revolute,
            axis,
            limits: Some(Limits {
                lower: -1.0,
                upper: 1.0,
                effort: 0.0,
                velocity: 0.0,
            }),
            ..fixed(name, origin)
        }
    }

    fn asset() -> MeshAsset {
        MeshAsset {
            path: PathBuf::from("/a.stl"),
            content_hash: 0,
            scale: 1.0,
            fix_up: None,
        }
    }

    /// base ─ arm(revolute Z at +1x) ─ hand(fixed, +1x, yawed 90°) ─ tip
    /// (fixed at +1x), plus `tail` under base. Every joint has a non-trivial
    /// origin so a wrong frame shows up in the FK.
    fn arm() -> (Robot, [LinkId; 4]) {
        let mut robot = Robot::new("r");
        let root = robot.root;
        let x1 = Pose::from_translation(DVec3::X);
        let arm = add(&mut robot, root, "arm", revolute("shoulder", x1, DVec3::Z));
        let hand = add(
            &mut robot,
            arm,
            "hand",
            fixed(
                "wrist",
                Pose::from_xyz_rpy(DVec3::X, DVec3::new(0.0, 0.0, FRAC_PI_2)),
            ),
        );
        let tip = add(&mut robot, hand, "tip", fixed("tip_joint", x1));
        let tail = add(
            &mut robot,
            root,
            "tail",
            fixed("tail_joint", Pose::from_translation(-DVec3::Y)),
        );
        assert_eq!(validate(&robot), Ok(()));
        (robot, [arm, hand, tip, tail])
    }

    /// Every geom of every link at `q = 0`, in world coordinates: what a
    /// frame move must leave alone.
    fn world_geoms(robot: &Robot) -> Vec<(LinkId, GeomId, Pose)> {
        let world = fk(robot, &JointState::default());
        let mut out = Vec::new();
        for (&link, l) in &robot.links {
            for geom in &l.visuals {
                out.push((link, geom.id, world[&link].compose(&geom.pose)));
            }
        }
        out
    }

    /// The `arm()` chain with a geom on every link, so a frame move has
    /// geometry to re-express as well as child joints.
    fn arm_with_geoms() -> (Robot, [LinkId; 4]) {
        let (mut robot, links) = arm();
        let mesh = robot.add_asset(asset());
        for (i, &link) in links.iter().enumerate() {
            let id: GeomId = robot.next_id.alloc();
            let pose = Pose::from_xyz_rpy(
                DVec3::new(0.1 * i as f64, -0.2, 0.3),
                DVec3::new(0.2, -0.4, 0.6),
            );
            apply(
                &mut robot,
                Command::AddGeom(
                    link,
                    Geom {
                        id,
                        mesh,
                        pose,
                        color: None,
                    },
                ),
            )
            .unwrap();
        }
        (robot, links)
    }

    #[test]
    fn moving_a_joint_frame_changes_no_world_pose_at_zero() {
        let (mut robot, links) = arm_with_geoms();
        let [_arm, hand, ..] = links;
        let wrist = robot.parent_joint(hand).unwrap();
        let before_links = fk(&robot, &JointState::default());
        let before_geoms = world_geoms(&robot);

        // A new pivot well away from the old one, turned as well as moved.
        let origin = Pose::from_xyz_rpy(DVec3::new(1.4, 0.25, -0.6), DVec3::new(0.3, 0.9, -0.2));
        apply(
            &mut robot,
            Command::MoveJointFrame {
                joint: wrist,
                origin,
                axis: DVec3::Y,
            },
        )
        .unwrap();

        assert_eq!(robot.joints[&wrist].origin, origin);
        assert_eq!(robot.joints[&wrist].axis, DVec3::Y);
        let after = fk(&robot, &JointState::default());
        for (link, pose) in &before_links {
            if *link == hand {
                // The moved link's own frame is the one thing that changes.
                continue;
            }
            assert_pose_eq(&after[link], pose);
        }
        assert_pose_eq(&after[&hand], &origin_in_world(&robot, hand));
        // Every geom, on the moved link and on its grandchildren, stays put.
        for (link, geom, pose) in before_geoms {
            let (_, _, now) = world_geoms(&robot)
                .into_iter()
                .find(|(l, g, _)| *l == link && *g == geom)
                .expect("the geom survives");
            assert_pose_eq(&now, &pose);
        }
    }

    /// `fk` recomputed for one link, as a second opinion on the map above.
    fn origin_in_world(robot: &Robot, link: LinkId) -> Pose {
        let joint = robot.parent_joint(link).unwrap();
        let parent = robot.joints[&joint].parent;
        let world = fk(robot, &JointState::default());
        world[&parent].compose(&robot.joints[&joint].origin)
    }

    #[test]
    fn moving_a_joint_frame_moves_the_pivot() {
        // The point of the command: at `q != 0` the child turns about the
        // new axis through the new origin.
        let (mut robot, [arm, ..]) = arm();
        let shoulder = robot.parent_joint(arm).unwrap();
        apply(
            &mut robot,
            Command::MoveJointFrame {
                joint: shoulder,
                origin: Pose::from_translation(DVec3::new(2.0, 0.0, 0.0)),
                axis: DVec3::Z,
            },
        )
        .unwrap();
        let mut q = JointState::default();
        q.set(shoulder, FRAC_PI_2);
        let world = fk(&robot, &q);
        // The link frame is now at (2,0,0) — the old one, at (1,0,0), is a
        // point of the child that swings to (2,-1,0) about the new pivot.
        assert_pose_eq(
            &world[&arm],
            &Pose::new(DVec3::new(2.0, 0.0, 0.0), DQuat::from_rotation_z(FRAC_PI_2)),
        );
        assert!(
            (world[&arm].transform_point(DVec3::new(-1.0, 0.0, 0.0)) - DVec3::new(2.0, -1.0, 0.0))
                .length()
                < EPS
        );
    }

    #[test]
    fn a_zero_axis_is_refused_and_changes_nothing() {
        let (mut robot, [arm, ..]) = arm();
        let shoulder = robot.parent_joint(arm).unwrap();
        let before = robot.clone();
        let err = apply(
            &mut robot,
            Command::MoveJointFrame {
                joint: shoulder,
                origin: Pose::IDENTITY,
                axis: DVec3::ZERO,
            },
        )
        .unwrap_err();
        assert_eq!(err, EditError::Invalid(ValidationError::ZeroAxis(shoulder)));
        assert_eq!(robot, before, "a refused command leaves nothing behind");

        // An unknown joint is an unknown id, not a panic.
        let ghost = JointId::from_raw(999);
        assert_eq!(
            apply(
                &mut robot,
                Command::MoveJointFrame {
                    joint: ghost,
                    origin: Pose::IDENTITY,
                    axis: DVec3::Z,
                },
            )
            .unwrap_err(),
            unknown(ghost)
        );
    }

    #[test]
    fn a_no_op_frame_move_adds_no_history_entry() {
        let (mut robot, [arm, ..]) = arm_with_geoms();
        let shoulder = robot.parent_joint(arm).unwrap();
        let joint = robot.joints[&shoulder].clone();
        let mut history = crate::History::new();
        history
            .apply(
                &mut robot,
                Command::MoveJointFrame {
                    joint: shoulder,
                    origin: joint.origin,
                    axis: joint.axis,
                },
            )
            .unwrap();
        assert_eq!(history.undo_depth(), 0, "nothing changed, nothing recorded");
        assert!(!history.can_undo());
    }

    #[test]
    fn add_link_allocates_ids_and_sets_the_edge() {
        let mut robot = Robot::new("r");
        let root = robot.root;
        let before = robot.next_id.peek();
        let arm = add(&mut robot, root, "arm", fixed("j", Pose::IDENTITY));
        assert_eq!(arm.raw(), before);
        let jid = robot.parent_joint(arm).unwrap();
        assert_eq!(jid.raw(), before + 1);
        let joint = &robot.joints[&jid];
        assert_eq!((joint.parent, joint.child), (root, arm));
        assert_eq!(robot.links[&arm].name, "arm");

        // The joint's own endpoints are overwritten, whatever they said.
        let bogus = Joint::fixed("k", LinkId::from_raw(99), LinkId::from_raw(98));
        let leaf = add(&mut robot, arm, "leaf", bogus);
        let j = &robot.joints[&robot.parent_joint(leaf).unwrap()];
        assert_eq!((j.parent, j.child), (arm, leaf));
    }

    #[test]
    fn add_link_refuses_unknown_parent_and_bad_names() {
        let mut robot = Robot::new("r");
        let root = robot.root;
        let ghost = LinkId::from_raw(42);
        assert_eq!(
            apply(
                &mut robot,
                Command::AddLink {
                    link: Box::new(Link::new("x")),
                    parent: ghost,
                    joint: fixed("j", Pose::IDENTITY),
                }
            ),
            Err(EditError::UnknownId {
                kind: "link",
                id: "l42".into()
            })
        );
        assert_eq!(
            apply(
                &mut robot,
                Command::AddLink {
                    link: Box::new(Link::new("base_link")),
                    parent: root,
                    joint: fixed("j", Pose::IDENTITY),
                }
            ),
            Err(ValidationError::DuplicateLinkName("base_link".into()).into())
        );
        let err = apply(
            &mut robot,
            Command::AddLink {
                link: Box::new(Link::new("ok")),
                parent: root,
                joint: revolute("j", Pose::IDENTITY, DVec3::ZERO),
            },
        )
        .unwrap_err();
        assert!(matches!(
            err,
            EditError::Invalid(ValidationError::ZeroAxis(_))
        ));
        assert_eq!(err.to_string(), "joint j2 has a zero axis");
    }

    #[test]
    fn remove_link_takes_the_subtree_and_its_frames() {
        let (mut robot, [arm, hand, tip, tail]) = arm();
        let frame: FrameId = robot.next_id.alloc();
        robot.frames.insert(
            frame,
            Frame {
                name: "tcp".into(),
                parent: tip,
                pose: Pose::IDENTITY,
            },
        );
        let links_before = robot.links.len();
        apply(&mut robot, Command::RemoveLink(arm)).unwrap();
        assert_eq!(robot.links.len(), links_before - 3);
        for l in [arm, hand, tip] {
            assert!(!robot.links.contains_key(&l));
            assert_eq!(robot.parent_joint(l), None);
        }
        assert!(robot.links.contains_key(&tail));
        assert_eq!(robot.joints.len(), 1, "only tail_joint is left");
        assert!(robot.frames.is_empty());
        assert_eq!(validate(&robot), Ok(()));
    }

    #[test]
    fn frame_commands_add_set_rename_remove() {
        let (mut robot, [arm, _hand, tip, _tail]) = arm();
        let pose = Pose::new(DVec3::new(0.0, 0.0, 0.05), DQuat::from_rotation_x(0.3));
        let tcp = apply(
            &mut robot,
            Command::AddFrame(Frame {
                name: "tcp".into(),
                parent: tip,
                pose,
            }),
        )
        .unwrap()
        .and_then(Created::frame)
        .expect("AddFrame returns the frame it created");
        assert_eq!(robot.frames[&tcp].parent, tip);
        assert_pose_eq(&robot.frames[&tcp].pose, &pose);

        // A frame on a link that is not there, and a name that is a link's,
        // are both refused and leave nothing behind.
        assert_eq!(
            apply(
                &mut robot,
                Command::AddFrame(Frame {
                    name: "grip".into(),
                    parent: LinkId::from_raw(999),
                    pose: Pose::IDENTITY,
                })
            ),
            Err(unknown(LinkId::from_raw(999)))
        );
        assert_eq!(
            apply(
                &mut robot,
                Command::AddFrame(Frame {
                    name: "hand".into(),
                    parent: tip,
                    pose: Pose::IDENTITY,
                })
            ),
            Err(EditError::Invalid(ValidationError::DuplicateFrameName(
                "hand".into()
            )))
        );
        assert_eq!(robot.frames.len(), 1, "both refusals changed nothing");

        // `SetFrame` writes name, parent and pose in one go.
        let moved = Pose::from_translation(DVec3::Y * 0.2);
        apply(
            &mut robot,
            Command::SetFrame(
                tcp,
                Frame {
                    name: "grip".into(),
                    parent: arm,
                    pose: moved,
                },
            ),
        )
        .unwrap();
        assert_eq!(robot.frames[&tcp].name, "grip");
        assert_eq!(robot.frames[&tcp].parent, arm);
        assert_pose_eq(&robot.frames[&tcp].pose, &moved);

        apply(&mut robot, Command::RenameFrame(tcp, "tool0".into())).unwrap();
        assert_eq!(robot.frames[&tcp].name, "tool0");
        assert_eq!(robot.frames[&tcp].parent, arm, "a rename moves nothing");

        // Unknown ids are refused by every one of them.
        let ghost = FrameId::from_raw(999);
        for command in [
            Command::RemoveFrame(ghost),
            Command::RenameFrame(ghost, "x".into()),
            Command::SetFrame(
                ghost,
                Frame {
                    name: "x".into(),
                    parent: arm,
                    pose: Pose::IDENTITY,
                },
            ),
        ] {
            assert_eq!(apply(&mut robot, command), Err(unknown(ghost)));
        }

        apply(&mut robot, Command::RemoveFrame(tcp)).unwrap();
        assert!(robot.frames.is_empty());
        assert_eq!(validate(&robot), Ok(()));
    }

    #[test]
    fn moving_a_joint_frame_leaves_its_links_frames_in_the_world() {
        let (mut robot, [_arm, hand, _tip, _tail]) = arm_with_geoms();
        let joint = robot.parent_joint(hand).unwrap();
        let tcp = apply(
            &mut robot,
            Command::AddFrame(Frame {
                name: "tcp".into(),
                parent: hand,
                pose: Pose::new(DVec3::new(0.1, 0.0, 0.0), DQuat::from_rotation_z(0.4)),
            }),
        )
        .unwrap()
        .and_then(Created::frame)
        .unwrap();
        let before = frames(&robot, &JointState::default())[&tcp];

        apply(
            &mut robot,
            Command::MoveJointFrame {
                joint,
                origin: Pose::new(
                    DVec3::new(0.5, 0.25, 0.0),
                    DQuat::from_rotation_x(FRAC_PI_2),
                ),
                axis: DVec3::Y,
            },
        )
        .unwrap();
        // The joint frame moved under it, so the stored pose changed…
        assert!(
            (robot.frames[&tcp].pose.t - DVec3::new(0.1, 0.0, 0.0)).length() > EPS,
            "the frame is expressed in the new link frame"
        );
        // …and the world pose did not.
        assert_pose_eq(&frames(&robot, &JointState::default())[&tcp], &before);
    }

    #[test]
    fn reparent_leaves_a_frame_on_its_link() {
        let (mut robot, [arm, hand, _tip, tail]) = arm();
        let pose = Pose::from_translation(DVec3::Z * 0.3);
        let tcp = apply(
            &mut robot,
            Command::AddFrame(Frame {
                name: "tcp".into(),
                parent: hand,
                pose,
            }),
        )
        .unwrap()
        .and_then(Created::frame)
        .unwrap();
        apply(
            &mut robot,
            Command::Reparent {
                link: arm,
                new_parent: tail,
                keep_world_pose: true,
            },
        )
        .unwrap();
        // The frame stays on `hand` with the pose it had: `Reparent` moves
        // a subtree, and a frame is part of the link it hangs on.
        assert_eq!(robot.frames[&tcp].parent, hand);
        assert_pose_eq(&robot.frames[&tcp].pose, &pose);
        assert_eq!(validate(&robot), Ok(()));
    }

    #[test]
    fn remove_link_refuses_the_root_and_unknown_ids() {
        let (mut robot, _) = arm();
        let root = robot.root;
        assert_eq!(
            apply(&mut robot, Command::RemoveLink(root)),
            Err(EditError::CannotRemoveRoot)
        );
        assert_eq!(
            apply(&mut robot, Command::RemoveLink(LinkId::from_raw(77))),
            Err(unknown(LinkId::from_raw(77)))
        );
    }

    #[test]
    fn rename_link_and_joint_are_validated() {
        let (mut robot, [arm, ..]) = arm();
        let shoulder = robot.parent_joint(arm).unwrap();
        apply(&mut robot, Command::RenameLink(arm, "upper_arm".into())).unwrap();
        assert_eq!(robot.links[&arm].name, "upper_arm");
        apply(&mut robot, Command::RenameJoint(shoulder, "j0".into())).unwrap();
        assert_eq!(robot.joints[&shoulder].name, "j0");
        assert_eq!(
            apply(&mut robot, Command::RenameLink(arm, "tail".into())),
            Err(ValidationError::DuplicateLinkName("tail".into()).into())
        );
        assert_eq!(
            apply(&mut robot, Command::RenameJoint(shoulder, "1".into())),
            Err(ValidationError::InvalidName {
                kind: "joint",
                name: "1".into()
            }
            .into())
        );
        assert_eq!(
            apply(
                &mut robot,
                Command::RenameJoint(JointId::from_raw(55), "x".into())
            ),
            Err(EditError::UnknownId {
                kind: "joint",
                id: "j55".into()
            })
        );
    }

    #[test]
    fn geom_add_move_remove() {
        let (mut robot, [arm, ..]) = arm();
        let mesh = robot.add_asset(asset());
        let gid: GeomId = robot.next_id.alloc();
        let geom = Geom {
            id: gid,
            mesh,
            pose: Pose::IDENTITY,
            color: None,
        };
        apply(&mut robot, Command::AddGeom(arm, geom.clone())).unwrap();
        assert_eq!(robot.links[&arm].visuals, vec![geom.clone()]);
        // Same id twice is a validation error.
        assert_eq!(
            apply(&mut robot, Command::AddGeom(arm, geom.clone())),
            Err(ValidationError::DuplicateGeomId {
                link: arm,
                geom: gid
            }
            .into())
        );
        // Unknown mesh too.
        let ghost = Geom {
            id: robot.next_id.alloc(),
            mesh: MeshId::from_raw(500),
            ..geom
        };
        assert!(matches!(
            apply(&mut robot, Command::AddGeom(arm, ghost)),
            Err(EditError::Invalid(ValidationError::DanglingMesh { .. }))
        ));
        let moved = Pose::from_translation(DVec3::Z);
        apply(&mut robot, Command::SetGeomPose(arm, gid, moved)).unwrap();
        assert_eq!(robot.links[&arm].visuals[0].pose, moved);
        assert_eq!(
            apply(
                &mut robot,
                Command::SetGeomPose(arm, GeomId::from_raw(9), moved)
            ),
            Err(unknown(GeomId::from_raw(9)))
        );
        assert_eq!(
            apply(&mut robot, Command::RemoveGeom(arm, GeomId::from_raw(9))),
            Err(unknown(GeomId::from_raw(9)))
        );
        apply(&mut robot, Command::RemoveGeom(arm, gid)).unwrap();
        assert!(robot.links[&arm].visuals.is_empty());
    }

    #[test]
    fn set_joint_keeps_the_endpoints() {
        let (mut robot, [arm, _, _, tail]) = arm();
        let shoulder = robot.parent_joint(arm).unwrap();
        let mut edited = robot.joints[&shoulder].clone();
        edited.kind = JointKind::Continuous;
        edited.limits = None;
        edited.axis = DVec3::Y;
        edited.parent = tail; // ignored
        edited.child = tail; // ignored
        apply(&mut robot, Command::SetJoint(shoulder, edited)).unwrap();
        let j = &robot.joints[&shoulder];
        assert_eq!(j.kind, JointKind::Continuous);
        assert_eq!(j.axis, DVec3::Y);
        assert_eq!((j.parent, j.child), (robot.root, arm));
        assert_eq!(validate(&robot), Ok(()));

        let mut bad = j.clone();
        bad.kind = JointKind::Prismatic;
        assert_eq!(
            apply(&mut robot, Command::SetJoint(shoulder, bad)),
            Err(ValidationError::MissingLimits(shoulder).into())
        );
    }

    #[test]
    fn reparent_refuses_root_self_and_descendants() {
        let (mut robot, [arm, _, tip, tail]) = arm();
        let root = robot.root;
        assert_eq!(
            apply(
                &mut robot,
                Command::Reparent {
                    link: root,
                    new_parent: tail,
                    keep_world_pose: false
                }
            ),
            Err(EditError::CannotReparentRoot)
        );
        for new_parent in [arm, tip] {
            assert_eq!(
                apply(
                    &mut robot,
                    Command::Reparent {
                        link: arm,
                        new_parent,
                        keep_world_pose: true
                    }
                ),
                Err(EditError::WouldCreateCycle {
                    link: arm,
                    new_parent
                })
            );
        }
        assert_eq!(validate(&robot), Ok(()));
    }

    #[test]
    fn reparent_without_keep_keeps_the_origin() {
        let (mut robot, [arm, hand, _, tail]) = arm();
        let wrist = robot.parent_joint(hand).unwrap();
        let origin = robot.joints[&wrist].origin;
        apply(
            &mut robot,
            Command::Reparent {
                link: hand,
                new_parent: tail,
                keep_world_pose: false,
            },
        )
        .unwrap();
        assert_eq!(robot.joints[&wrist].parent, tail);
        assert_eq!(robot.joints[&wrist].origin, origin);
        assert_eq!(robot.child_joints(arm).count(), 0);
        assert_eq!(validate(&robot), Ok(()));
    }

    #[test]
    fn reparent_with_keep_world_pose_leaves_fk_unchanged() {
        let (mut robot, [_, hand, _, tail]) = arm();
        let q = JointState::default();
        let before = fk(&robot, &q);
        apply(
            &mut robot,
            Command::Reparent {
                link: hand,
                new_parent: tail,
                keep_world_pose: true,
            },
        )
        .unwrap();
        assert_eq!(validate(&robot), Ok(()));
        let after = fk(&robot, &q);
        assert_eq!(before.len(), after.len());
        for (link, pose) in &before {
            assert_pose_eq(&after[link], pose);
        }
        // The rewritten origin is hand in tail's frame: tail sits at -y, hand
        // at (2, 0, 0) yawed 90°, so the origin is (2, 1, 0) with that yaw.
        let wrist = robot.parent_joint(hand).unwrap();
        assert_pose_eq(
            &robot.joints[&wrist].origin,
            &Pose::new(DVec3::new(2.0, 1.0, 0.0), DQuat::from_rotation_z(FRAC_PI_2)),
        );
    }

    #[test]
    fn material_commands() {
        let (mut robot, [arm, ..]) = arm();
        let rubbery = Material {
            density: 900.0,
            color: [0.0, 0.0, 0.0, 1.0],
        };
        apply(&mut robot, Command::UpsertMaterial("foam".into(), rubbery)).unwrap();
        assert_eq!(robot.materials["foam"], rubbery);
        apply(
            &mut robot,
            Command::SetLinkMaterial(arm, Some("foam".into())),
        )
        .unwrap();
        assert_eq!(
            apply(&mut robot, Command::RemoveMaterial("foam".into())),
            Err(EditError::MaterialInUse {
                material: "foam".into(),
                link: arm
            })
        );
        assert_eq!(
            apply(
                &mut robot,
                Command::SetLinkMaterial(arm, Some("gold".into()))
            ),
            Err(ValidationError::DanglingMaterial {
                link: arm,
                material: "gold".into()
            }
            .into())
        );
        apply(&mut robot, Command::SetLinkMaterial(arm, None)).unwrap();
        apply(&mut robot, Command::RemoveMaterial("foam".into())).unwrap();
        assert!(!robot.materials.contains_key("foam"));
        assert_eq!(
            apply(&mut robot, Command::RemoveMaterial("foam".into())),
            Err(EditError::UnknownMaterial("foam".into()))
        );
        assert_eq!(
            apply(
                &mut robot,
                Command::UpsertMaterial("no way".into(), rubbery)
            ),
            Err(ValidationError::InvalidName {
                kind: "material",
                name: "no way".into()
            }
            .into())
        );
    }

    #[test]
    fn set_asset_inertial_collision() {
        let (mut robot, [arm, ..]) = arm();
        let mesh = robot.add_asset(asset());
        let scaled = MeshAsset {
            scale: 0.001,
            ..asset()
        };
        apply(&mut robot, Command::SetAsset(mesh, scaled.clone())).unwrap();
        assert_eq!(robot.assets[&mesh], scaled);
        assert_eq!(
            apply(&mut robot, Command::SetAsset(MeshId::from_raw(300), scaled)),
            Err(unknown(MeshId::from_raw(300)))
        );
        apply(
            &mut robot,
            Command::SetInertial(arm, InertialSpec::Hybrid { mass: 2.0 }),
        )
        .unwrap();
        assert_eq!(
            robot.links[&arm].inertial,
            InertialSpec::Hybrid { mass: 2.0 }
        );
        apply(
            &mut robot,
            Command::SetCollision(arm, CollisionPolicy::None),
        )
        .unwrap();
        assert_eq!(robot.links[&arm].collision, CollisionPolicy::None);
    }

    /// Relative pose between every pair of links; what `SetRoot` must keep.
    fn relative_poses(robot: &Robot) -> BTreeMap<(LinkId, LinkId), Pose> {
        let world = fk(robot, &JointState::default());
        let mut out = BTreeMap::new();
        for (&a, pa) in &world {
            for (&b, pb) in &world {
                out.insert((a, b), pa.inverse().compose(pb));
            }
        }
        out
    }

    #[test]
    fn set_root_reverses_fixed_joints_and_keeps_relative_poses() {
        let (mut robot, [arm, hand, tip, tail]) = arm();
        let old_root = robot.root;
        let before = relative_poses(&robot);
        // tail is one fixed joint away from base.
        apply(&mut robot, Command::SetRoot(tail)).unwrap();
        assert_eq!(robot.root, tail);
        assert_eq!(validate(&robot), Ok(()));
        let j = &robot.joints[&robot.parent_joint(old_root).unwrap()];
        assert_eq!(j.name, "tail_joint");
        assert_eq!((j.parent, j.child), (tail, old_root));
        assert_pose_eq(&j.origin, &Pose::from_translation(DVec3::Y));
        let after = relative_poses(&robot);
        assert_eq!(before.len(), after.len());
        for (pair, pose) in &before {
            assert_pose_eq(&after[pair], pose);
        }
        // Across the revolute shoulder it is refused before anything moves:
        // tip → hand → arm are fixed, then the shoulder is not.
        let shoulder = robot.parent_joint(arm).unwrap();
        let snapshot = robot.clone();
        assert_eq!(
            apply(&mut robot, Command::SetRoot(tip)),
            Err(EditError::MovableJointOnRootPath(shoulder))
        );
        assert_eq!(robot, snapshot);
        assert_eq!(robot.joints[&robot.parent_joint(hand).unwrap()].parent, arm);
    }

    /// A follower whose leader is deleted keeps moving — freely. Removing
    /// a link is not the moment to refuse an edit elsewhere in the tree;
    /// turning a leader `Fixed` is (ADR-0013).
    #[test]
    fn removing_a_leaders_subtree_frees_its_followers() {
        let (mut robot, [arm, _, _, tail]) = arm();
        let shoulder = robot.parent_joint(arm).unwrap();
        let tail_joint = robot.parent_joint(tail).unwrap();
        let follower = robot.joints.get_mut(&tail_joint).unwrap();
        follower.kind = JointKind::Revolute;
        follower.axis = DVec3::Z;
        follower.limits = Some(Limits {
            lower: -1.0,
            upper: 1.0,
            effort: 0.0,
            velocity: 0.0,
        });
        follower.mimic = Some(Mimic {
            joint: shoulder,
            multiplier: 1.0,
            offset: 0.0,
        });
        assert_eq!(validate(&robot), Ok(()));

        // Turning the leader into a `Fixed` joint is refused, naming the
        // follower — the document never half-holds a broken coupling.
        let mut demoted = robot.joints[&shoulder].clone();
        demoted.kind = JointKind::Fixed;
        demoted.limits = None;
        let err = apply(&mut robot, Command::SetJoint(shoulder, demoted)).unwrap_err();
        assert!(
            matches!(
                err,
                EditError::Invalid(ValidationError::MimicLeaderFixed { joint, leader })
                    if joint == tail_joint && leader == shoulder
            ),
            "{err:?}"
        );
        assert_eq!(robot.joints[&shoulder].kind, JointKind::Revolute);

        // Deleting it is not: the follower is simply freed.
        apply(&mut robot, Command::RemoveLink(arm)).unwrap();
        assert_eq!(robot.joints[&tail_joint].mimic, None);
        assert_eq!(validate(&robot), Ok(()));
    }

    /// "Apply to every movable joint" is one command and one undo, and it
    /// skips what `validate` would refuse rather than failing (ADR-0014).
    #[test]
    fn set_actuators_gives_every_free_movable_joint_the_same_one() {
        let (mut robot, [arm, _, _, tail]) = arm();
        let shoulder = robot.parent_joint(arm).unwrap();
        let tail_joint = robot.parent_joint(tail).unwrap();
        for j in [shoulder, tail_joint] {
            let joint = robot.joints.get_mut(&j).unwrap();
            joint.kind = JointKind::Revolute;
            joint.axis = DVec3::Z;
            joint.limits = Some(Limits {
                lower: -1.0,
                upper: 1.0,
                effort: 0.0,
                velocity: 0.0,
            });
        }
        robot.joints.get_mut(&tail_joint).unwrap().mimic = Some(Mimic {
            joint: shoulder,
            multiplier: 1.0,
            offset: 0.0,
        });
        assert_eq!(validate(&robot), Ok(()));

        let motor = ActuatorSpec::Motor { gear: 50.0 };
        apply(&mut robot, Command::SetActuators(Some(motor))).unwrap();
        assert_eq!(robot.joints[&shoulder].actuator, Some(motor));
        assert_eq!(
            robot.joints[&tail_joint].actuator, None,
            "a follower is already driven by its equality"
        );
        assert!(
            robot
                .joints
                .values()
                .filter(|j| !j.kind.is_movable())
                .all(|j| j.actuator.is_none()),
            "a fixed joint has nothing to actuate"
        );
        assert_eq!(validate(&robot), Ok(()));

        apply(&mut robot, Command::SetActuators(None)).unwrap();
        assert!(robot.joints.values().all(|j| j.actuator.is_none()));
    }
}
