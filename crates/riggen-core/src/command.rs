//! Every edit of the document is a [`Command`] (docs/02-data-model.md
//! §Commands and history). A command is applied to a clone, the result is
//! validated, and only then does it replace the document — so a refused
//! command leaves nothing behind. Joints are tree edges (ADR-0005): a link
//! arrives with its parent joint and leaves with its subtree; "connect two
//! links" is [`Command::Reparent`].

use std::fmt;

use crate::fk::{JointState, fk};
use crate::ids::{GeomId, Id, JointId, LinkId, MeshId};
use crate::pose::Pose;
use crate::robot::{CollisionPolicy, Geom, InertialSpec, Joint, Link, Material, MeshAsset, Robot};
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
    /// Returns the link `AddLink` created.
    ///
    /// [`History::apply`]: crate::history::History::apply
    pub fn apply(self, robot: &mut Robot) -> Result<Option<LinkId>, EditError> {
        let created = self.mutate(robot)?;
        validate(robot)?;
        Ok(created)
    }

    fn mutate(self, robot: &mut Robot) -> Result<Option<LinkId>, EditError> {
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
                return Ok(Some(link_id));
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
        }
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fk::{JointState, fk};
    use crate::ids::FrameId;
    use crate::robot::{Frame, JointKind, Limits};
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
    fn apply(robot: &mut Robot, command: Command) -> Result<Option<LinkId>, EditError> {
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
}
