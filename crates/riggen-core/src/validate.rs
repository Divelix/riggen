//! The document invariants (docs/02-data-model.md §Core types). The command
//! layer never produces a violating state (ADR-0005); `validate` is the
//! safety net behind every command and the gate before save and export.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use crate::ids::{FrameId, GeomId, JointId, LinkId, MeshId};
use crate::robot::Robot;

#[derive(Debug, Clone, PartialEq)]
pub enum ValidationError {
    /// `Robot::root` names no link.
    RootMissing(LinkId),
    /// A joint has the root as its child.
    RootHasParent(JointId),
    /// A non-root link with no parent joint.
    Orphan(LinkId),
    /// Two joints claim the same child.
    MultipleParents {
        link: LinkId,
        first: JointId,
        second: JointId,
    },
    /// Links whose parent chain never reaches the root; `links` is the loop
    /// in parent order, starting from the lowest id in it.
    Cycle(Vec<LinkId>),
    /// A joint's `parent` or `child` names no link.
    DanglingJointLink {
        joint: JointId,
        link: LinkId,
    },
    /// A geom's `mesh` names no asset.
    DanglingMesh {
        link: LinkId,
        geom: GeomId,
        mesh: MeshId,
    },
    /// A link's `material` is not in `Robot::materials`.
    DanglingMaterial {
        link: LinkId,
        material: String,
    },
    /// A frame's `parent` names no link.
    DanglingFrameLink {
        frame: FrameId,
        link: LinkId,
    },
    /// Two geoms in one link share an id.
    DuplicateGeomId {
        link: LinkId,
        geom: GeomId,
    },
    DuplicateLinkName(String),
    DuplicateJointName(String),
    /// Two frames share a name, or a frame's name is also a link's: frames
    /// and links are **one namespace**, because a URDF frame is written as
    /// a `<link>` (ADR-0012).
    DuplicateFrameName(String),
    /// The fixed joint the URDF writer gives a frame, `<frame>_fixed`, is
    /// already a joint's name — the same one namespace (ADR-0012).
    FrameJointNameCollision {
        frame: String,
        joint: String,
    },
    /// Not an XML name / MJCF identifier: `[A-Za-z_][A-Za-z0-9_.-]*`.
    /// `kind` is "link", "joint", "frame" or "material".
    InvalidName {
        kind: &'static str,
        name: String,
    },
    /// A movable joint with a zero (or non-finite) axis.
    ZeroAxis(JointId),
    /// `Revolute` / `Prismatic` without limits.
    MissingLimits(JointId),
    /// `lower > upper`.
    LimitsUnordered {
        joint: JointId,
        lower: f64,
        upper: f64,
    },
    /// A non-finite number where the document needs a real one.
    NonFinite {
        what: String,
    },
    // ---- mimic joints (ADR-0013) -----------------------------------------
    /// A mimic's `joint` names no joint.
    DanglingMimicJoint {
        joint: JointId,
        leader: JointId,
    },
    /// A joint mimics itself.
    SelfMimic(JointId),
    /// A `Fixed` joint carries a mimic: it has no degree of freedom to
    /// drive, and MJCF writes no `<joint>` for it to couple.
    MimicOnFixedJoint(JointId),
    /// A mimic whose leader is `Fixed`, so there is nothing to follow.
    MimicLeaderFixed {
        joint: JointId,
        leader: JointId,
    },
    /// A mimic whose leader itself mimics. Chains are out of scope
    /// (ADR-0013): consumer support for them is a lottery and MuJoCo wants
    /// them flattened against the free joint anyway.
    MimicChain {
        joint: JointId,
        leader: JointId,
    },
    /// A mimic with a zero `multiplier`: the follower would be pinned to a
    /// constant, which is a `Fixed` joint spelled the hard way.
    ZeroMimicMultiplier(JointId),
    /// The leader's range mapped through `(multiplier, offset)` does not
    /// fit inside the follower's own limits; `lower` / `upper` are that
    /// mapped reach. MJCF would give the follower a `range` its equality
    /// constraint fights (ADR-0013).
    MimicExceedsLimits {
        joint: JointId,
        leader: JointId,
        lower: f64,
        upper: f64,
    },
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::RootMissing(l) => write!(f, "root link {l} does not exist"),
            Self::RootHasParent(j) => write!(f, "joint {j} makes the root link a child"),
            Self::Orphan(l) => write!(f, "link {l} has no parent joint"),
            Self::MultipleParents {
                link,
                first,
                second,
            } => write!(f, "link {link} has two parent joints, {first} and {second}"),
            Self::Cycle(links) => {
                let names: Vec<String> = links.iter().map(ToString::to_string).collect();
                write!(f, "links form a loop: {}", names.join(" → "))
            }
            Self::DanglingJointLink { joint, link } => {
                write!(f, "joint {joint} refers to missing link {link}")
            }
            Self::DanglingMesh { link, geom, mesh } => {
                write!(
                    f,
                    "geom {geom} of link {link} refers to missing mesh {mesh}"
                )
            }
            Self::DanglingMaterial { link, material } => {
                write!(f, "link {link} uses unknown material \"{material}\"")
            }
            Self::DanglingFrameLink { frame, link } => {
                write!(f, "frame {frame} refers to missing link {link}")
            }
            Self::DuplicateGeomId { link, geom } => {
                write!(f, "link {link} has two geoms with id {geom}")
            }
            Self::DuplicateLinkName(n) => write!(f, "two links are named \"{n}\""),
            Self::DuplicateJointName(n) => write!(f, "two joints are named \"{n}\""),
            Self::DuplicateFrameName(n) => write!(
                f,
                "\"{n}\" names a frame and something else: a frame name must be unique among frames and different from every link name"
            ),
            Self::FrameJointNameCollision { frame, joint } => write!(
                f,
                "frame \"{frame}\" exports a fixed joint named \"{joint}\", which is already a joint"
            ),
            Self::InvalidName { kind, name } => write!(
                f,
                "{kind} name \"{name}\" is not a valid identifier (letter or _ first, then letters, digits, _ . -)"
            ),
            Self::ZeroAxis(j) => write!(f, "joint {j} has a zero axis"),
            Self::MissingLimits(j) => write!(f, "joint {j} needs limits for its kind"),
            Self::LimitsUnordered {
                joint,
                lower,
                upper,
            } => write!(f, "joint {joint} limits are unordered: {lower} > {upper}"),
            Self::NonFinite { what } => write!(f, "{what} is not a finite number"),
            Self::DanglingMimicJoint { joint, leader } => {
                write!(f, "joint {joint} mimics missing joint {leader}")
            }
            Self::SelfMimic(j) => write!(f, "joint {j} mimics itself"),
            Self::MimicOnFixedJoint(j) => {
                write!(f, "fixed joint {j} cannot mimic: it has no value to drive")
            }
            Self::MimicLeaderFixed { joint, leader } => write!(
                f,
                "joint {joint} mimics fixed joint {leader}, which has no value to follow"
            ),
            Self::MimicChain { joint, leader } => write!(
                f,
                "joint {joint} mimics {leader}, which is itself a mimic: mimic chains are not supported"
            ),
            Self::ZeroMimicMultiplier(j) => {
                write!(f, "joint {j} has a zero mimic multiplier")
            }
            Self::MimicExceedsLimits {
                joint,
                leader,
                lower,
                upper,
            } => write!(
                f,
                "joint {joint} following {leader} reaches {lower}..{upper}, outside its own limits"
            ),
        }
    }
}

impl std::error::Error for ValidationError {}

/// Valid XML name and MJCF identifier: `[A-Za-z_][A-Za-z0-9_.-]*`.
pub fn is_valid_name(name: &str) -> bool {
    let mut chars = name.chars();
    match chars.next() {
        Some(c) if c.is_ascii_alphabetic() || c == '_' => {}
        _ => return false,
    }
    chars.all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '.' | '-'))
}

/// The first violated invariant, if any. The command layer wants one
/// error; export wants them all ([`validation_errors`]).
pub fn validate(robot: &Robot) -> Result<(), ValidationError> {
    match validation_errors(robot).into_iter().next() {
        Some(e) => Err(e),
        None => Ok(()),
    }
}

/// Every violated invariant, structural ones first.
pub fn validation_errors(robot: &Robot) -> Vec<ValidationError> {
    let mut errors = Vec::new();
    check_tree(robot, &mut errors);
    check_references(robot, &mut errors);
    check_names(robot, &mut errors);
    check_joints(robot, &mut errors);
    check_mimics(robot, &mut errors);
    errors
}

fn check_tree(robot: &Robot, errors: &mut Vec<ValidationError>) {
    if !robot.links.contains_key(&robot.root) {
        errors.push(ValidationError::RootMissing(robot.root));
    }
    // child → parent joint, catching a second claim on the same child.
    let mut parent_of: BTreeMap<LinkId, JointId> = BTreeMap::new();
    for (&jid, joint) in &robot.joints {
        if joint.child == robot.root {
            errors.push(ValidationError::RootHasParent(jid));
        }
        if let Some(&first) = parent_of.get(&joint.child) {
            errors.push(ValidationError::MultipleParents {
                link: joint.child,
                first,
                second: jid,
            });
        } else {
            parent_of.insert(joint.child, jid);
        }
    }
    // Reachability from the root; whatever is left is an orphan or a loop.
    let mut reached = BTreeSet::new();
    let mut stack = vec![robot.root];
    while let Some(link) = stack.pop() {
        if !reached.insert(link) {
            continue;
        }
        for (_, joint) in robot.joints.iter().filter(|(_, j)| j.parent == link) {
            stack.push(joint.child);
        }
    }
    let mut reported = BTreeSet::new();
    for &link in robot.links.keys() {
        if reached.contains(&link) || reported.contains(&link) {
            continue;
        }
        let Some(_) = parent_of.get(&link) else {
            errors.push(ValidationError::Orphan(link));
            reported.insert(link);
            continue;
        };
        // Follow parent pointers; a link that never reaches the root and has
        // a parent ends in a loop (possibly a tail leading into it).
        let mut chain = Vec::new();
        let mut cursor = link;
        let cycle = loop {
            if let Some(pos) = chain.iter().position(|&l| l == cursor) {
                break Some(chain[pos..].to_vec());
            }
            // Reached the root, or a loop already reported from another tail.
            if reached.contains(&cursor) || reported.contains(&cursor) {
                break None;
            }
            chain.push(cursor);
            match parent_of.get(&cursor) {
                Some(jid) => cursor = robot.joints[jid].parent,
                None => break None,
            }
        };
        reported.extend(chain.iter().copied());
        // `None`: the chain reached the root, a dangling link (reported by
        // `check_references`) or a loop already reported from another tail.
        if let Some(mut cycle) = cycle {
            let start = cycle
                .iter()
                .copied()
                .min()
                .and_then(|m| cycle.iter().position(|&l| l == m));
            if let Some(start) = start {
                cycle.rotate_left(start);
            }
            errors.push(ValidationError::Cycle(cycle));
        }
    }
}

fn check_references(robot: &Robot, errors: &mut Vec<ValidationError>) {
    for (&jid, joint) in &robot.joints {
        for link in [joint.parent, joint.child] {
            if !robot.links.contains_key(&link) {
                errors.push(ValidationError::DanglingJointLink { joint: jid, link });
            }
        }
    }
    for (&lid, link) in &robot.links {
        let mut seen = BTreeSet::new();
        for geom in link.visuals.iter().chain(link.collision.geoms()) {
            if !seen.insert(geom.id) {
                errors.push(ValidationError::DuplicateGeomId {
                    link: lid,
                    geom: geom.id,
                });
            }
            if !robot.assets.contains_key(&geom.mesh) {
                errors.push(ValidationError::DanglingMesh {
                    link: lid,
                    geom: geom.id,
                    mesh: geom.mesh,
                });
            }
        }
        if let Some(material) = &link.material
            && !robot.materials.contains_key(material)
        {
            errors.push(ValidationError::DanglingMaterial {
                link: lid,
                material: material.clone(),
            });
        }
    }
    for (&fid, frame) in &robot.frames {
        if !robot.links.contains_key(&frame.parent) {
            errors.push(ValidationError::DanglingFrameLink {
                frame: fid,
                link: frame.parent,
            });
        }
    }
}

fn check_names(robot: &Robot, errors: &mut Vec<ValidationError>) {
    let mut seen = BTreeSet::new();
    for link in robot.links.values() {
        if !is_valid_name(&link.name) {
            errors.push(ValidationError::InvalidName {
                kind: "link",
                name: link.name.clone(),
            });
        }
        if !seen.insert(link.name.as_str()) {
            errors.push(ValidationError::DuplicateLinkName(link.name.clone()));
        }
    }
    // Frames join that same set: MJCF keeps sites and bodies apart, URDF
    // writes both as `<link>`, and renaming behind the user's back at
    // export time is worse than one rule checked here (ADR-0012).
    for frame in robot.frames.values() {
        if !is_valid_name(&frame.name) {
            errors.push(ValidationError::InvalidName {
                kind: "frame",
                name: frame.name.clone(),
            });
        }
        if !seen.insert(frame.name.as_str()) {
            errors.push(ValidationError::DuplicateFrameName(frame.name.clone()));
        }
    }
    for name in robot.materials.keys() {
        if !is_valid_name(name) {
            errors.push(ValidationError::InvalidName {
                kind: "material",
                name: name.clone(),
            });
        }
    }
    let mut seen = BTreeSet::new();
    for joint in robot.joints.values() {
        if !is_valid_name(&joint.name) {
            errors.push(ValidationError::InvalidName {
                kind: "joint",
                name: joint.name.clone(),
            });
        }
        if !seen.insert(joint.name.as_str()) {
            errors.push(ValidationError::DuplicateJointName(joint.name.clone()));
        }
    }
    // …and the fixed joints the frames export to must not land on one.
    for frame in robot.frames.values() {
        let generated = format!("{}_fixed", frame.name);
        if seen.contains(generated.as_str()) {
            errors.push(ValidationError::FrameJointNameCollision {
                frame: frame.name.clone(),
                joint: generated,
            });
        }
    }
}

fn check_joints(robot: &Robot, errors: &mut Vec<ValidationError>) {
    // A frame's pose reaches the export untouched (ADR-0012), so a NaN in
    // it would become a NaN `pos` in the MJCF rather than an error here.
    for (&fid, frame) in &robot.frames {
        if !frame.pose.t.is_finite() || !frame.pose.r.is_finite() {
            errors.push(ValidationError::NonFinite {
                what: format!("pose of frame {fid}"),
            });
        }
    }
    for (name, material) in &robot.materials {
        if !material.density.is_finite() || material.density < 0.0 {
            errors.push(ValidationError::NonFinite {
                what: format!("density of material \"{name}\""),
            });
        }
    }
    for (&jid, joint) in &robot.joints {
        if !joint.origin.t.is_finite() || !joint.origin.r.is_finite() {
            errors.push(ValidationError::NonFinite {
                what: format!("origin of joint {jid}"),
            });
        }
        if joint.kind.is_movable() && (!joint.axis.is_finite() || joint.axis.length() == 0.0) {
            errors.push(ValidationError::ZeroAxis(jid));
        }
        match joint.limits {
            None if joint.kind.requires_limits() => {
                errors.push(ValidationError::MissingLimits(jid));
            }
            Some(limits) if !limits.lower.is_finite() || !limits.upper.is_finite() => {
                errors.push(ValidationError::NonFinite {
                    what: format!("limits of joint {jid}"),
                });
            }
            Some(limits) if limits.lower > limits.upper => {
                errors.push(ValidationError::LimitsUnordered {
                    joint: jid,
                    lower: limits.lower,
                    upper: limits.upper,
                });
            }
            _ => {}
        }
    }
}

/// Mimic joints (ADR-0013): the leader must exist, move, and not itself
/// mimic; the rule must be a real linear map; and the reach it gives the
/// follower must fit the limits the follower will be exported with.
fn check_mimics(robot: &Robot, errors: &mut Vec<ValidationError>) {
    for (&jid, joint) in &robot.joints {
        let Some(mimic) = joint.mimic else { continue };
        if !mimic.multiplier.is_finite() || !mimic.offset.is_finite() {
            errors.push(ValidationError::NonFinite {
                what: format!("mimic of joint {jid}"),
            });
            continue;
        }
        if mimic.multiplier == 0.0 {
            errors.push(ValidationError::ZeroMimicMultiplier(jid));
            continue;
        }
        if !joint.kind.is_movable() {
            errors.push(ValidationError::MimicOnFixedJoint(jid));
            continue;
        }
        if mimic.joint == jid {
            errors.push(ValidationError::SelfMimic(jid));
            continue;
        }
        let Some(leader) = robot.joints.get(&mimic.joint) else {
            errors.push(ValidationError::DanglingMimicJoint {
                joint: jid,
                leader: mimic.joint,
            });
            continue;
        };
        if !leader.kind.is_movable() {
            errors.push(ValidationError::MimicLeaderFixed {
                joint: jid,
                leader: mimic.joint,
            });
            continue;
        }
        if leader.mimic.is_some() {
            errors.push(ValidationError::MimicChain {
                joint: jid,
                leader: mimic.joint,
            });
            continue;
        }
        // A `Continuous` follower has no range to leave, so the check is
        // vacuous; a `Continuous` leader has an unbounded one, which no
        // bounded follower can contain.
        let Some(own) = joint.limits else { continue };
        let (lo, hi) = match leader.limits {
            Some(l) => (l.lower, l.upper),
            None => (f64::NEG_INFINITY, f64::INFINITY),
        };
        let ends = [
            mimic.multiplier * lo + mimic.offset,
            mimic.multiplier * hi + mimic.offset,
        ];
        let (lower, upper) = (ends[0].min(ends[1]), ends[0].max(ends[1]));
        if lower < own.lower || upper > own.upper {
            errors.push(ValidationError::MimicExceedsLimits {
                joint: jid,
                leader: mimic.joint,
                lower,
                upper,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::Id;
    use crate::pose::Pose;
    use crate::robot::{Frame, Geom, Joint, JointKind, Limits, Link, MeshAsset, Mimic};
    use riggen_mesh::glam::{DVec3, dvec3};
    use std::path::PathBuf;

    /// Appends `name` under `parent` with a fixed joint `<name>_joint`.
    fn add_link(robot: &mut Robot, parent: LinkId, name: &str) -> (LinkId, JointId) {
        let link: LinkId = robot.next_id.alloc();
        robot.links.insert(link, Link::new(name));
        let joint: JointId = robot.next_id.alloc();
        robot
            .joints
            .insert(joint, Joint::fixed(format!("{name}_joint"), parent, link));
        (link, joint)
    }

    /// base ─ arm ─ hand, plus a second child of base.
    fn chain() -> (Robot, LinkId, JointId, LinkId, JointId) {
        let mut robot = Robot::new("r");
        let root = robot.root;
        let (arm, arm_j) = add_link(&mut robot, root, "arm");
        let (hand, hand_j) = add_link(&mut robot, arm, "hand");
        add_link(&mut robot, root, "tail");
        assert_eq!(validate(&robot), Ok(()));
        (robot, arm, arm_j, hand, hand_j)
    }

    #[test]
    fn empty_and_chain_are_valid() {
        assert_eq!(validate(&Robot::new("r")), Ok(()));
        chain();
    }

    #[test]
    fn root_missing() {
        let mut robot = Robot::new("r");
        let root = robot.root;
        robot.links.clear();
        assert_eq!(validate(&robot), Err(ValidationError::RootMissing(root)));
    }

    #[test]
    fn root_has_parent() {
        let (mut robot, arm, _, _, _) = chain();
        let j: JointId = robot.next_id.alloc();
        robot
            .joints
            .insert(j, Joint::fixed("back", arm, robot.root));
        assert_eq!(validate(&robot), Err(ValidationError::RootHasParent(j)));
    }

    #[test]
    fn orphan() {
        let (mut robot, _, _, hand, hand_j) = chain();
        robot.joints.remove(&hand_j);
        assert_eq!(validate(&robot), Err(ValidationError::Orphan(hand)));
    }

    #[test]
    fn multiple_parents() {
        let (mut robot, _, arm_j, hand, hand_j) = chain();
        let second: JointId = robot.next_id.alloc();
        robot
            .joints
            .insert(second, Joint::fixed("again", robot.root, hand));
        assert_eq!(
            validate(&robot),
            Err(ValidationError::MultipleParents {
                link: hand,
                first: hand_j,
                second
            })
        );
        let _ = arm_j;
    }

    #[test]
    fn cycle_names_the_loop_in_parent_order() {
        // Detach arm from base and hang it under hand: arm → hand → arm.
        let (mut robot, arm, arm_j, hand, _) = chain();
        robot.joints.get_mut(&arm_j).unwrap().parent = hand;
        assert_eq!(
            validate(&robot),
            Err(ValidationError::Cycle(vec![arm, hand]))
        );
        assert_eq!(
            validate(&robot).unwrap_err().to_string(),
            format!("links form a loop: {arm} → {hand}")
        );
    }

    #[test]
    fn self_loop_is_a_cycle_of_one() {
        let (mut robot, arm, arm_j, _, _) = chain();
        robot.joints.get_mut(&arm_j).unwrap().parent = arm;
        // hand hangs off the loop and is reported by the same error.
        assert_eq!(
            validation_errors(&robot),
            vec![ValidationError::Cycle(vec![arm])]
        );
    }

    #[test]
    fn dangling_joint_link() {
        let (mut robot, _, _, hand, hand_j) = chain();
        robot.links.remove(&hand);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DanglingJointLink {
                joint: hand_j,
                link: hand
            })
        );
    }

    #[test]
    fn dangling_mesh_and_duplicate_geom_id() {
        let (mut robot, arm, _, _, _) = chain();
        let mesh = MeshId::from_raw(999);
        let geom = GeomId::from_raw(998);
        let g = Geom {
            id: geom,
            mesh,
            pose: Pose::IDENTITY,
            color: None,
        };
        robot.links.get_mut(&arm).unwrap().visuals.push(g.clone());
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DanglingMesh {
                link: arm,
                geom,
                mesh
            })
        );
        let real = robot.add_asset(MeshAsset {
            path: PathBuf::from("/a.stl"),
            content_hash: 0,
            scale: 1.0,
            fix_up: None,
        });
        let arm_link = robot.links.get_mut(&arm).unwrap();
        arm_link.visuals[0].mesh = real;
        arm_link.visuals.push(Geom { mesh: real, ..g });
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DuplicateGeomId { link: arm, geom })
        );
    }

    #[test]
    fn dangling_material() {
        let (mut robot, arm, _, _, _) = chain();
        robot.links.get_mut(&arm).unwrap().material = Some("unobtainium".into());
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DanglingMaterial {
                link: arm,
                material: "unobtainium".into()
            })
        );
        robot.links.get_mut(&arm).unwrap().material = Some("steel".into());
        assert_eq!(validate(&robot), Ok(()));
    }

    #[test]
    fn dangling_frame_link() {
        let (mut robot, ..) = chain();
        let frame: FrameId = robot.next_id.alloc();
        let link = LinkId::from_raw(777);
        robot.frames.insert(
            frame,
            Frame {
                name: "tcp".into(),
                parent: link,
                pose: Pose::IDENTITY,
            },
        );
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DanglingFrameLink { frame, link })
        );
    }

    #[test]
    fn frames_and_links_are_one_namespace() {
        let (mut robot, arm, ..) = chain();
        let add = |robot: &mut Robot, name: &str, parent: LinkId| -> FrameId {
            let id: FrameId = robot.next_id.alloc();
            robot.frames.insert(
                id,
                Frame {
                    name: name.into(),
                    parent,
                    pose: Pose::IDENTITY,
                },
            );
            id
        };
        add(&mut robot, "tcp", arm);
        assert_eq!(validate(&robot), Ok(()));

        // Two frames may not share a name…
        let root = robot.root;
        let second = add(&mut robot, "tcp", root);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DuplicateFrameName("tcp".into()))
        );
        // …nor may a frame take a link's, because URDF writes both as
        // `<link>` (ADR-0012).
        robot.frames.get_mut(&second).unwrap().name = "arm".into();
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DuplicateFrameName("arm".into()))
        );
        assert!(
            validate(&robot)
                .unwrap_err()
                .to_string()
                .contains("different from every link name")
        );
        // A frame may share a *joint*'s name — separate namespaces in both
        // formats — but not produce a second joint called `<name>_fixed`.
        robot.frames.get_mut(&second).unwrap().name = "hand_joint".into();
        assert_eq!(validate(&robot), Ok(()));
        robot.frames.get_mut(&second).unwrap().name = "grip".into();
        robot
            .joints
            .values_mut()
            .find(|j| j.name == "tail_joint")
            .unwrap()
            .name = "grip_fixed".into();
        assert_eq!(
            validate(&robot),
            Err(ValidationError::FrameJointNameCollision {
                frame: "grip".into(),
                joint: "grip_fixed".into(),
            })
        );
        // A frame name is an XML name like every other.
        robot.frames.get_mut(&second).unwrap().name = "2 hands".into();
        assert_eq!(
            validate(&robot),
            Err(ValidationError::InvalidName {
                kind: "frame",
                name: "2 hands".into()
            })
        );
        // …and its pose has to be a real one: it reaches the export as
        // written, so a NaN would land in the MJCF (ADR-0012).
        robot.frames.get_mut(&second).unwrap().name = "mount".into();
        robot.frames.get_mut(&second).unwrap().pose.t.x = f64::NAN;
        assert_eq!(
            validate(&robot),
            Err(ValidationError::NonFinite {
                what: format!("pose of frame {second}")
            })
        );
    }

    #[test]
    fn duplicate_names() {
        let (mut robot, arm, arm_j, _, _) = chain();
        robot.links.get_mut(&arm).unwrap().name = "base_link".into();
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DuplicateLinkName("base_link".into()))
        );
        robot.links.get_mut(&arm).unwrap().name = "arm".into();
        robot.joints.get_mut(&arm_j).unwrap().name = "hand_joint".into();
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DuplicateJointName("hand_joint".into()))
        );
    }

    #[test]
    fn invalid_names() {
        for bad in ["", "1arm", "arm joint", "arm:1", "ärm", "-x"] {
            let (mut robot, arm, _, _, _) = chain();
            robot.links.get_mut(&arm).unwrap().name = bad.into();
            assert_eq!(
                validate(&robot),
                Err(ValidationError::InvalidName {
                    kind: "link",
                    name: bad.into()
                }),
                "{bad:?}"
            );
        }
        for good in ["arm", "_arm", "Arm-2.b", "a_b_c"] {
            assert!(is_valid_name(good), "{good:?}");
        }
        let (mut robot, _, arm_j, _, _) = chain();
        robot.joints.get_mut(&arm_j).unwrap().name = "no way".into();
        assert!(matches!(
            validate(&robot),
            Err(ValidationError::InvalidName { kind: "joint", .. })
        ));
    }

    #[test]
    fn zero_axis_only_matters_for_movable_joints() {
        let (mut robot, _, arm_j, _, _) = chain();
        robot.joints.get_mut(&arm_j).unwrap().axis = DVec3::ZERO;
        assert_eq!(validate(&robot), Ok(()), "fixed joints ignore the axis");
        let j = robot.joints.get_mut(&arm_j).unwrap();
        j.kind = JointKind::Continuous;
        assert_eq!(validate(&robot), Err(ValidationError::ZeroAxis(arm_j)));
        let j = robot.joints.get_mut(&arm_j).unwrap();
        j.axis = dvec3(0.0, f64::NAN, 0.0);
        assert_eq!(validate(&robot), Err(ValidationError::ZeroAxis(arm_j)));
    }

    #[test]
    fn limits_required_and_ordered() {
        let (mut robot, _, arm_j, _, _) = chain();
        for kind in [JointKind::Revolute, JointKind::Prismatic] {
            let j = robot.joints.get_mut(&arm_j).unwrap();
            j.kind = kind;
            j.limits = None;
            assert_eq!(validate(&robot), Err(ValidationError::MissingLimits(arm_j)));
            let j = robot.joints.get_mut(&arm_j).unwrap();
            j.limits = Some(Limits {
                lower: 1.0,
                upper: -1.0,
                effort: 0.0,
                velocity: 0.0,
            });
            assert_eq!(
                validate(&robot),
                Err(ValidationError::LimitsUnordered {
                    joint: arm_j,
                    lower: 1.0,
                    upper: -1.0
                })
            );
            let j = robot.joints.get_mut(&arm_j).unwrap();
            j.limits = Some(Limits {
                lower: -1.0,
                upper: 1.0,
                effort: 0.0,
                velocity: 0.0,
            });
            assert_eq!(validate(&robot), Ok(()));
        }
        let j = robot.joints.get_mut(&arm_j).unwrap();
        j.kind = JointKind::Continuous;
        j.limits = None;
        assert_eq!(validate(&robot), Ok(()), "continuous needs no limits");
        let j = robot.joints.get_mut(&arm_j).unwrap();
        j.limits = Some(Limits {
            lower: f64::NAN,
            upper: 1.0,
            effort: 0.0,
            velocity: 0.0,
        });
        assert!(matches!(
            validate(&robot),
            Err(ValidationError::NonFinite { .. })
        ));
    }

    #[test]
    fn material_name_and_density() {
        let (mut robot, ..) = chain();
        robot.materials.insert(
            "no way".into(),
            crate::robot::Material {
                density: 1.0,
                color: [1.0; 4],
            },
        );
        assert_eq!(
            validate(&robot),
            Err(ValidationError::InvalidName {
                kind: "material",
                name: "no way".into()
            })
        );
        robot.materials.remove("no way");
        for density in [f64::NAN, f64::INFINITY, -1.0] {
            robot.materials.get_mut("steel").unwrap().density = density;
            assert_eq!(
                validate(&robot),
                Err(ValidationError::NonFinite {
                    what: "density of material \"steel\"".into()
                }),
                "{density}"
            );
        }
    }

    #[test]
    fn non_finite_origin() {
        let (mut robot, _, arm_j, _, _) = chain();
        robot.joints.get_mut(&arm_j).unwrap().origin.t.x = f64::INFINITY;
        assert_eq!(
            validate(&robot),
            Err(ValidationError::NonFinite {
                what: format!("origin of joint {arm_j}")
            })
        );
    }

    #[test]
    fn validation_errors_collects_all() {
        let (mut robot, arm, arm_j, hand, hand_j) = chain();
        robot.joints.remove(&hand_j);
        robot.links.get_mut(&arm).unwrap().name = "base_link".into();
        robot.joints.get_mut(&arm_j).unwrap().kind = JointKind::Revolute;
        assert_eq!(
            validation_errors(&robot),
            vec![
                ValidationError::Orphan(hand),
                ValidationError::DuplicateLinkName("base_link".into()),
                ValidationError::MissingLimits(arm_j),
            ]
        );
    }

    // ---- mimic joints (ADR-0013) -----------------------------------------

    /// base ─j0─ a ─j1─ b ─j2─ c, every joint revolute about Z with ±3
    /// limits, so any of them may lead or follow.
    fn movable_chain() -> (Robot, [JointId; 3]) {
        let mut robot = Robot::new("r");
        let root = robot.root;
        let (a, j0) = add_link(&mut robot, root, "a");
        let (b, j1) = add_link(&mut robot, a, "b");
        let (_, j2) = add_link(&mut robot, b, "c");
        for j in [j0, j1, j2] {
            let joint = robot.joints.get_mut(&j).unwrap();
            joint.kind = JointKind::Revolute;
            joint.axis = DVec3::Z;
            joint.limits = Some(Limits {
                lower: -3.0,
                upper: 3.0,
                effort: 0.0,
                velocity: 0.0,
            });
        }
        assert_eq!(validate(&robot), Ok(()));
        (robot, [j0, j1, j2])
    }

    fn mimic(robot: &mut Robot, follower: JointId, leader: JointId, multiplier: f64, offset: f64) {
        robot.joints.get_mut(&follower).unwrap().mimic = Some(Mimic {
            joint: leader,
            multiplier,
            offset,
        });
    }

    #[test]
    fn a_mimic_leader_must_exist_be_movable_and_not_be_the_follower() {
        let (mut robot, [j0, j1, _]) = movable_chain();
        mimic(&mut robot, j1, j0, -0.5, 0.1);
        assert_eq!(validate(&robot), Ok(()), "the ordinary case");

        let ghost = JointId::from_raw(999);
        mimic(&mut robot, j1, ghost, 1.0, 0.0);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DanglingMimicJoint {
                joint: j1,
                leader: ghost
            })
        );

        mimic(&mut robot, j1, j1, 1.0, 0.0);
        assert_eq!(validate(&robot), Err(ValidationError::SelfMimic(j1)));

        mimic(&mut robot, j1, j0, 1.0, 0.0);
        robot.joints.get_mut(&j0).unwrap().kind = JointKind::Fixed;
        robot.joints.get_mut(&j0).unwrap().limits = None;
        assert_eq!(
            validate(&robot),
            Err(ValidationError::MimicLeaderFixed {
                joint: j1,
                leader: j0
            })
        );
    }

    #[test]
    fn a_fixed_joint_cannot_follow_anything() {
        let (mut robot, [j0, j1, _]) = movable_chain();
        mimic(&mut robot, j1, j0, 1.0, 0.0);
        let follower = robot.joints.get_mut(&j1).unwrap();
        follower.kind = JointKind::Fixed;
        follower.limits = None;
        assert_eq!(
            validate(&robot),
            Err(ValidationError::MimicOnFixedJoint(j1))
        );
    }

    /// A follower whose leader follows: rejected outright, not resolved
    /// (ADR-0013).
    #[test]
    fn mimic_chains_are_rejected() {
        let (mut robot, [j0, j1, j2]) = movable_chain();
        mimic(&mut robot, j1, j0, 0.5, 0.0);
        mimic(&mut robot, j2, j1, 0.5, 0.0);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::MimicChain {
                joint: j2,
                leader: j1
            })
        );
    }

    #[test]
    fn a_mimic_rule_must_be_a_real_non_degenerate_line() {
        let (mut robot, [j0, j1, _]) = movable_chain();
        mimic(&mut robot, j1, j0, 0.0, 0.0);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::ZeroMimicMultiplier(j1))
        );
        for (multiplier, offset) in [(f64::NAN, 0.0), (f64::INFINITY, 0.0), (1.0, f64::NAN)] {
            mimic(&mut robot, j1, j0, multiplier, offset);
            let err = validate(&robot).unwrap_err();
            assert!(
                matches!(&err, ValidationError::NonFinite { what } if what.contains("mimic")),
                "{err:?}"
            );
        }
    }

    /// The leader's whole range, mapped, has to fit the range the follower
    /// is exported with, or MJCF's `range` fights the equality (ADR-0013).
    #[test]
    fn a_followers_reach_must_fit_its_own_limits() {
        let (mut robot, [j0, j1, _]) = movable_chain();
        for multiplier in [2.0, -2.0] {
            mimic(&mut robot, j1, j0, multiplier, 0.0);
            assert_eq!(
                validate(&robot),
                Err(ValidationError::MimicExceedsLimits {
                    joint: j1,
                    leader: j0,
                    lower: -6.0,
                    upper: 6.0
                }),
                "a negative multiplier flips the interval, it does not excuse it"
            );
        }
        // Shifted off one end: ±3 through (1, 0.5) reaches 3.5.
        mimic(&mut robot, j1, j0, 1.0, 0.5);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::MimicExceedsLimits {
                joint: j1,
                leader: j0,
                lower: -2.5,
                upper: 3.5
            })
        );
        mimic(&mut robot, j1, j0, 0.5, 1.0);
        assert_eq!(validate(&robot), Ok(()), "-0.5..2.5 fits inside ±3");

        // A `Continuous` follower has no range to leave…
        let follower = robot.joints.get_mut(&j1).unwrap();
        follower.kind = JointKind::Continuous;
        follower.limits = None;
        mimic(&mut robot, j1, j0, 10.0, 0.0);
        assert_eq!(validate(&robot), Ok(()));

        // …but a `Continuous` leader has an unbounded one, which no
        // bounded follower can hold.
        let (mut robot, [j0, j1, _]) = movable_chain();
        let leader = robot.joints.get_mut(&j0).unwrap();
        leader.kind = JointKind::Continuous;
        leader.limits = None;
        mimic(&mut robot, j1, j0, 1.0, 0.0);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::MimicExceedsLimits {
                joint: j1,
                leader: j0,
                lower: f64::NEG_INFINITY,
                upper: f64::INFINITY
            })
        );
    }
}
