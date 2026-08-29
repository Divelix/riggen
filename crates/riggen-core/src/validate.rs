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
    /// Not an XML name / MJCF identifier: `[A-Za-z_][A-Za-z0-9_.-]*`.
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
        for geom in &link.visuals {
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
}

fn check_joints(robot: &Robot, errors: &mut Vec<ValidationError>) {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::Id;
    use crate::pose::Pose;
    use crate::robot::{Frame, Geom, Joint, JointKind, Limits, Link, MeshAsset};
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
}
