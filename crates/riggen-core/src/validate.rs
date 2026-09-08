//! The document invariants (docs/02-data-model.md §Core types). The command
//! layer never produces a violating state (ADR-0005); `validate` is the
//! safety net behind every command and the gate before save and export.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use crate::ids::{ActuatorId, FrameId, GeomId, JointId, LinkId, MeshId, TendonId};
use crate::robot::{ActuatorSpec, ActuatorTarget, General, Robot};

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
    /// A ring of followers with no free leader at its head: each joint in
    /// `joints` follows the next, and the last follows the first. `joints`
    /// is in follow order, starting from the lowest id in it. A *chain*
    /// resolves (ADR-0025); a cycle has no value to resolve against.
    MimicCycle {
        joints: Vec<JointId>,
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
    // ---- fixed tendons (ADR-0025 §4) -------------------------------------
    /// Two tendons share a name. Unique **among tendons** only: MJCF's
    /// namespaces are per element type.
    DuplicateTendonName(String),
    /// A tendon with no joints: it has no length, and MJCF refuses a
    /// `<fixed>` without a `<joint>` child.
    EmptyTendon(TendonId),
    /// A tendon names a joint that is not in the document.
    DanglingTendonJoint {
        tendon: TendonId,
        joint: JointId,
    },
    /// One joint appears twice on the same tendon; the two coefficients
    /// would be a single sum written as two children.
    DuplicateTendonJoint {
        tendon: TendonId,
        joint: JointId,
    },
    /// A tendon runs over a `Fixed` joint, which has no value to weigh.
    /// This is what refuses `SetJoint` to `Fixed` on such a joint, the way
    /// [`MimicLeaderFixed`] refuses demoting a leader.
    ///
    /// [`MimicLeaderFixed`]: ValidationError::MimicLeaderFixed
    TendonOnFixedJoint {
        tendon: TendonId,
        joint: JointId,
    },
    /// A `coef` of zero: the joint contributes nothing to the length, so
    /// it is not on the tendon.
    ZeroTendonCoef {
        tendon: TendonId,
        joint: JointId,
    },
    /// A tendon `range` that bounds nothing: `lower` is not below `upper`.
    /// The numbers are in MJCF's own absolute terms (ADR-0025 §4), which
    /// is why the check needs no coordinates.
    InvalidTendonRange {
        tendon: TendonId,
        lower: f64,
        upper: f64,
    },
    // ---- actuators (ADR-0014, ADR-0023) ----------------------------------
    /// An actuator's `target` names no joint or tendon of the document. A
    /// name and a target are separate fields now, so each can go wrong on
    /// its own (ADR-0023, ADR-0025 §4).
    DanglingActuatorTarget {
        actuator: ActuatorId,
        target: ActuatorTarget,
    },
    /// Two actuators share a name. Unique **among actuators** only: MJCF's
    /// namespaces are per element type, so an actuator may answer to a
    /// joint's name — which is the default one (ADR-0023).
    DuplicateActuatorName(String),
    /// An actuator drives a `Fixed` joint: MJCF writes no `<joint>` for
    /// one, so there is nothing for the `<actuator>` to drive.
    ActuatorOnFixedJoint {
        actuator: ActuatorId,
        joint: JointId,
    },
    /// An actuator on a joint that mimics another. The follower is already
    /// driven by the `<equality>` the mimic writes (ADR-0013); actuating it
    /// too sets the two against each other.
    ActuatorOnMimicFollower {
        actuator: ActuatorId,
        joint: JointId,
        leader: JointId,
    },
    /// A gain MuJoCo cannot use: a negative `kp` / `kv`, or a zero `gear`.
    /// `what` names the gain and its value. (A *non-finite* one is
    /// [`NonFinite`], as for every other number in the document.)
    ///
    /// [`NonFinite`]: ValidationError::NonFinite
    InvalidActuatorGain {
        actuator: ActuatorId,
        what: String,
    },
    /// A `General` actuator's `dynprm` / `gainprm` / `biasprm` holds more
    /// than the ten entries MuJoCo has room for (ADR-0024). `what` names
    /// the vector, `len` how long it is.
    ActuatorPrmTooLong {
        actuator: ActuatorId,
        what: &'static str,
        len: usize,
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
            Self::MimicCycle { joints } => {
                let ring = joints
                    .iter()
                    .map(|j| j.to_string())
                    .collect::<Vec<_>>()
                    .join(" → ");
                write!(f, "joints {ring} follow each other in a cycle")
            }
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
            Self::DuplicateTendonName(n) => write!(f, "two tendons are named \"{n}\""),
            Self::EmptyTendon(t) => write!(f, "tendon {t} runs over no joints"),
            Self::DanglingTendonJoint { tendon, joint } => {
                write!(f, "tendon {tendon} runs over missing joint {joint}")
            }
            Self::DuplicateTendonJoint { tendon, joint } => {
                write!(f, "tendon {tendon} names joint {joint} twice")
            }
            Self::TendonOnFixedJoint { tendon, joint } => write!(
                f,
                "tendon {tendon} runs over fixed joint {joint}, which has no value to weigh"
            ),
            Self::ZeroTendonCoef { tendon, joint } => {
                write!(f, "tendon {tendon} gives joint {joint} a zero coefficient")
            }
            Self::InvalidTendonRange {
                tendon,
                lower,
                upper,
            } => write!(f, "tendon {tendon} range {lower}..{upper} bounds nothing"),
            Self::DanglingActuatorTarget { actuator, target } => {
                let (kind, id) = match target {
                    ActuatorTarget::Joint(j) => ("joint", j.to_string()),
                    ActuatorTarget::Tendon(t) => ("tendon", t.to_string()),
                };
                write!(f, "actuator {actuator} drives missing {kind} {id}")
            }
            Self::DuplicateActuatorName(n) => write!(f, "two actuators are named \"{n}\""),
            Self::ActuatorOnFixedJoint { actuator, joint } => write!(
                f,
                "actuator {actuator} cannot drive fixed joint {joint}: it has no degree of freedom"
            ),
            Self::ActuatorOnMimicFollower {
                actuator,
                joint,
                leader,
            } => write!(
                f,
                "actuator {actuator} cannot drive joint {joint}: it follows {leader}, which already drives it"
            ),
            Self::InvalidActuatorGain { actuator, what } => {
                write!(f, "actuator {actuator} has {what}")
            }
            Self::ActuatorPrmTooLong {
                actuator,
                what,
                len,
            } => write!(
                f,
                "actuator {actuator} has {len} {what} entries; MuJoCo holds at most {}",
                General::MAX_PRM
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
    check_tendons(robot, &mut errors);
    check_actuators(robot, &mut errors);
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
        // `qpos_ref` reaches MJCF's `<joint ref>` untouched (ADR-0025 §3),
        // and shifts the range the writer derives from it.
        if !joint.qpos_ref.is_finite() {
            errors.push(ValidationError::NonFinite {
                what: format!("qpos_ref of joint {jid}"),
            });
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

/// Mimic joints (ADR-0013, amended by ADR-0025): the leader must exist and
/// move, and may itself follow — only a *cycle* of followers is refused;
/// the rule must be a real linear map; and the reach the whole chain gives
/// the follower must fit the limits it will be exported with.
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
        // A `Continuous` follower has no range to leave, so the check is
        // vacuous; a `Continuous` leader has an unbounded one, which no
        // bounded follower can contain.
        let Some(own) = joint.limits else { continue };
        // Through a chain the reach is the *free* leader's range mapped by
        // the composed map (ADR-0025), so the promise ADR-0013 made holds
        // however long the chain is. A cycle is reported below and has no
        // free leader; `composed_leader` gives up on one.
        let Some((free, multiplier, offset)) = composed_leader(robot, jid) else {
            continue;
        };
        let (lo, hi) = match robot.joints[&free].limits {
            Some(l) => (l.lower, l.upper),
            None => (f64::NEG_INFINITY, f64::INFINITY),
        };
        let ends = [multiplier * lo + offset, multiplier * hi + offset];
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
    check_mimic_cycles(robot, errors);
}

/// The free joint at the head of `joint`'s chain of leaders and the affine
/// map from its value to `joint`'s: `q(joint) = multiplier · q(free) +
/// offset`. `None` when the chain runs into a cycle or a leader that is
/// not a joint of the document — both reported elsewhere.
fn composed_leader(robot: &Robot, joint: JointId) -> Option<(JointId, f64, f64)> {
    let (mut multiplier, mut offset) = (1.0, 0.0);
    let mut seen = BTreeSet::new();
    let mut cursor = joint;
    loop {
        if !seen.insert(cursor) {
            return None;
        }
        match robot.joints.get(&cursor)?.mimic {
            // q(joint) = m·q(cursor) + o, and q(cursor) = mu·q(leader) + off.
            Some(m) => {
                offset += multiplier * m.offset;
                multiplier *= m.multiplier;
                cursor = m.joint;
            }
            None => return Some((cursor, multiplier, offset)),
        }
    }
}

/// A ring of followers (ADR-0025). A self-mimic is a ring of one and is
/// already `SelfMimic`, so it is not reported twice.
fn check_mimic_cycles(robot: &Robot, errors: &mut Vec<ValidationError>) {
    let mut reported: BTreeSet<JointId> = robot
        .joints
        .iter()
        .filter(|(jid, j)| j.mimic.is_some_and(|m| m.joint == **jid))
        .map(|(&jid, _)| jid)
        .collect();
    for &start in robot.joints.keys() {
        if reported.contains(&start) {
            continue;
        }
        let mut path: Vec<JointId> = Vec::new();
        let mut cursor = start;
        let cycle = loop {
            if let Some(pos) = path.iter().position(|&j| j == cursor) {
                break Some(path[pos..].to_vec());
            }
            // Already walked from another tail, or the head of the chain.
            if reported.contains(&cursor) {
                break None;
            }
            match robot.joints.get(&cursor).and_then(|j| j.mimic) {
                Some(mimic) => {
                    path.push(cursor);
                    cursor = mimic.joint;
                }
                None => break None,
            }
        };
        reported.extend(path.iter().copied());
        if let Some(mut cycle) = cycle {
            let start = cycle
                .iter()
                .copied()
                .min()
                .and_then(|m| cycle.iter().position(|&j| j == m));
            if let Some(start) = start {
                cycle.rotate_left(start);
            }
            errors.push(ValidationError::MimicCycle { joints: cycle });
        }
    }
}

/// Fixed tendons (ADR-0025 §4): a tendon is a named linear combination of
/// joint values, so every joint it weighs must exist, move, appear once and
/// count for something — and a `range`, held in MJCF's own absolute terms,
/// must bound something. Nothing here needs coordinates: the document never
/// evaluates a tendon's length.
fn check_tendons(robot: &Robot, errors: &mut Vec<ValidationError>) {
    let mut seen_names = BTreeSet::new();
    for (&tid, tendon) in &robot.tendons {
        if !seen_names.insert(tendon.name.as_str()) {
            errors.push(ValidationError::DuplicateTendonName(tendon.name.clone()));
        }
        if tendon.joints.is_empty() {
            errors.push(ValidationError::EmptyTendon(tid));
        }
        let mut seen_joints = BTreeSet::new();
        for entry in &tendon.joints {
            let jid = entry.joint;
            if !seen_joints.insert(jid) {
                errors.push(ValidationError::DuplicateTendonJoint {
                    tendon: tid,
                    joint: jid,
                });
            }
            match robot.joints.get(&jid) {
                None => errors.push(ValidationError::DanglingTendonJoint {
                    tendon: tid,
                    joint: jid,
                }),
                Some(joint) if !joint.kind.is_movable() => {
                    errors.push(ValidationError::TendonOnFixedJoint {
                        tendon: tid,
                        joint: jid,
                    });
                }
                Some(_) => {}
            }
            if !entry.coef.is_finite() {
                errors.push(ValidationError::NonFinite {
                    what: format!("coef of joint {jid} on tendon {tid}"),
                });
            } else if entry.coef == 0.0 {
                errors.push(ValidationError::ZeroTendonCoef {
                    tendon: tid,
                    joint: jid,
                });
            }
        }
        for (what, value) in [
            ("stiffness", tendon.stiffness),
            ("damping", tendon.damping),
            ("frictionloss", tendon.frictionloss),
        ] {
            if !value.is_finite() {
                errors.push(ValidationError::NonFinite {
                    what: format!("{what} of tendon {tid}"),
                });
            }
        }
        if let Some([lower, upper]) = tendon.range {
            if !lower.is_finite() || !upper.is_finite() {
                errors.push(ValidationError::NonFinite {
                    what: format!("range of tendon {tid}"),
                });
            } else if lower >= upper {
                errors.push(ValidationError::InvalidTendonRange {
                    tendon: tid,
                    lower,
                    upper,
                });
            }
        }
    }
}

/// Actuators (ADR-0014, re-keyed onto the table by ADR-0023): the target
/// must exist, only a movable joint that is not already driven by a mimic
/// may be driven, the gains must be numbers MuJoCo can use — and every
/// actuator's name is its own, unique among actuators.
///
/// **Several actuators on one joint is legal**, deliberately: MuJoCo sums
/// their controls, and refusing it here would reject models MuJoCo accepts
/// and force the import to drop what a file said (ADR-0023 §3).
fn check_actuators(robot: &Robot, errors: &mut Vec<ValidationError>) {
    let mut seen = BTreeSet::new();
    for (&aid, entry) in &robot.actuators {
        if !seen.insert(entry.name.as_str()) {
            errors.push(ValidationError::DuplicateActuatorName(entry.name.clone()));
        }
        // A tendon target only has to exist: a tendon's joints are free
        // and movable by construction, so the joint-specific refusals below
        // — `Fixed`, a mimic follower — have nothing to say about it
        // (ADR-0025 §4). Its gains are checked like any other actuator's.
        let joint = match entry.target {
            ActuatorTarget::Tendon(tid) => {
                if !robot.tendons.contains_key(&tid) {
                    errors.push(ValidationError::DanglingActuatorTarget {
                        actuator: aid,
                        target: entry.target,
                    });
                    continue;
                }
                None
            }
            ActuatorTarget::Joint(jid) => match robot.joints.get(&jid) {
                Some(joint) => Some((jid, joint)),
                None => {
                    errors.push(ValidationError::DanglingActuatorTarget {
                        actuator: aid,
                        target: entry.target,
                    });
                    continue;
                }
            },
        };
        let actuator = &entry.spec;
        if let Some((jid, joint)) = joint {
            if !joint.kind.is_movable() {
                errors.push(ValidationError::ActuatorOnFixedJoint {
                    actuator: aid,
                    joint: jid,
                });
                continue;
            }
            if let Some(mimic) = joint.mimic {
                errors.push(ValidationError::ActuatorOnMimicFollower {
                    actuator: aid,
                    joint: jid,
                    leader: mimic.joint,
                });
                continue;
            }
        }
        // `(name, value, is usable)`. `kp` / `kv` may be zero — a position
        // servo with no damping is ordinary — but never negative, which
        // would push the joint away from its target; a `gear` may be
        // negative (it reverses the joint) but never zero, which is an
        // actuator that cannot move it at all. Those are statements about
        // the *presets*: a `General`'s gains are MuJoCo's to interpret
        // (ADR-0024), so only finiteness and the vectors' length are
        // riggen's to refuse.
        let gains: [Option<(&str, f64, bool)>; 2] = match *actuator {
            ActuatorSpec::Position { kp, kv } => {
                [Some(("kp", kp, kp >= 0.0)), Some(("kv", kv, kv >= 0.0))]
            }
            ActuatorSpec::Velocity { kv } => [Some(("kv", kv, kv >= 0.0)), None],
            ActuatorSpec::Motor { gear } => [Some(("gear", gear, gear != 0.0)), None],
            ActuatorSpec::General(ref general) => {
                for (what, prm) in general.prms() {
                    if prm.len() > General::MAX_PRM {
                        errors.push(ValidationError::ActuatorPrmTooLong {
                            actuator: aid,
                            what,
                            len: prm.len(),
                        });
                    }
                    for (i, value) in prm.iter().enumerate() {
                        if !value.is_finite() {
                            errors.push(ValidationError::NonFinite {
                                what: format!("{what}[{i}] of actuator {aid}"),
                            });
                        }
                    }
                }
                [Some(("gear", general.gear, true)), None]
            }
        };
        for (name, value, usable) in gains.into_iter().flatten() {
            if !value.is_finite() {
                errors.push(ValidationError::NonFinite {
                    what: format!("{name} of actuator {aid}"),
                });
            } else if !usable {
                errors.push(ValidationError::InvalidActuatorGain {
                    actuator: aid,
                    what: format!("{name} {value}"),
                });
            }
        }
        // The ranges the file said (ADR-0024) are numbers like any other
        // in the document: finite, or refused. Whether they bound anything
        // is MuJoCo's to decide under the flags written beside them.
        for (name, range) in [
            ("ctrlrange", entry.ranges.ctrl),
            ("forcerange", entry.ranges.force),
        ] {
            if range.is_some_and(|r| !r.iter().all(|v| v.is_finite())) {
                errors.push(ValidationError::NonFinite {
                    what: format!("{name} of actuator {aid}"),
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::Id;
    use crate::pose::Pose;
    use crate::robot::{
        Actuator, ActuatorRanges, ActuatorSpec, ActuatorTarget, BiasType, DynType, Frame, GainType,
        General, Geom, Joint, JointKind, Limits, Link, MeshAsset, Mimic, Tendon, TendonJoint,
    };
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

    /// A follower whose leader follows is a chain, and a chain resolves
    /// (ADR-0025). Only a ring with no free leader is refused.
    #[test]
    fn mimic_chains_are_accepted_and_only_cycles_are_refused() {
        let (mut robot, [j0, j1, j2]) = movable_chain();
        mimic(&mut robot, j1, j0, 0.5, 0.0);
        mimic(&mut robot, j2, j1, 0.5, 0.0);
        assert_eq!(validate(&robot), Ok(()), "a chain of three");

        // A ring of two: j0 follows j1 follows j0.
        mimic(&mut robot, j2, j1, 0.5, 0.0);
        mimic(&mut robot, j0, j1, 1.0, 0.0);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::MimicCycle {
                joints: vec![j0, j1]
            }),
            "the ring is reported once, in follow order from its lowest id"
        );
        // The chain hanging off the ring is not itself a cycle.
        assert_eq!(
            validation_errors(&robot)
                .iter()
                .filter(|e| matches!(e, ValidationError::MimicCycle { .. }))
                .count(),
            1
        );

        // A ring of three.
        mimic(&mut robot, j0, j2, 1.0, 0.0);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::MimicCycle {
                joints: vec![j0, j2, j1]
            })
        );

        // A joint that follows itself stays `SelfMimic`, not a ring of one.
        let (mut robot, [_, j1, _]) = movable_chain();
        mimic(&mut robot, j1, j1, 1.0, 0.0);
        assert_eq!(validate(&robot), Err(ValidationError::SelfMimic(j1)));
        assert!(
            !validation_errors(&robot)
                .iter()
                .any(|e| matches!(e, ValidationError::MimicCycle { .. }))
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

    /// Down a chain the reach is the **free** leader's range through the
    /// composed map, not the immediate leader's declared limits (ADR-0025):
    /// a leader that cannot reach its own limits does not lend them on.
    #[test]
    fn a_chains_reach_is_composed_from_its_free_leader() {
        let (mut robot, [j0, j1, j2]) = movable_chain();
        robot.joints.get_mut(&j0).unwrap().limits = Some(Limits {
            lower: -1.0,
            upper: 1.0,
            effort: 0.0,
            velocity: 0.0,
        });
        // j1 reaches ±0.5 of its own ±3, so j2 at ×4 reaches ±2 and fits —
        // against j1's *limits* it would have been ±12 and refused.
        mimic(&mut robot, j1, j0, 0.5, 0.0);
        mimic(&mut robot, j2, j1, 4.0, 0.0);
        assert_eq!(validate(&robot), Ok(()));

        // ×8 composes to ±4, outside j2's own ±3.
        mimic(&mut robot, j2, j1, 8.0, 0.0);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::MimicExceedsLimits {
                joint: j2,
                leader: j1,
                lower: -4.0,
                upper: 4.0
            })
        );
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

    // ---- actuators (ADR-0014, ADR-0023) ----------------------------------

    /// One actuator on `joint`, replacing whatever drove it, and its id.
    fn actuate(robot: &mut Robot, joint: JointId, spec: ActuatorSpec) -> ActuatorId {
        robot
            .actuators
            .retain(|_, a| a.target.joint() != Some(joint));
        let name = robot.default_actuator_name(joint);
        let id = robot.next_id.alloc();
        robot.actuators.insert(
            id,
            Actuator {
                name,
                target: ActuatorTarget::Joint(joint),
                spec,
                ranges: ActuatorRanges::default(),
            },
        );
        id
    }

    /// Takes every actuator off `joint`.
    fn unactuate(robot: &mut Robot, joint: JointId) {
        robot
            .actuators
            .retain(|_, a| a.target.joint() != Some(joint));
    }

    #[test]
    fn an_actuator_needs_a_movable_joint_that_nothing_else_drives() {
        let (mut robot, [j0, _, _]) = movable_chain();
        let a = actuate(
            &mut robot,
            j0,
            ActuatorSpec::Position { kp: 100.0, kv: 5.0 },
        );
        assert_eq!(validate(&robot), Ok(()), "the ordinary case");

        // A `Fixed` joint has no `<joint>` in the MJCF to drive, and the
        // refusal names the actuator, not only the joint (ADR-0023).
        robot.joints.get_mut(&j0).unwrap().kind = JointKind::Fixed;
        assert_eq!(
            validate(&robot),
            Err(ValidationError::ActuatorOnFixedJoint {
                actuator: a,
                joint: j0
            })
        );

        // A mimic follower is already driven by its `<equality>`.
        let (mut robot, [j0, j1, _]) = movable_chain();
        mimic(&mut robot, j1, j0, 0.5, 0.0);
        let a = actuate(&mut robot, j1, ActuatorSpec::Motor { gear: 1.0 });
        assert_eq!(
            validate(&robot),
            Err(ValidationError::ActuatorOnMimicFollower {
                actuator: a,
                joint: j1,
                leader: j0
            })
        );
        // The leader may carry one: that is how a coupled pair is driven.
        unactuate(&mut robot, j1);
        actuate(&mut robot, j0, ActuatorSpec::Motor { gear: 1.0 });
        assert_eq!(validate(&robot), Ok(()));
    }

    /// The one-actuator-per-joint assumption ADR-0014 never wrote down is
    /// dropped: MuJoCo sums the controls of every actuator targeting a
    /// joint, so `validate` must not refuse what a real file ships
    /// (ADR-0023 §3).
    #[test]
    fn several_actuators_may_drive_one_joint() {
        let (mut robot, [j0, _, _]) = movable_chain();
        let coarse = actuate(&mut robot, j0, ActuatorSpec::Motor { gear: 50.0 });
        let name = robot.default_actuator_name(j0);
        assert_eq!(name, "a_joint_2", "the second one takes a free suffix");
        let fine = robot.next_id.alloc();
        robot.actuators.insert(
            fine,
            Actuator {
                name,
                target: ActuatorTarget::Joint(j0),
                spec: ActuatorSpec::Position { kp: 5.0, kv: 0.0 },
                ranges: ActuatorRanges::default(),
            },
        );
        assert_eq!(validate(&robot), Ok(()));
        assert_eq!(
            robot.actuators_on(j0).map(|(id, _)| id).collect::<Vec<_>>(),
            vec![coarse, fine],
            "ActuatorId order"
        );
    }

    /// A name is unique **among actuators** and nowhere else: sharing the
    /// driven joint's name is the default, and MJCF's per-element
    /// namespaces are the reason (ADR-0023 §2).
    #[test]
    fn an_actuator_name_is_unique_among_actuators_only() {
        let (mut robot, [j0, j1, _]) = movable_chain();
        actuate(&mut robot, j0, ActuatorSpec::Motor { gear: 1.0 });
        let second = actuate(&mut robot, j1, ActuatorSpec::Motor { gear: 1.0 });
        assert_eq!(robot.actuators[&second].name, "b_joint");
        assert_eq!(validate(&robot), Ok(()), "each named after its joint");

        // A link's or a joint's name is fair game — every default one is
        // already a joint's.
        robot.actuators.get_mut(&second).unwrap().name = "a".to_owned();
        assert!(robot.links.values().any(|l| l.name == "a"));
        assert_eq!(validate(&robot), Ok(()));

        // Another actuator's is not.
        robot.actuators.get_mut(&second).unwrap().name = "a_joint".to_owned();
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DuplicateActuatorName("a_joint".into()))
        );
    }

    /// A target and a name are separate fields now, so a target can dangle
    /// on its own — a hand-edited file, or a joint deleted behind the
    /// command layer's back (ADR-0023 §3).
    #[test]
    fn an_actuator_target_must_be_a_joint_of_the_document() {
        let (mut robot, [j0, _, _]) = movable_chain();
        let a = actuate(&mut robot, j0, ActuatorSpec::Motor { gear: 1.0 });
        let ghost = JointId::from_raw(9999);
        robot.actuators.get_mut(&a).unwrap().target = ActuatorTarget::Joint(ghost);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DanglingActuatorTarget {
                actuator: a,
                target: ActuatorTarget::Joint(ghost)
            })
        );
        // A tendon target goes wrong the same way, through the same variant
        // (ADR-0025 §4).
        let ghost = TendonId::from_raw(9998);
        robot.actuators.get_mut(&a).unwrap().target = ActuatorTarget::Tendon(ghost);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DanglingActuatorTarget {
                actuator: a,
                target: ActuatorTarget::Tendon(ghost)
            })
        );
        assert_eq!(
            validate(&robot).unwrap_err().to_string(),
            format!("actuator {a} drives missing tendon t9998")
        );
    }

    #[test]
    fn actuator_gains_must_be_numbers_mujoco_can_use() {
        let (mut robot, [j0, _, _]) = movable_chain();
        // Zero gains are ordinary for a servo…
        actuate(&mut robot, j0, ActuatorSpec::Position { kp: 0.0, kv: 0.0 });
        assert_eq!(validate(&robot), Ok(()));
        // …but a negative one pushes the joint away from its target.
        let a = actuate(&mut robot, j0, ActuatorSpec::Position { kp: -1.0, kv: 5.0 });
        assert_eq!(
            validate(&robot),
            Err(ValidationError::InvalidActuatorGain {
                actuator: a,
                what: "kp -1".into()
            })
        );
        let a = actuate(&mut robot, j0, ActuatorSpec::Velocity { kv: -0.5 });
        assert_eq!(
            validate(&robot),
            Err(ValidationError::InvalidActuatorGain {
                actuator: a,
                what: "kv -0.5".into()
            })
        );
        // A negative `gear` merely reverses the joint; a zero one is an
        // actuator that cannot move it.
        actuate(&mut robot, j0, ActuatorSpec::Motor { gear: -50.0 });
        assert_eq!(validate(&robot), Ok(()));
        let a = actuate(&mut robot, j0, ActuatorSpec::Motor { gear: 0.0 });
        assert_eq!(
            validate(&robot),
            Err(ValidationError::InvalidActuatorGain {
                actuator: a,
                what: "gear 0".into()
            })
        );
        for actuator in [
            ActuatorSpec::Position {
                kp: f64::NAN,
                kv: 1.0,
            },
            ActuatorSpec::Velocity { kv: f64::INFINITY },
            ActuatorSpec::Motor {
                gear: f64::NEG_INFINITY,
            },
        ] {
            let a = actuate(&mut robot, j0, actuator);
            let err = validate(&robot).unwrap_err();
            assert!(
                matches!(&err, ValidationError::NonFinite { what }
                    if what.contains(&format!("actuator {a}"))),
                "{err}"
            );
        }
    }

    /// A `General` (ADR-0024) is refused for exactly two things: a `prm`
    /// vector longer than MuJoCo's ten, and a non-finite entry or `gear`.
    /// Its gains are otherwise MuJoCo's to interpret — a negative or zero
    /// one is **not** `InvalidActuatorGain`, which is about the presets.
    #[test]
    fn a_general_actuator_is_refused_only_for_length_and_finiteness() {
        let (mut robot, [j0, _, _]) = movable_chain();
        let filter = General {
            dyntype: DynType::Filter,
            gaintype: GainType::Fixed,
            biastype: BiasType::Affine,
            dynprm: vec![0.1],
            gainprm: vec![200.0],
            biasprm: vec![0.0, -200.0, -5.0],
            gear: 1.0,
        };
        actuate(&mut robot, j0, ActuatorSpec::General(filter.clone()));
        assert_eq!(validate(&robot), Ok(()));
        // Gains the presets would refuse are fine on a `General`.
        actuate(
            &mut robot,
            j0,
            ActuatorSpec::General(General {
                gainprm: vec![-200.0],
                gear: 0.0,
                ..filter.clone()
            }),
        );
        assert_eq!(validate(&robot), Ok(()), "MuJoCo's to interpret");
        // A bare one is MuJoCo's defaults and legal.
        actuate(&mut robot, j0, ActuatorSpec::General(General::default()));
        assert_eq!(validate(&robot), Ok(()));
        // Ten entries fit; an eleventh does not.
        actuate(
            &mut robot,
            j0,
            ActuatorSpec::General(General {
                biasprm: vec![1.0; General::MAX_PRM],
                ..filter.clone()
            }),
        );
        assert_eq!(validate(&robot), Ok(()));
        let a = actuate(
            &mut robot,
            j0,
            ActuatorSpec::General(General {
                biasprm: vec![1.0; General::MAX_PRM + 1],
                ..filter.clone()
            }),
        );
        let err = validate(&robot).unwrap_err();
        assert_eq!(
            err,
            ValidationError::ActuatorPrmTooLong {
                actuator: a,
                what: "biasprm",
                len: 11
            }
        );
        assert_eq!(
            err.to_string(),
            format!("actuator {a} has 11 biasprm entries; MuJoCo holds at most 10")
        );
        // A non-finite entry names its slot; a non-finite gear its name.
        let a = actuate(
            &mut robot,
            j0,
            ActuatorSpec::General(General {
                dynprm: vec![0.1, f64::NAN],
                ..filter.clone()
            }),
        );
        assert_eq!(
            validate(&robot),
            Err(ValidationError::NonFinite {
                what: format!("dynprm[1] of actuator {a}")
            })
        );
        let a = actuate(
            &mut robot,
            j0,
            ActuatorSpec::General(General {
                gear: f64::INFINITY,
                ..filter
            }),
        );
        assert_eq!(
            validate(&robot),
            Err(ValidationError::NonFinite {
                what: format!("gear of actuator {a}")
            })
        );
    }

    /// The ranges an actuator keeps from its file (ADR-0024) are numbers
    /// of the document and finite like every other; their flags and
    /// bounds are MuJoCo's business, so nothing else about them is
    /// refused.
    #[test]
    fn actuator_ranges_must_be_finite_and_nothing_more() {
        let (mut robot, [j0, _, _]) = movable_chain();
        let a = actuate(&mut robot, j0, ActuatorSpec::Motor { gear: 1.0 });
        let entry = robot.actuators.get_mut(&a).unwrap();
        entry.ranges = ActuatorRanges {
            ctrl: Some([1.0, -1.0]),
            force: Some([0.0, 0.0]),
            ctrl_limited: Some(true),
            force_limited: Some(false),
        };
        assert_eq!(
            validate(&robot),
            Ok(()),
            "an inverted or empty range is MuJoCo's to judge"
        );
        robot.actuators.get_mut(&a).unwrap().ranges.force = Some([-1.0, f64::INFINITY]);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::NonFinite {
                what: format!("forcerange of actuator {a}")
            })
        );
        robot.actuators.get_mut(&a).unwrap().ranges = ActuatorRanges {
            ctrl: Some([f64::NAN, 1.0]),
            ..ActuatorRanges::default()
        };
        assert_eq!(
            validate(&robot),
            Err(ValidationError::NonFinite {
                what: format!("ctrlrange of actuator {a}")
            })
        );
    }
    // ---- fixed tendons (ADR-0025 §4) -------------------------------------

    fn tendon_mut(robot: &mut Robot, id: TendonId) -> &mut Tendon {
        robot.tendons.get_mut(&id).unwrap()
    }

    fn with_tendon(
        joints: impl FnOnce([JointId; 3]) -> Vec<TendonJoint>,
    ) -> (Robot, [JointId; 3], TendonId) {
        let (mut robot, js) = movable_chain();
        let id: TendonId = robot.next_id.alloc();
        robot.tendons.insert(id, Tendon::new("grip", joints(js)));
        (robot, js, id)
    }

    /// A tendon over two free joints is the ordinary case, and every joint
    /// it weighs must exist, move, appear once and count for something.
    #[test]
    fn a_tendons_joints_must_exist_move_and_each_count_once() {
        let (mut robot, [j0, _, _], t) = with_tendon(|[j0, j1, _]| {
            vec![
                TendonJoint {
                    joint: j0,
                    coef: 1.0,
                },
                TendonJoint {
                    joint: j1,
                    coef: -1.0,
                },
            ]
        });
        assert_eq!(validate(&robot), Ok(()), "the ordinary case");

        tendon_mut(&mut robot, t).joints.clear();
        assert_eq!(validate(&robot), Err(ValidationError::EmptyTendon(t)));

        let ghost = JointId::from_raw(999);
        tendon_mut(&mut robot, t).joints = vec![TendonJoint {
            joint: ghost,
            coef: 1.0,
        }];
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DanglingTendonJoint {
                tendon: t,
                joint: ghost
            })
        );

        // A `Fixed` joint has no value to weigh — this is what refuses
        // demoting one of a tendon's joints, the way `MimicLeaderFixed`
        // refuses demoting a leader.
        tendon_mut(&mut robot, t).joints = vec![TendonJoint {
            joint: j0,
            coef: 1.0,
        }];
        robot.joints.get_mut(&j0).unwrap().kind = JointKind::Fixed;
        assert_eq!(
            validate(&robot),
            Err(ValidationError::TendonOnFixedJoint {
                tendon: t,
                joint: j0
            })
        );
        robot.joints.get_mut(&j0).unwrap().kind = JointKind::Revolute;

        tendon_mut(&mut robot, t).joints = vec![
            TendonJoint {
                joint: j0,
                coef: 1.0,
            },
            TendonJoint {
                joint: j0,
                coef: 2.0,
            },
        ];
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DuplicateTendonJoint {
                tendon: t,
                joint: j0
            })
        );

        tendon_mut(&mut robot, t).joints = vec![TendonJoint {
            joint: j0,
            coef: 0.0,
        }];
        assert_eq!(
            validate(&robot),
            Err(ValidationError::ZeroTendonCoef {
                tendon: t,
                joint: j0
            })
        );

        tendon_mut(&mut robot, t).joints[0].coef = f64::NAN;
        assert!(
            matches!(validate(&robot), Err(ValidationError::NonFinite { what }) if what.contains("coef")),
            "{:?}",
            validate(&robot)
        );
    }

    /// A tendon's name is its own — MJCF namespaces elements by type — and
    /// its `range`, held in MJCF's own absolute terms, must bound something.
    #[test]
    fn tendon_names_are_unique_among_tendons_and_a_range_must_bound_something() {
        let (mut robot, [j0, _, _], t) = with_tendon(|[j0, _, _]| {
            vec![TendonJoint {
                joint: j0,
                coef: 1.0,
            }]
        });
        // The joint it runs over is named "a_joint": one namespace each.
        robot.tendons.get_mut(&t).unwrap().name = robot.joints[&j0].name.clone();
        assert_eq!(validate(&robot), Ok(()));

        let second: TendonId = robot.next_id.alloc();
        let twin = robot.tendons[&t].clone();
        robot.tendons.insert(second, twin);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::DuplicateTendonName(
                robot.joints[&j0].name.clone()
            ))
        );
        robot.tendons.remove(&second);

        tendon_mut(&mut robot, t).range = Some([-0.5, 0.5]);
        assert_eq!(validate(&robot), Ok(()));
        tendon_mut(&mut robot, t).range = Some([0.5, 0.5]);
        assert_eq!(
            validate(&robot),
            Err(ValidationError::InvalidTendonRange {
                tendon: t,
                lower: 0.5,
                upper: 0.5
            })
        );
        tendon_mut(&mut robot, t).range = Some([0.5, f64::INFINITY]);
        assert!(
            matches!(validate(&robot), Err(ValidationError::NonFinite { what }) if what.contains("range")),
            "{:?}",
            validate(&robot)
        );
        tendon_mut(&mut robot, t).range = None;

        for set in [
            |t: &mut Tendon| t.stiffness = f64::NAN,
            |t: &mut Tendon| t.damping = f64::NAN,
            |t: &mut Tendon| t.frictionloss = f64::NAN,
        ] {
            let mut robot = robot.clone();
            set(robot.tendons.get_mut(&t).unwrap());
            assert!(
                matches!(validate(&robot), Err(ValidationError::NonFinite { .. })),
                "a non-finite number in a tendon is refused like every other"
            );
        }
    }

    /// The joint-specific refusals of `check_actuators` say nothing about a
    /// tendon actuator: a tendon's joints are free and movable by
    /// construction, so a `Fixed` joint or a mimic follower on the tendon is
    /// the *tendon's* refusal, not the actuator's (ADR-0025 §4).
    #[test]
    fn a_tendon_actuator_only_needs_its_tendon_and_its_gains() {
        let (mut robot, [j0, j1, _], t) = with_tendon(|[j0, j1, _]| {
            vec![
                TendonJoint {
                    joint: j0,
                    coef: 1.0,
                },
                TendonJoint {
                    joint: j1,
                    coef: 1.0,
                },
            ]
        });
        // `j1` follows `j0`, which would refuse an actuator *on* `j1` — the
        // tendon's actuator is unbothered.
        mimic(&mut robot, j1, j0, 1.0, 0.0);
        let a: ActuatorId = robot.next_id.alloc();
        robot.actuators.insert(
            a,
            Actuator {
                name: "grip".to_owned(),
                target: ActuatorTarget::Tendon(t),
                spec: ActuatorSpec::Motor { gear: 20.0 },
                ranges: ActuatorRanges::default(),
            },
        );
        assert_eq!(validate(&robot), Ok(()));

        // Its gains are checked like any other actuator's.
        robot.actuators.get_mut(&a).unwrap().spec = ActuatorSpec::Motor { gear: 0.0 };
        assert!(
            matches!(
                validate(&robot),
                Err(ValidationError::InvalidActuatorGain { actuator, .. }) if actuator == a
            ),
            "{:?}",
            validate(&robot)
        );
    }
}
