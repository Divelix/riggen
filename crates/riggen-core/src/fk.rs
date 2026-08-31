//! Forward kinematics (docs/02-data-model.md §Kinematics):
//! `world(child) = world(parent) ∘ joint.origin ∘ motion(kind, axis, q)`,
//! one depth-first pass from the root. This is the oracle the export
//! round-trip tests compare against (ADR-0004), and what `Reparent {
//! keep_world_pose }` reads to rewrite a joint origin.

use std::collections::BTreeMap;

use riggen_mesh::glam::{DQuat, DVec3};

use crate::ids::{FrameId, JointId, LinkId};
use crate::pose::Pose;
use crate::robot::{JointKind, Robot};

/// `q` per movable joint, radians or meters by kind. Derived UI state,
/// never saved; a joint absent from the map reads as `0`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct JointState(pub BTreeMap<JointId, f64>);

impl JointState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn get(&self, joint: JointId) -> f64 {
        self.0.get(&joint).copied().unwrap_or(0.0)
    }

    pub fn set(&mut self, joint: JointId, q: f64) {
        self.0.insert(joint, q);
    }
}

/// The child frame's displacement for joint value `q`: a rotation about
/// `axis` for `Revolute` / `Continuous`, a translation along it for
/// `Prismatic`, identity for `Fixed`. `axis` need not be unit length; a
/// zero axis (rejected by `validate`) yields identity rather than NaN.
pub fn motion(kind: JointKind, axis: DVec3, q: f64) -> Pose {
    let axis = axis.normalize_or_zero();
    match kind {
        JointKind::Fixed => Pose::IDENTITY,
        JointKind::Revolute | JointKind::Continuous => {
            if axis == DVec3::ZERO {
                Pose::IDENTITY
            } else {
                Pose::from_rotation(DQuat::from_axis_angle(axis, q))
            }
        }
        JointKind::Prismatic => Pose::from_translation(axis * q),
    }
}

/// `q` with every mimic joint's value replaced by the one its leader
/// implies: `q(follower) = multiplier * q(leader) + offset` (ADR-0013).
///
/// This is the **single implementation** of that rule — [`fk`], the Joints
/// window and `--fk-samples` all read it, so the derived number cannot
/// drift between what the viewport shows and what the export writes. A
/// follower's own entry in `q` is ignored rather than an error: it is
/// derived state the caller need not know about.
///
/// One pass, not a fixed point: `validate` rejects a leader that itself
/// mimics, so every leader's value is already the caller's.
pub fn resolve_q(robot: &Robot, q: &JointState) -> JointState {
    let mut out = q.clone();
    for (&jid, joint) in &robot.joints {
        if let Some(mimic) = joint.mimic {
            out.set(jid, mimic.multiplier * q.get(mimic.joint) + mimic.offset);
        }
    }
    out
}

/// World pose of every link reachable from the root for the given joint
/// values, mimic joints resolved through [`resolve_q`]. A link the tree
/// does not reach (only possible in a document `validate` rejects) is
/// simply absent from the result.
pub fn fk(robot: &Robot, q: &JointState) -> BTreeMap<LinkId, Pose> {
    let q = &resolve_q(robot, q);
    // parent link → its child joints, so the walk does not rescan `joints`
    // at every node.
    let mut children: BTreeMap<LinkId, Vec<JointId>> = BTreeMap::new();
    for (&jid, joint) in &robot.joints {
        children.entry(joint.parent).or_default().push(jid);
    }

    let mut world = BTreeMap::new();
    let mut stack = vec![(robot.root, Pose::IDENTITY)];
    while let Some((link, pose)) = stack.pop() {
        if world.insert(link, pose).is_some() {
            continue; // a loop; validate reports it, fk just terminates
        }
        for jid in children.get(&link).into_iter().flatten() {
            let joint = &robot.joints[jid];
            let local = joint
                .origin
                .compose(&motion(joint.kind, joint.axis, q.get(*jid)));
            stack.push((joint.child, pose.compose(&local)));
        }
    }
    world
}

/// World pose of every named frame: `world(frame.parent) ∘ frame.pose`
/// over one [`fk`] pass (ADR-0012). [`fk`] itself keeps returning links
/// only — its `BTreeMap<LinkId, Pose>` is the export oracle and the
/// round-trip tests' contract, and a frame is not a body.
///
/// A frame on a link the tree does not reach (only possible in a document
/// `validate` rejects) is absent from the result.
pub fn frames(robot: &Robot, q: &JointState) -> BTreeMap<FrameId, Pose> {
    let world = fk(robot, q);
    robot
        .frames
        .iter()
        .filter_map(|(&id, frame)| Some((id, world.get(&frame.parent)?.compose(&frame.pose))))
        .collect()
}

/// The joint origin that puts `link` at `world` in the **zero
/// configuration** — the inverse of one step of [`fk`].
///
/// `world(link) = world(parent) ∘ origin` at `q = 0`, so the origin wanted
/// is `world(parent)⁻¹ ∘ world`. This is what the link gizmo and the align
/// tool commit through a single `SetJoint`: the caller knows where the part
/// should end up in the world and needs the number the document stores.
///
/// `None` for the root (no parent joint to write) and for a link the tree
/// does not reach.
pub fn origin_for_world(robot: &Robot, link: LinkId, world: Pose) -> Option<Pose> {
    let joint = robot.parent_joint(link)?;
    let parent = robot.joints[&joint].parent;
    let poses = fk(robot, &JointState::default());
    Some(poses.get(&parent)?.inverse().compose(&world))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::Id;
    use crate::robot::{Joint, Limits, Link};
    use std::f64::consts::FRAC_PI_2;

    const EPS: f64 = 1e-9;

    fn assert_vec_eq(a: DVec3, b: DVec3) {
        assert!((a - b).length() < EPS, "{a} != {b}");
    }

    fn assert_rot_eq(a: DQuat, b: DQuat) {
        assert!(
            a.abs_diff_eq(b, EPS) || a.abs_diff_eq(-b, EPS),
            "{a} != {b}"
        );
    }

    fn assert_pose_eq(a: &Pose, b: &Pose) {
        assert_vec_eq(a.t, b.t);
        assert_rot_eq(a.r, b.r);
    }

    fn limits() -> Option<Limits> {
        Some(Limits {
            lower: -3.0,
            upper: 3.0,
            effort: 0.0,
            velocity: 0.0,
        })
    }

    fn attach(
        robot: &mut Robot,
        parent: LinkId,
        (link, joint): (LinkId, JointId),
        name: &str,
        kind: JointKind,
        origin: Pose,
        axis: DVec3,
    ) {
        robot.links.insert(link, Link::new(name));
        robot.joints.insert(
            joint,
            Joint {
                name: format!("{name}_joint"),
                kind,
                parent,
                child: link,
                origin,
                axis,
                limits: if kind.requires_limits() {
                    limits()
                } else {
                    None
                },
                dynamics: Default::default(),
                mimic: None,
                actuator: None,
            },
        );
    }

    /// base ─j1(revolute Z, origin +1x)─ l1 ─j2(revolute Y, origin +1x)─ l2
    /// ─j3(prismatic X, origin +1z)─ l3. `reversed` allocates the ids from
    /// the leaf up, so map order disagrees with tree order.
    fn chain(reversed: bool) -> (Robot, [LinkId; 3], [JointId; 3]) {
        let mut robot = Robot::new("chain");
        let mut ids: Vec<(LinkId, JointId)> = (0..3)
            .map(|_| (robot.next_id.alloc(), robot.next_id.alloc()))
            .collect();
        if reversed {
            ids.reverse();
        }
        let links = [ids[0].0, ids[1].0, ids[2].0];
        let joints = [ids[0].1, ids[1].1, ids[2].1];
        let root = robot.root;
        let x1 = Pose::from_translation(DVec3::X);
        attach(
            &mut robot,
            root,
            ids[0],
            "l1",
            JointKind::Revolute,
            x1,
            DVec3::Z,
        );
        attach(
            &mut robot,
            links[0],
            ids[1],
            "l2",
            JointKind::Revolute,
            x1,
            DVec3::Y,
        );
        attach(
            &mut robot,
            links[1],
            ids[2],
            "l3",
            JointKind::Prismatic,
            Pose::from_translation(DVec3::Z),
            DVec3::X,
        );
        assert_eq!(crate::validate(&robot), Ok(()));
        (robot, links, joints)
    }

    /// Hand-computed world poses for q = (90°, 90°, 0.5).
    fn expected() -> [Pose; 3] {
        let rz = DQuat::from_rotation_z(FRAC_PI_2);
        let rzy = rz * DQuat::from_rotation_y(FRAC_PI_2);
        [
            // l1: origin (1,0,0), turned 90° about Z.
            Pose::new(DVec3::new(1.0, 0.0, 0.0), rz),
            // l2: l1's frame carries (1,0,0) to (0,1,0); rotation Rz·Ry.
            Pose::new(DVec3::new(1.0, 1.0, 0.0), rzy),
            // l3: offset (0,0,1) in l2 → Ry maps z→x, Rz maps x→y: (0,1,0);
            // slide 0.5 along l3's x, which is world (0,0,-1): (0,0,-0.5).
            Pose::new(DVec3::new(1.0, 2.0, -0.5), rzy),
        ]
    }

    #[test]
    fn origin_for_world_round_trips_through_fk() {
        let (mut robot, links, _) = chain(false);
        // Every link of the chain, put somewhere awkward and read back.
        for (i, &link) in links.iter().enumerate() {
            let want = Pose::from_xyz_rpy(
                DVec3::new(0.5 * i as f64 - 1.0, 2.25, -0.75),
                DVec3::new(0.3, -0.9, 1.4),
            );
            let origin = origin_for_world(&robot, link, want).expect("a non-root link");
            let joint = robot.parent_joint(link).unwrap();
            robot.joints.get_mut(&joint).unwrap().origin = origin;
            assert_eq!(crate::validate(&robot), Ok(()));
            assert_pose_eq(&fk(&robot, &JointState::new())[&link], &want);
        }
        // …and the whole chain still hangs together afterwards.
        let world = fk(&robot, &JointState::new());
        assert_eq!(world.len(), 4);
    }

    #[test]
    fn origin_for_world_has_nothing_to_write_for_the_root() {
        let (robot, _, _) = chain(false);
        assert_eq!(origin_for_world(&robot, robot.root, Pose::IDENTITY), None);
        assert_eq!(
            origin_for_world(&robot, LinkId::from_raw(999), Pose::IDENTITY),
            None
        );
    }

    #[test]
    fn three_joint_chain_matches_hand_computed_poses() {
        let (robot, links, joints) = chain(false);
        let mut q = JointState::new();
        q.set(joints[0], FRAC_PI_2);
        q.set(joints[1], FRAC_PI_2);
        q.set(joints[2], 0.5);
        let world = fk(&robot, &q);
        assert_eq!(world.len(), 4);
        assert_eq!(world[&robot.root], Pose::IDENTITY);
        for (link, want) in links.iter().zip(expected()) {
            assert_pose_eq(&world[link], &want);
        }
        // The leaf's x axis points down in the world.
        assert_vec_eq(world[&links[2]].r * DVec3::X, -DVec3::Z);
    }

    #[test]
    fn chain_order_is_independent_of_id_order() {
        let (robot, links, joints) = chain(true);
        assert!(joints[0] > joints[2], "leaf ids allocated first");
        let mut q = JointState::new();
        q.set(joints[0], FRAC_PI_2);
        q.set(joints[1], FRAC_PI_2);
        q.set(joints[2], 0.5);
        let world = fk(&robot, &q);
        for (link, want) in links.iter().zip(expected()) {
            assert_pose_eq(&world[link], &want);
        }
    }

    #[test]
    fn absent_q_reads_as_zero_and_gives_the_origins() {
        let (robot, links, joints) = chain(false);
        let q = JointState::new();
        assert_eq!(q.get(joints[0]), 0.0);
        let world = fk(&robot, &q);
        assert_pose_eq(&world[&links[0]], &Pose::from_translation(DVec3::X));
        assert_pose_eq(
            &world[&links[1]],
            &Pose::from_translation(DVec3::new(2.0, 0.0, 0.0)),
        );
        assert_pose_eq(
            &world[&links[2]],
            &Pose::from_translation(DVec3::new(2.0, 0.0, 1.0)),
        );
    }

    #[test]
    fn fixed_joint_is_identity_whatever_q_says() {
        let (mut robot, links, joints) = chain(false);
        let j = robot.joints.get_mut(&joints[0]).unwrap();
        j.kind = JointKind::Fixed;
        j.limits = None;
        let mut q = JointState::new();
        q.set(joints[0], 1.0);
        let world = fk(&robot, &q);
        assert_pose_eq(&world[&links[0]], &Pose::from_translation(DVec3::X));
        assert_eq!(motion(JointKind::Fixed, DVec3::Z, 1.0), Pose::IDENTITY);
    }

    #[test]
    fn motion_by_kind() {
        let rot = motion(JointKind::Revolute, DVec3::Z * 3.0, FRAC_PI_2);
        assert_vec_eq(rot.transform_point(DVec3::X), DVec3::Y);
        let cont = motion(JointKind::Continuous, DVec3::Y, FRAC_PI_2);
        assert_vec_eq(cont.transform_point(DVec3::Z), DVec3::X);
        let slide = motion(JointKind::Prismatic, DVec3::new(0.0, 2.0, 0.0), 0.5);
        assert_pose_eq(&slide, &Pose::from_translation(DVec3::new(0.0, 0.5, 0.0)));
        assert_eq!(
            motion(JointKind::Revolute, DVec3::ZERO, 1.0),
            Pose::IDENTITY,
            "zero axis is identity, not NaN"
        );
    }

    #[test]
    fn unreachable_link_is_absent_and_a_loop_terminates() {
        let (mut robot, links, joints) = chain(false);
        // Hang l1 under l3: l1 → l2 → l3 → l1 is unreachable from the root.
        robot.joints.get_mut(&joints[0]).unwrap().parent = links[2];
        let world = fk(&robot, &JointState::new());
        assert_eq!(world.len(), 1);
        assert!(world.contains_key(&robot.root));
        let _ = LinkId::from_raw(0);
    }

    #[test]
    fn frames_ride_their_link_and_fk_still_returns_links_only() {
        let mut robot = Robot::new("r");
        let root = robot.root;
        let arm: LinkId = robot.next_id.alloc();
        robot.links.insert(arm, Link::new("arm"));
        let hinge: JointId = robot.next_id.alloc();
        robot.joints.insert(
            hinge,
            Joint {
                kind: JointKind::Revolute,
                axis: DVec3::Z,
                origin: Pose::from_translation(DVec3::X),
                limits: limits(),
                ..Joint::fixed("hinge", root, arm)
            },
        );
        let tcp: crate::ids::FrameId = robot.next_id.alloc();
        robot.frames.insert(
            tcp,
            crate::robot::Frame {
                name: "tcp".into(),
                parent: arm,
                pose: Pose::from_translation(DVec3::X * 0.5),
            },
        );
        // A frame on the root, to pin the identity case too.
        let base: crate::ids::FrameId = robot.next_id.alloc();
        robot.frames.insert(
            base,
            crate::robot::Frame {
                name: "base_mark".into(),
                parent: root,
                pose: Pose::from_translation(DVec3::Z * 0.1),
            },
        );

        let mut q = JointState::new();
        q.set(hinge, FRAC_PI_2);
        let world = fk(&robot, &q);
        assert_eq!(world.len(), 2, "fk returns links only");
        let f = frames(&robot, &q);
        assert_eq!(f.len(), 2);
        // The arm sits at +X turned 90° about Z, so its +X half-metre
        // points along +Y from there.
        assert_pose_eq(
            &f[&tcp],
            &world[&arm].compose(&Pose::from_translation(DVec3::X * 0.5)),
        );
        assert_vec_eq(f[&tcp].t, DVec3::new(1.0, 0.5, 0.0));
        assert_vec_eq(f[&base].t, DVec3::Z * 0.1);
        assert_rot_eq(f[&base].r, DQuat::IDENTITY);
    }

    /// A follower's pose is the one its leader implies, at every
    /// configuration, and whatever the caller put in its own slot
    /// (ADR-0013).
    #[test]
    fn a_mimic_joint_follows_its_leader_through_fk() {
        let (mut robot, links, joints) = chain(false);
        let (leader, follower) = (joints[0], joints[1]);
        robot.joints.get_mut(&follower).unwrap().mimic = Some(crate::robot::Mimic {
            joint: leader,
            multiplier: -0.5,
            offset: 0.1,
        });
        assert_eq!(crate::validate(&robot), Ok(()));

        // The same tree without the coupling, driven by hand.
        let mut free = robot.clone();
        free.joints.get_mut(&follower).unwrap().mimic = None;

        for driver in [0.0, 0.5, -1.2] {
            let mut q = JointState::new();
            q.set(leader, driver);
            // A stale value in the follower's own slot is ignored, not an
            // error: it is derived state.
            q.set(follower, 99.0);
            q.set(joints[2], 0.25);

            let derived = -0.5 * driver + 0.1;
            assert!((resolve_q(&robot, &q).get(follower) - derived).abs() < EPS);

            let mut by_hand = q.clone();
            by_hand.set(follower, derived);
            let coupled = fk(&robot, &q);
            let expected = fk(&free, &by_hand);
            for link in links {
                assert_pose_eq(&coupled[&link], &expected[&link]);
            }
            // …and it really moved: the follower is not stuck at zero.
            if driver != 0.0 {
                let mut still = q.clone();
                still.set(follower, 0.0);
                assert_ne!(coupled[&links[2]].t, fk(&free, &still)[&links[2]].t);
            }
        }
    }
}
