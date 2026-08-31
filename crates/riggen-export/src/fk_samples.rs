//! `<name>.fk.json`: the poses `riggen_core::fk` gives every link — and
//! every named frame — at a few joint configurations, which
//! `python/tests/test_mjcf_load.py` compares against MuJoCo's `mj_forward`
//! (ADR-0004 §2, ADR-0012). Written beside the export by `riggen --export
//! --fk-samples`.

use std::collections::BTreeMap;

use riggen_core::{ActuatorSpec, JointKind, JointState, Robot, fk, resolve_q};
use serde::Serialize;

use crate::xml::quat_wxyz;

/// Fractions of each joint's range (of ±π for a continuous one) the
/// samples sit at: rest, then four that move every joint differently.
pub const FRACTIONS: [&[f64]; 5] = [
    &[0.0],
    &[0.5],
    &[-0.5],
    &[0.9, -0.3, 0.6],
    &[-0.7, 0.8, -0.2],
];

#[derive(Debug, Serialize)]
pub struct Samples {
    /// Movable joint names, in the order `q` is given.
    pub joints: Vec<String>,
    /// What the MJCF's `<actuator>` block should hold (ADR-0014). Empty
    /// for a document with none — an imported URDF, say, which has no
    /// actuator to bring.
    pub actuators: Vec<SampledActuator>,
    pub samples: Vec<Sample>,
}

/// One `<actuator>` element, as the numbers rather than as XML: what
/// `test_mjcf_load.py` reads out of `MjModel` and compares. Derived from
/// the document here and from `ResolvedJoint` in `mjcf.rs` — two
/// statements of one rule, the way `fk` and `mj_forward` are.
#[derive(Debug, Serialize)]
pub struct SampledActuator {
    /// The element's name, which is its joint's name.
    pub name: String,
    /// `position`, `velocity` or `motor`.
    pub kind: String,
    /// The joint it drives — the same string, spelled out so the check is
    /// not a restatement of the naming rule.
    pub joint: String,
    /// `kp` / `kv` / `gear` by name.
    pub gains: BTreeMap<String, f64>,
    /// Absent where MJCF leaves the attribute out and MuJoCo's unbounded
    /// default stands.
    pub ctrlrange: Option<[f64; 2]>,
    pub forcerange: Option<[f64; 2]>,
}

#[derive(Debug, Serialize)]
pub struct Sample {
    pub q: Vec<f64>,
    /// World pose per link name (an MJCF `<body>`).
    pub links: BTreeMap<String, WorldPose>,
    /// World pose per frame name (an MJCF `<site>`, ADR-0012). Empty for a
    /// document with no frames — an imported URDF, say, where the dummy
    /// links came back as links.
    pub sites: BTreeMap<String, WorldPose>,
}

#[derive(Debug, Serialize)]
pub struct WorldPose {
    pub pos: [f64; 3],
    /// MuJoCo order, `w x y z`.
    pub quat: [f64; 4],
}

impl WorldPose {
    fn of(pose: &riggen_core::Pose) -> Self {
        Self {
            pos: pose.t.to_array(),
            quat: quat_wxyz(pose.r),
        }
    }
}

/// Every actuator in `robot`, in `JointId` order, with the ranges the MJCF
/// writer derives from the same joint (ADR-0014).
fn actuators(robot: &Robot) -> Vec<SampledActuator> {
    let symmetric = |v: f64| (v != 0.0).then_some([-v, v]);
    robot
        .joints
        .values()
        .filter_map(|joint| {
            let actuator = joint.actuator?;
            let (gains, ctrlrange) = match actuator {
                ActuatorSpec::Position { kp, kv } => (
                    BTreeMap::from([("kp".to_owned(), kp), ("kv".to_owned(), kv)]),
                    joint.limits.map(|l| [l.lower, l.upper]),
                ),
                ActuatorSpec::Velocity { kv } => (
                    BTreeMap::from([("kv".to_owned(), kv)]),
                    joint.limits.and_then(|l| symmetric(l.velocity)),
                ),
                ActuatorSpec::Motor { gear } => (
                    BTreeMap::from([("gear".to_owned(), gear)]),
                    Some([-1.0, 1.0]),
                ),
            };
            Some(SampledActuator {
                name: joint.name.clone(),
                kind: actuator.kind_name().to_owned(),
                joint: joint.name.clone(),
                gains,
                ctrlrange,
                forcerange: joint.limits.and_then(|l| symmetric(l.effort)),
            })
        })
        .collect()
}

/// Five configurations of `robot`'s movable joints (in `JointId` order)
/// and the FK at each.
pub fn samples(robot: &Robot) -> Samples {
    let movable: Vec<_> = robot
        .joints
        .iter()
        .filter(|(_, j)| j.kind.is_movable())
        .collect();
    let mut out = Samples {
        joints: movable.iter().map(|(_, j)| j.name.clone()).collect(),
        actuators: actuators(robot),
        samples: Vec::new(),
    };
    for fractions in FRACTIONS {
        let mut state = JointState::new();
        for (i, (id, joint)) in movable.iter().enumerate() {
            let f = fractions[i % fractions.len()];
            let value = match (joint.kind, joint.limits) {
                (JointKind::Continuous, _) | (_, None) => f * std::f64::consts::PI,
                (_, Some(l)) => 0.5 * (l.lower + l.upper) + f * 0.5 * (l.upper - l.lower),
            };
            state.set(**id, value);
        }
        // A follower's slot holds its **derived** value, not the fraction
        // rule's (ADR-0013), so `q` is a `qpos` MuJoCo can be given whole
        // — the equality is soft and would otherwise fight it.
        let state = resolve_q(robot, &state);
        let q: Vec<f64> = movable.iter().map(|(id, _)| state.get(**id)).collect();
        let world = fk(robot, &state);
        let links = robot
            .links
            .iter()
            .filter_map(|(id, link)| Some((link.name.clone(), WorldPose::of(world.get(id)?))))
            .collect();
        // `world(parent) ∘ frame.pose` — the same composition `fk::frames`
        // does, over the pass just made.
        let sites = robot
            .frames
            .values()
            .filter_map(|frame| {
                let parent = world.get(&frame.parent)?;
                Some((
                    frame.name.clone(),
                    WorldPose::of(&parent.compose(&frame.pose)),
                ))
            })
            .collect();
        out.samples.push(Sample { q, links, sites });
    }
    out
}

pub fn to_json(robot: &Robot) -> String {
    let mut json = serde_json::to_string_pretty(&samples(robot)).expect("plain data");
    json.push('\n');
    json
}

#[cfg(test)]
mod tests {
    use super::*;
    use riggen_core::glam::{DQuat, DVec3};
    use riggen_core::{Command, Joint, Limits, Link, Pose};

    #[test]
    fn five_samples_within_the_limits_and_the_rest_pose_first() {
        let mut robot = Robot::new("r");
        let root = robot.root;
        let mut arm = Link::new("arm");
        arm.material = Some("PLA".into());
        Command::AddLink {
            link: Box::new(arm),
            parent: root,
            joint: Joint {
                kind: JointKind::Revolute,
                axis: DVec3::Z,
                origin: Pose::from_translation(DVec3::X),
                limits: Some(Limits {
                    lower: -1.0,
                    upper: 2.0,
                    effort: 1.0,
                    velocity: 1.0,
                }),
                ..Joint::fixed("j1", root, root)
            },
        }
        .apply(&mut robot)
        .unwrap();
        let arm = *robot.links.iter().find(|(_, l)| l.name == "arm").unwrap().0;
        Command::AddLink {
            link: Box::new(Link::new("wheel")),
            parent: arm,
            joint: Joint {
                kind: JointKind::Continuous,
                axis: DVec3::Y,
                origin: Pose::from_translation(DVec3::X),
                ..Joint::fixed("j2", arm, arm)
            },
        }
        .apply(&mut robot)
        .unwrap();

        let wheel = *robot
            .links
            .iter()
            .find(|(_, l)| l.name == "wheel")
            .unwrap()
            .0;
        let frame: riggen_core::FrameId = robot.next_id.alloc();
        robot.frames.insert(
            frame,
            riggen_core::Frame {
                name: "tcp".into(),
                parent: wheel,
                pose: Pose::from_translation(DVec3::Z * 0.25),
            },
        );

        let s = samples(&robot);
        assert_eq!(s.joints, ["j1", "j2"]);
        assert_eq!(s.samples.len(), 5);
        assert_eq!(
            s.samples[0].q,
            [0.5, 0.0],
            "rest is the middle of the range"
        );
        for sample in &s.samples {
            assert!(sample.q[0] >= -1.0 && sample.q[0] <= 2.0, "{:?}", sample.q);
            assert!(sample.q[1].abs() <= std::f64::consts::PI);
            assert_eq!(sample.links.len(), 3);
            assert_eq!(sample.links["base_link"].pos, [0.0; 3]);
            assert_eq!(sample.links["base_link"].quat, [1.0, 0.0, 0.0, 0.0]);
            // The frame rides its link: `world(wheel) ∘ (0, 0, 0.25)`.
            assert_eq!(sample.sites.len(), 1);
            let wheel = &sample.links["wheel"];
            let want = DVec3::from_array(wheel.pos)
                + DQuat::from_xyzw(wheel.quat[1], wheel.quat[2], wheel.quat[3], wheel.quat[0])
                    * (DVec3::Z * 0.25);
            let tcp = DVec3::from_array(sample.sites["tcp"].pos);
            assert!((tcp - want).length() < 1e-12, "{tcp} != {want}");
            let dq: f64 = (0..4)
                .map(|i| (sample.sites["tcp"].quat[i] - wheel.quat[i]).abs())
                .fold(0.0, f64::max);
            assert!(
                dq < 1e-12,
                "a frame at identity turns with its link: {dq:e}"
            );
        }
        // Sample 1: j1 at 1.25 rad about Z puts the wheel at x=1 rotated.
        let q1 = s.samples[1].q[0];
        let wheel = &s.samples[1].links["wheel"];
        let want = DVec3::X + DQuat::from_rotation_z(q1) * DVec3::X;
        assert!((DVec3::from_array(wheel.pos) - want).length() < 1e-12);
        let json = to_json(&robot);
        assert!(
            json.contains("\"joints\": [\n    \"j1\",\n    \"j2\"\n  ]"),
            "{json}"
        );
    }

    /// A follower's `q` is the value its leader implies, not the fraction
    /// rule's — the `qpos` MuJoCo is handed has to satisfy the equality it
    /// is given, or the soft constraint fights it (ADR-0013).
    #[test]
    fn a_followers_q_is_the_derived_one() {
        let mut robot = Robot::new("r");
        let root = robot.root;
        let limits = Some(Limits {
            lower: -1.0,
            upper: 1.0,
            effort: 1.0,
            velocity: 1.0,
        });
        for (name, parent) in [("j1", root), ("j2", root)] {
            Command::AddLink {
                link: Box::new(Link::new(if name == "j1" { "a" } else { "b" })),
                parent,
                joint: Joint {
                    kind: JointKind::Revolute,
                    axis: DVec3::Z,
                    origin: Pose::from_translation(DVec3::X),
                    limits,
                    ..Joint::fixed(name, parent, parent)
                },
            }
            .apply(&mut robot)
            .unwrap();
        }
        let id = |n: &str| *robot.joints.iter().find(|(_, j)| j.name == n).unwrap().0;
        let (leader, follower) = (id("j1"), id("j2"));
        robot.joints.get_mut(&follower).unwrap().mimic = Some(riggen_core::Mimic {
            joint: leader,
            multiplier: -0.5,
            offset: 0.1,
        });
        riggen_core::validate(&robot).unwrap();

        let s = samples(&robot);
        assert_eq!(s.joints, ["j1", "j2"]);
        for sample in &s.samples {
            assert!(
                (sample.q[1] - (-0.5 * sample.q[0] + 0.1)).abs() < 1e-12,
                "{:?}",
                sample.q
            );
            // …and it is a value the follower's own range allows.
            assert!(sample.q[1] >= -1.0 && sample.q[1] <= 1.0, "{:?}", sample.q);
        }
        // The five configurations are still five different ones.
        let first: Vec<f64> = s.samples.iter().map(|s| s.q[0]).collect();
        assert_eq!(first.len(), 5);
        assert!(first.windows(2).any(|w| w[0] != w[1]));
    }

    /// The `actuators` block is what `test_mjcf_load.py` holds MuJoCo to,
    /// so it must be the numbers the MJCF writer derives — including the
    /// two attributes it leaves out (ADR-0014).
    #[test]
    fn the_actuators_block_carries_the_ranges_the_mjcf_writer_derives() {
        let mut robot = Robot::new("r");
        let root = robot.root;
        for (name, kind, limits) in [
            (
                "servo",
                JointKind::Revolute,
                Some(Limits {
                    lower: -1.0,
                    upper: 2.0,
                    effort: 5.0,
                    velocity: 3.0,
                }),
            ),
            ("free", JointKind::Continuous, None),
        ] {
            Command::AddLink {
                link: Box::new(Link::new(name)),
                parent: root,
                joint: Joint {
                    kind,
                    axis: DVec3::Z,
                    origin: Pose::from_translation(DVec3::X),
                    limits,
                    ..Joint::fixed(name, root, root)
                },
            }
            .apply(&mut robot)
            .unwrap();
        }
        let id =
            |robot: &Robot, n: &str| *robot.joints.iter().find(|(_, j)| j.name == n).unwrap().0;
        let (servo, free) = (id(&robot, "servo"), id(&robot, "free"));
        robot.joints.get_mut(&servo).unwrap().actuator = Some(ActuatorSpec::Position {
            kp: 100.0,
            kv: 10.0,
        });
        robot.joints.get_mut(&free).unwrap().actuator = Some(ActuatorSpec::Velocity { kv: 2.0 });
        riggen_core::validate(&robot).unwrap();

        let a = actuators(&robot);
        assert_eq!(a.len(), 2);
        assert_eq!(
            (a[0].name.as_str(), a[0].kind.as_str()),
            ("servo", "position")
        );
        assert_eq!(a[0].joint, "servo");
        assert_eq!(a[0].gains["kp"], 100.0);
        assert_eq!(a[0].gains["kv"], 10.0);
        assert_eq!(a[0].ctrlrange, Some([-1.0, 2.0]), "the joint's own range");
        assert_eq!(a[0].forcerange, Some([-5.0, 5.0]), "±effort");
        // A `Continuous` joint has no `Limits`, so neither range is
        // written and MuJoCo's unbounded defaults stand.
        assert_eq!(
            (a[1].name.as_str(), a[1].kind.as_str()),
            ("free", "velocity")
        );
        assert_eq!(a[1].gains["kv"], 2.0);
        assert_eq!((a[1].ctrlrange, a[1].forcerange), (None, None));
        assert!(to_json(&robot).contains("\"kind\": \"position\""));

        // A motor is normalised, whatever the joint says.
        robot.joints.get_mut(&servo).unwrap().actuator = Some(ActuatorSpec::Motor { gear: 50.0 });
        let a = actuators(&robot);
        assert_eq!(a[0].ctrlrange, Some([-1.0, 1.0]));
        assert_eq!(a[0].gains["gear"], 50.0);
    }
}
