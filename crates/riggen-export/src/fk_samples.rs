//! `<name>.fk.json`: the poses `riggen_core::fk` gives every link at a few
//! joint configurations, which `python/tests/test_mjcf_load.py` compares
//! against MuJoCo's `mj_forward` (ADR-0004 §2). Written beside the export
//! by `riggen --export --fk-samples`.

use std::collections::BTreeMap;

use riggen_core::{JointKind, JointState, Robot, fk};
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
    pub samples: Vec<Sample>,
}

#[derive(Debug, Serialize)]
pub struct Sample {
    pub q: Vec<f64>,
    /// World pose per link name.
    pub links: BTreeMap<String, LinkPose>,
}

#[derive(Debug, Serialize)]
pub struct LinkPose {
    pub pos: [f64; 3],
    /// MuJoCo order, `w x y z`.
    pub quat: [f64; 4],
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
        samples: Vec::new(),
    };
    for fractions in FRACTIONS {
        let mut state = JointState::new();
        let mut q = Vec::new();
        for (i, (id, joint)) in movable.iter().enumerate() {
            let f = fractions[i % fractions.len()];
            let value = match (joint.kind, joint.limits) {
                (JointKind::Continuous, _) | (_, None) => f * std::f64::consts::PI,
                (_, Some(l)) => 0.5 * (l.lower + l.upper) + f * 0.5 * (l.upper - l.lower),
            };
            state.set(**id, value);
            q.push(value);
        }
        let world = fk(robot, &state);
        let links = robot
            .links
            .iter()
            .filter_map(|(id, link)| {
                let pose = world.get(id)?;
                Some((
                    link.name.clone(),
                    LinkPose {
                        pos: pose.t.to_array(),
                        quat: quat_wxyz(pose.r),
                    },
                ))
            })
            .collect();
        out.samples.push(Sample { q, links });
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
}
