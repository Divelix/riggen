//! Test-only helpers shared by the resolver and writer tests.

use std::path::{Path, PathBuf};

use riggen_core::glam::{DQuat, DVec3};
use riggen_core::{
    ActuatorSpec, CollisionPolicy, Command, Frame, FrameId, Geom, Joint, JointKind, Limits, Link,
    LinkId, MeshAsset, MeshId, Pose, Primitive, Robot,
};
use riggen_mesh::TriMesh;

use crate::MeshStore;
use crate::resolve::{ComputeNow, ExportError, ExportOptions, ResolvedRobot, resolve};

pub(crate) fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/fixtures")
}

/// A robot whose meshes are unit cubes registered by name, not by file.
pub(crate) struct Builder {
    pub(crate) robot: Robot,
    pub(crate) store: MeshStore,
}

impl Builder {
    pub(crate) fn new() -> Self {
        let mut robot = Robot::new("test");
        robot.links.get_mut(&robot.root).unwrap().material = Some("aluminium".into());
        Self {
            robot,
            store: MeshStore::default(),
        }
    }

    pub(crate) fn mesh(&mut self, stem: &str, mesh: TriMesh) -> MeshId {
        let id = self.robot.add_asset(MeshAsset {
            path: PathBuf::from(format!("/nowhere/{stem}.stl")),
            content_hash: 0,
            scale: 1.0,
            fix_up: None,
        });
        self.store.insert(id, mesh);
        id
    }

    pub(crate) fn geom(&mut self, mesh: MeshId, pose: Pose) -> Geom {
        Geom {
            id: self.robot.next_id.alloc(),
            mesh,
            pose,
            color: None,
        }
    }

    pub(crate) fn link(
        &mut self,
        name: &str,
        parent: LinkId,
        kind: JointKind,
        mesh: Option<MeshId>,
    ) -> LinkId {
        let mut link = Link::new(name);
        link.material = Some("aluminium".into());
        if let Some(m) = mesh {
            let g = self.geom(m, Pose::IDENTITY);
            link.visuals.push(g);
        }
        let limits = kind.requires_limits().then_some(Limits {
            lower: -1.0,
            upper: 1.0,
            effort: 1.0,
            velocity: 1.0,
        });
        let joint = Joint {
            kind,
            axis: DVec3::Z,
            origin: Pose::from_translation(DVec3::Z * 0.1),
            limits,
            ..Joint::fixed(format!("{name}_joint"), parent, parent)
        };
        Command::AddLink {
            link: Box::new(link),
            parent,
            joint,
        }
        .apply(&mut self.robot)
        .unwrap();
        *self
            .robot
            .links
            .iter()
            .find(|(_, l)| l.name == name)
            .unwrap()
            .0
    }

    /// A named frame on `parent`, written straight into the document —
    /// the `AddFrame` command arrives in step 4.
    pub(crate) fn frame(&mut self, name: &str, parent: LinkId, pose: Pose) -> FrameId {
        let id: FrameId = self.robot.next_id.alloc();
        self.robot.frames.insert(
            id,
            Frame {
                name: name.to_owned(),
                parent,
                pose,
            },
        );
        id
    }

    pub(crate) fn resolve(&self) -> Result<ResolvedRobot, Vec<ExportError>> {
        resolve(
            &self.robot,
            &self.store,
            &ComputeNow,
            &ExportOptions::default(),
        )
    }
}

/// base ─(revolute)─ upper ─(prismatic)─ slider ─(continuous)─ wheel
/// ─(fixed)─ tip: every joint kind on one chain, an aluminium cube per
/// link, one primitive collision, a rotated geom, two named frames —
/// one on the root, one on the leaf, one of them rotated — one mimic (the
/// slider follows the hinge at `-0.5 q + 0.1`, ADR-0013, a reach of
/// -0.4..0.6 inside its own ±1) — and two actuators (ADR-0014): a position
/// servo on the hinge and a velocity one on the limitless wheel. The
/// slider follows, so it may carry none, and the golden keeps the
/// "need an <actuator>" comment beside the two that have one.
pub(crate) fn every_joint_kind() -> Builder {
    let mut b = Builder::new();
    let cube = b.mesh("cube", TriMesh::cube(0.05));
    let root = b.robot.root;
    let g = b.geom(cube, Pose::IDENTITY);
    riggen_core::Command::AddGeom(root, g)
        .apply(&mut b.robot)
        .unwrap();
    let upper = b.link("upper", root, JointKind::Revolute, Some(cube));
    let slider = b.link("slider", upper, JointKind::Prismatic, Some(cube));
    let wheel = b.link("wheel", slider, JointKind::Continuous, Some(cube));
    let tip = b.link("tip", wheel, JointKind::Fixed, None);
    // Damping on the hinge; a rotated visual on the wheel; a box on the
    // slider; nothing collides on the tip.
    let mut hinge = None;
    for (&id, j) in b.robot.joints.iter_mut() {
        if j.child == upper {
            j.dynamics.damping = 0.1;
            j.axis = DVec3::Y;
            hinge = Some(id);
        }
    }
    for j in b.robot.joints.values_mut() {
        if j.child == slider {
            j.mimic = Some(riggen_core::Mimic {
                joint: hinge.expect("the hinge is above the slider"),
                multiplier: -0.5,
                offset: 0.1,
            });
        }
        if j.child == upper {
            j.actuator = Some(ActuatorSpec::Position { kp: 100.0, kv: 5.0 });
        }
        if j.child == wheel {
            // `Continuous`: no limits, so no `ctrlrange` and no
            // `forcerange` — MuJoCo's unbounded defaults stand.
            j.actuator = Some(ActuatorSpec::Velocity { kv: 2.0 });
        }
    }
    b.robot.links.get_mut(&wheel).unwrap().visuals[0].pose = Pose::new(
        DVec3::new(0.0, 0.02, 0.0),
        DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
    );
    b.robot.links.get_mut(&slider).unwrap().collision =
        CollisionPolicy::Primitives(vec![Primitive::Box {
            pose: Pose::from_translation(DVec3::Z * 0.01),
            size: DVec3::new(0.1, 0.2, 0.3),
        }]);
    b.robot.links.get_mut(&tip).unwrap().collision = CollisionPolicy::None;
    b.frame(
        "camera_mount",
        root,
        Pose::new(
            DVec3::new(0.0, 0.03, 0.04),
            DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
        ),
    );
    b.frame("tcp", tip, Pose::from_translation(DVec3::Z * 0.05));
    b
}
