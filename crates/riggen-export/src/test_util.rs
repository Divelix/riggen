//! Test-only helpers shared by the resolver and writer tests.

use std::path::{Path, PathBuf};

use riggen_core::glam::DVec3;
use riggen_core::{
    Command, Geom, Joint, JointKind, Limits, Link, LinkId, MeshAsset, MeshId, Pose, Robot,
};
use riggen_mesh::TriMesh;

use crate::MeshStore;
use crate::resolve::{ExportError, ExportOptions, ResolvedRobot, resolve};

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

    pub(crate) fn resolve(&self) -> Result<ResolvedRobot, Vec<ExportError>> {
        resolve(&self.robot, &self.store, &ExportOptions::default())
    }
}
