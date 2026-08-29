//! The headless [`MeshLookup`]: every asset of a `Robot` read from disk and
//! brought to document meters, the way the app's mesh store does it
//! (`scale`, then `fix_up`). The export CLI and the tests use this; the app
//! implements `MeshLookup` on its own store.

use std::collections::BTreeMap;
use std::sync::Arc;

use riggen_core::glam::{DMat4, DQuat, DVec3};
use riggen_core::inertial::MeshLookup;
use riggen_core::{MeshAsset, MeshId, Robot};
use riggen_mesh::TriMesh;

use crate::resolve::ExportError;

#[derive(Debug, Default, Clone)]
pub struct MeshStore(pub BTreeMap<MeshId, Arc<TriMesh>>);

impl MeshStore {
    /// Loads every asset `robot` references. Assets that fail to load are
    /// reported and skipped, so `resolve` can still name every other
    /// problem in the same pass.
    pub fn load(robot: &Robot) -> (Self, Vec<ExportError>) {
        let mut store = BTreeMap::new();
        let mut errors = Vec::new();
        for id in robot.referenced_assets() {
            let Some(asset) = robot.assets.get(&id) else {
                continue; // `validate` reports the dangling reference
            };
            match riggen_mesh::load_mesh(&asset.path) {
                Ok(mesh) => {
                    store.insert(id, Arc::new(to_document_units(mesh, asset)));
                }
                Err(e) => errors.push(ExportError::UnloadableMesh {
                    mesh: id,
                    path: asset.path.clone(),
                    reason: e.to_string(),
                }),
            }
        }
        (Self(store), errors)
    }

    pub fn insert(&mut self, id: MeshId, mesh: TriMesh) {
        self.0.insert(id, Arc::new(mesh));
    }
}

/// `scale` then `fix_up`, as the viewport draws it.
pub fn to_document_units(mut mesh: TriMesh, asset: &MeshAsset) -> TriMesh {
    if asset.scale != 1.0 || asset.fix_up.is_some() {
        mesh.transform(&DMat4::from_scale_rotation_translation(
            DVec3::splat(asset.scale),
            asset.fix_up.unwrap_or(DQuat::IDENTITY),
            DVec3::ZERO,
        ));
    }
    mesh
}

impl MeshLookup for MeshStore {
    fn mesh(&self, id: MeshId) -> Option<&TriMesh> {
        self.0.mesh(id)
    }
}
