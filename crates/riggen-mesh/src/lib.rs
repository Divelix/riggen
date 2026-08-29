//! Mesh geometry: [`TriMesh`], STL/OBJ loaders, [`Aabb`], ray/triangle. No
//! egui, no wgpu (docs/01-architecture.md §Crates).
//!
//! `f64` throughout — the document is f64 and mass properties (M3) want it;
//! the GPU path narrows to `f32` at upload (docs/02-data-model.md).
//! `glam` is re-exported so no other crate names it directly.

pub use glam;

mod aabb;
mod error;
mod ray;
mod stl;
mod tri_mesh;

pub use aabb::Aabb;
pub use error::MeshError;
pub use ray::{Ray, ray_triangle};
pub use stl::load_stl;
pub use tri_mesh::TriMesh;

/// The most triangles one mesh may have: the viewport's pick id spends 20
/// bits on `triangle + 1`, with `0` reserved for "miss"
/// (docs/01-architecture.md §Picking). Loaders reject bigger meshes with
/// [`MeshError::TooManyTriangles`]; decimating them is a backlog item.
pub const MAX_TRIANGLES: usize = (1 << 20) - 1;

/// The checks every loader ends with: the invariants of [`TriMesh::validate`]
/// plus the pick-id triangle cap.
fn finish_loaded(mesh: TriMesh, path: &std::path::Path) -> Result<TriMesh, MeshError> {
    mesh.validate()?;
    if mesh.triangle_count() > MAX_TRIANGLES {
        return Err(MeshError::TooManyTriangles {
            path: path.display().to_string(),
            count: mesh.triangle_count(),
        });
    }
    Ok(mesh)
}
