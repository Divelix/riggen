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
mod tri_mesh;

pub use aabb::Aabb;
pub use error::MeshError;
pub use ray::{Ray, ray_triangle};
pub use tri_mesh::TriMesh;
