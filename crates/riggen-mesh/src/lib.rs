//! Mesh geometry: [`TriMesh`], STL/OBJ loaders, [`Aabb`], ray/triangle,
//! the [`feature`] module (welded adjacency, circle fits),
//! [`mass_properties`] (docs/02-data-model.md §Inertials), [`convex_hull`]
//! (quickhull), [`decompose`] (V-HACD, the [`decomp`] module) and [`fit`]
//! (box / sphere / cylinder / capsule) for collision. No egui, no wgpu
//! (docs/01-architecture.md §Crates).
//!
//! `f64` throughout — the document is f64 and mass properties want it; the
//! GPU path narrows to `f32` at upload (docs/02-data-model.md).
//! `glam` is re-exported so no other crate names it directly.

pub use glam;

mod aabb;
pub mod decomp;
mod error;
pub mod feature;
pub mod fit;
mod hull;
mod mass;
mod obj;
mod ray;
mod stl;
mod tri_mesh;

pub use aabb::Aabb;
pub use decomp::{DecompError, DecompParams, decompose};
pub use error::MeshError;
pub use hull::convex_hull;
pub use mass::{MassProps, mass_properties};
pub use obj::load_obj;
pub use ray::{Ray, ray_triangle};
pub use stl::{load_stl, write_binary};
pub use tri_mesh::TriMesh;

/// Loads a mesh by file extension, case-insensitively: `.stl` → [`load_stl`],
/// `.obj` → [`load_obj`], anything else → [`MeshError::UnsupportedFormat`].
pub fn load_mesh(path: &std::path::Path) -> Result<TriMesh, MeshError> {
    let extension = path
        .extension()
        .and_then(|ext| ext.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    match extension.as_str() {
        "stl" => load_stl(path),
        "obj" => load_obj(path),
        _ => Err(MeshError::UnsupportedFormat {
            path: path.display().to_string(),
            extension,
        }),
    }
}

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

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    fn fixture(name: &str) -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/fixtures")
            .join(name)
    }

    #[test]
    fn load_mesh_dispatches_on_extension_case_insensitively() {
        let stl = load_mesh(&fixture("cube_binary.stl")).unwrap();
        let obj = load_mesh(&fixture("cube.obj")).unwrap();
        assert_eq!(stl.aabb(), obj.aabb());

        // Same bytes under an upper-case extension.
        let dir = std::env::temp_dir().join(format!("riggen-mesh-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let upper = dir.join("CUBE.STL");
        std::fs::copy(fixture("cube_binary.stl"), &upper).unwrap();
        assert_eq!(load_mesh(&upper).unwrap(), stl);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn load_mesh_rejects_unknown_extensions() {
        for name in ["part.ply", "part", "part.STEP", "part.stl.bak"] {
            let err = load_mesh(Path::new(name)).unwrap_err();
            assert!(
                matches!(err, MeshError::UnsupportedFormat { .. }),
                "{name}: {err}"
            );
            assert!(err.to_string().contains(name), "{err}");
        }
    }
}
