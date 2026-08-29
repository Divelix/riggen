//! Wavefront OBJ via `tobj`.
//!
//! Every shape in the file is merged into one [`TriMesh`]: M0 shows a
//! dropped file as one instance, and a link's visual is one mesh
//! (docs/02-data-model.md). Faces are triangulated; `single_index` makes
//! `tobj` unweld wherever a position is used with different normals, so the
//! result is welded exactly as far as the file's normals allow. Normals are
//! taken from the file when every vertex has one and recomputed flat
//! otherwise. Materials (`.mtl`) are ignored — colour comes from the
//! document, not the file.

use std::path::Path;

use glam::DVec3;

use crate::{MeshError, TriMesh};

const LOAD_OPTIONS: tobj::LoadOptions = tobj::LoadOptions {
    single_index: true,
    triangulate: true,
    ignore_points: true,
    ignore_lines: true,
};

/// Reads an OBJ into one [`TriMesh`].
pub fn load_obj(path: &Path) -> Result<TriMesh, MeshError> {
    let bytes = std::fs::read(path).map_err(|err| MeshError::Io {
        path: path.display().to_string(),
        message: err.to_string(),
    })?;
    parse_obj(&bytes, path)
}

/// [`load_obj`] on bytes already in memory; `path` is only for messages.
pub(crate) fn parse_obj(bytes: &[u8], path: &Path) -> Result<TriMesh, MeshError> {
    let mut reader = std::io::Cursor::new(bytes);
    // The material callback is what `mtllib` lines resolve through; answering
    // "no materials" keeps the loader off the filesystem entirely.
    let (models, _materials) = tobj::load_obj_buf(&mut reader, &LOAD_OPTIONS, |_| {
        Ok((Vec::new(), Default::default()))
    })
    .map_err(|err| MeshError::Parse {
        path: path.display().to_string(),
        message: err.to_string(),
    })?;

    let mut mesh = TriMesh::default();
    let mut all_have_normals = true;
    for model in &models {
        let m = &model.mesh;
        let base = mesh.positions.len() as u32;
        mesh.positions.extend(
            m.positions
                .as_chunks::<3>()
                .0
                .iter()
                .map(|&[x, y, z]| DVec3::new(x, y, z)),
        );
        if m.normals.len() == m.positions.len() {
            mesh.normals.extend(
                m.normals
                    .as_chunks::<3>()
                    .0
                    .iter()
                    .map(|&[x, y, z]| DVec3::new(x, y, z).normalize_or_zero()),
            );
        } else {
            all_have_normals = false;
        }
        mesh.indices.extend(m.indices.iter().map(|&i| base + i));
    }

    if mesh.indices.is_empty() {
        return Err(MeshError::Parse {
            path: path.display().to_string(),
            message: "no faces".to_owned(),
        });
    }
    if !all_have_normals {
        mesh.normals.clear();
        mesh.validate()?;
        mesh.flat_normals();
    }
    crate::finish_loaded(mesh, path)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(name: &str) -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/fixtures")
            .join(name)
    }

    fn assert_is_unit_cube(mesh: &TriMesh) {
        mesh.validate().unwrap();
        assert_eq!(mesh.triangle_count(), 12);
        let aabb = mesh.aabb().unwrap();
        assert_eq!(aabb.min, DVec3::splat(-0.5));
        assert_eq!(aabb.max, DVec3::splat(0.5));
        for i in 0..12 {
            let winding = mesh.face_normal(i);
            for &index in &mesh.indices[3 * i..3 * i + 3] {
                let n = mesh.normals[index as usize];
                assert!(
                    (n - winding).length() < 1e-12,
                    "triangle {i}: {n} vs {winding}"
                );
            }
        }
    }

    #[test]
    fn fixture_is_the_unit_cube_with_file_normals() {
        let mesh = load_obj(&fixture("cube.obj")).unwrap();
        assert_is_unit_cube(&mesh);
        // 8 positions × 3 normals each: welded as far as the normals allow.
        assert_eq!(mesh.positions.len(), 24);
        assert_eq!(mesh.aabb(), TriMesh::cube(0.5).aabb());
    }

    #[test]
    fn missing_normals_are_recomputed_flat() {
        let src = std::fs::read_to_string(fixture("cube.obj")).unwrap();
        let stripped: String = src
            .lines()
            .filter(|l| !l.starts_with("vn "))
            .map(|l| {
                // `f a//n b//n c//n d//n` → `f a b c d`
                l.split_whitespace()
                    .map(|w| w.split("//").next().unwrap())
                    .collect::<Vec<_>>()
                    .join(" ")
            })
            .collect::<Vec<_>>()
            .join("\n");
        let mesh = parse_obj(stripped.as_bytes(), Path::new("flat.obj")).unwrap();
        assert_is_unit_cube(&mesh);
        assert_eq!(mesh.positions.len(), 36, "unwelded by flat_normals");
    }

    #[test]
    fn shapes_are_merged_and_mtl_is_ignored() {
        let src = "mtllib nowhere.mtl\no a\nv 0 0 0\nv 1 0 0\nv 0 1 0\nusemtl red\nf 1 2 3\n\
                   o b\nv 0 0 1\nv 1 0 1\nv 0 1 1\nf 4 5 6\n";
        let mesh = parse_obj(src.as_bytes(), Path::new("two.obj")).unwrap();
        assert_eq!(mesh.triangle_count(), 2);
        assert_eq!(mesh.positions.len(), 6);
        assert_eq!(mesh.triangle(1)[0], DVec3::new(0.0, 0.0, 1.0));
        assert!(mesh.normals.iter().all(|&n| n == DVec3::Z));
    }

    #[test]
    fn garbage_and_empty_are_errors() {
        let err = parse_obj(b"v 1 2\nf 1 2 3\n", Path::new("bad.obj")).unwrap_err();
        assert!(matches!(err, MeshError::Parse { .. }), "{err}");
        assert!(err.to_string().contains("bad.obj"));

        let err = parse_obj(b"# nothing but a comment\n", Path::new("empty.obj")).unwrap_err();
        assert!(err.to_string().contains("no faces"), "{err}");

        let err = load_obj(&fixture("does_not_exist.obj")).unwrap_err();
        assert!(matches!(err, MeshError::Io { .. }), "{err}");
    }
}
