//! `.msh`, MuJoCo's own binary mesh format.
//!
//! No spec doc — the layout below is `mjCMesh::LoadSDF`/`LoadMSH`'s own read
//! order (docs/plans/mjcf-mesh-geometry.md step 1, confirmed against a mesh
//! written by `mujoco.export_mesh`/`mj_saveMesh` and a Menagerie `.msh`
//! file): a header of four little-endian `int32` counts — `nvertex`,
//! `nnormal`, `ntexcoord`, `nface` — then `nvertex` × 3 `float32` positions,
//! `nnormal` × 3 `float32` normals, `ntexcoord` × 2 `float32` texture
//! coordinates, and `nface` × 3 `int32` vertex indices. No material or
//! colour.
//!
//! `TriMesh` only wants positions and face indices; normals and texcoords
//! are read (to keep the following sections aligned) and discarded — the
//! same treatment STL's own per-facet normals get, and for the same reason:
//! flat, winding-derived normals are what every other loader here produces.

use std::path::Path;

use glam::DVec3;

use crate::{MeshError, TriMesh};

const HEADER_INTS: usize = 4;
const HEADER_BYTES: usize = HEADER_INTS * 4;

/// Reads a `.msh` file into a [`TriMesh`].
pub fn load_msh(path: &Path) -> Result<TriMesh, MeshError> {
    let bytes = std::fs::read(path).map_err(|err| MeshError::Io {
        path: path.display().to_string(),
        message: err.to_string(),
    })?;
    parse_msh(&bytes, path)
}

/// [`load_msh`] on bytes already in memory; `path` is only for messages.
pub fn parse_msh(bytes: &[u8], path: &Path) -> Result<TriMesh, MeshError> {
    if bytes.len() < HEADER_BYTES {
        return Err(MeshError::Parse {
            path: path.display().to_string(),
            message: format!(
                "{} bytes is shorter than the {HEADER_BYTES}-byte header",
                bytes.len()
            ),
        });
    }

    let i32_at = |offset: usize| -> i32 {
        let b: [u8; 4] = bytes[offset..offset + 4].try_into().expect("four bytes");
        i32::from_le_bytes(b)
    };
    let counts: [i32; HEADER_INTS] = std::array::from_fn(|i| i32_at(i * 4));
    if let Some(&negative) = counts.iter().find(|&&c| c < 0) {
        return Err(MeshError::Parse {
            path: path.display().to_string(),
            message: format!("negative count {negative} in header {counts:?}"),
        });
    }
    let [nvertex, nnormal, ntexcoord, nface] = counts.map(|c| c as usize);

    let expected = HEADER_BYTES + nvertex * 12 + nnormal * 12 + ntexcoord * 8 + nface * 12;
    if bytes.len() != expected {
        return Err(MeshError::Parse {
            path: path.display().to_string(),
            message: format!(
                "header says {nvertex} vertices, {nnormal} normals, {ntexcoord} texcoords, \
                 {nface} faces ({expected} bytes) but the file is {} bytes",
                bytes.len()
            ),
        });
    }

    let f32_at = |offset: usize| -> f64 {
        let b: [u8; 4] = bytes[offset..offset + 4].try_into().expect("four bytes");
        f64::from(f32::from_le_bytes(b))
    };

    let vertex_base = HEADER_BYTES;
    let positions: Vec<DVec3> = (0..nvertex)
        .map(|i| {
            let base = vertex_base + i * 12;
            DVec3::new(f32_at(base), f32_at(base + 4), f32_at(base + 8))
        })
        .collect();

    let face_base = vertex_base + nvertex * 12 + nnormal * 12 + ntexcoord * 8;
    let mut indices = Vec::with_capacity(nface * 3);
    for i in 0..nface {
        let base = face_base + i * 12;
        for c in 0..3 {
            let index = i32_at(base + c * 4);
            if index < 0 {
                return Err(MeshError::Parse {
                    path: path.display().to_string(),
                    message: format!("negative vertex index {index} in face {i}"),
                });
            }
            indices.push(index as u32);
        }
    }

    let mut mesh = TriMesh {
        positions,
        normals: Vec::new(),
        indices,
    };
    // Indices come straight from the file, unlike STL's own by-construction
    // ones — validate before `flat_normals` walks them, which panics on an
    // out-of-range index rather than erroring gracefully.
    mesh.validate()?;
    mesh.flat_normals();
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

    /// Encodes `mesh` as `.msh` bytes: no normals or texcoords, one face per
    /// triangle, vertices in `mesh.positions`' own order — not a product
    /// writer (MJCF export keeps writing `.stl`, ADR-0008), just what the
    /// regenerator below and the round-trip tests need.
    fn to_msh(mesh: &TriMesh) -> Vec<u8> {
        let mut out = Vec::new();
        for count in [
            mesh.positions.len() as i32,
            0,
            0,
            mesh.triangle_count() as i32,
        ] {
            out.extend_from_slice(&count.to_le_bytes());
        }
        for p in &mesh.positions {
            for c in [p.x, p.y, p.z] {
                out.extend_from_slice(&(c as f32).to_le_bytes());
            }
        }
        for &i in &mesh.indices {
            out.extend_from_slice(&(i as i32).to_le_bytes());
        }
        out
    }

    /// Regenerates `assets/fixtures/cube.msh` from `TriMesh::cube(0.5)`,
    /// unwelded exactly like `cube_binary.stl` (`stl.rs:172`'s fixture is
    /// the same cube). Ignored: the fixture is committed and the tests below
    /// read it; run it by hand if the cube ever changes: `cargo test -p
    /// riggen-mesh write_cube_msh_fixture -- --ignored`.
    #[test]
    #[ignore = "writes the committed fixture; run on purpose"]
    fn write_cube_msh_fixture() {
        std::fs::write(fixture("cube.msh"), to_msh(&TriMesh::cube(0.5))).unwrap();
    }

    /// `assets/fixtures/arm/thing.msh`, the `.msh` the MJCF corpus
    /// (`menagerie_style.xml`) references as `grip`: the unit tetrahedron,
    /// wound counter-clockwise seen from outside. Four vertices is the
    /// floor — MuJoCo's own `LoadMSH` refuses fewer ("invalid sizes in MSH
    /// file"), and a re-export of it as `.stl` fails the same way ("at
    /// least 4 vertices required") — so the single triangle the fixture
    /// began as kept the corpus out of MuJoCo altogether
    /// (plans/actuator-escape-hatch step 2).
    fn thing() -> TriMesh {
        TriMesh {
            positions: vec![DVec3::ZERO, DVec3::X, DVec3::Y, DVec3::Z],
            normals: Vec::new(),
            indices: vec![0, 2, 1, 0, 3, 2, 0, 1, 3, 1, 2, 3],
        }
    }

    /// Regenerates `assets/fixtures/arm/thing.msh` from [`thing`]; ignored
    /// for the same reason as [`write_cube_msh_fixture`]: `cargo test -p
    /// riggen-mesh write_thing_msh_fixture -- --ignored`.
    #[test]
    #[ignore = "writes the committed fixture; run on purpose"]
    fn write_thing_msh_fixture() {
        std::fs::write(fixture("arm/thing.msh"), to_msh(&thing())).unwrap();
    }

    #[test]
    fn thing_fixture_is_a_solid_matching_its_generator() {
        assert_eq!(
            std::fs::read(fixture("arm/thing.msh")).unwrap(),
            to_msh(&thing())
        );
        let mesh = load_msh(&fixture("arm/thing.msh")).unwrap();
        mesh.validate().unwrap();
        assert_eq!(mesh.triangle_count(), 4);
        assert!(
            (0..4).all(|i| {
                let [a, b, c] = mesh.triangle(i);
                // Outward: every face normal points away from the centroid.
                mesh.face_normal(i)
                    .dot((a + b + c) / 3.0 - DVec3::splat(0.25))
                    > 0.0
            }),
            "every face wound counter-clockwise seen from outside"
        );
    }

    #[test]
    fn fixture_is_the_unit_cube() {
        let mesh = load_msh(&fixture("cube.msh")).unwrap();
        mesh.validate().unwrap();
        assert_eq!(mesh, TriMesh::cube(0.5));
    }

    #[test]
    fn fixture_matches_its_generator() {
        // A hand-edited fixture would silently drift from the cube the
        // test above compares against; this pins the bytes to the
        // generator, the same as `stl.rs`'s `fixtures_match_their_generators`.
        assert_eq!(
            std::fs::read(fixture("cube.msh")).unwrap(),
            to_msh(&TriMesh::cube(0.5))
        );
    }

    #[test]
    fn normals_and_texcoords_are_skipped_not_misread() {
        // One triangle, plus a normal and two texcoords full of numbers
        // that are not valid geometry — if the reader mis-sizes either
        // block, the face indices below come out wrong.
        let mut bytes = Vec::new();
        for count in [3i32, 1, 2, 1] {
            bytes.extend_from_slice(&count.to_le_bytes());
        }
        let positions = [
            DVec3::ZERO,
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(0.0, 1.0, 0.0),
        ];
        for p in positions {
            for c in [p.x, p.y, p.z] {
                bytes.extend_from_slice(&(c as f32).to_le_bytes());
            }
        }
        bytes.extend_from_slice(&[0xffu8; 12]); // one bogus normal
        bytes.extend_from_slice(&[0xffu8; 16]); // two bogus texcoords
        for i in [0i32, 1, 2] {
            bytes.extend_from_slice(&i.to_le_bytes());
        }

        let mesh = parse_msh(&bytes, Path::new("triangle.msh")).unwrap();
        mesh.validate().unwrap();
        assert_eq!(mesh.triangle_count(), 1);
        assert_eq!(mesh.triangle(0), positions);
    }

    #[test]
    fn truncated_header_is_an_error() {
        let err = parse_msh(&[0u8; 10], Path::new("short.msh")).unwrap_err();
        assert!(matches!(err, MeshError::Parse { .. }), "{err}");
        assert!(err.to_string().contains("short.msh"), "{err}");
        assert!(err.to_string().contains("header"), "{err}");
    }

    #[test]
    fn size_mismatch_is_an_error() {
        let mut bytes = to_msh(&TriMesh::cube(0.5));
        bytes.truncate(bytes.len() - 4); // one index short of a whole face
        let err = parse_msh(&bytes, Path::new("bad.msh")).unwrap_err();
        assert!(err.to_string().contains("but the file is"), "{err}");
    }

    #[test]
    fn negative_count_is_an_error() {
        let mut bytes = vec![0u8; HEADER_BYTES];
        bytes[0..4].copy_from_slice(&(-1i32).to_le_bytes());
        let err = parse_msh(&bytes, Path::new("negative.msh")).unwrap_err();
        assert!(err.to_string().contains("negative count"), "{err}");
    }

    #[test]
    fn missing_file_is_an_io_error() {
        let err = load_msh(&fixture("does_not_exist.msh")).unwrap_err();
        assert!(matches!(err, MeshError::Io { .. }), "{err}");
    }
}
