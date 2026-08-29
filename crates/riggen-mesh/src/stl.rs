//! STL, binary and ASCII.
//!
//! Sniffing is the whole difficulty. The ASCII form starts with `solid`, but
//! so do plenty of binary files — exporters write "solid <name>" into the
//! 80-byte binary header — so the prefix alone is not a verdict: a file is
//! ASCII only if it *also* parses as ASCII, and is otherwise tried as binary.
//! `stl_io` decides on the prefix alone and keeps its binary reader private,
//! so the binary path (80-byte header, `u32` count, 50 bytes per facet) is
//! parsed here and the size is checked against the count, which `stl_io`
//! would not do either.
//!
//! Vertices come out **unwelded** (three per triangle) with normals recomputed
//! from the winding: STL's per-facet normals are unreliable in practice and
//! the vertices define the geometry anyway.

use std::path::Path;

use glam::DVec3;

use crate::{MeshError, TriMesh};

const BINARY_HEADER: usize = 80;
const BINARY_COUNT: usize = 4;
const BINARY_FACET: usize = 50;

/// Reads a binary or ASCII STL into an unwelded, flat-shaded [`TriMesh`].
pub fn load_stl(path: &Path) -> Result<TriMesh, MeshError> {
    let bytes = std::fs::read(path).map_err(|err| MeshError::Io {
        path: path.display().to_string(),
        message: err.to_string(),
    })?;
    parse_stl(&bytes, path)
}

/// Serialises `mesh` as binary STL: an 80-byte zero header, the facet count,
/// then 50 bytes per triangle (normal from the winding, three `f32`
/// vertices, a zero attribute count).
///
/// The generator behind `assets/fixtures/` — the arm fixtures of M2 are
/// written by an `#[ignore]`d test rather than checked in as opaque bytes.
/// `f64` positions narrow to `f32`, which is all the format has.
pub fn write_binary(mesh: &TriMesh) -> Vec<u8> {
    let mut out = vec![0u8; BINARY_HEADER];
    out.extend_from_slice(&(mesh.triangle_count() as u32).to_le_bytes());
    for i in 0..mesh.triangle_count() {
        let normal = mesh.face_normal(i);
        for v in [normal].into_iter().chain(mesh.triangle(i)) {
            for c in [v.x, v.y, v.z] {
                out.extend_from_slice(&(c as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&[0, 0]);
    }
    out
}

/// [`load_stl`] on bytes already in memory; `path` is only for messages.
pub(crate) fn parse_stl(bytes: &[u8], path: &Path) -> Result<TriMesh, MeshError> {
    let ascii_error = if bytes.starts_with(b"solid") {
        match parse_ascii(bytes) {
            Ok(triangles) => return finish(triangles, path),
            Err(err) => Some(err),
        }
    } else {
        None
    };

    match parse_binary(bytes) {
        Ok(triangles) => finish(triangles, path),
        Err(binary_error) => Err(MeshError::Parse {
            path: path.display().to_string(),
            message: match ascii_error {
                Some(ascii_error) => {
                    format!("not ASCII STL ({ascii_error}) and not binary STL ({binary_error})")
                }
                None => format!("not binary STL ({binary_error})"),
            },
        }),
    }
}

fn finish(triangles: Vec<[DVec3; 3]>, path: &Path) -> Result<TriMesh, MeshError> {
    let mut mesh = TriMesh {
        positions: triangles.iter().flatten().copied().collect(),
        normals: Vec::new(),
        indices: (0..(triangles.len() * 3) as u32).collect(),
    };
    mesh.flat_normals();
    crate::finish_loaded(mesh, path)
}

fn parse_ascii(bytes: &[u8]) -> Result<Vec<[DVec3; 3]>, String> {
    let mut cursor = std::io::Cursor::new(bytes);
    let reader = stl_io::create_stl_reader(&mut cursor).map_err(|err| err.to_string())?;
    let mut triangles = Vec::new();
    for triangle in reader {
        let triangle = triangle.map_err(|err| err.to_string())?;
        triangles.push(triangle.vertices.map(|v| {
            let [x, y, z] = v.0;
            DVec3::new(f64::from(x), f64::from(y), f64::from(z))
        }));
    }
    if triangles.is_empty() {
        return Err("no facets".to_owned());
    }
    Ok(triangles)
}

fn parse_binary(bytes: &[u8]) -> Result<Vec<[DVec3; 3]>, String> {
    if bytes.len() < BINARY_HEADER + BINARY_COUNT {
        return Err(format!(
            "{} bytes is shorter than the {}-byte header",
            bytes.len(),
            BINARY_HEADER + BINARY_COUNT
        ));
    }
    let count_bytes: [u8; 4] = bytes[BINARY_HEADER..BINARY_HEADER + BINARY_COUNT]
        .try_into()
        .expect("four bytes");
    let count = u32::from_le_bytes(count_bytes) as usize;
    let expected = BINARY_HEADER + BINARY_COUNT + count * BINARY_FACET;
    if bytes.len() != expected {
        return Err(format!(
            "header says {count} facets ({expected} bytes) but the file is {} bytes",
            bytes.len()
        ));
    }

    let f32_at = |offset: usize| {
        let b: [u8; 4] = bytes[offset..offset + 4].try_into().expect("four bytes");
        f64::from(f32::from_le_bytes(b))
    };
    let vec_at = |offset: usize| DVec3::new(f32_at(offset), f32_at(offset + 4), f32_at(offset + 8));

    let triangles = (0..count)
        .map(|i| {
            // 12 bytes of normal (ignored), then three vertices, then the
            // two-byte attribute count.
            let base = BINARY_HEADER + BINARY_COUNT + i * BINARY_FACET + 12;
            [vec_at(base), vec_at(base + 12), vec_at(base + 24)]
        })
        .collect();
    Ok(triangles)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(name: &str) -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/fixtures")
            .join(name)
    }

    use super::write_binary as to_binary;

    fn to_ascii(mesh: &TriMesh) -> String {
        let mut s = String::from("solid cube\n");
        for i in 0..mesh.triangle_count() {
            let n = mesh.face_normal(i);
            s += &format!("  facet normal {} {} {}\n    outer loop\n", n.x, n.y, n.z);
            for v in mesh.triangle(i) {
                s += &format!("      vertex {} {} {}\n", v.x, v.y, v.z);
            }
            s += "    endloop\n  endfacet\n";
        }
        s += "endsolid cube\n";
        s
    }

    /// Regenerates `assets/fixtures/cube_{binary,ascii}.stl` from
    /// `TriMesh::cube(0.5)`. Ignored: the fixtures are committed and the
    /// tests below read them; run it by hand if the cube ever changes:
    /// `cargo test -p riggen-mesh write_cube_stl_fixtures -- --ignored`.
    #[test]
    #[ignore = "writes the committed fixtures; run on purpose"]
    fn write_cube_stl_fixtures() {
        let cube = TriMesh::cube(0.5);
        std::fs::write(fixture("cube_binary.stl"), to_binary(&cube)).unwrap();
        std::fs::write(fixture("cube_ascii.stl"), to_ascii(&cube)).unwrap();
    }

    fn assert_is_unit_cube(mesh: &TriMesh) {
        mesh.validate().unwrap();
        assert_eq!(mesh.triangle_count(), 12);
        assert_eq!(mesh.positions.len(), 36, "unwelded");
        let aabb = mesh.aabb().unwrap();
        assert_eq!(aabb.min, DVec3::splat(-0.5));
        assert_eq!(aabb.max, DVec3::splat(0.5));
        for i in 0..12 {
            let n = mesh.normals[mesh.indices[3 * i] as usize];
            assert!((n.length() - 1.0).abs() < 1e-12);
            assert!(n.dot(mesh.triangle(i)[0]) > 0.0, "normal {i} points inward");
        }
    }

    #[test]
    fn binary_fixture_is_the_unit_cube() {
        let mesh = load_stl(&fixture("cube_binary.stl")).unwrap();
        assert_is_unit_cube(&mesh);
        assert_eq!(mesh, TriMesh::cube(0.5));
    }

    #[test]
    fn ascii_fixture_is_the_unit_cube() {
        let mesh = load_stl(&fixture("cube_ascii.stl")).unwrap();
        assert_is_unit_cube(&mesh);
        assert_eq!(mesh, TriMesh::cube(0.5));
    }

    #[test]
    fn fixtures_match_their_generators() {
        // A hand-edited fixture would silently drift from the cube the
        // other tests compare against; this pins the bytes to the generator.
        let cube = TriMesh::cube(0.5);
        assert_eq!(
            std::fs::read(fixture("cube_binary.stl")).unwrap(),
            to_binary(&cube)
        );
        assert_eq!(
            std::fs::read_to_string(fixture("cube_ascii.stl")).unwrap(),
            to_ascii(&cube)
        );
    }

    #[test]
    fn binary_starting_with_solid_is_still_binary() {
        let mut bytes = to_binary(&TriMesh::cube(0.5));
        bytes[..11].copy_from_slice(b"solid cube ");
        let mesh = parse_stl(&bytes, Path::new("solid.stl")).unwrap();
        assert_is_unit_cube(&mesh);
    }

    #[test]
    fn normals_come_from_winding_not_the_file() {
        let mut bytes = to_binary(&TriMesh::cube(0.5));
        // Zero every stored normal.
        for i in 0..12 {
            let base = BINARY_HEADER + BINARY_COUNT + i * BINARY_FACET;
            bytes[base..base + 12].fill(0);
        }
        let mesh = parse_stl(&bytes, Path::new("nonormals.stl")).unwrap();
        assert_is_unit_cube(&mesh);
    }

    #[test]
    fn garbage_is_an_error() {
        let err = parse_stl(b"hello world", Path::new("junk.stl")).unwrap_err();
        assert!(matches!(err, MeshError::Parse { .. }), "{err}");
        assert!(err.to_string().contains("junk.stl"));

        let err =
            parse_stl(b"solid but not really\nfacet nope\n", Path::new("junk.stl")).unwrap_err();
        assert!(matches!(err, MeshError::Parse { .. }), "{err}");
        assert!(err.to_string().contains("not ASCII STL"), "{err}");

        let mut truncated = to_binary(&TriMesh::cube(0.5));
        truncated.truncate(300);
        let err = parse_stl(&truncated, Path::new("short.stl")).unwrap_err();
        assert!(err.to_string().contains("header says 12 facets"), "{err}");

        let err = parse_stl(b"solid empty\nendsolid empty\n", Path::new("empty.stl")).unwrap_err();
        assert!(err.to_string().contains("no facets"), "{err}");
    }

    #[test]
    fn missing_file_is_an_io_error() {
        let err = load_stl(&fixture("does_not_exist.stl")).unwrap_err();
        assert!(matches!(err, MeshError::Io { .. }), "{err}");
    }

    #[test]
    fn too_many_triangles_is_rejected() {
        let n = crate::MAX_TRIANGLES + 1;
        let mut bytes = vec![0u8; BINARY_HEADER];
        bytes.extend_from_slice(&(n as u32).to_le_bytes());
        bytes.resize(BINARY_HEADER + BINARY_COUNT + n * BINARY_FACET, 0);
        let err = parse_stl(&bytes, Path::new("huge.stl")).unwrap_err();
        assert_eq!(
            err,
            MeshError::TooManyTriangles {
                path: "huge.stl".into(),
                count: n
            }
        );
    }
}
