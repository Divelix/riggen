use glam::{DMat3, DMat4, DVec3};

use crate::{Aabb, MeshError};

/// An indexed triangle soup: the one mesh type every crate speaks
/// (docs/01-architecture.md §Crates).
///
/// Right-handed, Z-up, in whatever unit the file was in until M1's
/// `MeshAsset` scales it (AGENTS.md). Counter-clockwise winding seen from
/// outside is the convention every producer here follows; nothing enforces
/// it, and `flat_normals` derives normals from it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TriMesh {
    pub positions: Vec<DVec3>,
    /// Per-vertex, unit length; empty means "not computed yet".
    pub normals: Vec<DVec3>,
    /// Three per triangle, into `positions`.
    pub indices: Vec<u32>,
}

impl TriMesh {
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// The three corners of triangle `i`, in winding order.
    ///
    /// Panics if `i >= triangle_count()` or the mesh fails `validate()`.
    pub fn triangle(&self, i: usize) -> [DVec3; 3] {
        let base = 3 * i;
        [
            self.positions[self.indices[base] as usize],
            self.positions[self.indices[base + 1] as usize],
            self.positions[self.indices[base + 2] as usize],
        ]
    }

    /// Unit normal of triangle `i` from its winding; zero for a degenerate
    /// (zero-area) triangle rather than NaN.
    pub fn face_normal(&self, i: usize) -> DVec3 {
        let [a, b, c] = self.triangle(i);
        (b - a).cross(c - a).normalize_or_zero()
    }

    /// Gives every triangle its own flat-shaded normal, from its winding.
    ///
    /// The mesh is **unwelded** in the process — every triangle ends up with
    /// three private vertices — because a vertex shared between two faces
    /// cannot carry both their normals. STL comes in unwelded already, so
    /// this is a no-op on its topology; an OBJ without normals gets tripled.
    /// File normals are ignored on purpose: STL exporters write unreliable
    /// ones (docs/plans, M0 step 4).
    pub fn flat_normals(&mut self) {
        let count = self.triangle_count();
        let mut positions = Vec::with_capacity(count * 3);
        let mut normals = Vec::with_capacity(count * 3);
        for i in 0..count {
            let tri = self.triangle(i);
            let n = (tri[1] - tri[0]).cross(tri[2] - tri[0]).normalize_or_zero();
            positions.extend_from_slice(&tri);
            normals.extend_from_slice(&[n; 3]);
        }
        self.positions = positions;
        self.normals = normals;
        self.indices = (0..(count * 3) as u32).collect();
    }

    /// Checks the invariants everything else assumes: three indices per
    /// triangle, every index in range, normals absent or one per vertex.
    pub fn validate(&self) -> Result<(), MeshError> {
        if !self.indices.len().is_multiple_of(3) {
            return Err(MeshError::IndexCount {
                len: self.indices.len(),
            });
        }
        let vertex_count = self.positions.len();
        if let Some(&index) = self
            .indices
            .iter()
            .find(|&&index| index as usize >= vertex_count)
        {
            return Err(MeshError::IndexOutOfRange {
                index,
                vertex_count,
            });
        }
        if !self.normals.is_empty() && self.normals.len() != vertex_count {
            return Err(MeshError::NormalCount {
                normal_count: self.normals.len(),
                vertex_count,
            });
        }
        Ok(())
    }

    /// Applies a rigid-plus-uniform-scale transform in place: positions
    /// through `m`, normals through its rotation (re-normalised). What
    /// `MeshAsset::scale` / `fix_up` turn file units into document meters
    /// with, once at load. A non-uniform or mirroring `m` is not rejected
    /// but leaves normals only approximately right.
    pub fn transform(&mut self, m: &DMat4) {
        for p in &mut self.positions {
            *p = m.transform_point3(*p);
        }
        let rotation = DMat3::from_mat4(*m);
        for n in &mut self.normals {
            *n = (rotation * *n).normalize_or_zero();
        }
    }

    /// Bounds of every vertex; `None` for an empty mesh.
    pub fn aabb(&self) -> Option<Aabb> {
        Aabb::of_points(self.positions.iter().copied())
    }

    /// An axis-aligned cube of half-extent `half` centred on the origin, 12
    /// triangles, unwelded and flat-shaded, outward CCW winding. The fixture
    /// for tests across the workspace and the reference the STL/OBJ cube
    /// fixtures are compared against.
    pub fn cube(half: f64) -> Self {
        let h = half;
        // (normal, four corners CCW seen from outside)
        let faces: [(DVec3, [DVec3; 4]); 6] = [
            (
                DVec3::X,
                [
                    DVec3::new(h, -h, -h),
                    DVec3::new(h, h, -h),
                    DVec3::new(h, h, h),
                    DVec3::new(h, -h, h),
                ],
            ),
            (
                DVec3::NEG_X,
                [
                    DVec3::new(-h, h, -h),
                    DVec3::new(-h, -h, -h),
                    DVec3::new(-h, -h, h),
                    DVec3::new(-h, h, h),
                ],
            ),
            (
                DVec3::Y,
                [
                    DVec3::new(h, h, -h),
                    DVec3::new(-h, h, -h),
                    DVec3::new(-h, h, h),
                    DVec3::new(h, h, h),
                ],
            ),
            (
                DVec3::NEG_Y,
                [
                    DVec3::new(-h, -h, -h),
                    DVec3::new(h, -h, -h),
                    DVec3::new(h, -h, h),
                    DVec3::new(-h, -h, h),
                ],
            ),
            (
                DVec3::Z,
                [
                    DVec3::new(-h, -h, h),
                    DVec3::new(h, -h, h),
                    DVec3::new(h, h, h),
                    DVec3::new(-h, h, h),
                ],
            ),
            (
                DVec3::NEG_Z,
                [
                    DVec3::new(-h, h, -h),
                    DVec3::new(h, h, -h),
                    DVec3::new(h, -h, -h),
                    DVec3::new(-h, -h, -h),
                ],
            ),
        ];
        let mut mesh = Self::default();
        for (normal, quad) in faces {
            for tri in [[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]] {
                let base = mesh.positions.len() as u32;
                mesh.positions.extend_from_slice(&tri);
                mesh.normals.extend_from_slice(&[normal; 3]);
                mesh.indices.extend_from_slice(&[base, base + 1, base + 2]);
            }
        }
        mesh
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transform_scales_positions_and_rotates_normals() {
        let mut mesh = TriMesh::cube(0.5);
        let m = DMat4::from_scale_rotation_translation(
            DVec3::splat(0.001),
            glam::DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
            DVec3::ZERO,
        );
        mesh.transform(&m);
        let aabb = mesh.aabb().unwrap();
        assert!(
            (aabb.min - DVec3::splat(-0.0005)).length() < 1e-12,
            "{aabb:?}"
        );
        assert!(
            (aabb.max - DVec3::splat(0.0005)).length() < 1e-12,
            "{aabb:?}"
        );
        // The +Y face's normal is now +Z, and still unit length.
        assert!(
            mesh.normals.iter().any(|n| (*n - DVec3::Z).length() < 1e-9),
            "{:?}",
            mesh.normals
        );
        assert!(mesh.normals.iter().all(|n| (n.length() - 1.0).abs() < 1e-9));
    }

    #[test]
    fn cube_is_twelve_valid_triangles_with_unit_aabb() {
        let cube = TriMesh::cube(0.5);
        assert_eq!(cube.triangle_count(), 12);
        cube.validate().unwrap();
        let aabb = cube.aabb().unwrap();
        assert_eq!(aabb.min, DVec3::splat(-0.5));
        assert_eq!(aabb.max, DVec3::splat(0.5));
        assert_eq!(aabb.center(), DVec3::ZERO);
        assert!((aabb.half_diagonal() - 0.75f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn cube_winding_faces_outward() {
        let cube = TriMesh::cube(1.0);
        for i in 0..cube.triangle_count() {
            let n = cube.face_normal(i);
            let centroid = cube.triangle(i).iter().sum::<DVec3>() / 3.0;
            // Outward: the winding normal points away from the origin and
            // agrees with the stored normal.
            assert!(n.dot(centroid) > 0.0, "triangle {i} winds inward");
            assert!((n - cube.normals[cube.indices[3 * i] as usize]).length() < 1e-12);
        }
    }

    #[test]
    fn flat_normals_unwelds_and_matches_winding() {
        // A welded square: 4 vertices, 2 triangles sharing an edge.
        let mut mesh = TriMesh {
            positions: vec![DVec3::ZERO, DVec3::X, DVec3::new(1.0, 1.0, 0.0), DVec3::Y],
            normals: vec![],
            indices: vec![0, 1, 2, 0, 2, 3],
        };
        mesh.flat_normals();
        mesh.validate().unwrap();
        assert_eq!(mesh.positions.len(), 6);
        assert_eq!(mesh.indices, (0..6).collect::<Vec<_>>());
        assert!(mesh.normals.iter().all(|&n| n == DVec3::Z));
    }

    #[test]
    fn degenerate_triangle_has_zero_normal() {
        let mesh = TriMesh {
            positions: vec![DVec3::ZERO, DVec3::X, DVec3::X * 2.0],
            normals: vec![],
            indices: vec![0, 1, 2],
        };
        assert_eq!(mesh.face_normal(0), DVec3::ZERO);
    }

    #[test]
    fn validate_rejects_bad_index_counts_and_ranges() {
        let mut mesh = TriMesh::cube(1.0);
        mesh.indices.push(0);
        assert_eq!(mesh.validate(), Err(MeshError::IndexCount { len: 37 }));

        let mut mesh = TriMesh::cube(1.0);
        mesh.indices[5] = 99;
        assert_eq!(
            mesh.validate(),
            Err(MeshError::IndexOutOfRange {
                index: 99,
                vertex_count: 36
            })
        );

        let mut mesh = TriMesh::cube(1.0);
        mesh.normals.pop();
        assert_eq!(
            mesh.validate(),
            Err(MeshError::NormalCount {
                normal_count: 35,
                vertex_count: 36
            })
        );

        let mut mesh = TriMesh::cube(1.0);
        mesh.normals.clear();
        mesh.validate().unwrap();
        assert!(TriMesh::default().validate().is_ok());
        assert_eq!(TriMesh::default().aabb(), None);
    }
}
