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

    /// A closed cylinder along +Z, centred on the origin: `segments` quads
    /// of wall between `z = ±height/2`, plus a triangle fan cap at each end.
    /// Outward CCW winding, unwelded and flat-shaded like [`Self::cube`].
    ///
    /// Ring vertices are computed once and copied, so the two triangles of a
    /// quad share **bit-identical** positions — which is what
    /// [`crate::feature::adjacency`] welds on.
    pub fn cylinder(radius: f64, height: f64, segments: usize) -> Self {
        let (bottom, top) = (
            ring(radius, -height * 0.5, segments),
            ring(radius, height * 0.5, segments),
        );
        let mut mesh = Self::default();
        mesh.push_wall(&bottom, &top, false);
        mesh.push_fan(&top, height * 0.5, false);
        mesh.push_fan(&bottom, -height * 0.5, true);
        mesh.flat_normals();
        mesh
    }

    /// A hollow tube along +Z, centred on the origin: an outer wall with
    /// outward normals, an inner wall with **inward** normals, and an
    /// annular cap at each end. `inner` must be smaller than `outer`.
    pub fn tube(outer: f64, inner: f64, height: f64, segments: usize) -> Self {
        let (h, n) = (height * 0.5, segments);
        let (ob, ot) = (ring(outer, -h, n), ring(outer, h, n));
        let (ib, it) = (ring(inner, -h, n), ring(inner, h, n));
        let mut mesh = Self::default();
        mesh.push_wall(&ob, &ot, false);
        // Reversed winding: the inner wall faces the axis.
        mesh.push_wall(&ib, &it, true);
        mesh.push_annulus(&ot, &it, false);
        mesh.push_annulus(&ob, &ib, true);
        mesh.flat_normals();
        mesh
    }

    /// Quads between two rings of equal length; `flip` reverses the winding
    /// (an inner wall).
    fn push_wall(&mut self, bottom: &[DVec3], top: &[DVec3], flip: bool) {
        for i in 0..bottom.len() {
            let j = (i + 1) % bottom.len();
            self.push_quad([bottom[i], bottom[j], top[j], top[i]], flip);
        }
    }

    /// A triangle fan from the ring's centre at height `z`; `flip` turns it
    /// into a bottom cap.
    fn push_fan(&mut self, ring: &[DVec3], z: f64, flip: bool) {
        let centre = DVec3::new(0.0, 0.0, z);
        for i in 0..ring.len() {
            let j = (i + 1) % ring.len();
            self.push_tri([centre, ring[i], ring[j]], flip);
        }
    }

    /// The flat ring between two co-planar rings (a tube's end face).
    fn push_annulus(&mut self, outer: &[DVec3], inner: &[DVec3], flip: bool) {
        for i in 0..outer.len() {
            let j = (i + 1) % outer.len();
            self.push_quad([outer[i], outer[j], inner[j], inner[i]], flip);
        }
    }

    fn push_quad(&mut self, quad: [DVec3; 4], flip: bool) {
        self.push_tri([quad[0], quad[1], quad[2]], flip);
        self.push_tri([quad[0], quad[2], quad[3]], flip);
    }

    fn push_tri(&mut self, tri: [DVec3; 3], flip: bool) {
        let base = self.positions.len() as u32;
        if flip {
            self.positions.extend_from_slice(&[tri[0], tri[2], tri[1]]);
        } else {
            self.positions.extend_from_slice(&tri);
        }
        self.indices.extend_from_slice(&[base, base + 1, base + 2]);
    }
}

/// `segments` points on a circle of `radius` at height `z`, counter-clockwise
/// from +X seen from +Z.
fn ring(radius: f64, z: f64, segments: usize) -> Vec<DVec3> {
    (0..segments)
        .map(|i| {
            let a = std::f64::consts::TAU * i as f64 / segments as f64;
            DVec3::new(radius * a.cos(), radius * a.sin(), z)
        })
        .collect()
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
    fn cylinder_is_closed_and_winds_outward() {
        let n = 24;
        let mesh = TriMesh::cylinder(0.3, 2.0, n);
        mesh.validate().unwrap();
        // n quads of wall (2 triangles each) plus n per cap.
        assert_eq!(mesh.triangle_count(), 4 * n);
        let aabb = mesh.aabb().unwrap();
        assert!(
            (aabb.min - DVec3::new(-0.3, -0.3, -1.0)).length() < 1e-9,
            "{aabb:?}"
        );
        assert!((aabb.max.z - 1.0).abs() < 1e-12);

        for i in 0..mesh.triangle_count() {
            let n = mesh.face_normal(i);
            let c = mesh.triangle(i).iter().sum::<DVec3>() / 3.0;
            // Outward everywhere: the wall points away from the axis, the
            // caps away from the mid-plane.
            assert!(n.dot(c) > 0.0, "triangle {i} winds inward: n {n} at {c}");
            assert!((n.length() - 1.0).abs() < 1e-12);
        }
    }

    #[test]
    fn tube_inner_wall_faces_the_axis() {
        let n = 16;
        let mesh = TriMesh::tube(0.5, 0.25, 1.0, n);
        mesh.validate().unwrap();
        // Two walls and two annular caps, 2 triangles per segment each.
        assert_eq!(mesh.triangle_count(), 8 * n);

        let mut inner = 0;
        for i in 0..mesh.triangle_count() {
            let normal = mesh.face_normal(i);
            let c = mesh.triangle(i).iter().sum::<DVec3>() / 3.0;
            let radial = DVec3::new(c.x, c.y, 0.0);
            if radial.length() < 0.3 && normal.z.abs() < 0.5 {
                // Inner wall: the normal points back at the axis.
                assert!(normal.dot(radial) < 0.0, "triangle {i} faces outward");
                inner += 1;
            }
        }
        assert_eq!(inner, 2 * n, "every inner-wall triangle checked");
    }

    #[test]
    fn generators_repeat_ring_positions_exactly() {
        // `feature::adjacency` welds on exact bits: the two triangles of a
        // wall quad must not differ in the last ulp.
        let mesh = TriMesh::cylinder(1.0, 1.0, 8);
        let distinct: std::collections::HashSet<[u64; 3]> = mesh
            .positions
            .iter()
            .map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
            .collect();
        // 8 per ring, plus the two cap centres.
        assert_eq!(distinct.len(), 18);
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
