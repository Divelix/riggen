//! Rigid-body mass properties of a closed triangle mesh: the signed-tetrahedra
//! port of RoboCAD's `mass.rs` onto [`TriMesh`] / glam
//! (docs/02-data-model.md §Inertials).
//!
//! RoboCAD had truck compute a second, independent volume and used the
//! discrepancy as its "is this mesh closed?" signal. Riggen has the welded
//! topology instead: [`MassProps::is_closed`] is
//! [`feature::adjacency`](crate::feature::adjacency)`.is_closed()`, which
//! is exact rather than a tolerance. An open mesh still gets numbers —
//! they are what the tetrahedra sum to and mean nothing — and the caller
//! (`riggen-core::inertial`, M3) refuses them.

use glam::{DMat3, DVec3};

use crate::{TriMesh, feature};

/// Volume, mass, centre of mass and inertia tensor of a solid at a material
/// density, in the mesh's own axes and units (document meters, kg/m³, kg,
/// kg·m²).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassProps {
    /// Always ≥ 0: an inward-wound mesh sums to a negative signed volume,
    /// which is folded here and reported by `inward_winding`.
    pub volume: f64,
    /// `density * volume`.
    pub mass: f64,
    /// Centre of mass, mesh axes. Zero for a zero-volume mesh.
    pub com: DVec3,
    /// Inertia tensor about `com`, mesh axes. Symmetric; zero for a
    /// zero-volume mesh.
    pub inertia: DMat3,
    /// Every edge shared by exactly two triangles. `false` means the other
    /// fields are garbage: the tetrahedra of an open shell do not add up to
    /// any solid.
    pub is_closed: bool,
    /// The triangles wind clockwise seen from outside, so the signed volume
    /// came out negative and was folded. Not an error — a mirrored STL is a
    /// common sight — but the normals point the wrong way.
    pub inward_winding: bool,
}

/// Mass properties of `mesh` at `density` (kg/m³), via the standard "sum of
/// signed tetrahedra with the origin as apex" decomposition.
///
/// Each triangle `(a, b, c)` forms a tetrahedron with the origin of signed
/// volume `a · (b × c) / 6`; summed over every triangle these reconstruct
/// the solid's volume regardless of where the origin sits, and the same
/// decomposition extends to any polynomial moment. The per-tetrahedron
/// closed forms come from Dirichlet's integral over the unit simplex `{u,
/// v, w ≥ 0, u + v + w ≤ 1}`: `∫1 = 1/6`, `∫u = 1/24`, `∫u² = 1/60`, `∫uv
/// = 1/120`. Writing a point of the tetrahedron as `p = u·a + v·b + w·c`
/// (the origin contributes nothing, being the fourth vertex) and `J = 6 ·
/// signed volume`:
///
/// - `∫x_i dV = (J/24)(a_i + b_i + c_i)` — the first moment, for the CoM;
/// - `∫x_i x_j dV = (J/60)(a_i a_j + b_i b_j + c_i c_j) + (J/120)(a_i b_j +
///   a_j b_i + b_i c_j + b_j c_i + c_i a_j + c_j a_i)` — `pair` below,
///   valid for `i == j` too (the diagonal collapses the cross terms into
///   one, matching the `1/60` second-moment coefficient directly).
///
/// The tensor is accumulated about the origin and shifted to the CoM by the
/// parallel-axis theorem. `mesh` must pass [`TriMesh::validate`].
pub fn mass_properties(mesh: &TriMesh, density: f64) -> MassProps {
    let mut signed_volume = 0.0;
    // First moments ∫x, ∫y, ∫z about the origin.
    let mut first = DVec3::ZERO;
    // Second moments about the origin: ∫x², ∫y², ∫z², ∫xy, ∫yz, ∫zx.
    let (mut mxx, mut myy, mut mzz) = (0.0, 0.0, 0.0);
    let (mut mxy, mut myz, mut mzx) = (0.0, 0.0, 0.0);

    for i in 0..mesh.triangle_count() {
        let [a, b, c] = mesh.triangle(i);
        let j = a.dot(b.cross(c));
        signed_volume += j / 6.0;
        first += (a + b + c) * (j / 24.0);

        let pair = |ai: f64, aj: f64, bi: f64, bj: f64, ci: f64, cj: f64| -> f64 {
            j * ((ai * aj + bi * bj + ci * cj) / 60.0
                + (ai * bj + aj * bi + bi * cj + bj * ci + ci * aj + cj * ai) / 120.0)
        };
        mxx += pair(a.x, a.x, b.x, b.x, c.x, c.x);
        myy += pair(a.y, a.y, b.y, b.y, c.y, c.y);
        mzz += pair(a.z, a.z, b.z, b.z, c.z, c.z);
        mxy += pair(a.x, a.y, b.x, b.y, c.x, c.y);
        myz += pair(a.y, a.z, b.y, b.z, c.y, c.z);
        mzx += pair(a.z, a.x, b.z, b.x, c.z, c.x);
    }

    // An empty mesh has no open edges but is no solid either.
    let is_closed = mesh.triangle_count() > 0 && feature::adjacency(mesh).is_closed();
    let inward_winding = signed_volume < 0.0;
    // Folding the winding negates every moment: the integrand is the same,
    // only the orientation of the tetrahedra flipped.
    let sign = if inward_winding { -1.0 } else { 1.0 };
    let volume = sign * signed_volume;
    let mass = density * volume;

    if volume <= f64::EPSILON {
        return MassProps {
            volume,
            mass,
            com: DVec3::ZERO,
            inertia: DMat3::ZERO,
            is_closed,
            inward_winding,
        };
    }
    let com = sign * first / volume;

    // Inertia about the origin: I_ii = ρ∫(sum of the other two squared
    // coordinates), I_ij = -ρ∫(x_i x_j) for i ≠ j.
    let rho = sign * density;
    let ixx_o = rho * (myy + mzz);
    let iyy_o = rho * (mxx + mzz);
    let izz_o = rho * (mxx + myy);
    let ixy_o = -rho * mxy;
    let iyz_o = -rho * myz;
    let izx_o = -rho * mzx;

    // Parallel-axis theorem, origin → CoM: I_com = I_o - m[(r·r)·Id - r⊗r],
    // r = com.
    let r = com;
    let ixx = ixx_o - mass * (r.y * r.y + r.z * r.z);
    let iyy = iyy_o - mass * (r.x * r.x + r.z * r.z);
    let izz = izz_o - mass * (r.x * r.x + r.y * r.y);
    let ixy = ixy_o + mass * r.x * r.y;
    let iyz = iyz_o + mass * r.y * r.z;
    let izx = izx_o + mass * r.z * r.x;

    MassProps {
        volume,
        mass,
        com,
        inertia: DMat3::from_cols(
            DVec3::new(ixx, ixy, izx),
            DVec3::new(ixy, iyy, iyz),
            DVec3::new(izx, iyz, izz),
        ),
        is_closed,
        inward_winding,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DMat4;

    const DENSITY: f64 = 1000.0;

    fn assert_mat_close(actual: DMat3, expected: DMat3, tol: f64) {
        for c in 0..3 {
            for r in 0..3 {
                let (a, e) = (actual.col(c)[r], expected.col(c)[r]);
                assert!(
                    (a - e).abs() <= tol,
                    "[{r}][{c}]: {a} vs {e}\nactual {actual:?}\nexpected {expected:?}"
                );
            }
        }
    }

    #[test]
    fn cube_matches_the_analytic_tensor() {
        let side = 0.2;
        let props = mass_properties(&TriMesh::cube(side * 0.5), DENSITY);
        assert!(props.is_closed);
        assert!(!props.inward_winding);
        assert!(
            (props.volume - side.powi(3)).abs() < 1e-15,
            "{}",
            props.volume
        );
        assert!((props.mass - DENSITY * side.powi(3)).abs() < 1e-12);
        assert!(props.com.length() < 1e-15, "{}", props.com);
        // I_ii = (1/6) m s² about the centroid; products vanish by symmetry.
        let diag = props.mass * side * side / 6.0;
        assert_mat_close(
            props.inertia,
            DMat3::from_diagonal(DVec3::splat(diag)),
            1e-12,
        );
        // Symmetric exactly: the writer fills both triangles from one value.
        assert_eq!(props.inertia, props.inertia.transpose());
    }

    #[test]
    fn cylinder_matches_the_analytic_prism_tensor() {
        // `TriMesh::cylinder` is a regular n-gon prism, which has a closed
        // form: with θ = 2π/n and circumradius R, the polygon's area is
        // (n/2) R² sin θ and its polar second moment of area is
        // (n/12) R⁴ sin θ (2 + cos θ) (sum of n isosceles triangles about
        // their apex). Ixx = Iyy for any regular polygon, so each is half the
        // polar moment plus the m h²/12 of a prism.
        let (radius, height, n) = (0.05f64, 0.3f64, 8usize);
        let theta = std::f64::consts::TAU / n as f64;
        let area = 0.5 * n as f64 * radius.powi(2) * theta.sin();
        let polar = n as f64 / 12.0 * radius.powi(4) * theta.sin() * (2.0 + theta.cos());
        let volume = area * height;
        let mass = DENSITY * volume;
        let izz = DENSITY * height * polar;
        let ixx = izz / 2.0 + mass * height * height / 12.0;

        let props = mass_properties(&TriMesh::cylinder(radius, height, n), DENSITY);
        assert!(props.is_closed);
        assert!((props.volume - volume).abs() < 1e-15, "{}", props.volume);
        assert!(props.com.length() < 1e-15, "{}", props.com);
        assert_mat_close(
            props.inertia,
            DMat3::from_diagonal(DVec3::new(ixx, ixx, izz)),
            1e-12,
        );

        // And with many segments the prism is the cylinder: Izz = m R²/2,
        // Ixx = m (3R² + h²)/12, to the polygon's approximation error.
        let props = mass_properties(&TriMesh::cylinder(radius, height, 720), DENSITY);
        let mass = DENSITY * std::f64::consts::PI * radius * radius * height;
        let izz = mass * radius * radius / 2.0;
        let ixx = mass * (3.0 * radius * radius + height * height) / 12.0;
        assert!((props.mass - mass).abs() / mass < 1e-4);
        assert!((props.inertia.z_axis.z - izz).abs() / izz < 1e-4);
        assert!((props.inertia.x_axis.x - ixx).abs() / ixx < 1e-4);
    }

    #[test]
    fn translation_moves_the_com_and_leaves_the_tensor_about_it() {
        let centred = mass_properties(&TriMesh::cube(0.5), DENSITY);
        let offset = DVec3::new(1.5, -2.0, 0.25);
        let mut moved = TriMesh::cube(0.5);
        moved.transform(&DMat4::from_translation(offset));
        let props = mass_properties(&moved, DENSITY);

        assert!((props.volume - centred.volume).abs() < 1e-12);
        assert!((props.com - offset).length() < 1e-12, "{}", props.com);
        // The parallel-axis shift back to the CoM cancels the offset exactly
        // — a wrong sign anywhere in it shows up here as m·|offset|².
        assert_mat_close(props.inertia, centred.inertia, 1e-9);
    }

    #[test]
    fn rotation_rotates_the_tensor() {
        // A box, not a cube, so the tensor is not isotropic.
        let mut mesh = TriMesh::cylinder(0.1, 1.0, 8);
        let props = mass_properties(&mesh, DENSITY);
        let rotation = glam::DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2);
        mesh.transform(&DMat4::from_quat(rotation));
        let rotated = mass_properties(&mesh, DENSITY);
        let r = DMat3::from_quat(rotation);
        assert_mat_close(rotated.inertia, r * props.inertia * r.transpose(), 1e-12);
    }

    #[test]
    fn cube_minus_a_face_reports_open() {
        let mut mesh = TriMesh::cube(0.5);
        // Drop the last two triangles: the -Z face.
        mesh.indices.truncate(mesh.indices.len() - 6);
        let props = mass_properties(&mesh, DENSITY);
        assert!(!props.is_closed);
        assert!(mass_properties(&TriMesh::cube(0.5), DENSITY).is_closed);
    }

    #[test]
    fn inward_winding_is_folded_and_flagged() {
        let mut mesh = TriMesh::cube(0.5);
        for tri in mesh.indices.chunks_exact_mut(3) {
            tri.swap(1, 2);
        }
        let inward = mass_properties(&mesh, DENSITY);
        let outward = mass_properties(&TriMesh::cube(0.5), DENSITY);
        assert!(inward.inward_winding);
        assert!(inward.is_closed, "winding does not change the topology");
        assert!((inward.volume - outward.volume).abs() < 1e-15);
        assert!((inward.mass - outward.mass).abs() < 1e-12);
        assert!((inward.com - outward.com).length() < 1e-15);
        assert_mat_close(inward.inertia, outward.inertia, 1e-12);
    }

    #[test]
    fn zero_volume_meshes_give_zeros_not_nan() {
        for mesh in [
            TriMesh::default(),
            // A single triangle: a tetrahedron with the origin, but no solid.
            TriMesh {
                positions: vec![DVec3::ZERO, DVec3::X, DVec3::Y],
                normals: vec![],
                indices: vec![0, 1, 2],
            },
        ] {
            let props = mass_properties(&mesh, DENSITY);
            assert_eq!(props.volume, 0.0);
            assert_eq!(props.mass, 0.0);
            assert_eq!(props.com, DVec3::ZERO);
            assert_eq!(props.inertia, DMat3::ZERO);
            assert!(!props.is_closed);
            assert!(!props.inward_winding);
        }
    }
}
