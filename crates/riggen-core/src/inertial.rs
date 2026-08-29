//! A link's inertial from its geoms (docs/02-data-model.md §Inertials).
//!
//! Core stores no geometry, so the meshes come through [`MeshLookup`] —
//! the app's mesh store and the export CLI both implement it. Per geom,
//! [`riggen_mesh::mass_properties`] at the link's density; the tensor is
//! rotated into the link frame and parallel-axis shifted by the geom pose;
//! the geoms are summed; then the [`InertialSpec`] mode decides what the
//! consumers get. [`check`] is the export gate: the sanity checks MuJoCo
//! fails on silently (mass > 0, symmetric, positive-definite, triangle
//! inequality on the principal moments).

use std::collections::BTreeMap;
use std::fmt;

use riggen_mesh::glam::{DMat3, DVec3};
use riggen_mesh::{MassProps, TriMesh, mass_properties};

use crate::ids::{GeomId, MeshId};
use crate::robot::{Geom, InertialSpec, Link, Material};

/// Where the geometry lives; core never holds it.
pub trait MeshLookup {
    /// The loaded mesh for `id`, already scaled to meters and `fix_up`ped.
    fn mesh(&self, id: MeshId) -> Option<&TriMesh>;
}

impl MeshLookup for BTreeMap<MeshId, TriMesh> {
    fn mesh(&self, id: MeshId) -> Option<&TriMesh> {
        self.get(&id)
    }
}

impl MeshLookup for BTreeMap<MeshId, std::sync::Arc<TriMesh>> {
    fn mesh(&self, id: MeshId) -> Option<&TriMesh> {
        self.get(&id).map(|m| &**m)
    }
}

impl<T: MeshLookup> MeshLookup for &T {
    fn mesh(&self, id: MeshId) -> Option<&TriMesh> {
        (**self).mesh(id)
    }
}

/// Mass, centre of mass and inertia tensor about the CoM, all in the link
/// frame. What every consumer reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inertial {
    pub mass: f64,
    pub com: DVec3,
    pub inertia: DMat3,
}

impl Inertial {
    pub const ZERO: Self = Self {
        mass: 0.0,
        com: DVec3::ZERO,
        inertia: DMat3::ZERO,
    };
}

/// [`compose_inertial`]'s result: the value in effect, and what the meshes
/// say beside it for the properties panel's comparison readout (`None`
/// under `Override` when the meshes cannot be measured — no density, an
/// open mesh — since the override does not need them).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkInertial {
    pub inertial: Inertial,
    pub computed: Option<Inertial>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum InertialError {
    /// `Computed` / `Hybrid` with neither a material nor a density override.
    NoDensity,
    /// The link's material is not in `Robot::materials` (`validate` catches
    /// this first; here for a standalone call).
    UnknownMaterial(String),
    /// The lookup has no mesh for a geom.
    MissingMesh { geom: GeomId, mesh: MeshId },
    /// `Computed` / `Hybrid` met a mesh whose tetrahedra add up to no solid.
    OpenMesh { geom: GeomId },
    /// `Hybrid` has nothing to scale: the geoms enclose no volume.
    NoVolume,
    /// From [`check`]: mass ≤ 0 or not finite.
    NonPositiveMass(f64),
    /// From [`check`]: a value in `com` or `inertia` is not finite.
    NonFinite,
    /// From [`check`]: `inertia[i][j] != inertia[j][i]`.
    NotSymmetric,
    /// From [`check`]: a principal moment ≤ 0.
    NotPositiveDefinite { moments: [f64; 3] },
    /// From [`check`]: `I1 + I2 < I3` for some permutation — no rigid body
    /// has such a tensor, and MuJoCo refuses it.
    TriangleInequality { moments: [f64; 3] },
}

impl fmt::Display for InertialError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoDensity => write!(f, "no material and no density override"),
            Self::UnknownMaterial(m) => write!(f, "unknown material \"{m}\""),
            Self::MissingMesh { geom, mesh } => {
                write!(f, "geom {geom}: mesh {mesh} is not loaded")
            }
            Self::OpenMesh { geom } => write!(
                f,
                "geom {geom}: the mesh is not closed, so its mass properties are meaningless"
            ),
            Self::NoVolume => write!(f, "the geoms enclose no volume to scale to the mass"),
            Self::NonPositiveMass(m) => write!(f, "mass {m} is not positive"),
            Self::NonFinite => write!(f, "the inertial has a non-finite value"),
            Self::NotSymmetric => write!(f, "the inertia tensor is not symmetric"),
            Self::NotPositiveDefinite { moments } => write!(
                f,
                "the inertia tensor is not positive-definite (principal moments {:.3e}, {:.3e}, {:.3e})",
                moments[0], moments[1], moments[2]
            ),
            Self::TriangleInequality { moments } => write!(
                f,
                "the principal moments {:.3e}, {:.3e}, {:.3e} violate the triangle inequality (I1 + I2 ≥ I3)",
                moments[0], moments[1], moments[2]
            ),
        }
    }
}

impl std::error::Error for InertialError {}

/// The density `Computed` / `Hybrid` use: the override, else the material.
pub fn density(link: &Link, materials: &BTreeMap<String, Material>) -> Result<f64, InertialError> {
    if let InertialSpec::Computed {
        density_override: Some(d),
    } = link.inertial
    {
        return Ok(d);
    }
    match &link.material {
        Some(name) => materials
            .get(name)
            .map(|m| m.density)
            .ok_or_else(|| InertialError::UnknownMaterial(name.clone())),
        None => Err(InertialError::NoDensity),
    }
}

/// One geom's mass properties in the **link** frame: the tensor rotated by
/// the geom pose and still about the geom's CoM, which is moved by the
/// pose. `Err` when the mesh is missing or open.
fn geom_inertial(
    geom: &Geom,
    meshes: &impl MeshLookup,
    density: f64,
) -> Result<Inertial, InertialError> {
    let mesh = meshes.mesh(geom.mesh).ok_or(InertialError::MissingMesh {
        geom: geom.id,
        mesh: geom.mesh,
    })?;
    let props: MassProps = mass_properties(mesh, density);
    if !props.is_closed {
        return Err(InertialError::OpenMesh { geom: geom.id });
    }
    let r = DMat3::from_quat(geom.pose.r.normalize());
    Ok(Inertial {
        mass: props.mass,
        com: geom.pose.transform_point(props.com),
        inertia: r * props.inertia * r.transpose(),
    })
}

/// The rigid-body sum of `parts`, each about its own CoM: the combined CoM
/// is the mass-weighted mean, and every tensor is shifted to it by the
/// parallel-axis theorem (`I += m[(d·d)·Id − d⊗d]`, `d = c_i − c`).
pub fn sum_inertials(parts: &[Inertial]) -> Inertial {
    let mass: f64 = parts.iter().map(|p| p.mass).sum();
    if mass <= 0.0 {
        return Inertial::ZERO;
    }
    let com = parts.iter().map(|p| p.com * p.mass).sum::<DVec3>() / mass;
    let mut inertia = DMat3::ZERO;
    for p in parts {
        let d = p.com - com;
        let shift = DMat3::from_diagonal(DVec3::splat(d.dot(d))) - outer(d, d);
        inertia += p.inertia + shift * p.mass;
    }
    Inertial { mass, com, inertia }
}

fn outer(a: DVec3, b: DVec3) -> DMat3 {
    DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}

/// What the meshes say: every geom's properties at the link's density,
/// summed in the link frame. A link with no geoms is [`Inertial::ZERO`],
/// which is fine for a static body and a `ZeroMassMovableLink` export
/// error for a moving one.
pub fn computed_inertial(
    link: &Link,
    meshes: &impl MeshLookup,
    materials: &BTreeMap<String, Material>,
) -> Result<Inertial, InertialError> {
    let density = density(link, materials)?;
    let parts = link
        .visuals
        .iter()
        .map(|g| geom_inertial(g, meshes, density))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(sum_inertials(&parts))
}

/// The link's inertial under its [`InertialSpec`]: `Computed` is the mesh
/// sum; `Override` passes the stored values through (the sum, if it can be
/// made, rides along as `computed`); `Hybrid` scales the sum's mass and
/// tensor together to the weighed mass, keeping its CoM.
pub fn compose_inertial(
    link: &Link,
    meshes: &impl MeshLookup,
    materials: &BTreeMap<String, Material>,
) -> Result<LinkInertial, InertialError> {
    match &link.inertial {
        InertialSpec::Computed { .. } => {
            let computed = computed_inertial(link, meshes, materials)?;
            Ok(LinkInertial {
                inertial: computed,
                computed: Some(computed),
            })
        }
        InertialSpec::Override { mass, com, inertia } => Ok(LinkInertial {
            inertial: Inertial {
                mass: *mass,
                com: *com,
                inertia: *inertia,
            },
            computed: computed_inertial(link, meshes, materials).ok(),
        }),
        InertialSpec::Hybrid { mass } => {
            let computed = computed_inertial(link, meshes, materials)?;
            if computed.mass <= 0.0 {
                return Err(InertialError::NoVolume);
            }
            let scale = mass / computed.mass;
            Ok(LinkInertial {
                inertial: Inertial {
                    mass: *mass,
                    com: computed.com,
                    inertia: computed.inertia * scale,
                },
                computed: Some(computed),
            })
        }
    }
}

/// Every reason MuJoCo (or physics) would reject `inertial`, in the order
/// the export dialog lists them. Empty means good to export.
pub fn check(inertial: &Inertial) -> Vec<InertialError> {
    let mut errors = Vec::new();
    let Inertial { mass, com, inertia } = inertial;
    if !(mass.is_finite() && *mass > 0.0) {
        errors.push(InertialError::NonPositiveMass(*mass));
    }
    if !com.is_finite() || !inertia.to_cols_array().iter().all(|v| v.is_finite()) {
        errors.push(InertialError::NonFinite);
        return errors;
    }
    let scale = inertia
        .to_cols_array()
        .iter()
        .fold(0.0f64, |m, v| m.max(v.abs()))
        .max(f64::MIN_POSITIVE);
    let tol = 1e-9 * scale;
    let t = inertia.transpose();
    if (inertia.x_axis - t.x_axis).abs().max_element() > tol
        || (inertia.y_axis - t.y_axis).abs().max_element() > tol
        || (inertia.z_axis - t.z_axis).abs().max_element() > tol
    {
        errors.push(InertialError::NotSymmetric);
        return errors;
    }
    let moments = principal_moments(inertia);
    if moments.iter().any(|&m| m <= tol) {
        errors.push(InertialError::NotPositiveDefinite { moments });
        return errors;
    }
    let [a, b, c] = moments;
    if a + b < c - tol || b + c < a - tol || c + a < b - tol {
        errors.push(InertialError::TriangleInequality { moments });
    }
    errors
}

/// The eigenvalues of a symmetric 3×3 `m`, ascending, by cyclic Jacobi
/// rotations: each sweep zeroes the three off-diagonal entries in turn with
/// a rotation in their plane, and the off-diagonal mass shrinks
/// quadratically. Converges in a handful of sweeps for any input; the
/// principal axes are not returned because the MJCF writer hands MuJoCo the
/// full tensor (ADR-0008) and nothing else needs them.
pub fn principal_moments(m: &DMat3) -> [f64; 3] {
    let mut a = [
        [m.x_axis.x, m.y_axis.x, m.z_axis.x],
        [m.x_axis.y, m.y_axis.y, m.z_axis.y],
        [m.x_axis.z, m.y_axis.z, m.z_axis.z],
    ];
    for _ in 0..50 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        let diag = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if off <= 1e-15 * diag.max(f64::MIN_POSITIVE) {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            // The rotation angle that zeroes a[p][q].
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            // `signum(0.0)` is 1: equal diagonals get the 45° rotation.
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            // A' = Jᵀ A J for the Givens rotation J in the (p, q) plane.
            let mut b = a;
            for k in 0..3 {
                b[k][p] = c * a[k][p] - s * a[k][q];
                b[k][q] = s * a[k][p] + c * a[k][q];
            }
            let mut a2 = b;
            for k in 0..3 {
                a2[p][k] = c * b[p][k] - s * b[q][k];
                a2[q][k] = s * b[p][k] + c * b[q][k];
            }
            a2[p][q] = 0.0;
            a2[q][p] = 0.0;
            a = a2;
        }
    }
    let mut moments = [a[0][0], a[1][1], a[2][2]];
    moments.sort_by(f64::total_cmp);
    moments
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{Id, IdGen};
    use crate::pose::Pose;
    use crate::robot::Robot;
    use riggen_mesh::glam::{DMat4, DQuat};
    use std::f64::consts::FRAC_PI_2;

    const STEEL: f64 = 7850.0;

    fn assert_close(a: DMat3, b: DMat3, tol: f64) {
        for c in 0..3 {
            for r in 0..3 {
                assert!(
                    (a.col(c)[r] - b.col(c)[r]).abs() <= tol,
                    "[{r}][{c}]: {} vs {}\n{a:?}\n{b:?}",
                    a.col(c)[r],
                    b.col(c)[r]
                );
            }
        }
    }

    /// A link of `geoms` (mesh, pose) in steel, and the lookup behind it.
    fn link_with(
        geoms: Vec<(TriMesh, Pose)>,
        spec: InertialSpec,
    ) -> (Link, BTreeMap<MeshId, TriMesh>) {
        let mut ids = IdGen::new();
        let mut meshes = BTreeMap::new();
        let mut link = Link::new("l");
        link.material = Some("steel".into());
        link.inertial = spec;
        for (mesh, pose) in geoms {
            let id: MeshId = ids.alloc();
            meshes.insert(id, mesh);
            link.visuals.push(Geom {
                id: ids.alloc(),
                mesh: id,
                pose,
                color: None,
            });
        }
        (link, meshes)
    }

    fn materials() -> BTreeMap<String, Material> {
        Robot::default_materials()
    }

    fn box_inertia(mass: f64, size: DVec3) -> DMat3 {
        let s = size * size;
        DMat3::from_diagonal(DVec3::new(s.y + s.z, s.x + s.z, s.x + s.y) * (mass / 12.0))
    }

    #[test]
    fn two_cubes_side_by_side_are_one_box() {
        // Unit cubes at x = ±0.5 make the 2×1×1 box centred on the origin.
        let (link, meshes) = link_with(
            vec![
                (TriMesh::cube(0.5), Pose::from_translation(DVec3::X * 0.5)),
                (TriMesh::cube(0.5), Pose::from_translation(DVec3::X * -0.5)),
            ],
            InertialSpec::Computed {
                density_override: None,
            },
        );
        let out = compose_inertial(&link, &meshes, &materials()).unwrap();
        let mass = STEEL * 2.0;
        assert!((out.inertial.mass - mass).abs() < 1e-9);
        assert!(out.inertial.com.length() < 1e-12, "{}", out.inertial.com);
        assert_close(
            out.inertial.inertia,
            box_inertia(mass, DVec3::new(2.0, 1.0, 1.0)),
            1e-9,
        );
        assert_eq!(out.computed, Some(out.inertial));
        assert!(check(&out.inertial).is_empty());
    }

    #[test]
    fn a_rotated_geom_has_the_rotated_tensor() {
        // A tall prism along Z, posed with its axis along X.
        let prism = TriMesh::cylinder(0.1, 1.0, 12);
        let q = DQuat::from_rotation_y(FRAC_PI_2);
        let (link, meshes) = link_with(
            vec![(prism.clone(), Pose::from_rotation(q))],
            InertialSpec::Computed {
                density_override: None,
            },
        );
        let out = compose_inertial(&link, &meshes, &materials()).unwrap();
        let upright = mass_properties(&prism, STEEL).inertia;
        let r = DMat3::from_quat(q);
        assert_close(out.inertial.inertia, r * upright * r.transpose(), 1e-12);
        // The long axis is now X: the smallest moment is about X.
        assert!(out.inertial.inertia.x_axis.x < out.inertial.inertia.z_axis.z);

        // Same as transforming the mesh itself and measuring it in place.
        let mut moved = prism;
        moved.transform(&DMat4::from_quat(q));
        assert_close(
            out.inertial.inertia,
            mass_properties(&moved, STEEL).inertia,
            1e-12,
        );
    }

    #[test]
    fn density_override_beats_the_material_and_no_density_is_an_error() {
        let (mut link, meshes) = link_with(
            vec![(TriMesh::cube(0.5), Pose::IDENTITY)],
            InertialSpec::Computed {
                density_override: Some(100.0),
            },
        );
        let out = compose_inertial(&link, &meshes, &materials()).unwrap();
        assert!((out.inertial.mass - 100.0).abs() < 1e-12);

        link.material = None;
        assert!(
            (compose_inertial(&link, &meshes, &materials())
                .unwrap()
                .inertial
                .mass
                - 100.0)
                .abs()
                < 1e-12
        );
        link.inertial = InertialSpec::Computed {
            density_override: None,
        };
        assert_eq!(
            compose_inertial(&link, &meshes, &materials()).unwrap_err(),
            InertialError::NoDensity
        );
        link.material = Some("unobtainium".into());
        assert_eq!(
            compose_inertial(&link, &meshes, &materials()).unwrap_err(),
            InertialError::UnknownMaterial("unobtainium".into())
        );
    }

    #[test]
    fn hybrid_scales_mass_and_tensor_together() {
        let (link, meshes) = link_with(
            vec![(TriMesh::cube(0.5), Pose::from_translation(DVec3::Y))],
            InertialSpec::Hybrid { mass: 2.0 },
        );
        let out = compose_inertial(&link, &meshes, &materials()).unwrap();
        let computed = out.computed.unwrap();
        assert!((computed.mass - STEEL).abs() < 1e-9);
        assert_eq!(out.inertial.mass, 2.0);
        assert_eq!(out.inertial.com, computed.com);
        assert_close(
            out.inertial.inertia,
            computed.inertia * (2.0 / STEEL),
            1e-15,
        );
        assert_close(out.inertial.inertia, box_inertia(2.0, DVec3::ONE), 1e-12);

        // Nothing to scale.
        let (empty, meshes) = link_with(vec![], InertialSpec::Hybrid { mass: 2.0 });
        assert_eq!(
            compose_inertial(&empty, &meshes, &materials()).unwrap_err(),
            InertialError::NoVolume
        );
    }

    #[test]
    fn override_passes_through_and_keeps_the_computed_readout() {
        let inertia = DMat3::from_diagonal(DVec3::new(1.0, 2.0, 2.5));
        let spec = InertialSpec::Override {
            mass: 3.0,
            com: DVec3::new(0.1, 0.2, 0.3),
            inertia,
        };
        let (link, meshes) = link_with(vec![(TriMesh::cube(0.5), Pose::IDENTITY)], spec.clone());
        let out = compose_inertial(&link, &meshes, &materials()).unwrap();
        assert_eq!(out.inertial.mass, 3.0);
        assert_eq!(out.inertial.com, DVec3::new(0.1, 0.2, 0.3));
        assert_eq!(out.inertial.inertia, inertia);
        assert!((out.computed.unwrap().mass - STEEL).abs() < 1e-9);

        // An override does not need the meshes: open mesh, no readout, no error.
        let mut open = TriMesh::cube(0.5);
        open.indices.truncate(30);
        let (link, meshes) = link_with(vec![(open, Pose::IDENTITY)], spec);
        let out = compose_inertial(&link, &meshes, &materials()).unwrap();
        assert_eq!(out.inertial.mass, 3.0);
        assert_eq!(out.computed, None);
    }

    #[test]
    fn open_and_missing_meshes_are_errors_under_computed() {
        let mut open = TriMesh::cube(0.5);
        open.indices.truncate(30);
        let (link, mut meshes) = link_with(
            vec![
                (TriMesh::cube(0.5), Pose::IDENTITY),
                (open, Pose::from_translation(DVec3::X * 2.0)),
            ],
            InertialSpec::Computed {
                density_override: None,
            },
        );
        let open_geom = link.visuals[1].id;
        assert_eq!(
            compose_inertial(&link, &meshes, &materials()).unwrap_err(),
            InertialError::OpenMesh { geom: open_geom }
        );
        let missing = link.visuals[0].mesh;
        meshes.remove(&missing);
        assert_eq!(
            compose_inertial(&link, &meshes, &materials()).unwrap_err(),
            InertialError::MissingMesh {
                geom: link.visuals[0].id,
                mesh: missing
            }
        );
        assert!(missing.raw() < open_geom.raw());
    }

    #[test]
    fn a_link_without_geoms_is_zero_not_an_error() {
        let (link, meshes) = link_with(
            vec![],
            InertialSpec::Computed {
                density_override: None,
            },
        );
        let out = compose_inertial(&link, &meshes, &materials()).unwrap();
        assert_eq!(out.inertial, Inertial::ZERO);
        // Not exportable on a moving body, though — both the mass and the
        // zero tensor say so.
        assert_eq!(
            check(&out.inertial),
            vec![
                InertialError::NonPositiveMass(0.0),
                InertialError::NotPositiveDefinite { moments: [0.0; 3] }
            ]
        );
    }

    #[test]
    fn principal_moments_of_diagonal_and_rotated_tensors() {
        let diag = DMat3::from_diagonal(DVec3::new(3.0, 1.0, 2.0));
        assert_eq!(principal_moments(&diag), [1.0, 2.0, 3.0]);

        let r = DMat3::from_quat(DQuat::from_euler(
            riggen_mesh::glam::EulerRot::XYZ,
            0.4,
            -1.1,
            2.3,
        ));
        let rotated = r * diag * r.transpose();
        let m = principal_moments(&rotated);
        for (got, want) in m.iter().zip([1.0, 2.0, 3.0]) {
            assert!((got - want).abs() < 1e-12, "{m:?}");
        }
        assert_eq!(principal_moments(&DMat3::ZERO), [0.0; 3]);
        // Repeated eigenvalues do not stall the iteration.
        assert_eq!(principal_moments(&DMat3::IDENTITY), [1.0; 3]);
    }

    #[test]
    fn a_flat_plate_fails_the_triangle_inequality_only_after_a_bad_override() {
        // A thin plate: Izz ≈ Ixx + Iyy, on the edge of the inequality and
        // legitimately inside it.
        let plate = TriMesh::cube(0.5);
        let mut thin = plate;
        thin.transform(&DMat4::from_scale(DVec3::new(1.0, 1.0, 0.01)));
        let (link, meshes) = link_with(
            vec![(thin, Pose::IDENTITY)],
            InertialSpec::Computed {
                density_override: None,
            },
        );
        let computed = compose_inertial(&link, &meshes, &materials()).unwrap();
        assert!(
            check(&computed.inertial).is_empty(),
            "{:?}",
            check(&computed.inertial)
        );

        // Hand-typed "plate" numbers that no body can have: Izz > Ixx + Iyy.
        let bad = Inertial {
            mass: 1.0,
            com: DVec3::ZERO,
            inertia: DMat3::from_diagonal(DVec3::new(1.0, 1.0, 3.0)),
        };
        assert_eq!(
            check(&bad),
            vec![InertialError::TriangleInequality {
                moments: [1.0, 1.0, 3.0]
            }]
        );
        // And rotated, so the violation hides in the off-diagonals.
        let r = DMat3::from_quat(DQuat::from_rotation_x(0.7));
        let hidden = Inertial {
            inertia: r * bad.inertia * r.transpose(),
            ..bad
        };
        assert!(matches!(
            check(&hidden)[..],
            [InertialError::TriangleInequality { .. }]
        ));
    }

    #[test]
    fn check_catches_the_other_rejections() {
        let good = Inertial {
            mass: 1.0,
            com: DVec3::ZERO,
            inertia: DMat3::from_diagonal(DVec3::new(1.0, 2.0, 2.5)),
        };
        assert!(check(&good).is_empty());
        assert_eq!(
            check(&Inertial { mass: -1.0, ..good }),
            vec![InertialError::NonPositiveMass(-1.0)]
        );
        assert_eq!(
            check(&Inertial {
                com: DVec3::new(f64::NAN, 0.0, 0.0),
                ..good
            }),
            vec![InertialError::NonFinite]
        );
        let mut asym = good.inertia;
        asym.x_axis.y = 0.5;
        assert_eq!(
            check(&Inertial {
                inertia: asym,
                ..good
            }),
            vec![InertialError::NotSymmetric]
        );
        assert_eq!(
            check(&Inertial {
                inertia: DMat3::from_diagonal(DVec3::new(1.0, -2.0, 2.5)),
                ..good
            }),
            vec![InertialError::NotPositiveDefinite {
                moments: [-2.0, 1.0, 2.5]
            }]
        );
        // Errors compound: a zero mass and a bad tensor both show.
        assert_eq!(
            check(&Inertial {
                mass: 0.0,
                inertia: DMat3::from_diagonal(DVec3::new(1.0, 1.0, 3.0)),
                ..good
            })
            .len(),
            2
        );
    }

    #[test]
    fn sum_of_one_part_is_that_part_and_of_nothing_is_zero() {
        let part = Inertial {
            mass: 2.0,
            com: DVec3::new(1.0, 2.0, 3.0),
            inertia: DMat3::from_diagonal(DVec3::new(1.0, 2.0, 2.5)),
        };
        assert_eq!(sum_inertials(&[part]), part);
        assert_eq!(sum_inertials(&[]), Inertial::ZERO);
    }
}
