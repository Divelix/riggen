//! Collision primitives fitted to a point cloud, in the cloud's own frame
//! (plans/m3-sim-ready): the starting numbers for `CollisionPolicy::
//! Primitives`, which the user then moves and resizes. Deliberately the
//! simplest fits — every one starts from the axis-aligned bounding box —
//! since an oriented (PCA) fit is a backlog line, not an M3 need.
//!
//! No `Primitive` here: that is a core type with a `Pose`. These return the
//! numbers; `riggen-core` turns an axis into a pose whose Z is that axis.

use glam::DVec3;

use crate::Aabb;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxFit {
    pub center: DVec3,
    /// Full extents.
    pub size: DVec3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphereFit {
    pub center: DVec3,
    pub radius: f64,
}

/// A cylinder or capsule: `axis` is unit, `length` is the straight part
/// (for a capsule, without the two hemispherical caps).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxialFit {
    pub center: DVec3,
    pub axis: DVec3,
    pub radius: f64,
    pub length: f64,
}

/// The axis-aligned bounding box. `None` for an empty cloud.
pub fn box_fit(points: &[DVec3]) -> Option<BoxFit> {
    let aabb = Aabb::of_points(points.iter().copied())?;
    Some(BoxFit {
        center: aabb.center(),
        size: aabb.max - aabb.min,
    })
}

/// The AABB's centre and the farthest point from it.
pub fn sphere_fit(points: &[DVec3]) -> Option<SphereFit> {
    let center = Aabb::of_points(points.iter().copied())?.center();
    let radius = points
        .iter()
        .map(|p| (*p - center).length())
        .fold(0.0, f64::max);
    Some(SphereFit { center, radius })
}

/// Axis along the AABB's longest extent, length that extent, radius the
/// farthest radial distance from the axis.
pub fn cylinder_fit(points: &[DVec3]) -> Option<AxialFit> {
    let aabb = Aabb::of_points(points.iter().copied())?;
    let extent = aabb.max - aabb.min;
    let axis = longest_axis(extent);
    let center = aabb.center();
    Some(AxialFit {
        center,
        axis,
        radius: max_radial(points, center, axis),
        length: extent.dot(axis),
    })
}

/// Like [`cylinder_fit`], with the caps taken off the length: a capsule of
/// radius `r` spans `length + 2r` along its axis. A cloud shorter than
/// `2r` along the axis gets a zero-length capsule (a sphere).
pub fn capsule_fit(points: &[DVec3]) -> Option<AxialFit> {
    let fit = cylinder_fit(points)?;
    Some(AxialFit {
        length: (fit.length - 2.0 * fit.radius).max(0.0),
        ..fit
    })
}

/// The unit axis of the largest component; ties go Z, then Y, then X, so
/// an upright part stays upright.
fn longest_axis(extent: DVec3) -> DVec3 {
    if extent.z >= extent.x && extent.z >= extent.y {
        DVec3::Z
    } else if extent.y >= extent.x {
        DVec3::Y
    } else {
        DVec3::X
    }
}

fn max_radial(points: &[DVec3], center: DVec3, axis: DVec3) -> f64 {
    points
        .iter()
        .map(|p| {
            let d = *p - center;
            (d - axis * d.dot(axis)).length()
        })
        .fold(0.0, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TriMesh;

    #[test]
    fn each_fit_returns_its_own_generators_numbers() {
        let cube = TriMesh::cube(0.05);
        let b = box_fit(&cube.positions).unwrap();
        assert_eq!(b.center, DVec3::ZERO);
        assert!((b.size - DVec3::splat(0.1)).length() < 1e-15);

        let sphere = TriMesh::sphere(0.3, 24);
        let s = sphere_fit(&sphere.positions).unwrap();
        assert!(s.center.length() < 1e-15, "{}", s.center);
        assert!((s.radius - 0.3).abs() < 1e-12, "{}", s.radius);

        let cylinder = TriMesh::cylinder(0.2, 1.5, 16);
        let c = cylinder_fit(&cylinder.positions).unwrap();
        assert_eq!(c.axis, DVec3::Z);
        assert!(c.center.length() < 1e-15);
        assert!((c.radius - 0.2).abs() < 1e-12, "{}", c.radius);
        assert!((c.length - 1.5).abs() < 1e-12, "{}", c.length);

        let capsule = TriMesh::capsule(0.2, 1.0, 16);
        let k = capsule_fit(&capsule.positions).unwrap();
        assert_eq!(k.axis, DVec3::Z);
        assert!(k.center.length() < 1e-15);
        assert!((k.radius - 0.2).abs() < 1e-12, "{}", k.radius);
        assert!((k.length - 1.0).abs() < 1e-12, "{}", k.length);
    }

    #[test]
    fn a_lying_cylinder_is_fitted_along_its_length() {
        let mut mesh = TriMesh::cylinder(0.1, 2.0, 16);
        mesh.transform(&glam::DMat4::from_rotation_y(std::f64::consts::FRAC_PI_2));
        let c = cylinder_fit(&mesh.positions).unwrap();
        assert_eq!(c.axis, DVec3::X);
        assert!((c.length - 2.0).abs() < 1e-12);
        assert!((c.radius - 0.1).abs() < 1e-12);
        // Squat: the capsule has no straight part left.
        let k = capsule_fit(&TriMesh::cylinder(0.5, 0.5, 8).positions).unwrap();
        assert_eq!(k.length, 0.0);
    }

    #[test]
    fn empty_clouds_fit_nothing() {
        assert_eq!(box_fit(&[]), None);
        assert_eq!(sphere_fit(&[]), None);
        assert_eq!(cylinder_fit(&[]), None);
        assert_eq!(capsule_fit(&[]), None);
    }
}
