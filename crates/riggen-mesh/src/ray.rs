use glam::DVec3;

/// A half-line: `origin + t * dir`, `t >= 0`. `dir` need not be unit length;
/// the `t` returned by [`ray_triangle`] is in units of `dir`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray {
    pub origin: DVec3,
    pub dir: DVec3,
}

impl Ray {
    pub fn at(&self, t: f64) -> DVec3 {
        self.origin + self.dir * t
    }
}

/// Möller–Trumbore ray/triangle intersection: the `t` of the hit, or `None`
/// for a miss, a hit behind the origin, or a ray parallel to the triangle.
///
/// Two-sided on purpose. The ID buffer has already decided which triangle
/// the cursor is over (docs/01-architecture.md §Picking and snapping); this
/// only recovers the exact point on it, and must not lose the hit because
/// the renderer drew a back face.
pub fn ray_triangle(ray: &Ray, tri: &[DVec3; 3]) -> Option<f64> {
    const EPS: f64 = 1e-12;
    let [a, b, c] = *tri;
    let e1 = b - a;
    let e2 = c - a;
    let p = ray.dir.cross(e2);
    let det = e1.dot(p);
    if det.abs() < EPS {
        return None;
    }
    let inv_det = 1.0 / det;
    let s = ray.origin - a;
    let u = s.dot(p) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = ray.dir.dot(q) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(q) * inv_det;
    (t >= 0.0).then_some(t)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Unit right triangle in the z = 1 plane, CCW seen from +Z.
    const TRI: [DVec3; 3] = [
        DVec3::new(0.0, 0.0, 1.0),
        DVec3::new(1.0, 0.0, 1.0),
        DVec3::new(0.0, 1.0, 1.0),
    ];

    fn down_from(x: f64, y: f64) -> Ray {
        Ray {
            origin: DVec3::new(x, y, 5.0),
            dir: DVec3::NEG_Z,
        }
    }

    #[test]
    fn hits_inside_and_reports_distance() {
        let t = ray_triangle(&down_from(0.25, 0.25), &TRI).unwrap();
        assert!((t - 4.0).abs() < 1e-12);
        assert!((down_from(0.25, 0.25).at(t).z - 1.0).abs() < 1e-12);
    }

    #[test]
    fn misses_outside_and_behind() {
        assert_eq!(ray_triangle(&down_from(0.75, 0.75), &TRI), None);
        assert_eq!(ray_triangle(&down_from(-0.1, 0.5), &TRI), None);
        // Pointing away from the triangle: the hit would be at t < 0.
        let up = Ray {
            origin: DVec3::new(0.25, 0.25, 5.0),
            dir: DVec3::Z,
        };
        assert_eq!(ray_triangle(&up, &TRI), None);
    }

    #[test]
    fn parallel_ray_misses() {
        let ray = Ray {
            origin: DVec3::new(-1.0, 0.25, 1.0),
            dir: DVec3::X,
        };
        assert_eq!(ray_triangle(&ray, &TRI), None);
    }

    #[test]
    fn backface_still_hits() {
        // From below, the triangle's winding is clockwise; still a hit.
        let ray = Ray {
            origin: DVec3::new(0.25, 0.25, -3.0),
            dir: DVec3::Z,
        };
        let t = ray_triangle(&ray, &TRI).unwrap();
        assert!((t - 4.0).abs() < 1e-12);
    }

    #[test]
    fn unnormalised_dir_scales_t() {
        let ray = Ray {
            origin: DVec3::new(0.25, 0.25, 5.0),
            dir: DVec3::NEG_Z * 2.0,
        };
        let t = ray_triangle(&ray, &TRI).unwrap();
        assert!((t - 2.0).abs() < 1e-12);
    }

    #[test]
    fn cube_ray_hits_near_face_first() {
        let cube = crate::TriMesh::cube(1.0);
        let ray = Ray {
            origin: DVec3::new(0.1, 0.2, 10.0),
            dir: DVec3::NEG_Z,
        };
        let mut hits: Vec<f64> = (0..cube.triangle_count())
            .filter_map(|i| ray_triangle(&ray, &cube.triangle(i)))
            .collect();
        hits.sort_by(|a, b| a.partial_cmp(b).unwrap());
        // The +Z face at t = 9 and, two-sided, the −Z face at t = 11.
        assert_eq!(hits.len(), 2);
        assert!((hits[0] - 9.0).abs() < 1e-12);
        assert!((hits[1] - 11.0).abs() < 1e-12);
    }
}
