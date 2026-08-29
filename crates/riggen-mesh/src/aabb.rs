use glam::{DMat4, DVec3};

/// Axis-aligned bounding box. Both corners inclusive; a single point is a
/// valid (zero-volume) box.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: DVec3,
    pub max: DVec3,
}

impl Aabb {
    /// Tight bounds of `points`; `None` if there are none.
    pub fn of_points(points: impl IntoIterator<Item = DVec3>) -> Option<Self> {
        points.into_iter().fold(None, |acc, p| {
            Some(match acc {
                None => Self { min: p, max: p },
                Some(b) => Self {
                    min: b.min.min(p),
                    max: b.max.max(p),
                },
            })
        })
    }

    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Bounds of the eight transformed corners — the tight box of the
    /// transformed box, not of the original contents.
    pub fn transformed(&self, m: &DMat4) -> Self {
        let corners = (0..8).map(|i| {
            let p = DVec3::new(
                if i & 1 == 0 { self.min.x } else { self.max.x },
                if i & 2 == 0 { self.min.y } else { self.max.y },
                if i & 4 == 0 { self.min.z } else { self.max.z },
            );
            m.transform_point3(p)
        });
        Self::of_points(corners).expect("eight corners")
    }

    pub fn center(&self) -> DVec3 {
        (self.min + self.max) * 0.5
    }

    pub fn size(&self) -> DVec3 {
        self.max - self.min
    }

    /// Radius of the sphere around `center()` that contains the box — what
    /// zoom-to-fit frames.
    pub fn half_diagonal(&self) -> f64 {
        self.size().length() * 0.5
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn of_points_and_union() {
        assert_eq!(Aabb::of_points([]), None);
        let a = Aabb::of_points([DVec3::new(1.0, -2.0, 3.0), DVec3::new(-1.0, 2.0, 0.0)]).unwrap();
        assert_eq!(a.min, DVec3::new(-1.0, -2.0, 0.0));
        assert_eq!(a.max, DVec3::new(1.0, 2.0, 3.0));
        let b = Aabb::of_points([DVec3::splat(5.0)]).unwrap();
        let u = a.union(&b);
        assert_eq!(u.min, a.min);
        assert_eq!(u.max, DVec3::splat(5.0));
    }

    #[test]
    fn transformed_rotates_and_translates() {
        let unit = Aabb {
            min: DVec3::splat(-1.0),
            max: DVec3::new(2.0, 1.0, 1.0),
        };
        // 90° about Z maps x → y, then shift by (10, 0, 0).
        let m = DMat4::from_translation(DVec3::new(10.0, 0.0, 0.0))
            * DMat4::from_rotation_z(std::f64::consts::FRAC_PI_2);
        let t = unit.transformed(&m);
        let eps = 1e-12;
        assert!((t.min - DVec3::new(9.0, -1.0, -1.0)).length() < eps);
        assert!((t.max - DVec3::new(11.0, 2.0, 1.0)).length() < eps);
    }
}
