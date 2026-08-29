//! [`Pose`]: a rigid transform, "this frame expressed in the parent frame"
//! (docs/02-data-model.md §Conventions). Composition is `parent ∘ child`; a
//! matrix is derived, never stored.

use riggen_mesh::glam::{DMat3, DMat4, DQuat, DVec3};
use serde::{Deserialize, Serialize};

/// Translation then rotation: `p_parent = r * p_child + t`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pose {
    pub t: DVec3,
    pub r: DQuat,
}

impl Pose {
    pub const IDENTITY: Self = Self {
        t: DVec3::ZERO,
        r: DQuat::IDENTITY,
    };

    pub fn new(t: DVec3, r: DQuat) -> Self {
        Self { t, r }
    }

    pub fn from_translation(t: DVec3) -> Self {
        Self {
            t,
            r: DQuat::IDENTITY,
        }
    }

    pub fn from_rotation(r: DQuat) -> Self {
        Self { t: DVec3::ZERO, r }
    }

    /// URDF convention: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`, radians,
    /// `rpy = (roll, pitch, yaw)`. Both the properties panel (after its
    /// degree conversion) and the URDF writer go through here.
    pub fn from_xyz_rpy(xyz: DVec3, rpy: DVec3) -> Self {
        let r = DQuat::from_rotation_z(rpy.z)
            * DQuat::from_rotation_y(rpy.y)
            * DQuat::from_rotation_x(rpy.x);
        Self { t: xyz, r }
    }

    /// Inverse of [`from_xyz_rpy`](Self::from_xyz_rpy). Pitch is in
    /// `[-π/2, π/2]`; at gimbal lock (`|pitch| = π/2`) roll is reported as
    /// zero and yaw carries the whole twist. Angles round-trip as rotations,
    /// not necessarily as the same three numbers.
    pub fn to_xyz_rpy(&self) -> (DVec3, DVec3) {
        let m = DMat3::from_quat(self.r.normalize());
        // glam matrices are column-major: `m.x_axis.z` is row 2, column 0,
        // which for Rz·Ry·Rx equals -sin(pitch).
        let sin_pitch = (-m.x_axis.z).clamp(-1.0, 1.0);
        let pitch = sin_pitch.asin();
        let (roll, yaw) = if sin_pitch.abs() < 1.0 - 1e-12 {
            (
                m.y_axis.z.atan2(m.z_axis.z), // R21 / R22
                m.x_axis.y.atan2(m.x_axis.x), // R10 / R00
            )
        } else {
            (0.0, (-m.y_axis.x).atan2(m.y_axis.y)) // -R01 / R11
        };
        (self.t, DVec3::new(roll, pitch, yaw))
    }

    /// `self ∘ other`: `other` expressed in `self`'s parent frame.
    pub fn compose(&self, other: &Pose) -> Pose {
        Pose {
            t: self.t + self.r * other.t,
            r: (self.r * other.r).normalize(),
        }
    }

    pub fn inverse(&self) -> Pose {
        let r = self.r.normalize().inverse();
        Pose {
            t: -(r * self.t),
            r,
        }
    }

    pub fn transform_point(&self, p: DVec3) -> DVec3 {
        self.r * p + self.t
    }

    pub fn to_mat4(&self) -> DMat4 {
        DMat4::from_rotation_translation(self.r, self.t)
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    const EPS: f64 = 1e-9;

    fn assert_vec_eq(a: DVec3, b: DVec3) {
        assert!((a - b).length() < EPS, "{a} != {b}");
    }

    /// Quaternions `q` and `-q` are the same rotation.
    fn assert_rot_eq(a: DQuat, b: DQuat) {
        assert!(
            a.abs_diff_eq(b, EPS) || a.abs_diff_eq(-b, EPS),
            "{a} != {b}"
        );
    }

    #[test]
    fn identity_is_default_and_composes_trivially() {
        let p = Pose::from_xyz_rpy(DVec3::new(1.0, 2.0, 3.0), DVec3::new(0.1, 0.2, 0.3));
        assert_eq!(Pose::default(), Pose::IDENTITY);
        assert_eq!(Pose::IDENTITY.compose(&p), p);
        assert_eq!(p.compose(&Pose::IDENTITY), p);
        assert_eq!(Pose::IDENTITY.to_mat4(), DMat4::IDENTITY);
    }

    #[test]
    fn roll_pitch_yaw_are_the_urdf_axes() {
        // Roll about X takes +Y to +Z.
        let roll = Pose::from_xyz_rpy(DVec3::ZERO, DVec3::new(FRAC_PI_2, 0.0, 0.0));
        assert_vec_eq(roll.transform_point(DVec3::Y), DVec3::Z);
        // Pitch about Y takes +Z to +X.
        let pitch = Pose::from_xyz_rpy(DVec3::ZERO, DVec3::new(0.0, FRAC_PI_2, 0.0));
        assert_vec_eq(pitch.transform_point(DVec3::Z), DVec3::X);
        // Yaw about Z takes +X to +Y.
        let yaw = Pose::from_xyz_rpy(DVec3::ZERO, DVec3::new(0.0, 0.0, FRAC_PI_2));
        assert_vec_eq(yaw.transform_point(DVec3::X), DVec3::Y);
    }

    #[test]
    fn rpy_order_is_yaw_then_pitch_then_roll_applied_to_the_point() {
        // R = Rz(yaw)·Ry(pitch)·Rx(roll): the roll acts on the point first.
        // Roll 90° takes +Y to +Z; a following yaw of 90° leaves +Z alone.
        let p = Pose::from_xyz_rpy(DVec3::ZERO, DVec3::new(FRAC_PI_2, 0.0, FRAC_PI_2));
        assert_vec_eq(p.transform_point(DVec3::Y), DVec3::Z);
        // And +X: roll leaves it, yaw takes it to +Y.
        assert_vec_eq(p.transform_point(DVec3::X), DVec3::Y);
        // Hand-computed for (roll, pitch, yaw) = (90°, 90°, 0): Ry(90°)·Rx(90°)
        // maps +Y → +Z → +X.
        let q = Pose::from_xyz_rpy(DVec3::ZERO, DVec3::new(FRAC_PI_2, FRAC_PI_2, 0.0));
        assert_vec_eq(q.transform_point(DVec3::Y), DVec3::X);
    }

    #[test]
    fn rpy_round_trips_away_from_gimbal_lock() {
        let xyz = DVec3::new(0.5, -1.0, 2.0);
        for rpy in [
            DVec3::new(0.1, 0.2, 0.3),
            DVec3::new(-1.0, 0.7, 2.5),
            DVec3::new(3.0, -1.2, -3.0),
            DVec3::new(FRAC_PI_4, 0.0, 0.0),
            DVec3::new(0.0, 0.0, PI - 1e-3),
        ] {
            let (xyz_back, rpy_back) = Pose::from_xyz_rpy(xyz, rpy).to_xyz_rpy();
            assert_vec_eq(xyz_back, xyz);
            assert_vec_eq(rpy_back, rpy);
        }
    }

    #[test]
    fn rpy_round_trips_at_gimbal_lock_as_a_rotation() {
        for pitch in [FRAC_PI_2, -FRAC_PI_2] {
            let p = Pose::from_xyz_rpy(DVec3::ZERO, DVec3::new(0.4, pitch, -0.9));
            let (_, rpy) = p.to_xyz_rpy();
            assert!((rpy.y - pitch).abs() < 1e-6);
            assert_eq!(rpy.x, 0.0, "roll is folded into yaw at gimbal lock");
            assert_rot_eq(Pose::from_xyz_rpy(DVec3::ZERO, rpy).r, p.r);
        }
    }

    /// Deterministic LCG so the test needs no `rand` and never flakes.
    fn lcg(seed: &mut u64) -> f64 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*seed >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
    }

    #[test]
    fn random_rotations_round_trip_through_rpy() {
        let mut seed = 42;
        for _ in 0..1000 {
            let q = DQuat::from_xyzw(
                lcg(&mut seed),
                lcg(&mut seed),
                lcg(&mut seed),
                lcg(&mut seed),
            )
            .normalize();
            let pose = Pose::from_rotation(q);
            let (_, rpy) = pose.to_xyz_rpy();
            assert!(rpy.y.abs() <= FRAC_PI_2 + EPS);
            assert_rot_eq(Pose::from_xyz_rpy(DVec3::ZERO, rpy).r, q);
        }
    }

    #[test]
    fn compose_matches_matrix_product_and_inverse_cancels() {
        let a = Pose::from_xyz_rpy(DVec3::new(1.0, 0.0, 0.0), DVec3::new(0.0, 0.0, FRAC_PI_2));
        let b = Pose::from_xyz_rpy(DVec3::new(1.0, 0.0, 0.0), DVec3::new(FRAC_PI_2, 0.0, 0.0));
        let ab = a.compose(&b);
        // Hand-computed: b's origin (1,0,0) yawed by 90° is (0,1,0), plus a's (1,0,0).
        assert_vec_eq(ab.t, DVec3::new(1.0, 1.0, 0.0));
        let m = a.to_mat4() * b.to_mat4();
        let p = DVec3::new(0.3, -0.2, 0.7);
        assert_vec_eq(ab.transform_point(p), m.transform_point3(p));

        let round = ab.compose(&ab.inverse());
        assert_vec_eq(round.t, DVec3::ZERO);
        assert_rot_eq(round.r, DQuat::IDENTITY);
        assert_vec_eq(ab.inverse().transform_point(ab.transform_point(p)), p);
    }

    #[test]
    fn serde_shape_is_t_and_r_arrays() {
        let p = Pose::from_translation(DVec3::new(1.0, 2.0, 3.0));
        let json = serde_json::to_string(&p).unwrap();
        assert_eq!(json, "{\"t\":[1.0,2.0,3.0],\"r\":[0.0,0.0,0.0,1.0]}");
        assert_eq!(serde_json::from_str::<Pose>(&json).unwrap(), p);
        assert!(serde_json::from_str::<Pose>("{\"t\":[0,0,0],\"r\":[0,0,0,1],\"m\":1}").is_err());
    }
}
