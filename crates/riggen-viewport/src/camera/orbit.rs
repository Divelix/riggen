use web_time::Instant;

use riggen_mesh::glam::{Mat4, Vec3};

use super::animation::{CameraAnimation, CameraSample};
use super::orientation::{ISO_PITCH, MAX_PITCH, Projection, StandardView, ViewOrientation};

/// Turntable orbit camera around a focus point, Z-up (AGENTS.md).
///
/// `f32` throughout: this is the GPU side of the boundary
/// (docs/02-data-model.md). Angles are radians.
#[derive(Debug, Clone)]
pub struct OrbitCamera {
    pub target: Vec3,
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub fov_y: f32,
    pub near: f32,
    pub far: f32,
    pub projection: Projection,
    pub animation: Option<CameraAnimation>,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: Vec3::ZERO,
            distance: 3.0,
            yaw: Self::DEFAULT_YAW,
            pitch: Self::DEFAULT_PITCH,
            fov_y: 45f32.to_radians(),
            near: 0.01,
            far: 100.0,
            projection: Projection::Perspective,
            animation: None,
        }
    }
}

impl OrbitCamera {
    pub const DEFAULT_YAW: f32 = -std::f32::consts::FRAC_PI_4;
    pub const DEFAULT_PITCH: f32 = 0.5;

    pub fn eye(&self) -> Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        let dir = Vec3::new(cp * cy, cp * sy, sp);
        self.target + dir * self.distance
    }

    /// Forward/right/up basis at the current eye, Z-up except at the poles
    /// (exact top/bottom [`StandardView`]s), where `forward` is parallel to
    /// Z and `Z` would make `right` degenerate — Y stands in as the up hint
    /// there instead.
    pub fn basis(&self) -> (Vec3, Vec3, Vec3) {
        let forward = (self.target - self.eye()).normalize();
        let up_hint = if forward.z.abs() > 0.999 {
            Vec3::Y
        } else {
            Vec3::Z
        };
        let right = forward.cross(up_hint).normalize();
        let up = right.cross(forward).normalize();
        (forward, right, up)
    }

    pub fn view_matrix(&self) -> Mat4 {
        let (_, _, up) = self.basis();
        Mat4::look_at_rh(self.eye(), self.target, up)
    }

    /// Half-width/half-height of the orthographic frustum at the current
    /// distance, sized from the perspective FOV so toggling projection
    /// doesn't jump the apparent scale.
    pub fn ortho_half_extents(&self, aspect: f32) -> (f32, f32) {
        let half_height = self.distance * (self.fov_y * 0.5).tan();
        (half_height * aspect, half_height)
    }

    /// Projection with wgpu's `[0, 1]` clip depth — glam's `_rh`
    /// constructors produce it directly, so there is no OpenGL-to-wgpu
    /// remap here (robocad needed one for cgmath).
    pub fn proj_matrix(&self, aspect: f32) -> Mat4 {
        match self.projection {
            Projection::Perspective => {
                Mat4::perspective_rh(self.fov_y, aspect, self.near, self.far)
            }
            Projection::Orthographic => {
                let (half_width, half_height) = self.ortho_half_extents(aspect);
                Mat4::orthographic_rh(
                    -half_width,
                    half_width,
                    -half_height,
                    half_height,
                    self.near,
                    self.far,
                )
            }
        }
    }

    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        self.proj_matrix(aspect) * self.view_matrix()
    }

    /// View-projection for the corner axes-triad gizmo: same orientation as
    /// the main camera (so the triad orbits with it) but fixed distance and
    /// a small orthographic frustum, so panning/zooming the model never
    /// moves or scales it.
    pub fn axes_gizmo_view_proj(&self) -> Mat4 {
        let (_, _, up) = self.basis();
        let dir = (self.eye() - self.target).normalize();
        let eye = dir * 3.0;
        let view = Mat4::look_at_rh(eye, Vec3::ZERO, up);
        let proj = Mat4::orthographic_rh(-1.3, 1.3, -1.3, 1.3, 0.1, 10.0);
        proj * view
    }

    pub fn toggle_projection(&mut self) {
        self.projection = self.projection.toggled();
    }

    /// Snaps orientation to a [`StandardView`]; target and distance are
    /// unchanged.
    pub fn set_standard_view(&mut self, view: StandardView) {
        self.cancel_animation();
        use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};
        let (yaw, pitch) = match view {
            StandardView::Front => (FRAC_PI_2, 0.0),
            StandardView::Back => (-FRAC_PI_2, 0.0),
            StandardView::Right => (0.0, 0.0),
            StandardView::Left => (PI, 0.0),
            // Yaw is irrelevant once pitch reaches a pole; keep the current
            // one so a later orbit drag continues smoothly.
            StandardView::Top => (self.yaw, FRAC_PI_2),
            StandardView::Bottom => (self.yaw, -FRAC_PI_2),
            StandardView::Iso => (-FRAC_PI_4, ISO_PITCH),
        };
        self.yaw = yaw;
        self.pitch = pitch;
    }

    /// Snaps orientation to a [`ViewOrientation`]; target and distance are
    /// unchanged.
    pub fn set_orientation(&mut self, orientation: ViewOrientation) {
        self.cancel_animation();
        let (yaw, pitch) = orientation.yaw_pitch();
        self.yaw = yaw;
        self.pitch = pitch;
    }

    /// Starts a smooth animation to `target_yaw` and `target_pitch`.
    pub fn animate_to(&mut self, target_yaw: f32, target_pitch: f32) {
        self.animation = Some(CameraAnimation::new_orientation(
            self.target,
            self.distance,
            self.yaw,
            self.pitch,
            target_yaw,
            target_pitch,
        ));
    }

    /// Starts a smooth animation to a [`ViewOrientation`].
    pub fn animate_to_orientation(&mut self, orientation: ViewOrientation) {
        let (target_yaw, target_pitch) = orientation.yaw_pitch();
        self.animate_to(target_yaw, target_pitch);
    }

    /// Distance at which a sphere of `radius` fills the vertical FOV with a
    /// 20 % margin, clamped to the zoom range.
    fn fit_distance(&self, radius: f32) -> f32 {
        let radius = radius.max(1e-4);
        let fit = radius / (self.fov_y * 0.5).sin();
        (fit * 1.2).clamp(0.02, 50.0)
    }

    /// Starts a smooth animation to re-center on `center`, back off to fit
    /// `radius`, and transition to the default home orientation
    /// (`DEFAULT_YAW`, `DEFAULT_PITCH`).
    pub fn animate_home(&mut self, center: Vec3, radius: f32) {
        self.animate_home_with_orientation(center, radius, Self::DEFAULT_YAW, Self::DEFAULT_PITCH);
    }

    /// Starts a smooth animation to re-center on `center`, back off to fit
    /// `radius`, and transition to `(target_yaw, target_pitch)`.
    pub fn animate_home_with_orientation(
        &mut self,
        center: Vec3,
        radius: f32,
        target_yaw: f32,
        target_pitch: f32,
    ) {
        let target_distance = self.fit_distance(radius);
        self.animation = Some(CameraAnimation::from_samples(
            CameraSample::new(self.target, self.distance, self.yaw, self.pitch),
            CameraSample::new(center, target_distance, target_yaw, target_pitch),
            Instant::now(),
            CameraAnimation::DEFAULT_DURATION,
        ));
    }

    /// Starts a smooth transition to re-center on `center` and back off to
    /// fit `radius`.
    pub fn animate_frame_bounds(&mut self, center: Vec3, radius: f32) {
        let target_distance = self.fit_distance(radius);
        self.animate_to_target_and_distance(center, target_distance);
    }

    /// Starts a smooth animation to a new `target` position and `distance`.
    pub fn animate_to_target_and_distance(&mut self, target: Vec3, distance: f32) {
        self.animation = Some(CameraAnimation::new_bounds(
            self.target,
            self.distance,
            self.yaw,
            self.pitch,
            target,
            distance,
        ));
    }

    /// Steps the active camera animation, updating `target`, `distance`,
    /// `yaw` and `pitch`. Returns `true` if an animation is still in
    /// progress.
    pub fn step_animation(&mut self, now: Instant) -> bool {
        if let Some(anim) = &self.animation {
            let (sample, finished) = anim.sample(now);
            self.target = sample.target;
            self.distance = sample.distance;
            self.yaw = sample.yaw;
            self.pitch = sample.pitch;
            if finished {
                self.animation = None;
                false
            } else {
                true
            }
        } else {
            false
        }
    }

    /// Cancels any active camera animation in place without jumping.
    pub fn cancel_animation(&mut self) {
        self.animation = None;
    }

    /// Whether a camera transition animation is currently running.
    pub fn is_animating(&self) -> bool {
        self.animation.is_some()
    }

    /// The closest [`ViewOrientation`] to the current camera direction.
    pub fn closest_orientation(&self) -> ViewOrientation {
        ViewOrientation::from_direction(self.eye() - self.target)
    }

    /// Re-centers on `center` and backs off to `radius` (plus margin), for
    /// zoom-to-fit from scene bounds. Immediate — the animated form is
    /// [`Self::animate_frame_bounds`].
    pub fn frame_bounds(&mut self, center: Vec3, radius: f32) {
        self.cancel_animation();
        self.target = center;
        self.distance = self.fit_distance(radius);
    }

    /// `delta_yaw`/`delta_pitch` in radians.
    pub fn orbit(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.cancel_animation();
        self.yaw += delta_yaw;
        self.pitch = (self.pitch + delta_pitch).clamp(-MAX_PITCH, MAX_PITCH);
    }

    /// `delta_x`/`delta_y` in screen pixels.
    pub fn pan(&mut self, delta_x: f32, delta_y: f32) {
        self.cancel_animation();
        let (_, right, up) = self.basis();
        let scale = self.distance * 0.0015;
        self.target += right * (-delta_x * scale) + up * (delta_y * scale);
    }

    /// `scroll_delta` in the same units as egui's raw scroll delta; positive
    /// scrolls up (zoom in).
    pub fn zoom(&mut self, scroll_delta: f32) {
        self.cancel_animation();
        let factor = (1.0 - scroll_delta * 0.001).clamp(0.1, 10.0);
        self.distance = (self.distance * factor).clamp(0.02, 50.0);
    }

    /// World-space direction from the eye through normalized device
    /// coordinates `ndc` (x right, y up, both in `[-1, 1]`) — an
    /// approximation shared by both projections, exact for perspective.
    fn cursor_ray_dir(&self, ndc: (f32, f32), aspect: f32) -> Vec3 {
        let (forward, right, up) = self.basis();
        let tan_half_fov = (self.fov_y * 0.5).tan();
        (forward + right * (ndc.0 * tan_half_fov * aspect) + up * (ndc.1 * tan_half_fov))
            .normalize()
    }

    /// Zoom that dollies toward whatever is under the cursor instead of
    /// always toward `target`. Moves the eye by however much `distance` just
    /// changed along the cursor ray, then re-derives `target` behind it
    /// along the camera's (unchanged) orientation — so at screen-center,
    /// where the cursor ray coincides with the view direction, this
    /// degenerates to the plain target-anchored [`Self::zoom`].
    pub fn zoom_to_cursor(&mut self, scroll_delta: f32, ndc: (f32, f32), aspect: f32) {
        self.cancel_animation();
        let old_eye = self.eye();
        let orientation = (old_eye - self.target) / self.distance;
        let ray_dir = self.cursor_ray_dir(ndc, aspect);
        let old_distance = self.distance;
        self.zoom(scroll_delta);
        let delta = old_distance - self.distance;
        if delta.abs() < f32::EPSILON {
            return;
        }
        let new_eye = old_eye + ray_dir * delta;
        self.target = new_eye - orientation * self.distance;
    }
}
