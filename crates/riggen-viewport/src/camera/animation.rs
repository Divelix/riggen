use std::time::Duration;
use web_time::Instant;

use riggen_mesh::glam::Vec3;

/// Calculates the shortest angular difference in `[-π, π]` from `from` to
/// `to`.
pub fn shortest_angular_delta(from: f32, to: f32) -> f32 {
    use std::f32::consts::{PI, TAU};
    let diff = (to - from) % TAU;
    if diff > PI {
        diff - TAU
    } else if diff < -PI {
        diff + TAU
    } else {
        diff
    }
}

/// Sampled camera parameters at a specific instant during an animation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraSample {
    pub target: Vec3,
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
}

impl CameraSample {
    pub const fn new(target: Vec3, distance: f32, yaw: f32, pitch: f32) -> Self {
        Self {
            target,
            distance,
            yaw,
            pitch,
        }
    }
}

/// Smooth camera transition between two camera states (`target`,
/// `distance`, `yaw`, `pitch`).
///
/// Cubic ease-out, taking the shortest angular turn around the yaw axis.
#[derive(Debug, Clone)]
pub struct CameraAnimation {
    pub start_target: Vec3,
    pub target_target: Vec3,
    pub start_distance: f32,
    pub target_distance: f32,
    pub start_yaw: f32,
    pub start_pitch: f32,
    pub target_yaw: f32,
    pub target_pitch: f32,
    pub start_time: Instant,
    pub duration: Duration,
}

impl CameraAnimation {
    pub const DEFAULT_DURATION: Duration = Duration::from_millis(180);

    /// Creates an animation transitioning between two [`CameraSample`]
    /// states with custom timing.
    pub fn from_samples(
        start: CameraSample,
        target: CameraSample,
        start_time: Instant,
        duration: Duration,
    ) -> Self {
        let delta_yaw = shortest_angular_delta(start.yaw, target.yaw);
        Self {
            start_target: start.target,
            target_target: target.target,
            start_distance: start.distance,
            target_distance: target.distance,
            start_yaw: start.yaw,
            start_pitch: start.pitch,
            target_yaw: start.yaw + delta_yaw,
            target_pitch: target.pitch,
            start_time,
            duration,
        }
    }

    /// Creates an animation for changing orientation (yaw, pitch) only,
    /// keeping target and distance fixed.
    pub fn new_orientation(
        target: Vec3,
        distance: f32,
        start_yaw: f32,
        start_pitch: f32,
        target_yaw: f32,
        target_pitch: f32,
    ) -> Self {
        Self::from_samples(
            CameraSample::new(target, distance, start_yaw, start_pitch),
            CameraSample::new(target, distance, target_yaw, target_pitch),
            Instant::now(),
            Self::DEFAULT_DURATION,
        )
    }

    /// Creates an animation for framing bounds (target, distance) only,
    /// keeping orientation (yaw, pitch) fixed.
    pub fn new_bounds(
        start_target: Vec3,
        start_distance: f32,
        yaw: f32,
        pitch: f32,
        target_target: Vec3,
        target_distance: f32,
    ) -> Self {
        Self::from_samples(
            CameraSample::new(start_target, start_distance, yaw, pitch),
            CameraSample::new(target_target, target_distance, yaw, pitch),
            Instant::now(),
            Self::DEFAULT_DURATION,
        )
    }

    /// Convenience constructor for orientation-only animation starting
    /// immediately with default duration.
    pub fn new(start_yaw: f32, start_pitch: f32, target_yaw: f32, target_pitch: f32) -> Self {
        Self::with_duration(
            start_yaw,
            start_pitch,
            target_yaw,
            target_pitch,
            Instant::now(),
            Self::DEFAULT_DURATION,
        )
    }

    /// Convenience constructor for orientation-only animation with custom
    /// start time and duration.
    pub fn with_duration(
        start_yaw: f32,
        start_pitch: f32,
        target_yaw: f32,
        target_pitch: f32,
        start_time: Instant,
        duration: Duration,
    ) -> Self {
        Self::from_samples(
            CameraSample::new(Vec3::ZERO, 1.0, start_yaw, start_pitch),
            CameraSample::new(Vec3::ZERO, 1.0, target_yaw, target_pitch),
            start_time,
            duration,
        )
    }

    /// Samples current camera state at the given time `now`.
    /// Returns `(CameraSample, is_finished)`.
    pub fn sample(&self, now: Instant) -> (CameraSample, bool) {
        let elapsed = now.saturating_duration_since(self.start_time);
        if self.duration.is_zero() || elapsed >= self.duration {
            (
                CameraSample {
                    target: self.target_target,
                    distance: self.target_distance,
                    yaw: self.target_yaw,
                    pitch: self.target_pitch,
                },
                true,
            )
        } else {
            let t = (elapsed.as_secs_f32() / self.duration.as_secs_f32()).clamp(0.0, 1.0);
            let ease = 1.0 - (1.0 - t).powi(3);
            (
                CameraSample {
                    target: self.start_target.lerp(self.target_target, ease),
                    distance: self.start_distance
                        + (self.target_distance - self.start_distance) * ease,
                    yaw: self.start_yaw + (self.target_yaw - self.start_yaw) * ease,
                    pitch: self.start_pitch + (self.target_pitch - self.start_pitch) * ease,
                },
                false,
            )
        }
    }
}
