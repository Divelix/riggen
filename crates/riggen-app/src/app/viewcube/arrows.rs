//! The four step arrows on a ring around the cube: up, down, left and
//! right, each turning the view [`ARROW_STEP`] that way
//! (plans/viewcube-corner-axes).
//!
//! Layout, hit test and the camera call, as pure functions; the widget
//! paints and reads the pointer. Onshape's two curved arrows are not here:
//! they roll the view, which ADR-0028 §1 refuses.

use std::f32::consts::FRAC_PI_2;

use riggen_viewport::OrbitCamera;

use super::widget::drag_orbit;

/// What one click on an arrow turns the view by.
pub const ARROW_STEP: f32 = 15.0 * std::f32::consts::PI / 180.0;
/// The side of the square an arrow's triangle is drawn in, in points.
pub const ARROW_SIZE: f32 = 12.0;
/// From the cube rect's inscribed circle out to an arrow's centre, in
/// points: past the farthest an arm's letter reaches at any orientation.
pub const ARROW_RING_GAP: f32 = 28.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepArrow {
    Up,
    Right,
    Down,
    Left,
}

impl StepArrow {
    /// Clockwise from 12 o'clock.
    pub const ALL: [Self; 4] = [Self::Up, Self::Right, Self::Down, Self::Left];

    /// The unit screen direction the arrow points in, egui's +y down.
    pub fn direction(self) -> egui::Vec2 {
        match self {
            Self::Up => -egui::Vec2::Y,
            Self::Right => egui::Vec2::X,
            Self::Down => egui::Vec2::Y,
            Self::Left => -egui::Vec2::X,
        }
    }

    /// `(delta_yaw, delta_pitch)` in radians: the way a drag of the cube
    /// towards the arrow turns it, so arrows and drag cannot disagree.
    pub fn delta(self) -> (f32, f32) {
        let (yaw, pitch) = drag_orbit(self.direction());
        let step = |drag: f32| {
            if drag == 0.0 {
                0.0
            } else {
                drag.signum() * ARROW_STEP
            }
        };
        (step(yaw), step(pitch))
    }

    pub fn tooltip(self) -> &'static str {
        match self {
            Self::Up => "Turn the view 15° up",
            Self::Right => "Turn the view 15° right",
            Self::Down => "Turn the view 15° down",
            Self::Left => "Turn the view 15° left",
        }
    }
}

/// Where the arrows sit around a cube drawn in `cube_rect`: on a ring just
/// outside its inscribed circle, at 12, 3, 6 and 9 o'clock.
pub fn arrow_rects(cube_rect: egui::Rect) -> [(StepArrow, egui::Rect); 4] {
    let radius = cube_rect.width().min(cube_rect.height()) * 0.5 + ARROW_RING_GAP;
    StepArrow::ALL.map(|arrow| {
        let center = cube_rect.center() + arrow.direction() * radius;
        (
            arrow,
            egui::Rect::from_center_size(center, egui::Vec2::splat(ARROW_SIZE)),
        )
    })
}

/// The arrow under `pos`, if any.
pub fn hit_test_arrows(cube_rect: egui::Rect, pos: egui::Pos2) -> Option<StepArrow> {
    arrow_rects(cube_rect)
        .into_iter()
        .find(|(_, rect)| rect.contains(pos))
        .map(|(arrow, _)| arrow)
}

/// The isosceles triangle an arrow is drawn as, pointing outward.
pub fn arrow_triangle(arrow: StepArrow, rect: egui::Rect) -> [egui::Pos2; 3] {
    let out = arrow.direction();
    let side = out.rot90();
    let c = rect.center();
    let half = rect.width() * 0.5;
    [
        c + out * (half * 0.6),
        c - out * (half * 0.5) + side * half,
        c - out * (half * 0.5) - side * half,
    ]
}

/// Flies `camera` by one arrow's step from wherever it is now, a running
/// animation included. Pitch stops at the face views' ±90°, so turning
/// past a pole is a no-op rather than a flip.
pub fn step_camera(camera: &mut OrbitCamera, delta_yaw: f32, delta_pitch: f32) {
    camera.animate_to(
        camera.yaw + delta_yaw,
        (camera.pitch + delta_pitch).clamp(-FRAC_PI_2, FRAC_PI_2),
    );
}
