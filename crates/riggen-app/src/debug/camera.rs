//! Where the camera is, and the matrices everything is projected by.

use serde::Serialize;

use super::{round, round32};
use crate::app::RiggenApp;

/// The orbit camera's state plus the two matrices derived from it.
///
/// The matrices are here because an overlay-projection bug is usually a
/// disagreement between what the wgpu pass drew and what an egui-painter
/// overlay computed — and both start from these.
#[derive(Debug, Clone, Serialize)]
pub struct CameraDebug {
    pub eye: [f64; 3],
    pub target: [f64; 3],
    /// The `up` of `OrbitCamera::basis`, i.e. after the pole heuristic, not
    /// the raw world Z.
    pub up: [f64; 3],
    pub distance: f64,
    pub yaw_deg: f64,
    pub pitch_deg: f64,
    pub fov_y_deg: f64,
    pub projection: &'static str,
    /// `true` while a view transition is in flight. A snapshot taken then is
    /// not reproducible — the animation reads the wall clock — so scenarios
    /// avoid it and this field is how one would notice.
    pub animating: bool,
    /// Aspect the projection matrix below was built with. `None` before the
    /// viewport has been laid out once.
    pub aspect: Option<f64>,
    /// Columns: `view[0]` is the first column of the column-major matrix,
    /// i.e. `glam::Mat4::to_cols_array_2d`.
    pub view: [[f64; 4]; 4],
    /// `None` until there is an aspect to build the projection from.
    pub proj: Option<[[f64; 4]; 4]>,
}

impl CameraDebug {
    pub(super) fn capture(app: &RiggenApp) -> Self {
        let camera = &app.viewport.camera;
        let eye = camera.eye();
        let (_, _, up) = camera.basis();
        let aspect = app
            .viewport
            .viewport_rect()
            .and_then(|rect| (rect.height() > 0.0).then(|| (rect.width() / rect.height()) as f64));

        Self {
            eye: [round32(eye.x), round32(eye.y), round32(eye.z)],
            target: [
                round32(camera.target.x),
                round32(camera.target.y),
                round32(camera.target.z),
            ],
            up: [round32(up.x), round32(up.y), round32(up.z)],
            distance: round32(camera.distance),
            yaw_deg: round(camera.yaw.to_degrees() as f64),
            pitch_deg: round(camera.pitch.to_degrees() as f64),
            fov_y_deg: round(camera.fov_y.to_degrees() as f64),
            projection: camera.projection.label(),
            animating: camera.is_animating(),
            aspect: aspect.map(round),
            view: matrix(camera.view_matrix()),
            proj: aspect.map(|a| matrix(camera.proj_matrix(a as f32))),
        }
    }
}

fn matrix(m: riggen_mesh::glam::Mat4) -> [[f64; 4]; 4] {
    m.to_cols_array_2d().map(|col| col.map(round32))
}
