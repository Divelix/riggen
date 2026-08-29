//! What the app thinks it drew, as inspectable data (ADR-0003).
//!
//! The 3D viewport is one wgpu paint callback, so it contributes no AccessKit
//! nodes: a harness can photograph it but cannot ask it anything. This module
//! is the text side of that pair — a snapshot scenario writes a PNG *and*
//! this JSON, and a mismatch between the two localises the bug immediately.
//!
//! It is also the small public surface the out-of-crate snapshot suite needs
//! (`tests/visual/`), since the app's own fields are `pub(crate)`.
//!
//! **Every float is rounded** ([`round`]) before it is serialised. The JSON is
//! a committed golden, and an unrounded `f32`-to-`f64` widening churns in the
//! last digits between runs for reasons that have nothing to do with the code
//! under test.

use serde::Serialize;

use crate::app::RiggenApp;

/// Decimal places every serialised float is rounded to.
///
/// Internal units are meters (AGENTS.md), so six places is a micrometre — far
/// below anything a modelling or projection bug hides in, and far above the
/// noise floor that would make a golden churn. Screen coordinates are points,
/// where it is well under a pixel.
const PRECISION: i32 = 6;

/// Rounds to [`PRECISION`] decimal places. See the module doc for why.
pub fn round(x: f64) -> f64 {
    let scale = 10f64.powi(PRECISION);
    let r = (x * scale).round() / scale;
    // `-0.0` and `0.0` serialise differently and compare equal, which would
    // make a golden flip for no reason.
    if r == 0.0 { 0.0 } else { r }
}

/// Rounds an `f32` the same way, widening first.
pub fn round32(x: f32) -> f64 {
    round(x as f64)
}

/// A whole frame's worth of app state, as JSON. Grows a section per
/// milestone: `camera` and `instances` with the viewport (M0 step 8),
/// `selection` with picking (step 9).
#[derive(Debug, Clone, Serialize)]
pub struct DebugState {
    /// `[min_x, min_y, max_x, max_y]` of the viewport in egui logical points.
    /// `None` before the first frame has laid it out.
    pub viewport_rect: Option<[f64; 4]>,
}

impl RiggenApp {
    /// Snapshot of everything this module reports. Cheap enough to call per
    /// frame; the snapshot suite calls it once per scenario.
    pub fn debug_state(&self) -> DebugState {
        DebugState {
            viewport_rect: self.central_rect.map(|rect| {
                [
                    round32(rect.min.x),
                    round32(rect.min.y),
                    round32(rect.max.x),
                    round32(rect.max.y),
                ]
            }),
        }
    }

    /// [`Self::debug_state`] as pretty-printed JSON — what the snapshot
    /// goldens hold.
    pub fn debug_state_json(&self) -> String {
        serde_json::to_string_pretty(&self.debug_state())
            .unwrap_or_else(|err| format!("{{\"error\": \"{err}\"}}"))
    }

    /// Whether the status bar shows the frame-time readout.
    ///
    /// The snapshot suite turns it off: it reads the wall clock, so it
    /// differs on every frame.
    pub fn set_frame_hud_visible(&mut self, visible: bool) {
        self.show_frame_hud = visible;
    }

    /// Whether a snapshot taken now is reproducible: no pick readback in
    /// flight and no camera animation reading the wall clock. The harness
    /// pumps frames until this has held for a few in a row. Trivially true
    /// until the viewport lands (step 8).
    pub fn settled(&self) -> bool {
        true
    }

    /// Centre of the viewport rect — where a scenario aims a hover or a
    /// click. `None` before the first frame has laid it out.
    pub fn viewport_center(&self) -> Option<egui::Pos2> {
        self.central_rect.map(|rect| rect.center())
    }
}
