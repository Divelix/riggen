//! The window's two modes (ADR-0021): **View** is the posed robot, **Edit**
//! is the v0.3 editor at the zero configuration. `Tab` switches them
//! (`shortcuts.rs`).
//!
//! And **zen**, which is orthogonal to both: `Z` hides every panel and
//! both pieces of corner chrome, leaving the viewport filling the window
//! with the robot alone. It lives here because it is the same policy the
//! mode is — which panels are drawn — and is read in the same places. It
//! changes nothing else: not the mode, not the visibility row's toggles,
//! not one switch of the table (ADR-0021, amended).

use riggen_core::{JointKind, Limits};
use riggen_viewport::ViewOrientation;

use super::gizmo::{WHEEL_STEP, WHEEL_STEP_FINE, wheel_notches};
use super::panels::STEP_M;
use super::viewcube;
use super::{RiggenApp, Selection, Tool};

/// What the status bar says when a tool key is pressed in View: the tools
/// are Edit's, and the key does nothing else (ADR-0021 §1). Public so a
/// test asserts on the constant, not on prose.
pub const VIEW_TOOL_HINT: &str = "the tools are Edit's — Tab to edit";

/// A prismatic joint's wheel quantum, as a fraction of its travel: the
/// ring's 5° is 1.4 % of a turn, and a slide has no turn to take a
/// fraction of, so a notch is one percent of the way from one limit to the
/// other, never less than the properties panel's metre floor, and a tenth
/// of that with shift (plans/view-edit-modes OPEN 1).
const PRISMATIC_TRAVEL_FRACTION: f64 = 0.01;

/// What one wheel notch adds to a joint's `q` in View: 5° (1° with shift)
/// for a hinge, a fraction of the travel for a slide.
pub(crate) fn wheel_step(kind: JointKind, limits: Option<Limits>, fine: bool) -> f64 {
    match kind {
        JointKind::Prismatic => {
            let travel = limits.map_or(2.0, |l| (l.upper - l.lower).abs());
            let step = (travel * PRISMATIC_TRAVEL_FRACTION).max(STEP_M);
            if fine { step / 10.0 } else { step }
        }
        _ => (if fine { WHEEL_STEP_FINE } else { WHEEL_STEP }).to_radians(),
    }
}

/// Which of the two windows the user is looking at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// The joint tree, the glyphs alone under the cursor, the wheel posing
    /// a hovered joint.
    View,
    /// The link tree, the properties panel, the toolbar and the gizmos.
    Edit,
}

impl Mode {
    /// The name the control, the status bar and `debug_state` use.
    pub fn label(self) -> &'static str {
        match self {
            Mode::View => "View",
            Mode::Edit => "Edit",
        }
    }

    /// What `Tab` switches to.
    pub fn other(self) -> Mode {
        match self {
            Mode::View => Mode::Edit,
            Mode::Edit => Mode::View,
        }
    }
}

impl RiggenApp {
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Whether the window is in **zen**: every panel and both pieces of
    /// corner chrome hidden, the viewport filling the window with the
    /// robot alone (ADR-0021, amended).
    pub fn zen(&self) -> bool {
        self.zen
    }

    /// Enter or leave zen. Orthogonal to the mode: nothing else about the
    /// window changes, so there is no scene to sync and no selection to
    /// clear — only which panels `ui` draws. Never persisted.
    pub fn set_zen(&mut self, zen: bool) {
        self.zen = zen;
    }

    /// What `Z` does.
    pub fn toggle_zen(&mut self) {
        self.zen = !self.zen;
    }

    /// Switches modes. Edit is the zero configuration (ADR-0021 §2): going
    /// there stashes `q` and rewinds, coming back restores it — a rewind,
    /// not an edit, so no history entry either way. A selected joint is a
    /// thing in both modes and survives; a selected link or frame has no
    /// row in View and clears.
    pub fn set_mode(&mut self, mode: Mode) {
        if mode == self.mode {
            return;
        }
        self.mode = mode;
        match mode {
            Mode::Edit => {
                self.stashed_q = Some(std::mem::take(&mut self.q));
            }
            Mode::View => {
                if let Some(q) = self.stashed_q.take() {
                    self.q = q;
                    // Edit may have removed a joint or moved its limits.
                    self.clamp_q_to_document();
                }
                // The tools are Edit's: no gizmo, no snap, nothing for Esc
                // to leave.
                self.set_tool(Tool::Select);
            }
        }
        self.sync_scene();
        if matches!(self.selection, Selection::Link(_) | Selection::Frame(_)) {
            self.select(Selection::None);
        }
    }
}

impl RiggenApp {
    /// The corner chrome: the `View | Edit` control in the viewport's
    /// top-left in both modes with `Tab` in its tooltip, the toolbar to its
    /// right in Edit, the **visibility row** at the top-right
    /// (`overlays.rs`), and the **ViewCube** at the bottom-right
    /// (`viewcube/`, ADR-0028 §3). Drawn after the viewport in the same
    /// layer so egui's hit test gives it the pointer (`tool.rs`). Records
    /// all three rects in `chrome_rects`: camera blocked and picks
    /// suppressed under any of them, no glyph hovered through them. Not
    /// called at all in zen, where `chrome_rects` is cleared instead:
    /// nothing is drawn there, so nothing may go on blocking (ADR-0021,
    /// amended) — and the cube being one of the three is why zen has no
    /// projection readout at all (ADR-0028 §5).
    pub(crate) fn viewport_chrome(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        const MARGIN: f32 = 8.0;
        let corner = egui::Rect::from_min_max(rect.min + egui::Vec2::splat(MARGIN), rect.max);
        let mut chosen_mode = None;
        let mut chosen_tool = None;
        let response = ui.scope_builder(
            egui::UiBuilder::new()
                .max_rect(corner)
                .layout(egui::Layout::left_to_right(egui::Align::TOP)),
            |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        for mode in [Mode::View, Mode::Edit] {
                            if ui
                                .selectable_label(self.mode == mode, mode.label())
                                .on_hover_text(format!("{} (Tab)", mode.label()))
                                .clicked()
                            {
                                chosen_mode = Some(mode);
                            }
                        }
                    });
                });
                if self.mode == Mode::Edit {
                    chosen_tool = self.tool_bar(ui);
                }
            },
        );
        self.chrome_rects = vec![
            response.response.rect,
            self.overlay_row(ui, rect),
            self.view_cube(ui, rect),
        ];
        if let Some(mode) = chosen_mode {
            self.set_mode(mode);
        }
        if let Some(tool) = chosen_tool {
            self.set_tool(tool);
        }
    }

    /// The ViewCube in the bottom-right corner, and the camera call its
    /// action makes (ADR-0028 §3). Returns the rect it occupies — the cube
    /// plus its projection button — for `chrome_rects`.
    ///
    /// Bottom-right because top-right is the visibility row's and the only
    /// thing this corner held was the `persp` / `ortho` text the cube's own
    /// button now *is*. The block is laid out upwards from the bottom
    /// margin so the button, which hangs below the cube, stays inside the
    /// viewport.
    fn view_cube(&mut self, ui: &mut egui::Ui, rect: egui::Rect) -> egui::Rect {
        const MARGIN: f32 = 8.0;
        const SIZE: f32 = 92.0;
        let probe = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::Vec2::splat(SIZE));
        let block_height = SIZE + (viewcube::projection_button_rect(probe).max.y - probe.max.y);
        let cube_rect = egui::Rect::from_min_size(
            egui::pos2(
                rect.max.x - MARGIN - SIZE,
                rect.max.y - MARGIN - block_height,
            ),
            egui::Vec2::splat(SIZE),
        );
        self.viewcube_rect = Some(cube_rect);

        let camera = &self.viewport.camera;
        let out = viewcube::viewcube(ui, cube_rect, camera.yaw, camera.pitch, camera.projection);
        match out.action {
            Some(viewcube::ViewCubeAction::Select(orientation)) => {
                self.viewport.camera.animate_to_orientation(orientation);
            }
            Some(viewcube::ViewCubeAction::Orbit {
                delta_yaw,
                delta_pitch,
            }) => {
                self.viewport.camera.orbit(delta_yaw, delta_pitch);
            }
            Some(viewcube::ViewCubeAction::Home) => {
                self.viewport.animate_frame_scene();
            }
            Some(viewcube::ViewCubeAction::ToggleProjection) => {
                self.viewport.camera.toggle_projection();
            }
            None => {}
        }
        if out.action.is_some() {
            ui.ctx().request_repaint();
        }
        out.rect
    }

    /// Where a ViewCube facet is on screen, for a test that wants to click
    /// one — the analogue of `project_world` for the cube. `None` in zen,
    /// where the cube is not drawn, and for a facet the current view has
    /// culled.
    pub fn viewcube_facet_center(&self, orientation: ViewOrientation) -> Option<egui::Pos2> {
        let rect = self.viewcube_rect?;
        let camera = &self.viewport.camera;
        viewcube::project_viewcube(rect, camera.yaw, camera.pitch)
            .into_iter()
            .find(|facet| facet.orientation == orientation)
            .map(|facet| facet.center_2d)
    }

    /// In View, the wheel over a hovered glyph poses that joint — a notch
    /// at the rotate ring's quantum (ADR-0021 §1, ADR-0019 §2). The
    /// viewport has already been told not to zoom (`set_wheel_claimed`),
    /// so the notches are ours to read. Not a command: `q` is derived
    /// state, so there is no history entry and no burst to coalesce.
    pub(crate) fn step_hovered_joint_with_wheel(&mut self, ui: &egui::Ui) {
        if self.mode != Mode::View {
            return;
        }
        let Some(joint) = self.glyph_hover else {
            return;
        };
        let (notches, fine) = wheel_notches(ui);
        if notches == 0 {
            return;
        }
        let Some(j) = self.robot.joints.get(&joint) else {
            return;
        };
        // A follower's value is its leader's to set (ADR-0013).
        if j.mimic.is_some() {
            return;
        }
        let step = wheel_step(j.kind, j.limits, fine);
        let q = self.joint_value(joint) + f64::from(notches) * step;
        self.set_joint_value(joint, q);
        ui.ctx().request_repaint();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wheel_quantum_by_kind() {
        let deg = |d: f64| d.to_radians();
        assert!((wheel_step(JointKind::Revolute, None, false) - deg(5.0)).abs() < 1e-12);
        assert!((wheel_step(JointKind::Continuous, None, true) - deg(1.0)).abs() < 1e-12);
        let limits = |lower, upper| {
            Some(Limits {
                lower,
                upper,
                effort: 0.0,
                velocity: 0.0,
            })
        };
        // One percent of the travel, and a tenth of that with shift.
        assert!((wheel_step(JointKind::Prismatic, limits(-0.2, 0.3), false) - 0.005).abs() < 1e-12);
        assert!((wheel_step(JointKind::Prismatic, limits(-0.2, 0.3), true) - 0.0005).abs() < 1e-12);
        // Never under the metre floor: a 5 cm slide still steps a millimetre.
        assert_eq!(
            wheel_step(JointKind::Prismatic, limits(0.0, 0.05), false),
            STEP_M
        );
        // No limits: the ±1 m the slider assumes.
        assert!((wheel_step(JointKind::Prismatic, None, false) - 0.02).abs() < 1e-12);
    }
}
