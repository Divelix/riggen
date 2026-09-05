//! The joint tree: View's left panel (ADR-0021 §1). One row per movable
//! joint in kinematic order — a child joint's row nested under its
//! parent's, fixed joints collapsed through — each a scrubber: the name
//! with the value at the right, and under them a bar the width of the
//! row with the limits at its ends and the value as a fill from the zero
//! mark. A drag moves the value across the bar (a tenth as fast with
//! Ctrl), the wheel steps it by the rotate ring's quantum
//! (`mode::wheel_step`, ADR-0019 §2), and neither is a command: `q` is
//! derived state, never saved (01 §The document is the only state).
//!
//! A joint that follows another (ADR-0013) is not draggable: its row is
//! dimmed, sits at the value `fk::resolve_q` derives, and carries the rule
//! under it. The panel never computes that value itself — the viewport and
//! the export read the same helper.

use riggen_core::{JointId, JointKind, Limits, LinkId};

use crate::app::mode::wheel_step;
use crate::app::{RiggenApp, Selection, fmt_num};

/// Transient state of the joint tree panel.
#[derive(Debug, Clone, Default)]
pub(crate) struct JointTreeState {
    /// A scrubber bar was under the pointer while the last frame was
    /// drawn, so this frame's wheel is the row's and not the panel's
    /// scroll. One frame late, like every hover-derived switch.
    bar_hovered: bool,
}

/// What a row asked for. Collected while drawing, applied after, so the
/// document is not written while the tree is being walked.
enum RowAction {
    Select(JointId),
    Hover(JointId),
    /// Move the joint to this value; clamped by `set_joint_value`.
    Set(JointId, f64),
}

/// The scrubber bar's range and unit for a joint kind, in the unit the
/// row shows: `(lower, upper, suffix)`. Continuous: a full turn either
/// way; unlimited otherwise: ±π / ±1 m, what the properties panel defaults
/// a new limit to.
fn bar_range(kind: JointKind, limits: Option<Limits>) -> (f64, f64, &'static str) {
    match kind {
        JointKind::Prismatic => {
            let (lo, hi) = limits.map_or((-1.0, 1.0), |l| (l.lower, l.upper));
            (lo, hi, " m")
        }
        JointKind::Continuous => (-180.0, 180.0, "°"),
        _ => {
            let (lo, hi) = limits.map_or((-std::f64::consts::PI, std::f64::consts::PI), |l| {
                (l.lower, l.upper)
            });
            (lo.to_degrees(), hi.to_degrees(), "°")
        }
    }
}

/// Document ↔ bar units: radians are shown in degrees, metres as they are.
fn to_bar(kind: JointKind, q: f64) -> f64 {
    match kind {
        JointKind::Prismatic => q,
        _ => q.to_degrees(),
    }
}

fn from_bar(kind: JointKind, v: f64) -> f64 {
    match kind {
        JointKind::Prismatic => v,
        _ => v.to_radians(),
    }
}

/// The bar's height.
const BAR_HEIGHT: f32 = 14.0;

/// The value as the row prints it: a tenth of a degree, a millimetre —
/// a pose readout, not a field to be typed into.
fn shown(kind: JointKind, v: f64) -> String {
    let unit = match kind {
        JointKind::Prismatic => 1e-3,
        _ => 0.1,
    };
    fmt_num((v / unit).round() * unit)
}

impl RiggenApp {
    /// View's left panel: the heading, "Reset all", and the rows from the
    /// root down.
    pub(crate) fn joint_tree_panel(&mut self, ui: &mut egui::Ui) {
        let mut actions = Vec::new();
        let mut reset = false;
        let mut bar_hovered = false;
        // The same panel id as the link tree, so a width the user chose
        // survives `Tab`.
        egui::Panel::left("tree_panel")
            .resizable(true)
            .default_size(240.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.heading("Joints");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .button("Reset all")
                            .on_hover_text("Every joint back to zero")
                            .clicked()
                        {
                            reset = true;
                        }
                    });
                });
                ui.separator();
                if !self.has_movable_joint() {
                    ui.weak("no movable joints");
                    return;
                }
                // The wheel over a bar steps the joint; the area must not
                // scroll under it as well.
                let scroll_source = egui::containers::scroll_area::ScrollSource {
                    mouse_wheel: !self.joint_tree.bar_hovered,
                    ..Default::default()
                };
                egui::ScrollArea::vertical()
                    .scroll_source(scroll_source)
                    .show(ui, |ui| {
                        let root = self.robot.root;
                        self.joint_rows_under(ui, root, &mut actions, &mut bar_hovered);
                        ui.allocate_space(egui::vec2(ui.available_width(), 16.0));
                    });
            });
        self.joint_tree.bar_hovered = bar_hovered;
        for action in actions {
            match action {
                RowAction::Select(joint) => self.select(Selection::Joint(joint)),
                RowAction::Hover(joint) => self.tree.hovered_joint = Some(joint),
                RowAction::Set(joint, q) => self.set_joint_value(joint, q),
            }
        }
        if reset {
            self.reset_joint_values();
        }
    }

    /// The rows for the joints under `link`, in joint id order: a movable
    /// joint is a row with its own subtree nested under it, a fixed joint
    /// is collapsed through — its child's joints appear at this level.
    fn joint_rows_under(
        &self,
        ui: &mut egui::Ui,
        link: LinkId,
        actions: &mut Vec<RowAction>,
        bar_hovered: &mut bool,
    ) {
        let joints: Vec<JointId> = self.robot.child_joints(link).collect();
        for joint in joints {
            let child = self.robot.joints[&joint].child;
            if !self.robot.joints[&joint].kind.is_movable() {
                self.joint_rows_under(ui, child, actions, bar_hovered);
                continue;
            }
            self.joint_row(ui, joint, actions, bar_hovered);
            // A chain reads as a chain: the joints below this one are
            // indented under it. A leaf has nothing to indent.
            if self.robot.subtree(child).iter().any(|l| {
                self.robot
                    .child_joints(*l)
                    .any(|j| self.robot.joints[&j].kind.is_movable())
            }) {
                ui.indent(("joint_tree", joint), |ui| {
                    self.joint_rows_under(ui, child, actions, bar_hovered);
                });
            }
        }
    }

    /// One row: the name (click selects, hover lights the glyph) with the
    /// value at the right, the bar under them, a follower's rule under
    /// that.
    fn joint_row(
        &self,
        ui: &mut egui::Ui,
        joint: JointId,
        actions: &mut Vec<RowAction>,
        bar_hovered: &mut bool,
    ) {
        let j = &self.robot.joints[&joint];
        let selected = self.selection == Selection::Joint(joint);
        let hovered = self.hovered_joint == Some(joint);
        let follower = j.mimic.is_some();
        let (lo, hi, suffix) = bar_range(j.kind, j.limits);
        // `joint_value` resolves a follower, so the row shows what the
        // viewport is showing.
        let value = to_bar(j.kind, self.joint_value(joint));
        ui.horizontal(|ui| {
            let mut text = egui::RichText::new(&j.name);
            // Hovered but not selected reads as "this one", not as a second
            // selection — the link tree's rule.
            if hovered && !selected {
                text = text.color(egui::Color32::from_rgb(255, 236, 179));
            }
            let name = ui.add(egui::Button::selectable(selected, text));
            if name.hovered() {
                actions.push(RowAction::Hover(joint));
            }
            if name.clicked() {
                actions.push(RowAction::Select(joint));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let mut number = egui::RichText::new(format!("{}{suffix}", shown(j.kind, value)));
                if follower {
                    number = number.weak();
                }
                ui.label(number);
            });
        });
        let sense = if follower {
            egui::Sense::hover()
        } else {
            egui::Sense::click_and_drag()
        };
        let width = ui.available_width();
        let (rect, response) = ui.allocate_exact_size(egui::vec2(width, BAR_HEIGHT), sense);
        response.widget_info(|| egui::WidgetInfo::slider(!follower, value, j.name.clone()));
        self.paint_bar(ui, rect, &response, (lo, hi, suffix), value, follower);
        if response.hovered() {
            actions.push(RowAction::Hover(joint));
            *bar_hovered = true;
        }
        if let Some(m) = &j.mimic {
            ui.indent(("mimic_rule", joint), |ui| {
                ui.weak(self.mimic_rule(m));
            });
            return;
        }
        if response.clicked() {
            actions.push(RowAction::Select(joint));
        }
        // A drag moves the value across the bar: the whole width is the
        // whole range, a tenth of that with Ctrl.
        if response.dragged() {
            let dx = f64::from(response.drag_delta().x);
            if dx != 0.0 {
                let fine = ui.input(|i| i.modifiers.ctrl);
                let per_point = (hi - lo) / f64::from(rect.width().max(1.0));
                let dv = dx * per_point * if fine { 0.1 } else { 1.0 };
                actions.push(RowAction::Set(joint, from_bar(j.kind, value + dv)));
            }
        }
        if response.hovered() {
            let (notches, fine) = crate::app::gizmo::wheel_notches(ui);
            if notches != 0 {
                let step = wheel_step(j.kind, j.limits, fine);
                let q = self.joint_value(joint) + f64::from(notches) * step;
                actions.push(RowAction::Set(joint, q));
            }
        }
    }

    /// The bar: the range as a trough, the value as a fill from the zero
    /// mark (or from the nearer end when zero is outside the limits), a
    /// tick at zero, the limits in small text at the ends.
    #[allow(clippy::too_many_arguments)]
    fn paint_bar(
        &self,
        ui: &egui::Ui,
        rect: egui::Rect,
        response: &egui::Response,
        (lo, hi, suffix): (f64, f64, &str),
        value: f64,
        follower: bool,
    ) {
        let visuals = ui.style().interact(response);
        let painter = ui.painter();
        let rounding = 3.0;
        painter.rect_filled(rect, rounding, visuals.bg_fill);
        let span = (hi - lo).max(f64::EPSILON);
        let x_of = |v: f64| {
            let t = ((v - lo) / span).clamp(0.0, 1.0) as f32;
            rect.left() + t * rect.width()
        };
        let zero = x_of(0.0);
        let at = x_of(value);
        let fill = egui::Rect::from_min_max(
            egui::pos2(zero.min(at), rect.top()),
            egui::pos2(zero.max(at), rect.bottom()),
        );
        let mut color = ui.visuals().selection.bg_fill;
        if follower {
            color = color.gamma_multiply(0.5);
        }
        painter.rect_filled(fill, 0.0, color);
        painter.vline(
            zero,
            rect.y_range(),
            egui::Stroke::new(1.0, ui.visuals().weak_text_color()),
        );
        painter.rect_stroke(rect, rounding, visuals.bg_stroke, egui::StrokeKind::Inside);
        let small = egui::FontId::proportional(9.0);
        let weak = ui.visuals().weak_text_color();
        painter.text(
            rect.left_center() + egui::vec2(3.0, 0.0),
            egui::Align2::LEFT_CENTER,
            format!("{}{suffix}", fmt_num(lo)),
            small.clone(),
            weak,
        );
        painter.text(
            rect.right_center() - egui::vec2(3.0, 0.0),
            egui::Align2::RIGHT_CENTER,
            format!("{}{suffix}", fmt_num(hi)),
            small,
            weak,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bar_ranges_by_kind() {
        let limits = |lower, upper| {
            Some(Limits {
                lower,
                upper,
                effort: 0.0,
                velocity: 0.0,
            })
        };
        let (lo, hi, unit) = bar_range(
            JointKind::Revolute,
            limits(-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2),
        );
        assert!((lo + 90.0).abs() < 1e-9 && (hi - 90.0).abs() < 1e-9);
        assert_eq!(unit, "°");
        assert_eq!(bar_range(JointKind::Continuous, None), (-180.0, 180.0, "°"));
        assert_eq!(
            bar_range(JointKind::Prismatic, limits(-0.2, 0.3)),
            (-0.2, 0.3, " m")
        );
        assert_eq!(bar_range(JointKind::Prismatic, None), (-1.0, 1.0, " m"));
        assert!(
            (from_bar(JointKind::Revolute, to_bar(JointKind::Revolute, 1.0)) - 1.0).abs() < 1e-12
        );
        assert_eq!(to_bar(JointKind::Prismatic, 0.25), 0.25);
    }

    #[test]
    fn readout_rounds_to_the_pose_unit() {
        assert_eq!(shown(JointKind::Revolute, -4.270421), "-4.3");
        assert_eq!(shown(JointKind::Revolute, 30.0), "30");
        assert_eq!(shown(JointKind::Prismatic, 0.12345), "0.123");
        assert_eq!(shown(JointKind::Prismatic, 0.0), "0");
    }
}
