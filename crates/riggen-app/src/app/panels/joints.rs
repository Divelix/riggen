//! The joint sliders window: one slider per movable joint, bounded by its
//! limits (Continuous: ±180°), degrees for revolute kinds and meters for
//! prismatic. Dragging writes `q` and re-syncs the scene every frame — a
//! joint value is derived state, never a command and never saved
//! (docs/01-architecture.md §The document is the only state).
//!
//! A joint that follows another (ADR-0013) is not draggable: its slider is
//! disabled, sits at the value `fk::resolve_q` derives, and carries the
//! rule underneath it. The window never computes that value itself — the
//! viewport and the export read the same helper.

use riggen_core::{JointId, JointKind, Limits, Mimic};

use crate::app::RiggenApp;

#[derive(Debug, Clone, Default)]
pub(crate) struct JointsWindow {
    pub(crate) open: bool,
}

/// One row of the window, read off the document before it is drawn.
type Row = (JointId, String, JointKind, Option<Limits>, Option<Mimic>);

/// A slider's range and unit for a joint kind: `(lower, upper, suffix)`
/// in the slider's unit, with the document ↔ slider conversions.
fn slider_range(kind: JointKind, limits: Option<Limits>) -> (f64, f64, &'static str) {
    match kind {
        JointKind::Prismatic => {
            let l = limits.unwrap_or(Limits {
                lower: -1.0,
                upper: 1.0,
                effort: 0.0,
                velocity: 0.0,
            });
            (l.lower, l.upper, " m")
        }
        JointKind::Continuous => (-180.0, 180.0, "°"),
        _ => {
            let l = limits.unwrap_or(Limits {
                lower: -std::f64::consts::PI,
                upper: std::f64::consts::PI,
                effort: 0.0,
                velocity: 0.0,
            });
            (l.lower.to_degrees(), l.upper.to_degrees(), "°")
        }
    }
}

fn to_slider(kind: JointKind, q: f64) -> f64 {
    match kind {
        JointKind::Prismatic => q,
        _ => q.to_degrees(),
    }
}

/// Six decimals, trailing zeros dropped, no `-0` — the same shape the
/// properties panel writes a number in.
fn num(v: f64) -> String {
    let r = (v * 1e6).round() / 1e6;
    format!("{}", if r == 0.0 { 0.0 } else { r })
}

fn from_slider(kind: JointKind, v: f64) -> f64 {
    match kind {
        JointKind::Prismatic => v,
        _ => v.to_radians(),
    }
}

impl RiggenApp {
    pub fn joints_window_open(&self) -> bool {
        self.joints_window.open
    }

    pub fn set_joints_window_open(&mut self, open: bool) {
        self.joints_window.open = open;
    }

    /// `= -0.5 × upper_joint + 0.1`, the rule as the document holds it —
    /// radians or meters, the units `Mimic` is in, not the slider's
    /// (ADR-0013). A zero offset is left off rather than written `+ 0`.
    fn mimic_rule(&self, m: &Mimic) -> String {
        let leader = self
            .robot
            .joints
            .get(&m.joint)
            .map_or_else(|| m.joint.to_string(), |j| j.name.clone());
        let rule = format!("= {} × {leader}", num(m.multiplier));
        match m.offset {
            0.0 => rule,
            o if o < 0.0 => format!("{rule} - {}", num(-o)),
            o => format!("{rule} + {}", num(o)),
        }
    }

    /// Draws the window if it is open. Called after the panels so it
    /// floats over the viewport.
    pub(crate) fn joints_window(&mut self, ctx: &egui::Context) {
        if !self.joints_window.open {
            return;
        }
        let movable: Vec<Row> = self
            .robot
            .joints
            .iter()
            .filter(|(_, j)| j.kind.is_movable())
            .map(|(&id, j)| (id, j.name.clone(), j.kind, j.limits, j.mimic))
            .collect();

        // Top-right of the viewport, out of the way of the tree and the
        // properties panel; the user can drag it anywhere from there.
        let anchor = self
            .viewport
            .viewport_rect()
            .unwrap_or_else(|| ctx.content_rect());
        let mut open = self.joints_window.open;
        let mut changes: Vec<(JointId, f64)> = Vec::new();
        let mut reset = false;
        egui::Window::new("Joints")
            .open(&mut open)
            .default_pos(egui::pos2(anchor.right() - 336.0, anchor.top() + 16.0))
            .default_width(300.0)
            .resizable(false)
            .show(ctx, |ui| {
                if movable.is_empty() {
                    ui.weak("no movable joints");
                    return;
                }
                egui::Grid::new("joint_sliders")
                    .num_columns(2)
                    .show(ui, |ui| {
                        for (id, name, kind, limits, mimic) in &movable {
                            ui.label(name);
                            let (lo, hi, suffix) = slider_range(*kind, *limits);
                            // `joint_value` resolves a follower, so the
                            // slider shows what the viewport is showing.
                            let mut v = to_slider(*kind, self.joint_value(*id));
                            let slider = egui::Slider::new(&mut v, lo..=hi)
                                .suffix(suffix)
                                .fixed_decimals(1);
                            match mimic {
                                None => {
                                    if ui.add(slider).changed() {
                                        changes.push((*id, from_slider(*kind, v)));
                                    }
                                }
                                Some(m) => {
                                    // A read-out, not a control: the value
                                    // is its leader's to set.
                                    ui.vertical(|ui| {
                                        ui.add_enabled(false, slider);
                                        ui.weak(self.mimic_rule(m));
                                    });
                                }
                            }
                            ui.end_row();
                        }
                    });
                ui.separator();
                if ui.button("Reset all").clicked() {
                    reset = true;
                }
            });
        self.joints_window.open = open;
        for (id, q) in changes {
            self.set_joint_value(id, q);
        }
        if reset {
            self.reset_joint_values();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ranges_by_kind() {
        let limits = Some(Limits {
            lower: -std::f64::consts::FRAC_PI_2,
            upper: std::f64::consts::FRAC_PI_2,
            effort: 0.0,
            velocity: 0.0,
        });
        let (lo, hi, unit) = slider_range(JointKind::Revolute, limits);
        assert!((lo + 90.0).abs() < 1e-9 && (hi - 90.0).abs() < 1e-9);
        assert_eq!(unit, "°");
        assert_eq!(
            slider_range(JointKind::Continuous, None),
            (-180.0, 180.0, "°")
        );
        let (lo, hi, unit) = slider_range(
            JointKind::Prismatic,
            Some(Limits {
                lower: -0.2,
                upper: 0.3,
                effort: 0.0,
                velocity: 0.0,
            }),
        );
        assert_eq!((lo, hi, unit), (-0.2, 0.3, " m"));
        assert!(
            (from_slider(JointKind::Revolute, to_slider(JointKind::Revolute, 1.0)) - 1.0).abs()
                < 1e-12
        );
        assert_eq!(to_slider(JointKind::Prismatic, 0.25), 0.25);
    }
}
