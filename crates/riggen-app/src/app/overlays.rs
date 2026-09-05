//! The visibility row: five class toggles in the viewport's top-right
//! corner (docs/01-architecture.md §Panels and menus).
//!
//! Everything riggen draws over the robot used to be drawn always, and
//! `View › Collision geometry` was the one thing anybody could turn off —
//! a checkbox two clicks deep in a menu, which is the wrong distance from
//! the thing being decluttered. The row is Blender's overlay toggles for
//! riggen's things, beside the picture they act on.
//!
//! **A hidden thing answers nothing** (ADR-0021, amended): a toggle takes
//! away the drawing *and* the pointer target together. That is ADR-0020
//! §5's reasoning one layer up — a run behind geometry is dimmed rather
//! than dropped because a target the user can hit but cannot see is worse
//! than a visible one, and a joint the user has switched off that still
//! took their click would be exactly that.
//!
//! Visibility is app state, remembered through eframe storage and never in
//! the document: what a robot *is* does not depend on what the window is
//! showing (docs/02-data-model.md).

use super::RiggenApp;

/// One class of thing the viewport draws, and one button in the row.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Overlay {
    Joints,
    JointNames,
    Links,
    Frames,
    Collision,
}

impl Overlay {
    /// Left to right in the row: the two that are geometry last, so the
    /// three overlay toggles sit together.
    pub const ALL: [Self; 5] = [
        Self::Joints,
        Self::JointNames,
        Self::Frames,
        Self::Links,
        Self::Collision,
    ];

    /// What the toggle is called — in its tooltip, in the status bar's
    /// "… hidden" line and in `debug_state().ui.overlays`.
    pub fn name(self) -> &'static str {
        match self {
            Self::Joints => "joints",
            Self::JointNames => "joint names",
            Self::Frames => "frames",
            Self::Links => "links",
            Self::Collision => "collision",
        }
    }

    /// What it costs to switch off, said once in the tooltip so the ADR's
    /// rule is not something the user has to discover.
    fn tooltip(self) -> &'static str {
        match self {
            Self::Joints => "Joints — the glyphs, and in View the only thing the cursor can hit",
            Self::JointNames => "Joint names — the mimic and actuator labels, and frame names",
            Self::Frames => "Frames — the named triads (ADR-0012)",
            Self::Links => "Links — the visual meshes, which the Edit tools aim at",
            Self::Collision => "Collision geometry — the translucent hulls",
        }
    }

    /// Off by default only for collision: it is a check on a robot, not a
    /// part of it, and it was off in the menu it comes from.
    fn default_on(self) -> bool {
        self != Self::Collision
    }

    /// eframe storage key. One per toggle rather than one packed string,
    /// so a key added later defaults on its own.
    fn key(self) -> &'static str {
        match self {
            Self::Joints => "riggen.overlays.joints",
            Self::JointNames => "riggen.overlays.joint_names",
            Self::Frames => "riggen.overlays.frames",
            Self::Links => "riggen.overlays.links",
            Self::Collision => "riggen.overlays.collision",
        }
    }
}

/// What the viewport is currently drawing, class by class.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Overlays {
    pub joints: bool,
    pub joint_names: bool,
    pub frames: bool,
    pub links: bool,
    pub collision: bool,
}

impl Default for Overlays {
    fn default() -> Self {
        let mut overlays = Self {
            joints: false,
            joint_names: false,
            frames: false,
            links: false,
            collision: false,
        };
        for overlay in Overlay::ALL {
            overlays.set(overlay, overlay.default_on());
        }
        overlays
    }
}

impl Overlays {
    pub fn get(self, overlay: Overlay) -> bool {
        match overlay {
            Overlay::Joints => self.joints,
            Overlay::JointNames => self.joint_names,
            Overlay::Frames => self.frames,
            Overlay::Links => self.links,
            Overlay::Collision => self.collision,
        }
    }

    fn set(&mut self, overlay: Overlay, on: bool) {
        match overlay {
            Overlay::Joints => self.joints = on,
            Overlay::JointNames => self.joint_names = on,
            Overlay::Frames => self.frames = on,
            Overlay::Links => self.links = on,
            Overlay::Collision => self.collision = on,
        }
    }

    /// The names of what is switched off, in the row's order. What
    /// `debug_state()` reports and what the status bar reads out — a
    /// viewport that answers nothing is then a state the user can read
    /// rather than a fault they have to guess at.
    pub fn hidden(self) -> Vec<&'static str> {
        Overlay::ALL
            .into_iter()
            .filter(|o| !self.get(*o))
            .map(Overlay::name)
            .collect()
    }

    /// Read back from the last session; a key that is not there keeps the
    /// default, so a profile written before the row existed loads.
    pub(crate) fn load(storage: Option<&dyn eframe::Storage>) -> Self {
        let mut overlays = Self::default();
        let Some(storage) = storage else {
            return overlays;
        };
        for overlay in Overlay::ALL {
            if let Some(value) = storage.get_string(overlay.key()) {
                overlays.set(overlay, value == "true");
            }
        }
        overlays
    }

    pub(crate) fn save(self, storage: &mut dyn eframe::Storage) {
        for overlay in Overlay::ALL {
            storage.set_string(overlay.key(), self.get(overlay).to_string());
        }
    }
}

/// The side of one toggle button, in points.
const BUTTON: f32 = 20.0;
/// The margin between a mark and its button's edge.
const MARK_INSET: f32 = 4.0;

impl RiggenApp {
    pub fn overlays(&self) -> Overlays {
        self.overlays
    }

    /// Switch one class of thing on or off. `sync_scene` because two of
    /// the five are scene instances rather than overlay items; the other
    /// three are read where the glyphs are built.
    pub fn set_overlay(&mut self, overlay: Overlay, on: bool) {
        if self.overlays.get(overlay) != on {
            self.overlays.set(overlay, on);
            self.sync_scene();
        }
    }

    /// The row itself, in the viewport's top-right — the corner the Joints
    /// window vacated (ADR-0021). Drawn in both modes: what is on screen
    /// is not a question about which mode the window is in. Returns its
    /// rect, which joins the mode control's as **corner chrome**: the
    /// camera is blocked and the picks suppressed under both
    /// (01 §Picking and snapping).
    pub(crate) fn overlay_row(&mut self, ui: &mut egui::Ui, rect: egui::Rect) -> egui::Rect {
        const MARGIN: f32 = 8.0;
        let corner = egui::Rect::from_min_max(
            egui::pos2(rect.min.x, rect.min.y + MARGIN),
            egui::pos2(rect.max.x - MARGIN, rect.max.y),
        );
        let mut toggled = None;
        let response = ui.scope_builder(
            egui::UiBuilder::new()
                .max_rect(corner)
                .layout(egui::Layout::right_to_left(egui::Align::TOP)),
            |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        // Reversed because the row is laid out from the
                        // right edge — the corner it lives in is the right
                        // one, and a layout that starts there hands its
                        // children out right to left. `ALL` stays in
                        // reading order, which is what `hidden()` reports.
                        for overlay in Overlay::ALL.into_iter().rev() {
                            if self.toggle(ui, overlay) {
                                toggled = Some(overlay);
                            }
                        }
                    });
                });
            },
        );
        if let Some(overlay) = toggled {
            let on = !self.overlays.get(overlay);
            self.set_overlay(overlay, on);
        }
        response.response.rect
    }

    /// One button: the mark, lit when its class is drawn and dimmed when
    /// it is not. Returns whether it was clicked.
    fn toggle(&self, ui: &mut egui::Ui, overlay: Overlay) -> bool {
        let on = self.overlays.get(overlay);
        let (rect, response) =
            ui.allocate_exact_size(egui::Vec2::splat(BUTTON), egui::Sense::click());
        // Named, so the harness can click one by its label and a screen
        // reader has something to say (`get_by_label`).
        response.widget_info(|| {
            egui::WidgetInfo::selected(egui::WidgetType::Checkbox, true, on, overlay.name())
        });
        if ui.is_rect_visible(rect) {
            let visuals = ui.style().interact_selectable(&response, on);
            ui.painter()
                .rect_filled(rect, visuals.corner_radius, visuals.weak_bg_fill);
            let color = if on {
                visuals.fg_stroke.color
            } else {
                ui.visuals().weak_text_color()
            };
            paint_mark(ui.painter(), rect.shrink(MARK_INSET), overlay, color);
        }
        response.on_hover_text(overlay.tooltip()).clicked()
    }
}

/// The five marks, drawn rather than typed: egui bundles no icon set worth
/// the name, and each of these is the thing it switches as the viewport
/// draws it — a band and its spoke, a triad, a box, a hull round a box.
/// `joint names` is the exception and is simply an `A`: a name has no
/// shape of its own.
fn paint_mark(painter: &egui::Painter, rect: egui::Rect, overlay: Overlay, color: egui::Color32) {
    let stroke = egui::Stroke::new(1.2, color);
    let center = rect.center();
    match overlay {
        Overlay::Joints => {
            let radius = rect.width() * 0.42;
            painter.circle_stroke(center, radius, stroke);
            painter.line_segment([center, center + egui::vec2(radius * 1.6, 0.0)], stroke);
        }
        Overlay::JointNames => {
            painter.text(
                center,
                egui::Align2::CENTER_CENTER,
                "A",
                egui::FontId::proportional(rect.height()),
                color,
            );
        }
        Overlay::Frames => {
            // The link tree's own frame mark (`⌖ tcp`, ADR-0012): a triad
            // drawn at twelve points reads as an arrow, and the app
            // already has a character for this.
            painter.text(
                center,
                egui::Align2::CENTER_CENTER,
                "\u{2316}",
                egui::FontId::proportional(rect.height() * 1.2),
                color,
            );
        }
        Overlay::Links => {
            painter.rect_filled(rect.shrink(1.0), 1.0, color);
        }
        Overlay::Collision => {
            // A hull round a part: the box, and the shape that wraps it.
            painter.rect_filled(rect.shrink(3.0), 1.0, color.gamma_multiply(0.55));
            painter.rect_stroke(rect.shrink(0.5), 2.0, stroke, egui::StrokeKind::Inside);
        }
    }
}

impl RiggenApp {
    /// What the status bar says about the row: the classes the user has
    /// switched off, so a viewport with nothing in it — or a View that
    /// answers nothing because its joints are hidden (ADR-0021, amended)
    /// — is a state that can be read rather than a fault to guess at.
    ///
    /// Collision alone is the default and is not news; the note is for
    /// something that was turned off on purpose.
    pub(crate) fn hidden_note(&self) -> Option<String> {
        let hidden = self.overlays.hidden();
        if hidden.is_empty() || hidden == [Overlay::Collision.name()] {
            return None;
        }
        Some(format!("hidden: {}", hidden.join(", ")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_everything_but_collision() {
        let overlays = Overlays::default();
        for overlay in Overlay::ALL {
            assert_eq!(overlays.get(overlay), overlay.default_on(), "{overlay:?}");
        }
        assert_eq!(overlays.hidden(), vec!["collision"]);
    }

    #[test]
    fn hidden_lists_what_is_off_in_the_rows_order() {
        let mut overlays = Overlays::default();
        overlays.set(Overlay::Collision, true);
        assert!(overlays.hidden().is_empty());
        overlays.set(Overlay::Links, false);
        overlays.set(Overlay::Joints, false);
        assert_eq!(overlays.hidden(), vec!["joints", "links"]);
    }

    #[test]
    fn every_toggle_has_its_own_storage_key() {
        let mut keys: Vec<&str> = Overlay::ALL.iter().map(|o| o.key()).collect();
        keys.sort_unstable();
        keys.dedup();
        assert_eq!(keys.len(), Overlay::ALL.len());
    }
}
