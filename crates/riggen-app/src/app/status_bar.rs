//! Bottom status bar: `riggen | units: file | hover: i3/t120 | selected: … |
//! 4.10 ms (244 fps)` (docs/03-roadmap.md §M0).
//!
//! `hovered`/`selected` reflect the *previous* frame's viewport state — this
//! panel is drawn before the viewport so the central panel gets whatever
//! screen space is left (egui lays out side/bottom panels before the central
//! one). One frame of lag on a status readout is imperceptible.

/// Everything the bar shows, pre-formatted by the app so this module never
/// names viewport types.
pub(crate) struct StatusView<'a> {
    /// `i3/t120`-style readout of the hovered instance and triangle.
    pub hovered: Option<&'a str>,
    pub selected: Option<&'a str>,
    /// A one-off message — a load error, an export destination.
    pub message: Option<&'a str>,
    /// Seconds between the last two frames; `None` hides the readout (the
    /// snapshot suite, or the first frame).
    pub frame_dt: Option<f32>,
}

pub(crate) fn status_bar(ui: &mut egui::Ui, view: &StatusView<'_>) {
    egui::Panel::bottom("status_bar").show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label("riggen");
            ui.separator();
            // M0 shows dropped files in file units as-is; M1's `MeshAsset`
            // scaling turns this into a real unit readout.
            ui.label("units: file");
            ui.separator();
            match view.hovered {
                Some(hit) => ui.label(format!("hover: {hit}")),
                None => ui.weak("hover: —"),
            };
            ui.separator();
            match view.selected {
                Some(hit) => ui.label(format!("selected: {hit}")),
                None => ui.weak("selected: —"),
            };
            if let Some(message) = view.message {
                ui.separator();
                ui.label(message);
            }
            if let Some(dt) = view.frame_dt {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.monospace(format!("{:.2} ms ({:.0} fps)", dt * 1000.0, 1.0 / dt));
                });
            }
        });
    });
}
