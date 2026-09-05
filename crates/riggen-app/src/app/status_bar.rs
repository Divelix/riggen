//! Bottom status bar: `riggen | View | pendulum.riggen* | import: mm |
//! hover: arm (i1/t120) | selected: … | 2 instances | 4.10 ms (244 fps)`.
//!
//! `hovered`/`selected` reflect the *previous* frame's viewport state — this
//! panel is drawn before the viewport so the central panel gets whatever
//! screen space is left (egui lays out side/bottom panels before the central
//! one). One frame of lag on a status readout is imperceptible.

/// Everything the bar shows, pre-formatted by the app so this module never
/// names viewport or document types.
pub(crate) struct StatusView<'a> {
    /// `View` or `Edit` (ADR-0021 §3).
    pub mode: &'a str,
    /// `name.riggen`, with `*` when there are unsaved changes.
    pub document: &'a str,
    /// What a dropped mesh is read as: `mm`, `m`, …
    pub import_units: &'a str,
    /// `arm (i1/t120)`-style readout of the hovered link and triangle.
    pub hovered: Option<&'a str>,
    pub selected: Option<&'a str>,
    /// `hidden: joints, frames` — what the visibility row has switched
    /// off, absent when the row is at its default (`app/overlays.rs`).
    pub hidden: Option<&'a str>,
    pub instance_count: usize,
    /// A one-off message — a load error, an export destination.
    pub message: Option<&'a str>,
    /// Seconds between the last two frames; `None` hides the readout (the
    /// snapshot suite, or the first frame).
    pub frame_dt: Option<f32>,
}

/// `mm` / `cm` / `m` / `in` for the common import scales, `×0.5` otherwise.
pub(crate) fn import_units_label(scale: f64) -> String {
    let close = |x: f64| (scale - x).abs() < 1e-12;
    if close(0.001) {
        "mm".into()
    } else if close(0.01) {
        "cm".into()
    } else if close(1.0) {
        "m".into()
    } else if close(0.0254) {
        "in".into()
    } else {
        format!("×{scale}")
    }
}

pub(crate) fn status_bar(ui: &mut egui::Ui, view: &StatusView<'_>) {
    egui::Panel::bottom("status_bar").show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label("riggen");
            ui.separator();
            ui.label(view.mode);
            ui.separator();
            ui.label(view.document);
            ui.separator();
            ui.label(format!("import: {}", view.import_units));
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
            ui.separator();
            ui.label(format!("{} instances", view.instance_count));
            if let Some(hidden) = view.hidden {
                ui.separator();
                ui.label(hidden);
            }
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

#[cfg(test)]
mod tests {
    use super::import_units_label;

    #[test]
    fn import_units() {
        assert_eq!(import_units_label(0.001), "mm");
        assert_eq!(import_units_label(0.01), "cm");
        assert_eq!(import_units_label(1.0), "m");
        assert_eq!(import_units_label(0.0254), "in");
        assert_eq!(import_units_label(0.5), "×0.5");
    }
}
