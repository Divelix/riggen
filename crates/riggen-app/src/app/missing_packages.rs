//! The Missing packages window (plans/package-map-ui): after a URDF import
//! whose `package://` meshes did not load, one row per package — its name,
//! how many meshes it cost, **Choose folder…** — and **Dismiss**. Choosing
//! a folder imports the same file again through every folder chosen so far
//! (`file_io.rs`, [`RiggenApp::set_package_dir`]).
//!
//! Non-modal on purpose: the broken model stays in view behind it, which a
//! `Modal` would dim. Native only — a browser drop has no folders to choose
//! (ADR-0017 §3), so the module is compiled out on `wasm32`.

use super::RiggenApp;

const TITLE: &str = "Missing packages";
const CHOOSE_FOLDER: &str = "Choose folder…";
const DISMISS: &str = "Dismiss";

impl RiggenApp {
    /// Draws the window while an import is waiting on a folder; after the
    /// panels, over the viewport.
    pub(crate) fn missing_packages_window(&mut self, ctx: &egui::Context) {
        let Some(missing) = self.missing_packages().cloned() else {
            return;
        };
        // Bottom centre of the viewport: clear of the corner chrome at the
        // top and of the left panel in either mode.
        let anchor = self
            .viewport
            .viewport_rect()
            .unwrap_or_else(|| ctx.content_rect());
        let mut chosen: Option<String> = None;
        let mut dismissed = false;
        egui::Window::new(TITLE)
            .pivot(egui::Align2::CENTER_BOTTOM)
            .default_pos(anchor.center_bottom() - egui::vec2(0.0, 16.0))
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                ui.label("These package:// paths were not found beside the file.");
                ui.label("Choose the folder each package is in to import it again.");
                ui.add_space(4.0);
                egui::Grid::new("missing_packages_rows")
                    .num_columns(3)
                    .spacing([12.0, 6.0])
                    .show(ui, |ui| {
                        for (package, &meshes) in &missing {
                            ui.monospace(package);
                            ui.label(format!(
                                "{meshes} mesh{}",
                                if meshes == 1 { "" } else { "es" }
                            ));
                            if ui.button(CHOOSE_FOLDER).clicked() {
                                chosen = Some(package.clone());
                            }
                            ui.end_row();
                        }
                    });
                ui.separator();
                if ui.button(DISMISS).clicked() {
                    dismissed = true;
                }
            });
        if dismissed {
            self.dismiss_missing_packages();
        } else if let Some(package) = chosen
            && let Some(dir) = rfd::FileDialog::new()
                .set_title(format!("Folder of package://{package}"))
                .pick_folder()
        {
            self.set_package_dir(&package, dir);
        }
    }
}
