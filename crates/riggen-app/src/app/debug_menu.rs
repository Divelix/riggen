//! The Debug menu: egui's own layout overlays (debug builds only), plus the
//! two ways out of the running app for `debug_state()` (ADR-0003).
//!
//! The overlays are a human's tool on their own; combined with a snapshot
//! they become an agent's, because a frame captured with `show_widget_hits`
//! on is a picture of the layout skeleton including the parts that never
//! paint. Copy / Save are the runtime route to the same JSON the snapshot
//! goldens hold, for a state reached by hand rather than by a scenario.

use super::RiggenApp;

/// The status-bar message after Copy state (JSON). The suite asserts on it.
pub const COPIED_STATUS: &str = "debug state copied";

impl RiggenApp {
    pub(crate) fn debug_menu(&mut self, ui: &mut egui::Ui) {
        // egui compiles `Style::debug` only under `debug_assertions`, so the
        // overlays exist in `cargo run` and the test suite, and the release
        // binary in the wheel has just the two state items.
        #[cfg(debug_assertions)]
        {
            // Read the *active* theme's style but write both, so toggling an
            // overlay doesn't silently un-toggle itself when the user
            // switches between the light and dark styles.
            let mut debug = ui.ctx().style_of(ui.ctx().theme()).debug;
            let before = debug;

            ui.checkbox(&mut debug.debug_on_hover, "Debug on hover");
            ui.checkbox(&mut debug.show_widget_hits, "Show widget hits");
            ui.checkbox(
                &mut debug.show_interactive_widgets,
                "Show interactive widgets",
            );
            ui.checkbox(&mut debug.show_expand_width, "Show width expansion");
            ui.checkbox(&mut debug.show_expand_height, "Show height expansion");
            ui.checkbox(&mut debug.show_resize, "Show resize");
            ui.checkbox(&mut debug.show_unaligned, "Show unaligned");

            if debug != before {
                ui.ctx().all_styles_mut(|style| style.debug = debug);
            }

            ui.separator();
        }

        if ui.button("Copy state (JSON)").clicked() {
            ui.close();
            ui.ctx().copy_text(self.debug_state_json());
            self.status = Some(COPIED_STATUS.into());
        }
        if ui.button("Save state (JSON)…").clicked() {
            ui.close();
            self.save_debug_state();
        }
    }

    /// Native only, for the same reason File › Save As is: `rfd` is not
    /// built for wasm.
    #[cfg(not(target_arch = "wasm32"))]
    fn save_debug_state(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .set_file_name("debug-state.json")
            .add_filter("JSON", &["json"])
            .save_file()
        else {
            return;
        };
        self.status = Some(match std::fs::write(&path, self.debug_state_json()) {
            Ok(()) => format!("debug state written to {}", path.display()),
            Err(err) => format!("could not write debug state: {err}"),
        });
    }

    /// The browser has nowhere to write, so the state is offered as a
    /// download instead (ADR-0017 §6).
    #[cfg(target_arch = "wasm32")]
    fn save_debug_state(&mut self) {
        let json = self.debug_state_json();
        let name = "debug-state.json";
        self.status = Some(
            match crate::download::offer(name, json.as_bytes(), crate::download::JSON) {
                Ok(()) => format!("downloading {name}"),
                Err(err) => format!("could not download the debug state: {err}"),
            },
        );
    }
}
