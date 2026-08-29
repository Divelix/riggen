//! The materials table window: name / density / colour rows, add and
//! remove, edits through `UpsertMaterial` / `RemoveMaterial`. The link
//! material combo in the properties panel reads the same table, and the
//! viewport tints every instance with its link's material colour.

use std::collections::HashMap;

use riggen_core::{Command, Material};

use crate::app::RiggenApp;

/// The density a new material starts with (water) and its colour.
const NEW_MATERIAL: Material = Material {
    density: 1000.0,
    color: [0.6, 0.6, 0.6, 1.0],
};

#[derive(Debug, Clone, Default)]
pub(crate) struct MaterialsWindow {
    pub(crate) open: bool,
    /// The "new material" name field.
    new_name: String,
    /// Density text being typed, per material, while the field has focus.
    density_drafts: HashMap<String, String>,
    /// A colour being picked: the picker changes every frame, the command
    /// is sent once when the popup closes (one gesture = one command).
    color_draft: Option<(String, [f32; 4])>,
}

impl RiggenApp {
    pub fn materials_window_open(&self) -> bool {
        self.materials_window.open
    }

    pub fn set_materials_window_open(&mut self, open: bool) {
        self.materials_window.open = open;
    }

    /// Draws the window if it is open; after the panels, over the viewport.
    pub(crate) fn materials_window(&mut self, ctx: &egui::Context) {
        if !self.materials_window.open {
            return;
        }
        let materials: Vec<(String, Material)> = self
            .robot
            .materials
            .iter()
            .map(|(n, m)| (n.clone(), *m))
            .collect();
        let anchor = self
            .viewport
            .viewport_rect()
            .unwrap_or_else(|| ctx.content_rect());
        let state = &mut self.materials_window;
        let mut open = state.open;
        let mut commands: Vec<Command> = Vec::new();
        egui::Window::new("Materials")
            .open(&mut open)
            .default_pos(egui::pos2(anchor.left() + 16.0, anchor.top() + 16.0))
            .resizable(false)
            .show(ctx, |ui| {
                egui::Grid::new("materials_table")
                    .num_columns(4)
                    .striped(true)
                    .show(ui, |ui| {
                        ui.strong("name");
                        ui.strong("density kg/m³");
                        ui.strong("colour");
                        ui.label("");
                        ui.end_row();
                        for (name, material) in &materials {
                            ui.label(name);

                            let shown = format!("{}", material.density);
                            let mut text = state
                                .density_drafts
                                .get(name)
                                .cloned()
                                .unwrap_or_else(|| shown.clone());
                            let field = ui.add(
                                egui::TextEdit::singleline(&mut text)
                                    .id_salt(("density", name))
                                    .desired_width(72.0),
                            );
                            if field.changed() {
                                state.density_drafts.insert(name.clone(), text.clone());
                            }
                            if field.lost_focus() {
                                let draft = state.density_drafts.remove(name);
                                let escaped = ui.input(|i| i.key_pressed(egui::Key::Escape));
                                if !escaped
                                    && let Some(density) =
                                        draft.unwrap_or(text).trim().parse::<f64>().ok()
                                    && density != material.density
                                {
                                    commands.push(Command::UpsertMaterial(
                                        name.clone(),
                                        Material {
                                            density,
                                            ..*material
                                        },
                                    ));
                                }
                            }

                            let mut color = match &state.color_draft {
                                Some((n, c)) if n == name => *c,
                                _ => material.color,
                            };
                            if ui.color_edit_button_rgba_unmultiplied(&mut color).changed() {
                                state.color_draft = Some((name.clone(), color));
                            }

                            if ui.small_button("Remove").clicked() {
                                commands.push(Command::RemoveMaterial(name.clone()));
                            }
                            ui.end_row();
                        }
                    });
                ui.separator();
                ui.horizontal(|ui| {
                    ui.add(
                        egui::TextEdit::singleline(&mut state.new_name)
                            .hint_text("new material")
                            .desired_width(120.0),
                    );
                    if ui.button("Add").clicked() && !state.new_name.trim().is_empty() {
                        commands.push(Command::UpsertMaterial(
                            state.new_name.trim().to_owned(),
                            NEW_MATERIAL,
                        ));
                        state.new_name.clear();
                    }
                });
            });
        state.open = open;

        // The colour picker closed: commit the colour it left behind.
        if let Some((name, color)) = state.color_draft.clone()
            && !ctx.any_popup_open()
        {
            state.color_draft = None;
            if let Some(material) = self.robot.materials.get(&name)
                && material.color != color
            {
                commands.push(Command::UpsertMaterial(
                    name,
                    Material { color, ..*material },
                ));
            }
        }
        // While the picker is open the draft colour tints the viewport.
        let preview = self.materials_window.color_draft.clone();
        for command in commands {
            let _ = self.apply(command);
        }
        if let Some((name, color)) = preview {
            self.preview_material_color(&name, color);
        }
    }
}
