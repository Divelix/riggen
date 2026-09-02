//! The materials table window: name / density / colour rows, add and
//! remove, edits through `UpsertMaterial` / `RemoveMaterial`, and the name
//! renamed inline (double-click or F2, the tree's idiom) through
//! `RenameMaterial`. The link material combo in the properties panel reads
//! the same table, and the viewport tints every instance with its link's
//! material colour.

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
    /// The name being renamed inline and the text so far.
    renaming: Option<(String, String)>,
    /// Set when a rename starts so the field grabs focus on its first frame.
    focus_rename: bool,
    /// The name label under the cursor last frame: what F2 renames
    /// (`handle_shortcuts` runs before the window draws).
    pub(crate) hovered_name: Option<String>,
}

impl MaterialsWindow {
    /// Starts renaming `name` inline (F2 over its label, a double-click).
    pub(crate) fn start_rename(&mut self, name: String) {
        self.renaming = Some((name.clone(), name));
        self.focus_rename = true;
    }
}

impl RiggenApp {
    pub fn materials_window_open(&self) -> bool {
        self.materials_window.open
    }

    pub fn set_materials_window_open(&mut self, open: bool) {
        self.materials_window.open = open;
    }

    /// The material whose name is being renamed inline, if any
    /// (`debug_state().ui.renaming` reports it as `material <name>`).
    pub fn renaming_material(&self) -> Option<&str> {
        self.materials_window
            .renaming
            .as_ref()
            .map(|(name, _)| name.as_str())
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
        let mut hovered_name: Option<String> = None;
        // Rename outcome, applied after the draw: `Some((from, Some(to)))`
        // commits, `Some((_, None))` cancels.
        let mut rename_done: Option<(String, Option<String>)> = None;
        let mut rename_typed: Option<String> = None;
        // A double-click starts a rename *after* the draw, so the focus
        // request survives to the frame the field first appears on.
        let mut rename_start: Option<String> = None;
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
                            match &state.renaming {
                                Some((renaming, text)) if renaming == name => {
                                    let mut text = text.clone();
                                    let edit = ui.add(
                                        egui::TextEdit::singleline(&mut text)
                                            .id_salt(("rename_material", name))
                                            .desired_width(100.0),
                                    );
                                    if state.focus_rename {
                                        edit.request_focus();
                                    }
                                    // Escape surrenders focus too: checked first.
                                    let escaped = ui.input(|i| i.key_pressed(egui::Key::Escape));
                                    let entered = ui.input(|i| i.key_pressed(egui::Key::Enter));
                                    if escaped {
                                        rename_done = Some((name.clone(), None));
                                    } else if entered || edit.lost_focus() {
                                        rename_done = Some((name.clone(), Some(text)));
                                    } else if edit.changed() {
                                        rename_typed = Some(text);
                                    }
                                }
                                _ => {
                                    let label = ui
                                        .add(egui::Label::new(name).sense(egui::Sense::click()))
                                        .on_hover_text("double-click or F2 to rename");
                                    if label.hovered() {
                                        hovered_name = Some(name.clone());
                                    }
                                    if label.double_clicked() {
                                        rename_start = Some(name.clone());
                                    }
                                }
                            }

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
        // The field asked for focus on the frame it appeared; once.
        state.focus_rename = false;
        state.hovered_name = hovered_name;
        if let Some(name) = rename_start {
            state.start_rename(name);
        }
        if let Some(text) = rename_typed
            && let Some((_, draft)) = &mut state.renaming
        {
            *draft = text;
        }
        if let Some((from, to)) = rename_done {
            state.renaming = None;
            if let Some(to) = to {
                let to = to.trim().to_owned();
                if !to.is_empty() && to != from {
                    commands.push(Command::RenameMaterial { from, to });
                }
            }
        }

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
