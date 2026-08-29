//! The active tool and the toolbar that floats in the viewport's top-left
//! corner (docs/01-architecture.md §Panels and menus).
//!
//! A tool is *modal*: it decides what a click and a drag in the viewport
//! mean. `Select` is the M1 behaviour and the resting state — `Esc` always
//! comes back to it. The four editing tools rewrite frames, and every
//! frame-rewriting command in `riggen-core` works in the **zero
//! configuration** (plans/m2-placement-ux OPEN 1), so entering one with a
//! joint off zero resets the sliders first and says so in the status bar.

use super::RiggenApp;

/// What a viewport gesture means.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Tool {
    /// Click picks a link, drag orbits: the M1 viewport.
    #[default]
    Select,
    /// Gizmo: drag translates the selected link (its parent joint's origin)
    /// or the selected joint's pivot.
    Move,
    /// The same gizmo, rotating.
    Rotate,
    /// Click a feature to put the selected joint's origin and axis on it.
    PlaceJoint,
    /// Click a feature on the selected link, then one anywhere, to bring
    /// the first onto the second.
    Align,
}

impl Tool {
    /// Toolbar order, which is also the order the buttons are found by
    /// label in the snapshot suite.
    pub const ALL: [Tool; 5] = [
        Tool::Select,
        Tool::Move,
        Tool::Rotate,
        Tool::PlaceJoint,
        Tool::Align,
    ];

    /// The toolbar button's text, and the name `debug_state` reports.
    pub fn label(self) -> &'static str {
        match self {
            Tool::Select => "Select",
            Tool::Move => "Move",
            Tool::Rotate => "Rotate",
            Tool::PlaceJoint => "Place joint",
            Tool::Align => "Align",
        }
    }

    /// Whether the tool commits frame-rewriting commands, and therefore
    /// needs the zero configuration.
    pub fn edits_frames(self) -> bool {
        !matches!(self, Tool::Select)
    }
}

/// What the status bar says when entering an editing tool rewound the
/// sliders. Public so a test can assert on it rather than on prose.
pub const ZERO_CONFIG_STATUS: &str =
    "joint values reset to zero — placement tools edit the zero configuration";

impl RiggenApp {
    pub fn tool(&self) -> Tool {
        self.tool
    }

    /// Switches tools, resetting `q` first when the new one edits frames
    /// and something is off zero (OPEN 1).
    pub fn set_tool(&mut self, tool: Tool) {
        if tool.edits_frames() && self.q.0.values().any(|q| *q != 0.0) {
            self.reset_joint_values();
            self.status = Some(ZERO_CONFIG_STATUS.to_owned());
        }
        self.tool = tool;
    }

    /// The toolbar, drawn over the viewport's top-left corner **after** the
    /// viewport itself so egui's hit test gives it the pointer: a widget
    /// registered later in a layer is the top-most one under the cursor,
    /// which is the same rule the gizmo relies on.
    pub(crate) fn tool_bar(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        const MARGIN: f32 = 8.0;
        let corner = egui::Rect::from_min_max(rect.min + egui::Vec2::splat(MARGIN), rect.max);
        let mut chosen = None;
        let response = ui.scope_builder(
            egui::UiBuilder::new()
                .max_rect(corner)
                .layout(egui::Layout::top_down(egui::Align::LEFT)),
            |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        for tool in Tool::ALL {
                            if ui
                                .selectable_label(self.tool == tool, tool.label())
                                .clicked()
                            {
                                chosen = Some(tool);
                            }
                        }
                    });
                });
            },
        );
        // Remembered so a joint glyph *behind* the toolbar is not treated as
        // hovered through it (`update_glyph_hover`).
        self.toolbar_rect = Some(response.response.rect);
        if let Some(tool) = chosen {
            self.set_tool(tool);
        }
    }
}
