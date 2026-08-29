//! Keyboard shortcuts, handled once per frame *before* the panels so a
//! panel never sees a key the app already acted on.
//!
//! Two rules, kept from RoboCAD's `consume_key` lesson: a shortcut that a
//! text field would also want (Delete, and the undo/redo pair of step 11)
//! yields while a text field has focus, so `TextEdit`'s own editing keeps
//! working; and `consume_key` is matched from the most specific modifier
//! set down, because egui matches modifiers logically and a bare pattern
//! swallows its shifted variant.

use crate::app::{RiggenApp, Selection};

impl RiggenApp {
    pub(crate) fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        use egui::{Key, Modifiers};
        // File shortcuts fire always, text field or not. The shifted
        // pattern is matched before the bare one.
        let cmd = Modifiers::COMMAND;
        if ctx.input_mut(|i| i.consume_key(cmd | Modifiers::SHIFT, Key::S)) {
            self.save_as_dialog();
        } else if ctx.input_mut(|i| i.consume_key(cmd, Key::S)) {
            self.save();
        }
        if ctx.input_mut(|i| i.consume_key(cmd, Key::N)) {
            self.request_new();
        }
        if ctx.input_mut(|i| i.consume_key(cmd, Key::O)) {
            self.request_open_dialog();
        }
        // A focused text field owns Delete / F2 / letters.
        if text_field_focused(ctx) {
            return;
        }
        let delete = ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Delete));
        if delete {
            self.remove_selected();
        }
        let rename = ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::F2));
        if rename && let Selection::Link(link) = self.selection {
            self.start_rename(link);
        }
    }
}

/// Whether the widget with keyboard focus is a `TextEdit` (a button that
/// was clicked can hold focus too, and that one should not block Delete).
pub(crate) fn text_field_focused(ctx: &egui::Context) -> bool {
    ctx.memory(|m| m.focused())
        .is_some_and(|id| egui::TextEdit::load_state(ctx, id).is_some())
}
