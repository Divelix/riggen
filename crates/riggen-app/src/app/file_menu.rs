//! New / Open / Save / Save As / Quit, the unsaved-changes confirm, the
//! window title and the import-units choice. Every route that would drop
//! the current document asks first when it is dirty; the answer decides
//! whether the pending action runs (docs/03-roadmap.md §M1).

use std::path::{Path, PathBuf};

use riggen_core::Robot;

use super::RiggenApp;
use super::file_io::DOCUMENT_EXTENSION;

/// What the user asked for while the document was dirty, waiting on the
/// Save / Don't save / Cancel answer.
#[derive(Debug, Clone, PartialEq)]
pub enum PendingAction {
    New,
    /// `None`: the open dialog; `Some`: these files (a drop, the CLI).
    Open(Option<Vec<PathBuf>>),
    Quit,
}

/// The import-units choices in the File menu: `(label, scale)`.
pub(crate) const IMPORT_UNITS: [(&str, f64); 4] =
    [("mm", 0.001), ("cm", 0.01), ("m", 1.0), ("in", 0.0254)];

/// eframe storage key for the remembered import scale.
pub(crate) const IMPORT_SCALE_KEY: &str = "riggen.import_scale";

impl RiggenApp {
    /// `pendulum.riggen* — riggen`: what the OS window shows.
    pub fn window_title(&self) -> String {
        format!(
            "{}{} — riggen",
            self.document_label(),
            if self.history.is_dirty() { "*" } else { "" }
        )
    }

    /// The confirm the user is being asked, if any.
    pub fn pending_action(&self) -> Option<&PendingAction> {
        self.pending.as_ref()
    }

    /// Whether the app has agreed to close (the OS close request or Quit
    /// went through the dirty check).
    pub fn quit_confirmed(&self) -> bool {
        self.quit_confirmed
    }

    /// File › New: an empty document, after the dirty check.
    pub fn request_new(&mut self) {
        self.request(PendingAction::New);
    }

    /// File › Open…: the dialog, after the dirty check.
    pub fn request_open_dialog(&mut self) {
        self.request(PendingAction::Open(None));
    }

    /// Opening these files (a drop, the CLI), after the dirty check if
    /// one of them is a document. Meshes alone never ask: they add to
    /// the document rather than replace it.
    pub fn request_open(&mut self, paths: Vec<PathBuf>) {
        let replaces = paths
            .iter()
            .any(|p| super::file_io::extension_of(p) == DOCUMENT_EXTENSION);
        if replaces {
            self.request(PendingAction::Open(Some(paths)));
        } else {
            self.load_files(&paths);
        }
    }

    /// File › Quit and the OS close button, after the dirty check.
    pub fn request_quit(&mut self) {
        self.request(PendingAction::Quit);
    }

    fn request(&mut self, action: PendingAction) {
        if self.history.is_dirty() {
            self.pending = Some(action);
        } else {
            self.run_pending(action);
        }
    }

    fn run_pending(&mut self, action: PendingAction) {
        match action {
            PendingAction::New => self.new_document(),
            PendingAction::Open(None) => self.open_dialog(),
            PendingAction::Open(Some(paths)) => self.load_files(&paths),
            PendingAction::Quit => self.quit_confirmed = true,
        }
    }

    /// An empty document named `robot`, untitled.
    pub fn new_document(&mut self) {
        self.replace_document(Robot::new("robot"), None);
        self.status = None;
    }

    /// File › Save: to the document's file, or Save As when untitled.
    /// `true` when the document is on disk afterwards.
    pub fn save(&mut self) -> bool {
        match self.file.clone() {
            Some(path) => self.save_to(&path),
            None => self.save_as_dialog(),
        }
    }

    /// Writes the document to `path` (given the `.riggen` extension if it
    /// has none), marks it saved and makes it the document's file.
    pub fn save_to(&mut self, path: &Path) -> bool {
        let path = if super::file_io::extension_of(path) == DOCUMENT_EXTENSION {
            path.to_owned()
        } else {
            path.with_extension(DOCUMENT_EXTENSION)
        };
        match riggen_core::save(&self.robot, &path) {
            Ok(()) => {
                self.history.mark_saved();
                self.file = Some(path);
                self.status = Some(format!("saved {}", self.document_label()));
                true
            }
            Err(err) => {
                self.status = Some(err.to_string());
                false
            }
        }
    }

    /// File › Save As…: the dialog, then [`Self::save_to`].
    pub fn save_as_dialog(&mut self) -> bool {
        #[cfg(target_arch = "wasm32")]
        {
            self.status = Some("no filesystem in the browser".into());
            false
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let mut dialog = rfd::FileDialog::new()
                .add_filter("Riggen documents", &[DOCUMENT_EXTENSION])
                .set_file_name(format!("{}.{DOCUMENT_EXTENSION}", self.robot.name));
            if let Some(dir) = self.file.as_ref().and_then(|f| f.parent()) {
                dialog = dialog.set_directory(dir);
            }
            match dialog.save_file() {
                Some(path) => self.save_to(&path),
                None => false,
            }
        }
    }

    /// The modal's "Save" answer: save (a dialog when untitled), and run
    /// the pending action only if that succeeded.
    pub fn answer_save(&mut self) {
        if let Some(action) = self.pending.take()
            && self.save()
        {
            self.run_pending(action);
        }
    }

    /// The modal's "Don't save" answer.
    pub fn answer_discard(&mut self) {
        if let Some(action) = self.pending.take() {
            self.run_pending(action);
        }
    }

    /// The modal's "Cancel" answer (also Escape).
    pub fn answer_cancel(&mut self) {
        self.pending = None;
    }

    /// The Save / Don't save / Cancel modal while an action is pending.
    pub(crate) fn unsaved_changes_modal(&mut self, ctx: &egui::Context) {
        let Some(action) = self.pending.clone() else {
            return;
        };
        let what = match action {
            PendingAction::New => "starting a new document",
            PendingAction::Open(_) => "opening another document",
            PendingAction::Quit => "quitting",
        };
        let mut answer: Option<fn(&mut Self)> = None;
        let modal = egui::Modal::new(egui::Id::new("unsaved_changes")).show(ctx, |ui| {
            ui.set_width(360.0);
            ui.heading("Unsaved changes");
            ui.label(format!(
                "Save the changes to {} before {what}?",
                self.document_label()
            ));
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.button("Save").clicked() {
                    answer = Some(Self::answer_save);
                }
                if ui.button("Don't save").clicked() {
                    answer = Some(Self::answer_discard);
                }
                if ui.button("Cancel").clicked() {
                    answer = Some(Self::answer_cancel);
                }
            });
        });
        if answer.is_none() && modal.should_close() {
            answer = Some(Self::answer_cancel);
        }
        if let Some(answer) = answer {
            answer(self);
        }
    }

    /// The OS close button: refused while the dirty check has not been
    /// answered, so the modal gets its turn; once answered, the next
    /// request goes through.
    pub(crate) fn handle_close_request(&mut self, ctx: &egui::Context) {
        if self.quit_confirmed {
            return;
        }
        let requested = ctx.input(|i| i.viewport().close_requested());
        if requested && self.history.is_dirty() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.pending = Some(PendingAction::Quit);
        }
    }

    /// Pushes the title to the OS window when it changed.
    pub(crate) fn update_title(&mut self, ctx: &egui::Context) {
        let title = self.window_title();
        if self.last_title.as_deref() != Some(title.as_str()) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));
            self.last_title = Some(title);
        }
    }

    /// The File menu.
    pub(crate) fn file_menu(&mut self, ui: &mut egui::Ui) {
        if ui.button("New").clicked() {
            ui.close();
            self.request_new();
        }
        if ui.button("Open…").clicked() {
            ui.close();
            self.request_open_dialog();
        }
        ui.separator();
        if ui.button("Save").clicked() {
            ui.close();
            self.save();
        }
        if ui.button("Save As…").clicked() {
            ui.close();
            self.save_as_dialog();
        }
        ui.separator();
        ui.menu_button("Import units", |ui| {
            for (label, scale) in IMPORT_UNITS {
                let selected = (self.import_scale - scale).abs() < 1e-12;
                if ui.radio(selected, label).clicked() {
                    self.set_import_scale(scale);
                    ui.close();
                }
            }
        });
        ui.separator();
        if ui.button("Quit").clicked() {
            ui.close();
            self.request_quit();
        }
    }
}
