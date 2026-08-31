//! File › Export…: the modal that turns the document into MJCF / URDF /
//! SDF (docs/02-data-model.md §`ResolvedRobot`, ADR-0008, ADR-0016). It
//! resolves the document when it opens and whenever an option changes,
//! lists every `ExportError` with the link it names, and keeps the Export
//! button disabled while any exist — the sanity checks MuJoCo fails
//! silently on block here, in words, instead.

use std::path::{Path, PathBuf};

use riggen_export::{ExportOptions, MeshPathStyle, ResolvedRobot};

use super::RiggenApp;
use super::document::{AppDecomp, AppMeshes};

/// The modal's state. `resolved` is `Some` exactly when `errors` is empty.
#[derive(Default)]
pub struct ExportDialog {
    pub open: bool,
    pub options: ExportOptions,
    /// The `package://` name typed for `MeshPathStyle::Package`.
    pub package: String,
    pub dir: Option<PathBuf>,
    pub errors: Vec<String>,
    resolved: Option<ResolvedRobot>,
    /// Set by an option change; the next frame re-resolves.
    stale: bool,
}

impl RiggenApp {
    /// File › Export…: opens the modal with the last options, the
    /// directory beside the document when it has one, and a fresh resolve.
    pub fn open_export_dialog(&mut self) {
        if self.export_dialog.dir.is_none()
            && let Some(dir) = self.file.as_ref().and_then(|f| f.parent())
        {
            self.export_dialog.dir = Some(dir.join(format!("{}_export", self.robot.name)));
        }
        self.export_dialog.open = true;
        self.resolve_for_export();
    }

    pub fn export_dialog(&self) -> &ExportDialog {
        &self.export_dialog
    }

    pub fn set_export_dir(&mut self, dir: &Path) {
        self.export_dialog.dir = Some(dir.to_owned());
    }

    pub fn set_export_options(&mut self, options: ExportOptions) {
        self.export_dialog.options = options;
        self.export_dialog.stale = true;
    }

    /// Resolves the document with the dialog's options, loading any mesh
    /// the scene has not needed yet (a collision-only mesh with the view
    /// off), and fills `errors` / `resolved`.
    fn resolve_for_export(&mut self) {
        for mesh in self.robot.referenced_assets() {
            self.ensure_loaded(mesh);
        }
        // The dialog may be the first thing to want a decomposition (a
        // link whose collision view was never on), so ask before resolving.
        self.request_decompositions();
        let mut options = self.export_dialog.options.clone();
        if let MeshPathStyle::Package(name) = &mut options.mesh_paths {
            *name = self.export_dialog.package.clone();
        }
        // The cache, never a computation: a decomposition the job thread
        // has not delivered is `ExportError::DecompositionPending` in the
        // list below, and the line clears itself when the job lands and
        // the user presses the button again (plans OPEN 3).
        match riggen_export::resolve(
            &self.robot,
            &AppMeshes(&self.mesh_store),
            &AppDecomp(&self.decomp),
            &options,
        ) {
            Ok(resolved) => {
                self.export_dialog.errors.clear();
                self.export_dialog.resolved = Some(resolved);
            }
            Err(errors) => {
                self.export_dialog.errors = errors.iter().map(ToString::to_string).collect();
                self.export_dialog.resolved = None;
            }
        }
        self.export_dialog.stale = false;
    }

    /// The Export button: writes the files and closes the modal. `false`
    /// when nothing was written (errors, no directory, an I/O failure) —
    /// the status bar says why.
    pub fn run_export(&mut self) -> bool {
        if self.export_dialog.stale {
            self.resolve_for_export();
        }
        let Some(dir) = self.export_dialog.dir.clone() else {
            self.status = Some("choose an export directory".into());
            return false;
        };
        let Some(resolved) = self.export_dialog.resolved.as_ref() else {
            self.status = Some(format!(
                "cannot export: {}",
                self.export_dialog
                    .errors
                    .first()
                    .cloned()
                    .unwrap_or_default()
            ));
            return false;
        };
        let mut options = self.export_dialog.options.clone();
        if let MeshPathStyle::Package(name) = &mut options.mesh_paths {
            *name = self.export_dialog.package.clone();
        }
        match riggen_export::export(resolved, &options, &dir) {
            Ok(written) => {
                self.status = Some(format!(
                    "exported {} file{} to {}",
                    written.len(),
                    if written.len() == 1 { "" } else { "s" },
                    dir.display()
                ));
                self.export_dialog.open = false;
                true
            }
            Err(err) => {
                self.status = Some(err.to_string());
                false
            }
        }
    }

    /// The modal, while open.
    pub(crate) fn export_modal(&mut self, ctx: &egui::Context) {
        if !self.export_dialog.open {
            return;
        }
        if self.export_dialog.stale {
            self.resolve_for_export();
        }
        let mut action: Option<fn(&mut Self)> = None;
        let mut choose_dir = false;
        let modal = egui::Modal::new(egui::Id::new("export")).show(ctx, |ui| {
            ui.set_width(460.0);
            ui.heading("Export to MJCF / URDF / SDF");
            let d = &mut self.export_dialog;
            egui::Grid::new("export_grid")
                .num_columns(2)
                .show(ui, |ui| {
                    // Checkboxes, not radio buttons: the three writers are
                    // a set (ADR-0016), so any combination is askable and
                    // "both" is no longer a whole answer.
                    ui.label("format");
                    ui.horizontal(|ui| {
                        let f = &mut d.options.format;
                        for (on, label) in [
                            (&mut f.mjcf, "MJCF"),
                            (&mut f.urdf, "URDF"),
                            (&mut f.sdf, "SDF"),
                        ] {
                            if ui.checkbox(on, label).changed() {
                                d.stale = true;
                            }
                        }
                    });
                    ui.end_row();

                    ui.label("directory");
                    ui.horizontal(|ui| {
                        let shown = d
                            .dir
                            .as_ref()
                            .map(|p| p.display().to_string())
                            .unwrap_or_default();
                        ui.add(egui::Label::new(if shown.is_empty() {
                            egui::RichText::new("(none chosen)").weak()
                        } else {
                            egui::RichText::new(shown)
                        }));
                        if ui.button("Choose…").clicked() {
                            choose_dir = true;
                        }
                    });
                    ui.end_row();

                    let mesh_paths_used =
                        d.options.format.writes_urdf() || d.options.format.writes_sdf();
                    ui.add_enabled_ui(mesh_paths_used, |ui| {
                        ui.label("mesh paths");
                    });
                    ui.add_enabled_ui(mesh_paths_used, |ui| {
                        ui.horizontal(|ui| {
                            let is_package =
                                matches!(d.options.mesh_paths, MeshPathStyle::Package(_));
                            if ui
                                .radio(d.options.mesh_paths == MeshPathStyle::Relative, "relative")
                                .clicked()
                            {
                                d.options.mesh_paths = MeshPathStyle::Relative;
                            }
                            if ui.radio(is_package, "package://").clicked() && !is_package {
                                d.options.mesh_paths = MeshPathStyle::Package(d.package.clone());
                            }
                            if is_package {
                                ui.add(
                                    egui::TextEdit::singleline(&mut d.package)
                                        .desired_width(120.0)
                                        .hint_text("package name"),
                                );
                            }
                            if ui
                                .radio(d.options.mesh_paths == MeshPathStyle::Absolute, "absolute")
                                .clicked()
                            {
                                d.options.mesh_paths = MeshPathStyle::Absolute;
                            }
                        });
                    });
                    ui.end_row();

                    ui.add_enabled_ui(d.options.format.writes_mjcf(), |ui| {
                        ui.label("MJCF");
                    });
                    ui.add_enabled_ui(d.options.format.writes_mjcf(), |ui| {
                        if ui
                            .checkbox(&mut d.options.floating_base, "floating base (freejoint)")
                            .changed()
                        {
                            d.stale = true;
                        }
                    });
                    ui.end_row();
                });

            ui.add_space(8.0);
            if d.errors.is_empty() {
                ui.weak(match &d.resolved {
                    Some(r) => format!(
                        "{} links, {} joints, {} mesh files — ready",
                        r.links.len(),
                        r.joints.len(),
                        r.meshes.len()
                    ),
                    None => "resolving…".to_owned(),
                });
            } else {
                ui.colored_label(
                    ui.visuals().warn_fg_color,
                    format!(
                        "{} problem{} block the export:",
                        d.errors.len(),
                        if d.errors.len() == 1 { "" } else { "s" }
                    ),
                );
                for error in &d.errors {
                    ui.label(format!("• {error}"));
                }
            }

            ui.add_space(8.0);
            ui.horizontal(|ui| {
                // Nothing ticked would write a `meshes/` folder and no
                // file that reads it, so it is not a ready export.
                let ready = d.errors.is_empty() && d.dir.is_some() && d.options.format.writes_any();
                if ui.add_enabled(ready, egui::Button::new("Export")).clicked() {
                    action = Some(|app: &mut Self| {
                        app.run_export();
                    });
                }
                if ui.button("Cancel").clicked() {
                    action = Some(|app: &mut Self| app.export_dialog.open = false);
                }
            });
        });
        if choose_dir {
            self.choose_export_dir();
        }
        if action.is_none() && modal.should_close() {
            action = Some(|app: &mut Self| app.export_dialog.open = false);
        }
        if let Some(action) = action {
            action(self);
        }
    }

    fn choose_export_dir(&mut self) {
        #[cfg(target_arch = "wasm32")]
        {
            self.status = Some("no filesystem in the browser".into());
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let mut dialog = rfd::FileDialog::new();
            if let Some(dir) = self.export_dialog.dir.as_ref().and_then(|d| d.parent()) {
                dialog = dialog.set_directory(dir);
            }
            if let Some(dir) = dialog.pick_folder() {
                self.export_dialog.dir = Some(dir);
            }
        }
    }
}
