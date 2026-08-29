//! Getting files into the app: CLI arguments, drag-and-drop, File › Open.
//! Every route ends in [`RiggenApp::load_files`]; only
//! [`RiggenApp::open_path`] is lower, and it is the harness's primitive.
//!
//! A `.riggen` replaces the document; an STL/OBJ becomes a `MeshAsset` plus
//! a new link named after the file stem, `Fixed` joint at identity, under
//! the selected link or the root (plan m1-document-tree-joints, decided by
//! the human at step 5).

use std::path::{Path, PathBuf};

use riggen_core::{Geom, Link, LinkId, MeshAsset, Pose};

use super::document::name_from_stem;
use super::{LoadedMesh, RiggenApp};

/// Extensions the open dialog offers, matching `riggen_mesh::load_mesh`.
const MESH_EXTENSIONS: [&str; 2] = ["stl", "obj"];
/// The document's own extension.
pub(crate) const DOCUMENT_EXTENSION: &str = "riggen";

fn extension_of(path: &Path) -> String {
    path.extension()
        .and_then(|e| e.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase()
}

impl RiggenApp {
    /// Opens one file by extension: a `.riggen` replaces the document and
    /// returns `None`; a mesh becomes a new link and returns its id. The
    /// camera is not moved; callers decide whether to fit. Errors are
    /// returned *and* shown in the status bar, since every caller wants
    /// both.
    pub fn open_path(&mut self, path: &Path) -> Result<Option<LinkId>, String> {
        let result = if extension_of(path) == DOCUMENT_EXTENSION {
            self.open_document_path(path).map(|()| None)
        } else {
            self.open_mesh_path(path).map(Some)
        };
        if let Err(err) = &result {
            self.status = Some(err.clone());
        }
        result
    }

    /// Replaces the document with the file's. Warnings (a mesh that changed
    /// or went missing) go to the status bar; the document still opens.
    fn open_document_path(&mut self, path: &Path) -> Result<(), String> {
        let abs = std::path::absolute(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let (robot, warnings) = riggen_core::load(&abs).map_err(|e| e.to_string())?;
        // Step 10 adds the unsaved-changes confirm in front of this.
        self.replace_document(robot, Some(abs));
        if let Some(first) = warnings.first() {
            self.status = Some(match warnings.len() {
                1 => first.to_string(),
                n => format!("{first} (+{} more warnings)", n - 1),
            });
        }
        Ok(())
    }

    /// Registers the mesh as an asset (not a command) and adds a link for
    /// it under the selection or the root through `AddLink`, so undo
    /// removes the link and the asset stays registered for redo.
    fn open_mesh_path(&mut self, path: &Path) -> Result<LinkId, String> {
        let abs = std::path::absolute(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let raw = riggen_mesh::load_mesh(&abs).map_err(|e| e.to_string())?;
        let content_hash =
            riggen_core::hash_file(&abs).map_err(|e| format!("{}: {e}", abs.display()))?;
        let asset = MeshAsset {
            path: abs.clone(),
            content_hash,
            scale: self.import_scale,
            fix_up: None,
        };
        let mesh = self.robot.add_asset(asset.clone());
        self.mesh_store.insert(mesh, LoadedMesh::new(raw, &asset));

        let stem = abs
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        let mut link = Link::new(name_from_stem(&stem));
        link.visuals.push(Geom {
            id: self.robot.next_id.alloc(),
            mesh,
            pose: Pose::IDENTITY,
            color: None,
        });
        let parent = self.insertion_parent();
        self.add_link(link, parent).map_err(|e| e.to_string())
    }

    /// Opens every path, then fits the view to whatever is now in the scene.
    /// One bad file does not stop the others; the status bar reports the
    /// first failure, or how many files landed. Loading is synchronous in
    /// M1 — the `jobs` thread comes with M3's hull work.
    pub fn load_files(&mut self, paths: &[PathBuf]) {
        if paths.is_empty() {
            return;
        }
        let mut opened = 0usize;
        let mut first_error: Option<String> = None;
        let mut warning: Option<String> = None;
        for path in paths {
            match self.open_path(path) {
                Ok(_) => {
                    opened += 1;
                    // A document that opened with warnings left them here.
                    if extension_of(path) == DOCUMENT_EXTENSION {
                        warning = self.status.take();
                    }
                }
                Err(err) => {
                    first_error.get_or_insert(err);
                }
            }
        }
        if opened > 0 {
            self.viewport.animate_frame_scene();
        }
        self.status = match (first_error, warning) {
            (Some(err), _) if paths.len() > 1 => Some(format!(
                "opened {opened} of {} files; first error: {err}",
                paths.len()
            )),
            (Some(err), _) => Some(err),
            (None, Some(warning)) => Some(warning),
            (None, None) => Some(format!(
                "opened {opened} file{}",
                if opened == 1 { "" } else { "s" }
            )),
        };
    }

    /// File › Open…: a native multi-file dialog filtered to `.riggen` and
    /// STL/OBJ. The browser has no filesystem to reach for, so the wasm
    /// build says so.
    pub(crate) fn open_dialog(&mut self) {
        #[cfg(target_arch = "wasm32")]
        {
            self.status = Some("no filesystem in the browser; drop files onto the window".into());
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            if let Some(paths) = rfd::FileDialog::new()
                .add_filter("Riggen documents", &[DOCUMENT_EXTENSION])
                .add_filter("Meshes (STL, OBJ)", &MESH_EXTENSIONS)
                .pick_files()
            {
                self.load_files(&paths);
            }
        }
    }

    /// Drag-and-drop: a tinted "drop to open" overlay while files hover
    /// the window, and one link (or document) per file on release.
    pub(crate) fn handle_file_drops(&mut self, ctx: &egui::Context) {
        let hovering = ctx.input(|i| !i.raw.hovered_files.is_empty());
        if hovering {
            let rect = ctx.content_rect();
            let painter = ctx.layer_painter(egui::LayerId::new(
                egui::Order::Foreground,
                egui::Id::new("riggen file drop overlay"),
            ));
            painter.rect_filled(
                rect,
                0.0,
                egui::Color32::from_rgba_unmultiplied(80, 140, 220, 60),
            );
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "drop to open",
                egui::FontId::proportional(28.0),
                egui::Color32::WHITE,
            );
        }

        let dropped: Vec<PathBuf> = ctx.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .map(|file| file.path().to_path_buf())
                .collect()
        });
        self.load_files(&dropped);
    }
}
