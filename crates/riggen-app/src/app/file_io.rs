//! Getting files into the app: CLI arguments, drag-and-drop, File › Open.
//! Every route ends in [`RiggenApp::load_files`]; only
//! [`RiggenApp::open_path`] is lower, and it is the harness's primitive.

use std::path::{Path, PathBuf};

use riggen_viewport::InstanceId;

use super::RiggenApp;

/// Extensions the open dialog offers, matching `riggen_mesh::load_mesh`.
const MESH_EXTENSIONS: [&str; 2] = ["stl", "obj"];

impl RiggenApp {
    /// Loads a mesh file as a new instance at the origin, in file units
    /// (M1's `MeshAsset` owns scaling). The camera is not moved; callers
    /// decide whether to fit. Errors are returned *and* shown in the status
    /// bar, since every caller wants both.
    pub fn open_path(&mut self, path: &Path) -> Result<InstanceId, String> {
        let result = riggen_mesh::load_mesh(path)
            .map_err(|err| err.to_string())
            .and_then(|mesh| {
                let id = InstanceId(self.next_instance);
                self.viewport
                    .set_instance(id, &mesh)
                    .map_err(|err| err.to_string())?;
                self.next_instance += 1;
                Ok(id)
            });
        self.status = result.as_ref().err().cloned();
        result
    }

    /// Opens every path, then fits the view to whatever is now in the scene.
    /// One bad file does not stop the others; the status bar reports the
    /// first failure, or how many files landed. Loading is synchronous in
    /// M0 — the `jobs` thread comes with M3's hull work.
    pub fn load_files(&mut self, paths: &[PathBuf]) {
        if paths.is_empty() {
            return;
        }
        let mut opened = 0usize;
        let mut first_error: Option<String> = None;
        for path in paths {
            match self.open_path(path) {
                Ok(_) => opened += 1,
                Err(err) => {
                    first_error.get_or_insert(err);
                }
            }
        }
        if opened > 0 {
            self.viewport.animate_frame_scene();
        }
        self.status = match first_error {
            Some(err) if paths.len() > 1 => Some(format!(
                "opened {opened} of {} files; first error: {err}",
                paths.len()
            )),
            Some(err) => Some(err),
            None => Some(format!(
                "opened {opened} file{}",
                if opened == 1 { "" } else { "s" }
            )),
        };
    }

    /// File › Open…: a native multi-file dialog filtered to STL/OBJ. The
    /// browser has no filesystem to reach for, so the wasm build says so.
    pub(crate) fn open_dialog(&mut self) {
        #[cfg(target_arch = "wasm32")]
        {
            self.status = Some("no filesystem in the browser; drop files onto the window".into());
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            if let Some(paths) = rfd::FileDialog::new()
                .add_filter("Meshes (STL, OBJ)", &MESH_EXTENSIONS)
                .pick_files()
            {
                self.load_files(&paths);
            }
        }
    }

    /// Drag-and-drop: a tinted "drop to open" overlay while files hover
    /// the window, and one instance per file on release.
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
