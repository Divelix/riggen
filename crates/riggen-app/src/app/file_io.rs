//! Getting files into the app: CLI arguments, drag-and-drop, File › Open.
//! Every route ends in [`RiggenApp::load_files`]; only
//! [`RiggenApp::open_path`] is lower, and it is the harness's primitive.
//!
//! A `.riggen` replaces the document; an STL/OBJ becomes a `MeshAsset` plus
//! a new link named after the file stem, `Fixed` joint at identity, under
//! the selected link or the root (plan m1-document-tree-joints, decided by
//! the human at step 5).

use std::path::{Path, PathBuf};

use riggen_core::{Command, Geom, GeomId, Link, LinkId, MeshAsset, MeshId, Pose};

use super::document::name_from_stem;
use super::{LoadedMesh, RiggenApp};

/// Extensions the open dialog offers, matching `riggen_mesh::load_mesh`.
const MESH_EXTENSIONS: [&str; 2] = ["stl", "obj"];
/// The document's own extension.
pub(crate) const DOCUMENT_EXTENSION: &str = "riggen";
/// A URDF opens as a new document through `riggen_export::urdf_in`.
pub(crate) const URDF_EXTENSION: &str = "urdf";
/// An MJCF opens as a new document through `riggen_export::mjcf_in`
/// (ADR-0015). MJCF has no extension of its own; `.xml` is what MuJoCo
/// ships and what our own export writes.
pub(crate) const MJCF_EXTENSION: &str = "xml";

/// Whether opening `path` replaces the document (a `.riggen`, a `.urdf` or
/// an `.xml`) rather than adding a link to it.
pub(crate) fn replaces_document(path: &Path) -> bool {
    let ext = extension_of(path);
    ext == DOCUMENT_EXTENSION || ext == URDF_EXTENSION || ext == MJCF_EXTENSION
}

pub(crate) fn extension_of(path: &Path) -> String {
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
        let ext = extension_of(path);
        let result = if ext == DOCUMENT_EXTENSION {
            self.open_document_path(path).map(|()| None)
        } else if ext == URDF_EXTENSION {
            self.open_urdf_path(path).map(|()| None)
        } else if ext == MJCF_EXTENSION {
            self.open_mjcf_path(path).map(|()| None)
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
        let abs = riggen_core::absolute(path).map_err(|e| format!("{}: {e}", path.display()))?;
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

    /// File › Import URDF… (and a dropped `.urdf`): the file becomes a new,
    /// untitled document; what the import dropped goes to the status bar.
    fn open_urdf_path(&mut self, path: &Path) -> Result<(), String> {
        let imported = riggen_export::urdf_in::load(path, &riggen_export::PackageMap::default());
        self.finish_import(path, imported)
    }

    /// File › Import MJCF… (and a dropped `.xml`), the same way through
    /// `riggen_export::mjcf_in` (ADR-0015). One import vocabulary means one
    /// status line for both.
    fn open_mjcf_path(&mut self, path: &Path) -> Result<(), String> {
        let imported = riggen_export::mjcf_in::load(path);
        self.finish_import(path, imported)
    }

    /// What both imports do with their result: a new, untitled document,
    /// and what was dropped in the status bar.
    fn finish_import(
        &mut self,
        path: &Path,
        imported: Result<
            (riggen_core::Robot, Vec<riggen_export::ImportWarning>),
            riggen_export::ImportError,
        >,
    ) -> Result<(), String> {
        let (robot, warnings) = imported.map_err(|e| e.to_string())?;
        self.replace_document(robot, None);
        let name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        self.status = Some(match warnings.as_slice() {
            [] => format!("imported {name}"),
            [first] => format!("imported {name}: {first}"),
            [first, rest @ ..] => {
                format!("imported {name}: {first} (+{} more warnings)", rest.len())
            }
        });
        Ok(())
    }

    /// Loads a mesh file and registers it as an asset at the import scale.
    /// Not a command: the asset stays for the session, so undoing the
    /// link or geom that uses it and redoing never reloads the file.
    fn register_mesh(&mut self, path: &Path) -> Result<(MeshId, PathBuf), String> {
        let abs = riggen_core::absolute(path).map_err(|e| format!("{}: {e}", path.display()))?;
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
        Ok((mesh, abs))
    }

    fn geom_for(&mut self, mesh: MeshId) -> Geom {
        Geom {
            id: self.robot.next_id.alloc(),
            mesh,
            pose: Pose::IDENTITY,
            color: None,
        }
    }

    /// A dropped mesh: a new link named after the file under the selection
    /// or the root, through `AddLink`.
    fn open_mesh_path(&mut self, path: &Path) -> Result<LinkId, String> {
        let (mesh, abs) = self.register_mesh(path)?;
        let stem = abs
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        let mut link = Link::new(name_from_stem(&stem));
        let geom = self.geom_for(mesh);
        link.visuals.push(geom);
        let parent = self.insertion_parent();
        let added = self.add_link(link, parent).map_err(|e| e.to_string())?;
        // An open shell has no volume to weigh: say so at the drop, since
        // the export will refuse it later (docs/02-data-model.md §Inertials).
        let closed = self
            .mesh_store
            .get_mut(&mesh)
            .is_none_or(|loaded| loaded.adjacency().is_closed());
        self.status = (!closed).then(|| open_mesh_warning(&abs));
        Ok(added)
    }

    /// "Add mesh to this link…": the file as another visual geom of
    /// `link`, at identity in the link frame, through `AddGeom`.
    pub fn add_mesh_to_link(&mut self, link: LinkId, path: &Path) -> Result<GeomId, String> {
        let (mesh, _) = self.register_mesh(path)?;
        let geom = self.geom_for(mesh);
        let id = geom.id;
        self.apply(Command::AddGeom(link, geom))
            .map(|_| id)
            .map_err(|e| e.to_string())
    }

    /// The dialog behind "Add mesh to this link…".
    pub(crate) fn add_mesh_dialog(&mut self, link: LinkId) {
        #[cfg(target_arch = "wasm32")]
        {
            let _ = link;
            self.status = Some("no filesystem in the browser; drop files onto the window".into());
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("Meshes (STL, OBJ)", &MESH_EXTENSIONS)
                .pick_file()
                && let Err(err) = self.add_mesh_to_link(link, &path)
            {
                self.status = Some(err);
            }
        }
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
                    // A document or URDF that opened with warnings, or an
                    // open mesh, left a warning here.
                    if let Some(w) = self.status.take() {
                        warning = Some(w);
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
                .add_filter("URDF", &[URDF_EXTENSION])
                .add_filter("MJCF", &[MJCF_EXTENSION])
                .pick_files()
            {
                self.load_files(&paths);
            }
        }
    }

    /// File › Import URDF…: the dialog, then the dirty check (a URDF
    /// replaces the document).
    pub(crate) fn import_urdf_dialog(&mut self) {
        self.import_dialog("URDF", URDF_EXTENSION);
    }

    /// File › Import MJCF…, the same (ADR-0015).
    pub(crate) fn import_mjcf_dialog(&mut self) {
        self.import_dialog("MJCF", MJCF_EXTENSION);
    }

    fn import_dialog(&mut self, label: &str, extension: &str) {
        #[cfg(target_arch = "wasm32")]
        {
            let _ = (label, extension);
            self.status =
                Some("no filesystem in the browser; drop the file onto the window".into());
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter(label, &[extension])
                .pick_file()
            {
                self.request_open(vec![path]);
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
        if !dropped.is_empty() {
            // A dropped `.riggen` replaces the document: dirty check first.
            self.request_open(dropped);
        }
    }
}

/// The status-bar line for a dropped mesh that is not closed.
pub fn open_mesh_warning(path: &Path) -> String {
    format!(
        "{}: mesh is not closed, so its mass properties cannot be computed",
        path.file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| path.display().to_string())
    )
}
