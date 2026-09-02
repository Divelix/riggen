//! Getting files into the app: CLI arguments, drag-and-drop, File › Open.
//! Every route ends in [`RiggenApp::load_files`]; only
//! [`RiggenApp::open_path`] is lower, and it is the harness's primitive.
//!
//! A `.riggen` replaces the document; an STL/OBJ becomes a `MeshAsset` plus
//! a new link named after the file stem, `Fixed` joint at identity, under
//! the selected link or the root (plan m1-document-tree-joints, decided by
//! the human at step 5).
//!
//! There are two ways in and one reader behind them (ADR-0017).
//! [`RiggenApp::load_files`] takes paths and reads them off the disk;
//! [`RiggenApp::load_dropped`] takes the bytes of one gesture and reads
//! them out of a [`DroppedSet`]. Which one the app is living on is
//! [`Files`], and every mesh the app ever loads goes through it — so the
//! browser, which has no filesystem, runs the same `riggen_core::load_from`
//! and the same `urdf_in::load` as the desktop.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use riggen_core::{Command, Disk, FileSource, Geom, GeomId, Link, LinkId, MeshAsset, MeshId, Pose};

use super::document::name_from_stem;
use super::{LoadedMesh, RiggenApp};

/// The directory dropped files are given, so that every path in the
/// document is absolute exactly as it is on disk (docs/01-architecture.md
/// §File format). No such directory exists anywhere; [`DroppedSet`] never
/// looks at it.
pub(crate) const DROPPED_ROOT: &str = "/dropped";

/// The files of a drop gesture, resolved by **file name** (ADR-0017).
///
/// A browser hands us a flat set of files per gesture and no directory
/// tree, so `meshes/base.stl`, `../base.stl` and `base.stl` all mean the
/// same thing here: the file called `base.stl` that came with this drop.
/// A reference the set does not carry is missing, and the reader that
/// asked says so in the vocabulary it already had — `file::Warning`,
/// `ImportWarning::MeshNotFound`, `ExportError::UnloadableMesh`.
#[derive(Debug, Default, Clone)]
pub struct DroppedSet(BTreeMap<String, Vec<u8>>);

impl DroppedSet {
    /// The gesture's files, keyed by name. A later file with a name an
    /// earlier one already used wins: that is what a second drop of a
    /// re-exported mesh means.
    pub fn new<I, P>(files: I) -> Self
    where
        I: IntoIterator<Item = (P, Vec<u8>)>,
        P: AsRef<Path>,
    {
        let mut set = Self::default();
        set.extend(files);
        set
    }

    pub fn extend<I, P>(&mut self, files: I)
    where
        I: IntoIterator<Item = (P, Vec<u8>)>,
        P: AsRef<Path>,
    {
        for (path, bytes) in files {
            if let Some(name) = file_name(path.as_ref()) {
                self.0.insert(name, bytes);
            }
        }
    }

    /// Where a file of this set lives, as a path: `/dropped/<name>`.
    pub fn path_of(name: &Path) -> PathBuf {
        Path::new(DROPPED_ROOT).join(name.file_name().unwrap_or(name.as_os_str()))
    }
}

fn file_name(path: &Path) -> Option<String> {
    path.file_name().map(|n| n.to_string_lossy().into_owned())
}

impl FileSource for DroppedSet {
    fn read(&self, path: &Path) -> std::io::Result<Vec<u8>> {
        file_name(path)
            .and_then(|name| self.0.get(&name).cloned())
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!(
                        "{} was not among the dropped files",
                        file_name(path).unwrap_or_default()
                    ),
                )
            })
    }

    fn exists(&self, path: &Path) -> bool {
        file_name(path).is_some_and(|name| self.0.contains_key(&name))
    }
}

/// Where the app reads bytes from. The desktop reads the filesystem; the
/// browser reads what was dropped on it (ADR-0017). One field, so no
/// reader in the app has to know which world it is in.
#[derive(Debug, Clone)]
pub enum Files {
    Disk,
    Dropped(DroppedSet),
}

impl FileSource for Files {
    fn read(&self, path: &Path) -> std::io::Result<Vec<u8>> {
        match self {
            Self::Disk => Disk.read(path),
            Self::Dropped(set) => set.read(path),
        }
    }

    fn exists(&self, path: &Path) -> bool {
        match self {
            Self::Disk => Disk.exists(path),
            Self::Dropped(set) => set.exists(path),
        }
    }
}

/// Extensions the open dialog offers, matching `riggen_mesh::load_mesh`.
/// Native only: the browser has no dialog to filter (ADR-0017).
#[cfg(not(target_arch = "wasm32"))]
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
        let abs = match riggen_core::absolute(path) {
            Ok(abs) => abs,
            Err(e) => {
                let err = format!("{}: {e}", path.display());
                self.status = Some(err.clone());
                return Err(err);
            }
        };
        self.open_at(&abs, Some(abs.clone()))
    }

    /// [`RiggenApp::open_path`] for a file that arrived as bytes rather
    /// than as a path — a browser drop, the bundled example. The bytes
    /// join the app's [`Files`], and the same dispatch runs over them
    /// (ADR-0017). A document opened this way is untitled: there is no
    /// path to save it back to.
    pub fn open_bytes(&mut self, name: &Path, bytes: &[u8]) -> Result<Option<LinkId>, String> {
        let files = vec![(name.to_owned(), bytes.to_vec())];
        self.install_dropped(files, replaces_document(name));
        self.open_at(&DroppedSet::path_of(name), None)
    }

    /// Puts `files` into the app's source (ADR-0017 §3).
    ///
    /// A gesture that carries a **document** *replaces* the set: the new
    /// document's meshes are the ones it arrived with, and a mesh it does
    /// not carry is missing — the same answer a moved file gives on disk.
    /// A gesture of meshes alone *adds* to it: those meshes are joining the
    /// document already open, not replacing what it is made of.
    fn install_dropped(&mut self, files: Vec<(PathBuf, Vec<u8>)>, replace: bool) {
        match &mut self.files {
            Files::Dropped(set) if !replace => set.extend(files),
            slot => *slot = Files::Dropped(DroppedSet::new(files)),
        }
    }

    /// The one dispatch, over a path that [`Files`] can resolve. `file` is
    /// what the document's own path becomes: the file on disk, or `None`
    /// for bytes with no filesystem behind them.
    fn open_at(&mut self, at: &Path, file: Option<PathBuf>) -> Result<Option<LinkId>, String> {
        let ext = extension_of(at);
        let result = if ext == DOCUMENT_EXTENSION {
            self.open_document(at, file).map(|()| None)
        } else if ext == URDF_EXTENSION {
            self.open_urdf(at).map(|()| None)
        } else if ext == MJCF_EXTENSION {
            self.open_mjcf(at).map(|()| None)
        } else {
            self.open_mesh(at).map(Some)
        };
        if let Err(err) = &result {
            self.status = Some(err.clone());
        }
        result
    }

    /// Replaces the document with the file's. Warnings (a mesh that changed
    /// or went missing) go to the status bar; the document still opens.
    fn open_document(&mut self, at: &Path, file: Option<PathBuf>) -> Result<(), String> {
        let text = self.read_text(at)?;
        let (robot, warnings) =
            riggen_core::load_from(&text, at, &self.files).map_err(|e| e.to_string())?;
        self.replace_document(robot, file);
        if let Some(first) = warnings.first() {
            self.status = Some(match warnings.len() {
                1 => first.to_string(),
                n => format!("{first} (+{} more warnings)", n - 1),
            });
        }
        Ok(())
    }

    fn read_text(&self, at: &Path) -> Result<String, String> {
        let bytes = self
            .files
            .read(at)
            .map_err(|e| format!("{}: {e}", at.display()))?;
        String::from_utf8(bytes).map_err(|e| format!("{}: {e}", at.display()))
    }

    /// File › Import URDF… (and a dropped `.urdf`): the file becomes a new,
    /// untitled document; what the import dropped goes to the status bar.
    fn open_urdf(&mut self, at: &Path) -> Result<(), String> {
        let imported =
            riggen_export::urdf_in::load(at, &riggen_export::PackageMap::default(), &self.files);
        self.finish_import(at, imported)
    }

    /// File › Import MJCF… (and a dropped `.xml`), the same way through
    /// `riggen_export::mjcf_in` (ADR-0015). One import vocabulary means one
    /// status line for both.
    fn open_mjcf(&mut self, at: &Path) -> Result<(), String> {
        let imported = riggen_export::mjcf_in::load(at, &self.files);
        self.finish_import(at, imported)
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
    fn register_mesh(&mut self, at: &Path) -> Result<(MeshId, PathBuf), String> {
        // Before the read: a `.ply` is a format we do not read, and saying
        // so beats saying the file is missing.
        riggen_mesh::supported_format(at).map_err(|e| e.to_string())?;
        let abs = at.to_owned();
        let bytes = self
            .files
            .read(&abs)
            .map_err(|e| format!("{}: {e}", abs.display()))?;
        let raw = riggen_mesh::load_mesh_bytes(&abs, &bytes).map_err(|e| e.to_string())?;
        let asset = MeshAsset {
            path: abs.clone(),
            content_hash: riggen_core::content_hash(&bytes),
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
    fn open_mesh(&mut self, at: &Path) -> Result<LinkId, String> {
        let (mesh, abs) = self.register_mesh(at)?;
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
        let at = riggen_core::absolute(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let (mesh, _) = self.register_mesh(&at)?;
        let geom = self.geom_for(mesh);
        let id = geom.id;
        self.apply(Command::AddGeom(link, geom))
            .map(|_| id)
            .map_err(|e| e.to_string())
    }

    /// A collision mesh of the link's own (`CollisionPolicy::Meshes`), read
    /// through the file seam like a visual (ADR-0017): appended to the
    /// list, or starting one when the policy was something else. One
    /// `SetCollision`.
    pub fn add_collision_mesh_to_link(
        &mut self,
        link: LinkId,
        path: &Path,
    ) -> Result<GeomId, String> {
        let at = riggen_core::absolute(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut geoms = match self.robot.links.get(&link).map(|l| &l.collision) {
            Some(riggen_core::CollisionPolicy::Meshes(geoms)) => geoms.clone(),
            Some(_) => Vec::new(),
            None => return Err(format!("no link {link}")),
        };
        let (mesh, _) = self.register_mesh(&at)?;
        let geom = self.geom_for(mesh);
        let id = geom.id;
        geoms.push(geom);
        self.apply(Command::SetCollision(
            link,
            riggen_core::CollisionPolicy::Meshes(geoms),
        ))
        .map(|_| id)
        .map_err(|e| e.to_string())
    }

    /// The dialog behind Collision › "Add file…", the twin of
    /// [`add_mesh_dialog`](Self::add_mesh_dialog).
    pub(crate) fn add_collision_mesh_dialog(&mut self, link: LinkId) {
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
                && let Err(err) = self.add_collision_mesh_to_link(link, &path)
            {
                self.status = Some(err);
            }
        }
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
        self.status = report(opened, paths.len(), first_error, warning);
    }

    /// One drop gesture's worth of bytes (ADR-0017).
    ///
    /// The set is the resolution scope: if it carries a document — a
    /// `.riggen`, a `.urdf` or an `.xml` — the meshes beside it are that
    /// document's, not four more links, and every mesh reference resolves
    /// by file name against this same set. A set of meshes alone is what it
    /// has always been: one link per file. This is the one rule that
    /// differs from [`RiggenApp::load_files`], and it differs because on
    /// disk a mesh reference resolves whether or not the mesh was dropped,
    /// and here it does not.
    pub fn load_dropped(&mut self, files: Vec<(PathBuf, Vec<u8>)>) {
        if files.is_empty() {
            return;
        }
        let documents: Vec<PathBuf> = files
            .iter()
            .map(|(path, _)| path.clone())
            .filter(|path| replaces_document(path))
            .collect();
        let replaces = !documents.is_empty();
        let to_open: Vec<PathBuf> = if replaces {
            documents
        } else {
            files.iter().map(|(path, _)| path.clone()).collect()
        };
        self.install_dropped(files, replaces);

        let mut opened = 0usize;
        let mut first_error: Option<String> = None;
        let mut warning: Option<String> = None;
        for name in &to_open {
            match self.open_at(&DroppedSet::path_of(name), None) {
                Ok(_) => {
                    opened += 1;
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
        self.status = report(opened, to_open.len(), first_error, warning);
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

        #[cfg(not(target_arch = "wasm32"))]
        {
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
        // The browser reads a dropped file asynchronously and gives us no
        // path to read it from later, so the whole gesture is read at once
        // and lands in the inbox a frame or two on (ADR-0017).
        #[cfg(target_arch = "wasm32")]
        {
            let files = ctx.input(|i| i.raw.dropped_files.clone());
            if !files.is_empty() {
                let inbox = self.inbox.clone();
                let ctx = ctx.clone();
                wasm_bindgen_futures::spawn_local(async move {
                    let mut batch = Vec::with_capacity(files.len());
                    for file in files {
                        match file.bytes_async().await {
                            Ok(bytes) => batch.push((file.path().to_path_buf(), bytes)),
                            Err(err) => log_drop_error(&file.path().display().to_string(), &err),
                        }
                    }
                    inbox.borrow_mut().push(batch);
                    ctx.request_repaint();
                });
            }
        }
    }

    /// Whatever the browser finished reading since the last frame, opened
    /// as one gesture each. A no-op everywhere else.
    pub(crate) fn drain_dropped(&mut self) {
        #[cfg(target_arch = "wasm32")]
        {
            let batches: Vec<Vec<(PathBuf, Vec<u8>)>> =
                std::mem::take(&mut *self.inbox.borrow_mut());
            for batch in batches {
                if batch.iter().any(|(path, _)| replaces_document(path)) {
                    self.request_open_dropped(batch);
                } else {
                    self.load_dropped(batch);
                }
            }
        }
    }
}

#[cfg(target_arch = "wasm32")]
fn log_drop_error(name: &str, err: &str) {
    web_sys::console::error_1(&format!("riggen: cannot read {name}: {err}").into());
}

/// The status-bar line after a batch of files: the first error if there
/// was one, else the first warning, else the count.
fn report(
    opened: usize,
    asked: usize,
    first_error: Option<String>,
    warning: Option<String>,
) -> Option<String> {
    match (first_error, warning) {
        (Some(err), _) if asked > 1 => Some(format!(
            "opened {opened} of {asked} files; first error: {err}"
        )),
        (Some(err), _) => Some(err),
        (None, Some(warning)) => Some(warning),
        (None, None) => Some(format!(
            "opened {opened} file{}",
            if opened == 1 { "" } else { "s" }
        )),
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
