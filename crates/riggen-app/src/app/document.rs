//! The document and everything derived from it (docs/01-architecture.md
//! §The document is the only state): commands go through `History`, and
//! after every change [`RiggenApp::sync_scene`] makes the viewport's
//! instance table match the document's visual geoms at the FK pose for
//! the current joint values. Nothing else writes to the viewport's scene.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::PathBuf;
use std::sync::Arc;

use riggen_core::glam::{DMat4, DQuat, DVec3};
use riggen_core::inertial::{InertialError, LinkInertial, MeshLookup, compose_inertial};
use riggen_core::{
    CollisionPolicy, Command, Created, EditError, FrameId, GeomId, History, Joint, JointId,
    JointState, Link, LinkId, MeshAsset, MeshId, Pose, Primitive, Robot, fk,
};
use riggen_export::{DecompMiss, DecompSource};
use riggen_mesh::feature::Adjacency;
use riggen_mesh::{DecompParams, TriMesh};
use riggen_viewport::{InstanceId, RenderGroup};

/// The translucent orange collision geometry draws in — the MJCF
/// collision class's rgba, a little more opaque so it reads on a light
/// background.
pub const COLLISION_COLOR: [f32; 4] = [0.9, 0.4, 0.1, 0.35];

/// What a collision instance's mesh was built from, so `sync_scene`
/// re-uploads only when that changes (a pose change is a matrix write).
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum CollisionSource {
    /// The convex hull of a visual mesh (`CollisionPolicy::ConvexHull`).
    Hull(MeshId),
    /// A collision-only mesh (`CollisionPolicy::Meshes`).
    Mesh(MeshId),
    /// One piece of a `CollisionPolicy::ConvexDecomposition` of a visual
    /// mesh: the mesh, the parameters that produced the pieces, and which
    /// piece. All three are the key, so re-editing a parameter re-uploads.
    Piece(MeshId, DecompParams, usize),
    Primitive(Primitive),
}

/// A decomposition in the app's cache: requested, back, or hopeless. The
/// document holds the *parameters* and never this (ADR-0011), so the map is
/// derived state like `mesh_store` and is thrown away with the app.
#[derive(Debug, Clone)]
pub(crate) enum DecompState {
    /// A job is in flight; ask again next frame.
    Pending,
    Ready(Vec<Arc<TriMesh>>),
    /// V-HACD found nothing at these parameters; the message is the
    /// panel's and the export's.
    Failed(String),
}

/// The app's [`DecompSource`]: the cache and nothing else. `resolve` must
/// not start a decomposition — the frame that asks has already requested
/// it (`request_decompositions`), and an entry that has not landed blocks
/// the export with `DecompositionPending` (plans/convex-decomposition
/// OPEN 3, decided by the human: no modal, the line clears itself).
pub(crate) struct AppDecomp<'a>(pub(crate) &'a HashMap<(MeshId, DecompParams), DecompState>);

impl DecompSource for AppDecomp<'_> {
    fn pieces(
        &self,
        mesh: MeshId,
        _source: &TriMesh,
        params: DecompParams,
    ) -> Result<Vec<Arc<TriMesh>>, DecompMiss> {
        match self.0.get(&(mesh, params)) {
            Some(DecompState::Ready(pieces)) => Ok(pieces.clone()),
            Some(DecompState::Failed(reason)) => Err(DecompMiss::Degenerate(reason.clone())),
            Some(DecompState::Pending) | None => Err(DecompMiss::Pending),
        }
    }
}

use super::RiggenApp;
use super::file_io::Files;

/// A mesh asset's bytes through the app's [`Files`], parsed by the name it
/// carries — the one place the app turns a `MeshAsset::path` into geometry.
fn read_mesh(files: &Files, path: &std::path::Path) -> Result<TriMesh, String> {
    use riggen_core::FileSource as _;
    let bytes = files
        .read(path)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    riggen_mesh::load_mesh_bytes(path, &bytes).map_err(|e| e.to_string())
}

/// What the user has picked, in document terms. The viewport's own
/// selection is the instance view of this, kept in step both ways.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Selection {
    #[default]
    None,
    Link(LinkId),
    Joint(JointId),
    /// A named frame on a link (ADR-0012). Has no instance in the viewport
    /// — its glyph is drawn by the overlay.
    Frame(FrameId),
}

impl Selection {
    /// `"link l3"` / `"joint j7"` / `"frame f2"` for the status bar and
    /// `debug_state()`.
    pub fn describe(self) -> Option<String> {
        match self {
            Self::None => None,
            Self::Link(l) => Some(format!("link {l}")),
            Self::Joint(j) => Some(format!("joint {j}")),
            Self::Frame(f) => Some(format!("frame {f}")),
        }
    }
}

/// A mesh file as loaded (`raw`, file units) and as the document wants it
/// drawn (`mesh`: `scale` and `fix_up` applied). Keeping `raw` means a
/// scale edit re-derives instead of re-reading the file.
pub(crate) struct LoadedMesh {
    raw: Arc<TriMesh>,
    scale: f64,
    fix_up: Option<DQuat>,
    pub(crate) mesh: Arc<TriMesh>,
    /// Welded topology of `mesh`, built the first time snapping asks for it
    /// and dropped whenever `mesh` is re-derived (`riggen_mesh::feature`).
    adjacency: Option<Adjacency>,
    /// The convex hull of `mesh`, built the first time the collision view
    /// asks for it; `Some(None)` remembers that it is degenerate. Dropped
    /// with `adjacency` when `mesh` is re-derived.
    hull: Option<Option<Arc<TriMesh>>>,
}

impl LoadedMesh {
    pub(crate) fn new(raw: TriMesh, asset: &MeshAsset) -> Self {
        let raw = Arc::new(raw);
        let mesh = Self::derive(&raw, asset);
        Self {
            raw,
            scale: asset.scale,
            fix_up: asset.fix_up,
            mesh,
            adjacency: None,
            hull: None,
        }
    }

    fn derive(raw: &Arc<TriMesh>, asset: &MeshAsset) -> Arc<TriMesh> {
        if asset.scale == 1.0 && asset.fix_up.is_none() {
            return raw.clone();
        }
        let mut mesh = TriMesh::clone(raw);
        mesh.transform(&DMat4::from_scale_rotation_translation(
            DVec3::splat(asset.scale),
            asset.fix_up.unwrap_or(DQuat::IDENTITY),
            DVec3::ZERO,
        ));
        Arc::new(mesh)
    }

    /// Re-derives `mesh` if the asset's scale or fix-up moved. `true` when
    /// it did, so the instances drawing it get re-uploaded.
    fn refresh(&mut self, asset: &MeshAsset) -> bool {
        if self.scale == asset.scale && self.fix_up == asset.fix_up {
            return false;
        }
        self.scale = asset.scale;
        self.fix_up = asset.fix_up;
        self.mesh = Self::derive(&self.raw, asset);
        self.adjacency = None;
        self.hull = None;
        true
    }
}

impl LoadedMesh {
    /// The welded adjacency of the drawn mesh, built on first use. Every
    /// circle fit needs it and it is the expensive half, so it outlives the
    /// per-triangle memo in `snap.rs`.
    pub(crate) fn adjacency(&mut self) -> &Adjacency {
        if self.adjacency.is_none() {
            self.adjacency = Some(riggen_mesh::feature::adjacency(&self.mesh));
        }
        self.adjacency.as_ref().expect("just built")
    }

    /// The convex hull of the drawn mesh, built on first use and cached
    /// (plans/m3-sim-ready: synchronous, per `MeshId`). `None` for a mesh
    /// that spans no volume.
    pub(crate) fn hull(&mut self) -> Option<Arc<TriMesh>> {
        if self.hull.is_none() {
            self.hull = Some(
                riggen_mesh::convex_hull(&self.mesh.positions)
                    .ok()
                    .map(|mut h| {
                        h.flat_normals();
                        Arc::new(h)
                    }),
            );
        }
        self.hull.as_ref().expect("just built").clone()
    }
}

/// The app's mesh store as core's [`MeshLookup`]: the drawn meshes, in
/// meters, which is what `compose_inertial` and `resolve` read.
pub(crate) struct AppMeshes<'a>(pub(crate) &'a HashMap<MeshId, LoadedMesh>);

impl MeshLookup for AppMeshes<'_> {
    fn mesh(&self, id: MeshId) -> Option<&TriMesh> {
        self.0.get(&id).map(|l| &*l.mesh)
    }
}

/// The mesh a primitive draws as, at its size, centred on its own frame.
pub(crate) fn primitive_mesh(p: &Primitive) -> TriMesh {
    match p {
        Primitive::Box { size, .. } => {
            let mut mesh = TriMesh::cube(0.5);
            mesh.transform(&DMat4::from_scale(*size));
            mesh
        }
        Primitive::Cylinder { radius, length, .. } => TriMesh::cylinder(*radius, *length, 32),
        Primitive::Sphere { radius, .. } => TriMesh::sphere(*radius, 24),
        Primitive::Capsule { radius, length, .. } => TriMesh::capsule(*radius, *length, 24),
    }
}

pub(crate) fn primitive_pose(p: &Primitive) -> Pose {
    match p {
        Primitive::Box { pose, .. }
        | Primitive::Cylinder { pose, .. }
        | Primitive::Sphere { pose, .. }
        | Primitive::Capsule { pose, .. } => *pose,
    }
}

/// Turns a file stem into a valid link name (`validate::is_valid_name`):
/// every other character becomes `_`, a leading digit gets one in front,
/// nothing at all is `part`.
pub(crate) fn name_from_stem(stem: &str) -> String {
    let mut name: String = stem
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || matches!(c, '_' | '.' | '-') {
                c
            } else {
                '_'
            }
        })
        .collect();
    match name.chars().next() {
        None => name = "part".into(),
        Some(c) if !(c.is_ascii_alphabetic() || c == '_') => name.insert(0, '_'),
        _ => {}
    }
    name
}

/// `base`, or the first of `base_2`, `base_3`, … that `taken` does not
/// contain.
pub(crate) fn unique_name(base: &str, taken: &BTreeSet<&str>) -> String {
    if !taken.contains(base) {
        return base.to_owned();
    }
    (2..)
        .map(|n| format!("{base}_{n}"))
        .find(|candidate| !taken.contains(candidate.as_str()))
        .expect("the integers do not run out")
}

impl RiggenApp {
    pub fn robot(&self) -> &Robot {
        &self.robot
    }

    pub fn history(&self) -> &History {
        &self.history
    }

    /// The `.riggen` file the document came from or was last saved to.
    pub fn file(&self) -> Option<&PathBuf> {
        self.file.as_ref()
    }

    /// `name.riggen` or `untitled`, for the status bar and the title.
    pub(crate) fn document_label(&self) -> String {
        self.file
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "untitled".into())
    }

    pub fn selection(&self) -> Selection {
        self.selection
    }

    /// The scale a dropped mesh gets (`MeshAsset::scale`); `0.001` reads a
    /// millimetre file into meters.
    pub fn import_scale(&self) -> f64 {
        self.import_scale
    }

    pub fn set_import_scale(&mut self, scale: f64) {
        if scale.is_finite() && scale > 0.0 {
            self.import_scale = scale;
        }
    }

    /// Runs a command through the history and re-syncs the scene. A
    /// refused command changes nothing and puts its reason in the status
    /// bar. Returns what `AddLink` / `AddFrame` created.
    pub fn apply(&mut self, command: Command) -> Result<Option<Created>, EditError> {
        let result = self.history.apply(&mut self.robot, command);
        match &result {
            Ok(_) => self.after_document_change(),
            Err(err) => self.status = Some(err.to_string()),
        }
        result
    }

    /// `false` when there was nothing to undo.
    pub fn undo(&mut self) -> bool {
        let done = self.history.undo(&mut self.robot);
        if done {
            self.after_document_change();
        }
        done
    }

    /// `false` when there was nothing to redo.
    pub fn redo(&mut self) -> bool {
        let done = self.history.redo(&mut self.robot);
        if done {
            self.after_document_change();
        }
        done
    }

    /// Swaps in a new document (New, Open, a dropped `.riggen`): fresh
    /// history, no selection, joint values at zero, meshes loaded from the
    /// assets. The old mesh store is dropped — a file's assets are its own.
    pub(crate) fn replace_document(&mut self, robot: Robot, file: Option<PathBuf>) {
        self.robot = robot;
        self.file = file;
        self.history = History::new();
        self.mesh_store.clear();
        self.instances.clear();
        self.collision_instances.clear();
        self.viewport.clear_scene();
        self.q = JointState::default();
        self.selection = Selection::None;
        self.last_viewport_selected = None;
        self.sync_scene();
    }

    /// Sets one joint's value (a slider) and moves the instances; not a
    /// command — `q` is derived state, never saved. Clamped to the limits.
    pub fn set_joint_value(&mut self, joint: JointId, q: f64) {
        let Some(j) = self.robot.joints.get(&joint) else {
            return;
        };
        let q = match j.limits {
            Some(limits) => q.clamp(limits.lower, limits.upper),
            None => q,
        };
        self.q.set(joint, q);
        self.sync_scene();
    }

    /// What the joint is actually at — for one that follows another, the
    /// value its leader implies (ADR-0013), which is what `fk` used and
    /// what the viewport is showing, not the stale slot in `self.q`.
    /// `fk::resolve_q` is the one place that rule lives.
    pub fn joint_value(&self, joint: JointId) -> f64 {
        riggen_core::resolve_q(&self.robot, &self.q).get(joint)
    }

    /// Tints every instance whose link uses `material` with `color`
    /// without touching the document — the colour picker's live preview.
    /// The next `sync_scene` restores the document's colour.
    pub(crate) fn preview_material_color(&mut self, material: &str, color: [f32; 4]) {
        for (&(link, geom), &id) in &self.instances {
            let Some(l) = self.robot.links.get(&link) else {
                continue;
            };
            let own = l.visuals.iter().any(|g| g.id == geom && g.color.is_some());
            if !own && l.material.as_deref() == Some(material) {
                self.viewport.set_instance_color(id, color);
            }
        }
    }

    /// Every joint back to zero ("Reset all").
    pub fn reset_joint_values(&mut self) {
        self.q = JointState::default();
        self.sync_scene();
    }

    /// Where a new link goes: under the selected link, under a selected
    /// joint's child, else under the root.
    pub fn insertion_parent(&self) -> LinkId {
        match self.selection {
            Selection::Link(l) => l,
            Selection::Joint(j) => self.robot.joints[&j].child,
            // A frame's own link: "add here" means beside the thing shown.
            Selection::Frame(f) => self.robot.frames[&f].parent,
            Selection::None => self.robot.root,
        }
    }

    /// Adds `link` under `parent` with a `Fixed` joint at identity. The
    /// link's name is made unique (`arm`, `arm_2`) and the joint is named
    /// `<name>_joint`, unique the same way. The selection is left alone so
    /// a batch of dropped files lands side by side, not chained.
    pub fn add_link(&mut self, mut link: Link, parent: LinkId) -> Result<LinkId, EditError> {
        let link_names: BTreeSet<&str> =
            self.robot.links.values().map(|l| l.name.as_str()).collect();
        link.name = unique_name(&link.name, &link_names);
        let joint_names: BTreeSet<&str> = self
            .robot
            .joints
            .values()
            .map(|j| j.name.as_str())
            .collect();
        let joint_name = unique_name(&format!("{}_joint", link.name), &joint_names);
        let joint = Joint::fixed(joint_name, parent, parent);
        let created = self.apply(Command::AddLink {
            link: Box::new(link),
            parent,
            joint,
        })?;
        Ok(created
            .and_then(Created::link)
            .expect("AddLink returns the new link"))
    }

    /// Adds a named frame at `link`'s origin, its name made unique across
    /// the one namespace frames and links share (`frame`, `frame_2` —
    /// ADR-0012). `None` when the command was refused.
    pub fn add_frame(&mut self, link: LinkId) -> Option<FrameId> {
        let taken: BTreeSet<&str> = self
            .robot
            .links
            .values()
            .map(|l| l.name.as_str())
            .chain(self.robot.frames.values().map(|f| f.name.as_str()))
            .collect();
        let frame = riggen_core::Frame {
            name: unique_name("frame", &taken),
            parent: link,
            pose: Pose::IDENTITY,
        };
        self.apply(Command::AddFrame(frame))
            .ok()
            .flatten()
            .and_then(Created::frame)
    }

    /// Removes what is selected: a link with its subtree; for a joint, the
    /// link it leads to (the joint is the edge); for a frame, the frame
    /// alone. The root is refused with the reason in the status bar.
    /// Nothing selected: nothing.
    pub fn remove_selected(&mut self) {
        let link = match self.selection {
            Selection::Link(l) => l,
            Selection::Joint(j) => self.robot.joints[&j].child,
            Selection::Frame(f) => {
                let _ = self.apply(Command::RemoveFrame(f));
                return;
            }
            Selection::None => return,
        };
        let _ = self.apply(Command::RemoveLink(link));
    }

    /// Selects in document terms and mirrors it into the viewport (a link's
    /// first visual instance; joints and frames have none).
    pub fn select(&mut self, selection: Selection) {
        if selection != self.selection {
            self.props.clear();
            // The align gesture is about the link that was selected when it
            // started; selecting another abandons it.
            self.cancel_align();
        }
        self.selection = selection;
        let instance = match selection {
            Selection::Link(link) => self
                .instances
                .iter()
                .find(|((l, _), _)| *l == link)
                .map(|(_, id)| *id),
            _ => None,
        };
        self.viewport.set_selected(instance);
        self.last_viewport_selected = self.viewport.selected();
    }

    /// The link whose instance the viewport's last click hit. Called once
    /// per frame after `Viewport::ui`, when a resolved select pick may
    /// have changed the viewport's idea of the selection.
    pub(crate) fn sync_selection_from_viewport(&mut self) {
        let hit = self.viewport.selected();
        if hit == self.last_viewport_selected {
            return;
        }
        self.last_viewport_selected = hit;
        self.selection = hit
            .and_then(|h| self.link_of_instance(h.instance))
            .map_or(Selection::None, Selection::Link);
    }

    pub(crate) fn link_of_instance(&self, instance: InstanceId) -> Option<LinkId> {
        self.instances
            .iter()
            .find(|(_, id)| **id == instance)
            .map(|((link, _), _)| *link)
    }

    /// The link a *collision* instance belongs to (`debug_state()`); the
    /// pick pass never hits one, so nothing else asks.
    pub(crate) fn collision_link_of_instance(&self, instance: InstanceId) -> Option<LinkId> {
        self.collision_instances
            .iter()
            .find(|(_, (id, _))| *id == instance)
            .map(|((link, _), _)| *link)
    }

    /// The link's inertial under its `InertialSpec`, from the loaded meshes
    /// (docs/02-data-model.md §Inertials). What the properties panel's
    /// Inertial block shows and the export resolves.
    pub fn link_inertial(&self, link: LinkId) -> Result<LinkInertial, InertialError> {
        let data = self
            .robot
            .links
            .get(&link)
            .ok_or(InertialError::NoDensity)?;
        compose_inertial(data, &AppMeshes(&self.mesh_store), &self.robot.materials)
    }

    /// View › Collision geometry.
    pub fn show_collision(&self) -> bool {
        self.show_collision
    }

    pub fn set_show_collision(&mut self, show: bool) {
        if self.show_collision != show {
            self.show_collision = show;
            self.sync_scene();
        }
    }

    pub(crate) fn geom_of_instance(&self, instance: InstanceId) -> Option<GeomId> {
        self.instances
            .iter()
            .find(|(_, id)| **id == instance)
            .map(|((_, geom), _)| *geom)
    }

    /// `"arm"` for a hit, so the status bar can say what was hit rather
    /// than only which instance.
    pub(crate) fn link_name_of_instance(&self, instance: InstanceId) -> Option<&str> {
        self.link_of_instance(instance)
            .and_then(|l| self.robot.links.get(&l))
            .map(|l| l.name.as_str())
    }

    fn after_document_change(&mut self) {
        // Joints that vanished take their q with them; the rest stay within
        // limits that may just have moved.
        let joints = &self.robot.joints;
        self.q.0.retain(|j, _| joints.contains_key(j));
        for (jid, joint) in joints {
            if let Some(limits) = joint.limits
                && let Some(q) = self.q.0.get_mut(jid)
            {
                *q = q.clamp(limits.lower, limits.upper);
            }
        }
        if let Selection::Link(l) = self.selection
            && !self.robot.links.contains_key(&l)
        {
            self.selection = Selection::None;
        }
        if let Selection::Joint(j) = self.selection
            && !self.robot.joints.contains_key(&j)
        {
            self.selection = Selection::None;
        }
        if let Selection::Frame(f) = self.selection
            && !self.robot.frames.contains_key(&f)
        {
            self.selection = Selection::None;
        }
        self.sync_scene();
    }

    /// Makes the viewport match the document: one instance per visual geom,
    /// each at `fk(robot, q)[link] ∘ geom.pose`. Adds what is new, removes
    /// what is gone, re-uploads a mesh whose asset scale changed, and
    /// writes every model matrix. Cheap enough to run after every command
    /// and every slider tick — a matrix write uploads nothing.
    pub(crate) fn sync_scene(&mut self) {
        let wanted: Vec<((LinkId, GeomId), MeshId)> = self
            .robot
            .links
            .iter()
            .flat_map(|(&l, link)| link.visuals.iter().map(move |g| ((l, g.id), g.mesh)))
            .collect();
        let live: BTreeSet<(LinkId, GeomId)> = wanted.iter().map(|(k, _)| *k).collect();

        let gone: Vec<(LinkId, GeomId)> = self
            .instances
            .keys()
            .filter(|k| !live.contains(k))
            .copied()
            .collect();
        for key in gone {
            if let Some(id) = self.instances.remove(&key) {
                self.viewport.remove_instance(id);
            }
        }

        let mut reupload: BTreeSet<MeshId> = BTreeSet::new();
        let mut first_error: Option<String> = None;
        for (key, mesh_id) in &wanted {
            let Some(asset) = self.robot.assets.get(mesh_id) else {
                continue; // validate rejects this; nothing to draw
            };
            match self.mesh_store.get_mut(mesh_id) {
                Some(loaded) => {
                    if loaded.refresh(asset) {
                        reupload.insert(*mesh_id);
                    }
                }
                // The asset's bytes come from wherever the app is living:
                // the filesystem, or the dropped set (ADR-0017).
                None => match read_mesh(&self.files, &asset.path) {
                    Ok(raw) => {
                        self.mesh_store
                            .insert(*mesh_id, LoadedMesh::new(raw, asset));
                    }
                    Err(err) => {
                        first_error.get_or_insert(err);
                        continue;
                    }
                },
            }
            let needs_upload = !self.instances.contains_key(key) || reupload.contains(mesh_id);
            if needs_upload {
                let id = *self.instances.entry(*key).or_insert_with(|| {
                    let id = InstanceId(self.next_instance);
                    self.next_instance += 1;
                    id
                });
                let mesh = self.mesh_store[mesh_id].mesh.clone();
                if let Err(err) = self.viewport.set_instance(id, &mesh) {
                    first_error.get_or_insert_with(|| err.to_string());
                    self.instances.remove(key);
                }
            }
        }
        if let Some(err) = first_error {
            self.status = Some(err);
        }

        let mut world = fk(&self.robot, &self.q);
        // A gizmo drag previews a link's world pose without touching the
        // document: correct that link and everything under it, so the
        // subtree follows the handle exactly as it will after the commit.
        if let Some((link, pose)) = self.preview_world
            && let Some(current) = world.get(&link).copied()
        {
            let correction = pose.compose(&current.inverse());
            for l in self.robot.subtree(link) {
                if let Some(p) = world.get_mut(&l) {
                    *p = correction.compose(p);
                }
            }
        }
        for (&(link, geom), &id) in &self.instances {
            let Some(link_pose) = world.get(&link) else {
                continue;
            };
            let Some(l) = self.robot.links.get(&link) else {
                continue;
            };
            let Some(g) = l.visuals.iter().find(|g| g.id == geom) else {
                continue;
            };
            self.viewport
                .set_instance_model(id, link_pose.compose(&g.pose).to_mat4());
            // The geom's own colour wins, then the link's material, then
            // the viewport default.
            let color = g
                .color
                .or_else(|| {
                    l.material
                        .as_ref()
                        .and_then(|m| self.robot.materials.get(m))
                        .map(|m| m.color)
                })
                .unwrap_or(riggen_viewport::DEFAULT_INSTANCE_COLOR);
            self.viewport.set_instance_color(id, color);
        }

        self.sync_collision(&world);

        // The viewport drops a selection whose instance vanished; the
        // document side has to notice, and a link selection whose link
        // still exists but lost its instance is still a valid selection.
        let hit = self.viewport.selected();
        if hit != self.last_viewport_selected && hit.is_none() {
            self.last_viewport_selected = None;
        }
    }

    /// The translucent collision instances, from each link's policy at the
    /// same FK poses as the visuals: nothing for `None` / `SameAsVisual`
    /// (the visuals already show it), a hull per visual for `ConvexHull`,
    /// each primitive, each `Meshes` geom. All removed when the view is
    /// off. Uploads only when a shape's source changed.
    fn sync_collision(&mut self, world: &BTreeMap<LinkId, Pose>) {
        // (key, source, mesh to upload if the source is new, pose in link)
        let mut wanted: Vec<((LinkId, usize), CollisionSource, Pose)> = Vec::new();
        if self.show_collision {
            for (&lid, link) in &self.robot.links {
                let mut shapes: Vec<(CollisionSource, Pose)> = Vec::new();
                match &link.collision {
                    CollisionPolicy::None | CollisionPolicy::SameAsVisual => {}
                    CollisionPolicy::ConvexHull => {
                        for g in &link.visuals {
                            shapes.push((CollisionSource::Hull(g.mesh), g.pose));
                        }
                    }
                    CollisionPolicy::ConvexDecomposition {
                        max_hulls,
                        resolution,
                        concavity,
                    } => {
                        let params = DecompParams {
                            max_hulls: *max_hulls,
                            resolution: *resolution,
                            concavity: *concavity,
                        };
                        for g in &link.visuals {
                            // Nothing until the job lands; the frame it
                            // does, `wake` has already asked for a repaint.
                            let Some(DecompState::Ready(pieces)) =
                                self.decomp.get(&(g.mesh, params))
                            else {
                                continue;
                            };
                            for i in 0..pieces.len() {
                                shapes.push((CollisionSource::Piece(g.mesh, params, i), g.pose));
                            }
                        }
                    }
                    CollisionPolicy::Primitives(ps) => {
                        for p in ps {
                            shapes.push((CollisionSource::Primitive(p.clone()), primitive_pose(p)));
                        }
                    }
                    CollisionPolicy::Meshes(geoms) => {
                        for g in geoms {
                            shapes.push((CollisionSource::Mesh(g.mesh), g.pose));
                        }
                    }
                }
                for (i, (source, pose)) in shapes.into_iter().enumerate() {
                    wanted.push(((lid, i), source, pose));
                }
            }
        }

        let live: BTreeSet<(LinkId, usize)> = wanted.iter().map(|(k, _, _)| *k).collect();
        let gone: Vec<(LinkId, usize)> = self
            .collision_instances
            .keys()
            .filter(|k| !live.contains(k))
            .copied()
            .collect();
        for key in gone {
            if let Some((id, _)) = self.collision_instances.remove(&key) {
                self.viewport.remove_instance(id);
            }
        }

        for (key, source, pose) in wanted {
            let current = self.collision_instances.get(&key).cloned();
            let needs_upload = current.as_ref().is_none_or(|(_, s)| *s != source);
            if needs_upload {
                let mesh: Option<Arc<TriMesh>> = match &source {
                    CollisionSource::Hull(mesh_id) => {
                        self.ensure_loaded(*mesh_id).and_then(|l| l.hull())
                    }
                    CollisionSource::Mesh(mesh_id) => {
                        self.ensure_loaded(*mesh_id).map(|l| l.mesh.clone())
                    }
                    CollisionSource::Piece(mesh_id, params, i) => {
                        match self.decomp.get(&(*mesh_id, *params)) {
                            Some(DecompState::Ready(pieces)) => pieces.get(*i).cloned(),
                            _ => None,
                        }
                    }
                    CollisionSource::Primitive(p) => Some(Arc::new(primitive_mesh(p))),
                };
                let Some(mesh) = mesh else {
                    // Degenerate hull or unloadable mesh: nothing to draw;
                    // export reports it properly.
                    if let Some((id, _)) = self.collision_instances.remove(&key) {
                        self.viewport.remove_instance(id);
                    }
                    continue;
                };
                let id = match current {
                    Some((id, _)) => id,
                    None => {
                        let id = InstanceId(self.next_instance);
                        self.next_instance += 1;
                        id
                    }
                };
                if let Err(err) = self.viewport.set_instance(id, &mesh) {
                    self.status = Some(err.to_string());
                    self.collision_instances.remove(&key);
                    continue;
                }
                self.viewport
                    .set_instance_group(id, RenderGroup::Translucent);
                self.viewport.set_instance_color(id, COLLISION_COLOR);
                self.collision_instances.insert(key, (id, source));
            }
            let Some((id, _)) = self.collision_instances.get(&key) else {
                continue;
            };
            if let Some(link_pose) = world.get(&key.0) {
                self.viewport
                    .set_instance_model(*id, link_pose.compose(&pose).to_mat4());
            }
        }
    }

    /// The store entry for `mesh_id`, loading the file on first use.
    /// `None` when the asset is missing or the file does not load — the
    /// visual sync already put that error in the status bar.
    /// The decomposition parameters every link asks for, as
    /// `(mesh, params)` pairs — one per visual of every link whose policy
    /// is `ConvexDecomposition`.
    pub(crate) fn wanted_decompositions(&self) -> Vec<(MeshId, DecompParams)> {
        let mut wanted = Vec::new();
        for link in self.robot.links.values() {
            let CollisionPolicy::ConvexDecomposition {
                max_hulls,
                resolution,
                concavity,
            } = link.collision
            else {
                continue;
            };
            let params = DecompParams {
                max_hulls,
                resolution,
                concavity,
            };
            for g in &link.visuals {
                if !wanted.contains(&(g.mesh, params)) {
                    wanted.push((g.mesh, params));
                }
            }
        }
        wanted
    }

    /// Asks the job thread for every decomposition the document wants and
    /// does not have. Idempotent: `Jobs::request` drops a key already in
    /// flight, and a cached entry is never asked for again. Called once per
    /// frame and before an export resolves.
    pub(crate) fn request_decompositions(&mut self) {
        for (mesh, params) in self.wanted_decompositions() {
            if self.decomp.contains_key(&(mesh, params)) {
                continue;
            }
            let Some(source) = self.ensure_loaded(mesh).map(|l| l.mesh.clone()) else {
                continue; // Unloadable; `resolve` reports it properly.
            };
            if self.jobs.request(crate::jobs::Job::Decompose {
                mesh,
                params,
                source,
            }) {
                self.decomp.insert((mesh, params), DecompState::Pending);
            }
        }
    }

    /// Moves everything the job thread finished into the cache, and
    /// re-syncs the scene if anything landed — a decomposition changes what
    /// the collision view draws, and the document did not change, so
    /// nothing else would. Called once per frame at the top of `ui`, on the
    /// frame the worker's `wake` asked for.
    pub(crate) fn drain_jobs(&mut self) {
        let mut landed = false;
        for result in self.jobs.drain() {
            let crate::jobs::JobResult::Decomposed {
                mesh,
                params,
                pieces,
            } = result;
            let state = match pieces {
                Ok(pieces) => DecompState::Ready(pieces),
                Err(reason) => DecompState::Failed(reason),
            };
            self.decomp.insert((mesh, params), state);
            landed = true;
        }
        if landed {
            self.sync_scene();
        }
    }

    pub(crate) fn ensure_loaded(&mut self, mesh_id: MeshId) -> Option<&mut LoadedMesh> {
        let asset = self.robot.assets.get(&mesh_id)?;
        if !self.mesh_store.contains_key(&mesh_id) {
            let raw = read_mesh(&self.files, &asset.path).ok()?;
            self.mesh_store.insert(mesh_id, LoadedMesh::new(raw, asset));
        }
        let loaded = self.mesh_store.get_mut(&mesh_id)?;
        loaded.refresh(asset);
        Some(loaded)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stems_become_valid_names() {
        assert_eq!(name_from_stem("arm"), "arm");
        assert_eq!(name_from_stem("my part v2"), "my_part_v2");
        assert_eq!(name_from_stem("3d_print"), "_3d_print");
        assert_eq!(name_from_stem(""), "part");
        assert_eq!(name_from_stem("ärm"), "_rm");
        for stem in ["arm", "my part v2", "3d_print", "", "ärm", "a.b-c"] {
            assert!(riggen_core::validate::is_valid_name(&name_from_stem(stem)));
        }
    }

    #[test]
    fn unique_names_count_up() {
        let taken: BTreeSet<&str> = ["arm", "arm_2", "leg"].into_iter().collect();
        assert_eq!(unique_name("hand", &taken), "hand");
        assert_eq!(unique_name("arm", &taken), "arm_3");
        assert_eq!(unique_name("leg", &taken), "leg_2");
    }
}
