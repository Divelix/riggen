//! The document and everything derived from it (docs/01-architecture.md
//! §The document is the only state): commands go through `History`, and
//! after every change [`RiggenApp::sync_scene`] makes the viewport's
//! instance table match the document's visual geoms at the FK pose for
//! the current joint values. Nothing else writes to the viewport's scene.

use std::collections::BTreeSet;
use std::path::PathBuf;
use std::sync::Arc;

use riggen_core::glam::{DMat4, DQuat, DVec3};
use riggen_core::{
    Command, EditError, GeomId, History, Joint, JointId, JointState, Link, LinkId, MeshAsset,
    MeshId, Robot, fk,
};
use riggen_mesh::TriMesh;
use riggen_viewport::InstanceId;

use super::RiggenApp;

/// What the user has picked, in document terms. The viewport's own
/// selection is the instance view of this, kept in step both ways.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Selection {
    #[default]
    None,
    Link(LinkId),
    Joint(JointId),
}

impl Selection {
    /// `"link l3"` / `"joint j7"` for the status bar and `debug_state()`.
    pub fn describe(self) -> Option<String> {
        match self {
            Self::None => None,
            Self::Link(l) => Some(format!("link {l}")),
            Self::Joint(j) => Some(format!("joint {j}")),
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
        true
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
    /// bar. Returns the link `AddLink` created.
    pub fn apply(&mut self, command: Command) -> Result<Option<LinkId>, EditError> {
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

    pub fn joint_value(&self, joint: JointId) -> f64 {
        self.q.get(joint)
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
        Ok(created.expect("AddLink returns the new link"))
    }

    /// Removes the selected link with its subtree; for a selected joint,
    /// the link it leads to (the joint is the edge). The root is refused
    /// with the reason in the status bar. Nothing selected: nothing.
    pub fn remove_selected(&mut self) {
        let link = match self.selection {
            Selection::Link(l) => l,
            Selection::Joint(j) => self.robot.joints[&j].child,
            Selection::None => return,
        };
        let _ = self.apply(Command::RemoveLink(link));
    }

    /// Selects in document terms and mirrors it into the viewport (a link's
    /// first visual instance; joints have no instance).
    pub fn select(&mut self, selection: Selection) {
        if selection != self.selection {
            self.props.clear();
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
                None => match riggen_mesh::load_mesh(&asset.path) {
                    Ok(raw) => {
                        self.mesh_store
                            .insert(*mesh_id, LoadedMesh::new(raw, asset));
                    }
                    Err(err) => {
                        first_error.get_or_insert_with(|| err.to_string());
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

        let world = fk(&self.robot, &self.q);
        for (&(link, geom), &id) in &self.instances {
            let Some(link_pose) = world.get(&link) else {
                continue;
            };
            let Some(g) = self
                .robot
                .links
                .get(&link)
                .and_then(|l| l.visuals.iter().find(|g| g.id == geom))
            else {
                continue;
            };
            self.viewport
                .set_instance_model(id, link_pose.compose(&g.pose).to_mat4());
        }

        // The viewport drops a selection whose instance vanished; the
        // document side has to notice, and a link selection whose link
        // still exists but lost its instance is still a valid selection.
        let hit = self.viewport.selected();
        if hit != self.last_viewport_selected && hit.is_none() {
            self.last_viewport_selected = None;
        }
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
