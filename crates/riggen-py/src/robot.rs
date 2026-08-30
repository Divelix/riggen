//! `riggen._riggen.Robot`: the document, one method per `Command`
//! (docs/02-data-model.md §Commands and history), read access in the
//! schema's shape ([`crate::doc`]), ids as ints.
//!
//! Every edit runs on a clone and replaces the document only on success —
//! `Command::apply` can leave a half-edited robot behind a validation
//! failure, which is why `History::apply` clones too. A refused edit raises
//! and changes nothing, the id counter included.

use std::path::{Path, PathBuf};

use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::collections::BTreeMap;

use pyo3::exceptions::{PyOSError, PyValueError};
use riggen_core::glam::{DQuat, DVec3};
use riggen_core::{
    CollisionPolicy, Command, EditError, Geom, GeomId, Id, InertialSpec, Joint, JointId,
    JointState, Link, LinkId, Material, MeshAsset, MeshId, Pose, Robot, compose_inertial,
    validation_errors,
};
use riggen_export::{ExportError, ExportOptions, Format, MeshPathStyle, MeshStore, PackageMap};
use serde::{Deserialize, Serialize};
use serde_json::json;

use crate::doc::{from_doc, from_doc_with, to_doc};
use crate::errors::{edit_error, raise};

/// The `.riggen` envelope, as `riggen_core::file` writes it; here for
/// `to_json` / `from_json`, which carry no paths to rebase.
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Envelope {
    schema_version: u32,
    robot: Robot,
}

/// The document (`riggen_core::Robot`): links, joints, materials and the
/// mesh files it references. Every edit method applies one command on a
/// copy and keeps it only on success; a refused edit raises a
/// `riggen.EditError` subclass and changes nothing. Values are the
/// `.riggen` file's shape with ids as ints; `riggen.Robot` is the API over
/// this.
#[pyclass(name = "Robot", module = "riggen._riggen")]
pub struct PyRobot {
    pub(crate) inner: Robot,
}

/// The placeholders a joint dict may omit: `AddLink` and `SetJoint` both
/// overwrite the endpoints.
fn joint_defaults() -> [(&'static str, serde_json::Value); 2] {
    [("parent", json!("l0")), ("child", json!("l0"))]
}

/// Registers a mesh file the way the app's drop does: the absolute,
/// normalised path and the FNV-1a hash of the bytes (`MeshAsset`).
fn register(robot: &mut Robot, path: &Path, scale: f64, fix_up: Option<DQuat>) -> PyResult<MeshId> {
    let path = riggen_core::absolute(path).map_err(|e| with_path(path, e))?;
    let content_hash = riggen_core::hash_file(&path).map_err(|e| with_path(&path, e))?;
    Ok(robot.add_asset(MeshAsset {
        path,
        content_hash,
        scale,
        fix_up,
    }))
}

/// An I/O error that names the file: `std::io::Error` alone says "No such
/// file or directory (os error 2)", and the exception it becomes
/// (`FileNotFoundError`, `PermissionError`, …) keeps the kind.
fn with_path(path: &Path, e: std::io::Error) -> std::io::Error {
    std::io::Error::new(e.kind(), format!("{}: {e}", path.display()))
}

fn fix_up_from(obj: Option<&Bound<'_, PyAny>>) -> PyResult<Option<DQuat>> {
    obj.map(|q| from_doc::<DQuat>(q, "fix_up")).transpose()
}

fn pose_from(obj: Option<&Bound<'_, PyAny>>) -> PyResult<Pose> {
    obj.map_or(Ok(Pose::IDENTITY), |p| from_doc::<Pose>(p, "pose"))
}

impl PyRobot {
    /// Runs `f` on a clone; the document changes only if `f` succeeds.
    fn commit<T>(&mut self, f: impl FnOnce(&mut Robot) -> PyResult<T>) -> PyResult<T> {
        let mut next = self.inner.clone();
        let out = f(&mut next)?;
        self.inner = next;
        Ok(out)
    }

    /// One command, on a clone, as its own edit.
    fn edit(&mut self, py: Python<'_>, command: Command) -> PyResult<Option<LinkId>> {
        self.commit(|robot| apply(py, robot, command))
    }

    fn require_link(&self, py: Python<'_>, link: u32) -> PyResult<LinkId> {
        let id = LinkId::from_raw(link);
        if self.inner.links.contains_key(&id) {
            Ok(id)
        } else {
            Err(edit_error(
                py,
                EditError::UnknownId {
                    kind: LinkId::KIND,
                    id: id.to_string(),
                },
            ))
        }
    }

    fn map<I: Id, T: Serialize>(
        &self,
        py: Python<'_>,
        entries: impl Iterator<Item = (I, T)>,
    ) -> PyResult<Py<PyDict>> {
        let dict = PyDict::new(py);
        for (id, value) in entries {
            dict.set_item(id.raw(), to_doc(py, &value)?)?;
        }
        Ok(dict.unbind())
    }
}

fn apply(py: Python<'_>, robot: &mut Robot, command: Command) -> PyResult<Option<LinkId>> {
    command.apply(robot).map_err(|e| edit_error(py, e))
}

/// Every resolve error on its own line, spelled as `riggen --export` prints
/// them.
fn join_export_errors(errors: &[ExportError]) -> String {
    errors
        .iter()
        .map(|e| format!("cannot export: {e}"))
        .collect::<Vec<_>>()
        .join("\n")
}

fn format_from(name: &str) -> PyResult<Format> {
    match name {
        "mjcf" => Ok(Format::Mjcf),
        "urdf" => Ok(Format::Urdf),
        "both" => Ok(Format::Both),
        other => Err(PyValueError::new_err(format!(
            "format: {other:?} is not \"mjcf\", \"urdf\" or \"both\""
        ))),
    }
}

fn mesh_paths_from(style: &str) -> PyResult<MeshPathStyle> {
    match style {
        "relative" => Ok(MeshPathStyle::Relative),
        "absolute" => Ok(MeshPathStyle::Absolute),
        other => match other.strip_prefix("package://") {
            Some(name) if !name.is_empty() => Ok(MeshPathStyle::Package(name.to_owned())),
            _ => Err(PyValueError::new_err(format!(
                "mesh_paths: {other:?} is not \"relative\", \"absolute\" or \"package://<name>\""
            ))),
        },
    }
}

#[pymethods]
impl PyRobot {
    /// An empty document named `name`: one root link `base_link` and the
    /// default materials.
    #[new]
    fn new(name: &str) -> Self {
        Self {
            inner: Robot::new(name),
        }
    }

    /// Reads a `.riggen` file: mesh paths resolved against it, the document
    /// validated, every mesh file checked against its recorded hash. Returns
    /// the robot and the warnings (a changed or missing mesh), as strings.
    /// Raises `riggen.FileError`.
    #[staticmethod]
    fn load(py: Python<'_>, path: PathBuf) -> PyResult<(Self, Vec<String>)> {
        let (inner, warnings) =
            riggen_core::load(&path).map_err(|e| raise(py, "FileError", e.to_string()))?;
        let warnings = warnings.iter().map(ToString::to_string).collect();
        Ok((Self { inner }, warnings))
    }

    /// Writes the `.riggen` file, mesh paths rebased relative to it, assets
    /// no geom references dropped. Raises `riggen.FileError`.
    fn save(&self, py: Python<'_>, path: PathBuf) -> PyResult<()> {
        riggen_core::save(&self.inner, &path).map_err(|e| raise(py, "FileError", e.to_string()))
    }

    /// The document as the `.riggen` JSON text (`schema_version` envelope
    /// included), paths as held in memory — absolute. For diffing.
    fn to_json(&self) -> PyResult<String> {
        let envelope = Envelope {
            schema_version: riggen_core::file::SCHEMA_VERSION,
            robot: self.inner.clone(),
        };
        serde_json::to_string_pretty(&envelope)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))
    }

    /// The inverse of `to_json`: parses, validates, resolves no paths.
    /// Malformed JSON is `ValueError`; a document that breaks an invariant
    /// is `riggen.ValidationError`.
    #[staticmethod]
    fn from_json(py: Python<'_>, text: &str) -> PyResult<Self> {
        let envelope: Envelope = serde_json::from_str(text)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
        if envelope.schema_version != riggen_core::file::SCHEMA_VERSION {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "schema version {} is not the {} this build reads",
                envelope.schema_version,
                riggen_core::file::SCHEMA_VERSION
            )));
        }
        riggen_core::validate(&envelope.robot)
            .map_err(|e| raise(py, "ValidationError", e.to_string()))?;
        Ok(Self {
            inner: envelope.robot,
        })
    }

    /// An independent copy of the document.
    fn copy(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }

    // ---- read access ------------------------------------------------------

    #[getter]
    fn name(&self) -> &str {
        &self.inner.name
    }

    #[setter]
    fn set_name(&mut self, name: String) {
        self.inner.name = name;
    }

    /// The root link's id.
    #[getter]
    fn root(&self) -> u32 {
        self.inner.root.raw()
    }

    /// The value the next allocated id will have (`Robot::next_id`).
    #[getter]
    fn next_id(&self) -> u32 {
        self.inner.next_id.peek()
    }

    /// `{link id: link}`, each link in the schema's shape.
    fn links(&self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        self.map(py, self.inner.links.iter().map(|(id, l)| (*id, l)))
    }

    /// `{joint id: joint}`, `parent` / `child` as link ids.
    fn joints(&self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        self.map(py, self.inner.joints.iter().map(|(id, j)| (*id, j)))
    }

    /// `{frame id: frame}`.
    fn frames(&self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        self.map(py, self.inner.frames.iter().map(|(id, f)| (*id, f)))
    }

    /// `{mesh id: asset}`, `path` absolute.
    fn assets(&self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        self.map(py, self.inner.assets.iter().map(|(id, a)| (*id, a)))
    }

    /// `{name: {"density": kg/m³, "color": [r, g, b, a]}}`.
    fn materials(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        to_doc(py, &self.inner.materials)
    }

    /// The id of the link called `name`, or `None`.
    fn link(&self, name: &str) -> Option<u32> {
        self.inner
            .links
            .iter()
            .find(|(_, l)| l.name == name)
            .map(|(id, _)| id.raw())
    }

    /// The id of the joint called `name`, or `None`.
    fn joint(&self, name: &str) -> Option<u32> {
        self.inner
            .joints
            .iter()
            .find(|(_, j)| j.name == name)
            .map(|(id, _)| id.raw())
    }

    /// The joint whose child is `link`; `None` for the root.
    fn parent_joint(&self, py: Python<'_>, link: u32) -> PyResult<Option<u32>> {
        let link = self.require_link(py, link)?;
        Ok(self.inner.parent_joint(link).map(Id::raw))
    }

    /// Joints whose parent is `link`, in id order.
    fn child_joints(&self, py: Python<'_>, link: u32) -> PyResult<Vec<u32>> {
        let link = self.require_link(py, link)?;
        Ok(self.inner.child_joints(link).map(Id::raw).collect())
    }

    /// `link` and every descendant, depth-first, parents before children.
    fn subtree(&self, py: Python<'_>, link: u32) -> PyResult<Vec<u32>> {
        let link = self.require_link(py, link)?;
        Ok(self.inner.subtree(link).into_iter().map(Id::raw).collect())
    }

    // ---- assets -----------------------------------------------------------

    /// Registers a mesh file (absolute path, content hash) and returns its
    /// id. Not a command: an unreferenced asset is dropped on save.
    #[pyo3(signature = (path, *, scale = 1.0, fix_up = None))]
    fn add_asset(
        &mut self,
        path: PathBuf,
        scale: f64,
        fix_up: Option<&Bound<'_, PyAny>>,
    ) -> PyResult<u32> {
        let fix_up = fix_up_from(fix_up)?;
        Ok(register(&mut self.inner, &path, scale, fix_up)?.raw())
    }

    /// `SetAsset`: replaces an asset's `path` / `scale` / `fix_up`; the
    /// path is absolutised, the hash recomputed.
    fn set_asset(&mut self, py: Python<'_>, mesh: u32, asset: &Bound<'_, PyAny>) -> PyResult<()> {
        let mut asset: MeshAsset = from_doc_with(asset, "asset", &[("content_hash", json!(0))])?;
        asset.path = riggen_core::absolute(&asset.path).map_err(|e| with_path(&asset.path, e))?;
        asset.content_hash =
            riggen_core::hash_file(&asset.path).map_err(|e| with_path(&asset.path, e))?;
        self.edit(py, Command::SetAsset(MeshId::from_raw(mesh), asset))?;
        Ok(())
    }

    // ---- links and geoms --------------------------------------------------

    /// `AddLink`: a new link under `parent` with `joint` as its parent joint
    /// (a joint dict; `parent` / `child` in it are ignored). With `mesh`,
    /// the file is registered first and one geom at identity is the link's
    /// visual. Returns the new link's id; `parent_joint` gives the joint's.
    #[pyo3(signature = (name, parent, joint, *, mesh = None, scale = 1.0, fix_up = None, material = None))]
    #[allow(clippy::too_many_arguments)]
    fn add_link(
        &mut self,
        py: Python<'_>,
        name: &str,
        parent: u32,
        joint: &Bound<'_, PyAny>,
        mesh: Option<PathBuf>,
        scale: f64,
        fix_up: Option<&Bound<'_, PyAny>>,
        material: Option<String>,
    ) -> PyResult<u32> {
        let joint: Joint = from_doc_with(joint, "joint", &joint_defaults())?;
        let fix_up = fix_up_from(fix_up)?;
        let mut link = Link::new(name);
        link.material = material;
        self.commit(|robot| {
            if let Some(path) = &mesh {
                let mesh = register(robot, path, scale, fix_up)?;
                let id: GeomId = robot.next_id.alloc();
                link.visuals.push(Geom {
                    id,
                    mesh,
                    pose: Pose::IDENTITY,
                    color: None,
                });
            }
            let command = Command::AddLink {
                link: Box::new(link),
                parent: LinkId::from_raw(parent),
                joint,
            };
            let created = apply(py, robot, command)?;
            Ok(created.expect("AddLink returns the link it created").raw())
        })
    }

    /// `RemoveLink`: the link, its parent joint and its whole subtree.
    fn remove_link(&mut self, py: Python<'_>, link: u32) -> PyResult<()> {
        self.edit(py, Command::RemoveLink(LinkId::from_raw(link)))?;
        Ok(())
    }

    fn rename_link(&mut self, py: Python<'_>, link: u32, name: String) -> PyResult<()> {
        self.edit(py, Command::RenameLink(LinkId::from_raw(link), name))?;
        Ok(())
    }

    fn rename_joint(&mut self, py: Python<'_>, joint: u32, name: String) -> PyResult<()> {
        self.edit(py, Command::RenameJoint(JointId::from_raw(joint), name))?;
        Ok(())
    }

    /// `AddGeom`: a visual of asset `mesh` on `link`, at `pose` (identity
    /// by default). Returns the geom's id.
    #[pyo3(signature = (link, mesh, *, pose = None, color = None))]
    fn add_geom(
        &mut self,
        py: Python<'_>,
        link: u32,
        mesh: u32,
        pose: Option<&Bound<'_, PyAny>>,
        color: Option<[f32; 4]>,
    ) -> PyResult<u32> {
        let pose = pose_from(pose)?;
        self.commit(|robot| {
            let id: GeomId = robot.next_id.alloc();
            let geom = Geom {
                id,
                mesh: MeshId::from_raw(mesh),
                pose,
                color,
            };
            apply(py, robot, Command::AddGeom(LinkId::from_raw(link), geom))?;
            Ok(id.raw())
        })
    }

    fn remove_geom(&mut self, py: Python<'_>, link: u32, geom: u32) -> PyResult<()> {
        self.edit(
            py,
            Command::RemoveGeom(LinkId::from_raw(link), GeomId::from_raw(geom)),
        )?;
        Ok(())
    }

    /// `SetGeomPose`: the geom in the link frame.
    fn set_geom_pose(
        &mut self,
        py: Python<'_>,
        link: u32,
        geom: u32,
        pose: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        let pose = from_doc::<Pose>(pose, "pose")?;
        self.edit(
            py,
            Command::SetGeomPose(LinkId::from_raw(link), GeomId::from_raw(geom), pose),
        )?;
        Ok(())
    }

    // ---- joints -----------------------------------------------------------

    /// `SetJoint`: replaces everything about the joint except its
    /// endpoints — `parent` / `child` in the dict are ignored.
    fn set_joint(&mut self, py: Python<'_>, joint: u32, value: &Bound<'_, PyAny>) -> PyResult<()> {
        let value: Joint = from_doc_with(value, "joint", &joint_defaults())?;
        self.edit(py, Command::SetJoint(JointId::from_raw(joint), value))?;
        Ok(())
    }

    /// `MoveJointFrame`: the pivot moves, nothing in the world does.
    fn move_joint_frame(
        &mut self,
        py: Python<'_>,
        joint: u32,
        origin: &Bound<'_, PyAny>,
        axis: [f64; 3],
    ) -> PyResult<()> {
        let origin = from_doc::<Pose>(origin, "origin")?;
        self.edit(
            py,
            Command::MoveJointFrame {
                joint: JointId::from_raw(joint),
                origin,
                axis: DVec3::from_array(axis),
            },
        )?;
        Ok(())
    }

    /// `Reparent`: hangs `link` (with its subtree) under `new_parent`. With
    /// `keep_world_pose` the joint origin is rewritten so nothing moves at
    /// `q = 0`; without it the origin is kept and the part jumps.
    #[pyo3(signature = (link, new_parent, *, keep_world_pose = false))]
    fn reparent(
        &mut self,
        py: Python<'_>,
        link: u32,
        new_parent: u32,
        keep_world_pose: bool,
    ) -> PyResult<()> {
        self.edit(
            py,
            Command::Reparent {
                link: LinkId::from_raw(link),
                new_parent: LinkId::from_raw(new_parent),
                keep_world_pose,
            },
        )?;
        Ok(())
    }

    /// `SetRoot`: reverses the fixed joints on the path; refuses a movable
    /// one.
    fn set_root(&mut self, py: Python<'_>, link: u32) -> PyResult<()> {
        self.edit(py, Command::SetRoot(LinkId::from_raw(link)))?;
        Ok(())
    }

    // ---- materials, inertial, collision -----------------------------------

    #[pyo3(signature = (link, material))]
    fn set_link_material(
        &mut self,
        py: Python<'_>,
        link: u32,
        material: Option<String>,
    ) -> PyResult<()> {
        self.edit(
            py,
            Command::SetLinkMaterial(LinkId::from_raw(link), material),
        )?;
        Ok(())
    }

    /// `UpsertMaterial`: adds or replaces `{"density": …, "color": […]}`.
    fn upsert_material(
        &mut self,
        py: Python<'_>,
        name: String,
        material: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        let material = from_doc::<Material>(material, "material")?;
        self.edit(py, Command::UpsertMaterial(name, material))?;
        Ok(())
    }

    /// `RemoveMaterial`: refused while a link uses it.
    fn remove_material(&mut self, py: Python<'_>, name: String) -> PyResult<()> {
        self.edit(py, Command::RemoveMaterial(name))?;
        Ok(())
    }

    /// `SetInertial`: an `InertialSpec` in the schema's shape —
    /// `{"Computed": {"density_override": None}}`, `{"Override": {...}}`,
    /// `{"Hybrid": {"mass": …}}`.
    fn set_inertial(&mut self, py: Python<'_>, link: u32, spec: &Bound<'_, PyAny>) -> PyResult<()> {
        let spec = from_doc::<InertialSpec>(spec, "inertial")?;
        self.edit(py, Command::SetInertial(LinkId::from_raw(link), spec))?;
        Ok(())
    }

    /// `SetCollision`: a `CollisionPolicy` in the schema's shape —
    /// `"None"`, `"SameAsVisual"`, `"ConvexHull"`, `{"Primitives": […]}`,
    /// `{"Meshes": […]}`.
    fn set_collision(
        &mut self,
        py: Python<'_>,
        link: u32,
        policy: &Bound<'_, PyAny>,
    ) -> PyResult<()> {
        let policy = from_doc::<CollisionPolicy>(policy, "collision")?;
        self.edit(py, Command::SetCollision(LinkId::from_raw(link), policy))?;
        Ok(())
    }

    // ---- kinematics -------------------------------------------------------

    /// Every invariant the document breaks, as messages (`validation_errors`).
    /// Empty for any document the edit methods, `load` or `from_json` let
    /// through — they validate — so this is for a document assembled some
    /// other way.
    fn validate(&self) -> Vec<String> {
        validation_errors(&self.inner)
            .iter()
            .map(ToString::to_string)
            .collect()
    }

    /// `validate()`, raising `riggen.ValidationError` (every message, one
    /// per line) when the list is not empty.
    fn check(&self, py: Python<'_>) -> PyResult<()> {
        let errors = self.validate();
        if errors.is_empty() {
            Ok(())
        } else {
            Err(raise(py, "ValidationError", errors.join("\n")))
        }
    }

    /// Forward kinematics: the world pose of every link for the joint values
    /// `q` (`{joint id: radians or meters}`, missing joints at zero), as
    /// `{link id: pose}`.
    fn fk(&self, py: Python<'_>, q: BTreeMap<u32, f64>) -> PyResult<Py<PyDict>> {
        let mut state = JointState::new();
        for (joint, value) in q {
            let id = JointId::from_raw(joint);
            if !self.inner.joints.contains_key(&id) {
                return Err(edit_error(
                    py,
                    EditError::UnknownId {
                        kind: JointId::KIND,
                        id: id.to_string(),
                    },
                ));
            }
            state.set(id, value);
        }
        let world = riggen_core::fk(&self.inner, &state);
        self.map(py, world.iter().map(|(id, pose)| (*id, pose)))
    }

    /// The joint origin that puts `link` at `world` in the zero
    /// configuration — what to `set_joint` so a part lands where wanted.
    /// `None` for the root.
    fn origin_for_world(
        &self,
        py: Python<'_>,
        link: u32,
        world: &Bound<'_, PyAny>,
    ) -> PyResult<Option<Py<PyAny>>> {
        let link = self.require_link(py, link)?;
        let world = from_doc::<Pose>(world, "world")?;
        riggen_core::origin_for_world(&self.inner, link, world)
            .map(|pose| to_doc(py, &pose))
            .transpose()
    }

    /// The link's inertial under its spec — `(mass, com, inertia)`, the
    /// tensor about the CoM in link axes as three rows — with every
    /// referenced mesh read from disk (`MeshStore`). Raises
    /// `riggen.InertialError`.
    #[allow(clippy::type_complexity)]
    fn inertial(&self, py: Python<'_>, link: u32) -> PyResult<(f64, [f64; 3], [[f64; 3]; 3])> {
        let id = self.require_link(py, link)?;
        let (store, load_errors) = MeshStore::load(&self.inner);
        let link = &self.inner.links[&id];
        let composed = compose_inertial(link, &store, &self.inner.materials).map_err(|e| {
            let mut message = e.to_string();
            for load_error in &load_errors {
                message.push_str(&format!("\n{load_error}"));
            }
            raise(py, "InertialError", message)
        })?;
        let i = composed.inertial;
        Ok((
            i.mass,
            i.com.to_array(),
            [
                i.inertia.row(0).to_array(),
                i.inertia.row(1).to_array(),
                i.inertia.row(2).to_array(),
            ],
        ))
    }

    // ---- export and import ------------------------------------------------

    /// Writes the export directory (ADR-0008): `<name>.xml` and/or
    /// `<name>.urdf` beside `meshes/` in meters; with `fk_samples`,
    /// `<name>.fk.json` too. `format` is `"mjcf"`, `"urdf"` or `"both"`;
    /// `mesh_paths` (URDF only) `"relative"`, `"absolute"` or
    /// `"package://<name>"`. Returns every path written. Raises
    /// `riggen.ExportError` listing every reason the document cannot be
    /// exported, exactly as `riggen --export` prints them.
    #[pyo3(signature = (dir, *, format = "both", mesh_paths = "relative", floating_base = false, fk_samples = false))]
    fn export(
        &self,
        py: Python<'_>,
        dir: PathBuf,
        format: &str,
        mesh_paths: &str,
        floating_base: bool,
        fk_samples: bool,
    ) -> PyResult<Vec<PathBuf>> {
        let options = ExportOptions {
            format: format_from(format)?,
            mesh_paths: mesh_paths_from(mesh_paths)?,
            floating_base,
        };
        let (store, load_errors) = MeshStore::load(&self.inner);
        let resolved = match riggen_export::resolve(&self.inner, &store, &options) {
            Ok(r) if load_errors.is_empty() => r,
            Ok(_) => return Err(raise(py, "ExportError", join_export_errors(&load_errors))),
            Err(mut errors) => {
                errors.extend(load_errors);
                return Err(raise(py, "ExportError", join_export_errors(&errors)));
            }
        };
        let mut written = riggen_export::export(&resolved, &options, &dir)
            .map_err(|e| PyOSError::new_err(e.to_string()))?;
        if fk_samples {
            let path = dir.join(format!("{}.fk.json", self.inner.name));
            std::fs::write(&path, riggen_export::fk_samples::to_json(&self.inner))
                .map_err(|e| PyOSError::new_err(format!("{}: {e}", path.display())))?;
            written.push(path);
        }
        Ok(written)
    }

    /// The `<name>.fk.json` text `export(fk_samples=True)` writes: five
    /// joint configurations and the FK at each, by name.
    fn fk_samples_json(&self) -> String {
        riggen_export::fk_samples::to_json(&self.inner)
    }

    /// Imports a URDF (docs/02-data-model.md §URDF import): mesh paths
    /// resolved against the file and `packages` (`{name: directory}` for
    /// `package://name/…`). Returns the document and the warnings — what
    /// the URDF held that the document does not. Raises
    /// `riggen.UrdfImportError`.
    #[staticmethod]
    #[pyo3(signature = (path, packages = None))]
    fn load_urdf(
        py: Python<'_>,
        path: PathBuf,
        packages: Option<BTreeMap<String, PathBuf>>,
    ) -> PyResult<(Self, Vec<String>)> {
        let packages = PackageMap(packages.unwrap_or_default());
        let (inner, warnings) = riggen_export::urdf_in::load(&path, &packages)
            .map_err(|e| raise(py, "UrdfImportError", e.to_string()))?;
        let warnings = warnings.iter().map(ToString::to_string).collect();
        Ok((Self { inner }, warnings))
    }

    fn __repr__(&self) -> String {
        format!(
            "Robot('{}': {} links, {} joints)",
            self.inner.name,
            self.inner.links.len(),
            self.inner.joints.len()
        )
    }
}
