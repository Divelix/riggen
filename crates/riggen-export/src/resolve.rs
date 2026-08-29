//! `Robot` → [`ResolvedRobot`] (docs/02-data-model.md §`ResolvedRobot`):
//! links in topological order, joints beside them, every geom a mesh in
//! meters or a primitive, every inertial composed and checked. Everything
//! that can block an export is found here, all of it, so the export dialog
//! lists every problem at once (step 12).

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::path::PathBuf;
use std::sync::Arc;

use riggen_core::glam::DVec3;
use riggen_core::inertial::{self, Inertial, InertialError, MeshLookup};
use riggen_core::{
    CollisionPolicy, Dynamics, Geom, JointKind, Limits, LinkId, MeshId, Pose, Primitive, Robot,
    ValidationError, validation_errors,
};
use riggen_mesh::TriMesh;

/// Which file(s) `export` writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Format {
    Mjcf,
    Urdf,
    #[default]
    Both,
}

impl Format {
    pub fn writes_mjcf(self) -> bool {
        matches!(self, Self::Mjcf | Self::Both)
    }

    pub fn writes_urdf(self) -> bool {
        matches!(self, Self::Urdf | Self::Both)
    }
}

/// How a URDF `<mesh filename>` names its file. MJCF ignores this — it has
/// `meshdir`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum MeshPathStyle {
    /// `meshes/<stem>.stl`, relative to the URDF.
    #[default]
    Relative,
    /// `package://<name>/meshes/<stem>.stl`.
    Package(String),
    /// The absolute path of the written file.
    Absolute,
}

/// Everything the export dialog decides (docs/02-data-model.md
/// §`ResolvedRobot`). Not a document field: two exports of one document may
/// differ in all of it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ExportOptions {
    pub format: Format,
    pub mesh_paths: MeshPathStyle,
    /// MJCF only: the root body gets a `<freejoint/>`, which makes it a
    /// moving body that needs mass (OPEN 3).
    pub floating_base: bool,
}

/// Why a document cannot be exported. `resolve` returns every one it finds.
#[derive(Debug, Clone, PartialEq)]
pub enum ExportError {
    Invalid(ValidationError),
    /// The link's inertial could not be composed or fails [`inertial::check`].
    Inertial {
        link: LinkId,
        name: String,
        error: InertialError,
    },
    /// A body that moves (its parent joint is movable, or it is a floating
    /// root) with no mass: MuJoCo refuses it. An empty static body is fine
    /// and gets no `<inertial>`.
    ZeroMassMovableLink {
        link: LinkId,
        name: String,
    },
    /// A referenced mesh is not in the lookup (the file did not load).
    UnloadableMesh {
        mesh: MeshId,
        path: PathBuf,
        reason: String,
    },
    /// `ConvexHull` of a mesh that spans no volume (a flat plate, a line).
    DegenerateHull {
        mesh: MeshId,
        path: PathBuf,
        reason: String,
    },
    /// A collision policy the writers do not handle yet.
    Unsupported {
        link: LinkId,
        name: String,
        what: &'static str,
    },
}

impl fmt::Display for ExportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Invalid(e) => write!(f, "{e}"),
            Self::Inertial { name, error, .. } => write!(f, "link \"{name}\": {error}"),
            Self::ZeroMassMovableLink { name, .. } => write!(
                f,
                "link \"{name}\" moves but has no mass (add a mesh, a material, or an override)"
            ),
            Self::UnloadableMesh { path, reason, .. } => {
                write!(f, "mesh {}: {reason}", path.display())
            }
            Self::DegenerateHull { path, reason, .. } => {
                write!(f, "mesh {}: {reason}", path.display())
            }
            Self::Unsupported { name, what, .. } => {
                write!(f, "link \"{name}\": {what} is not supported yet")
            }
        }
    }
}

impl std::error::Error for ExportError {}

impl From<ValidationError> for ExportError {
    fn from(e: ValidationError) -> Self {
        Self::Invalid(e)
    }
}

/// A geom the writers can serialise: a mesh file (already in meters, named
/// by its stem — `meshes/<name>.stl` once written) at a pose in the link
/// frame, or a primitive.
#[derive(Debug, Clone, PartialEq)]
pub enum ResolvedGeom {
    Mesh {
        name: String,
        mesh: Arc<TriMesh>,
        pose: Pose,
    },
    Primitive(Primitive),
}

#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedLink {
    pub name: String,
    pub visuals: Vec<ResolvedGeom>,
    pub collisions: Vec<ResolvedGeom>,
    /// `None` for an empty static body.
    pub inertial: Option<Inertial>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedJoint {
    pub name: String,
    pub kind: JointKind,
    /// Indices into `ResolvedRobot::links`.
    pub parent: usize,
    pub child: usize,
    /// Child link frame in the parent link frame.
    pub origin: Pose,
    /// Unit, child frame.
    pub axis: DVec3,
    pub limits: Option<Limits>,
    pub dynamics: Dynamics,
}

/// The pure-numeric, convention-fixed intermediate every writer reads.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedRobot {
    pub name: String,
    /// Topological order, root first.
    pub links: Vec<ResolvedLink>,
    /// `joints[i]` is the parent joint of `links[i + 1]`.
    pub joints: Vec<ResolvedJoint>,
    /// Every mesh file to write, by stem: the union of what the geoms name.
    pub meshes: BTreeMap<String, Arc<TriMesh>>,
    pub floating_base: bool,
}

impl ResolvedRobot {
    /// The parent joint of `links[link]`; `None` for the root.
    pub fn parent_joint(&self, link: usize) -> Option<&ResolvedJoint> {
        link.checked_sub(1).map(|i| &self.joints[i])
    }

    /// Joints whose parent is `links[link]`, in order.
    pub fn child_joints(&self, link: usize) -> impl Iterator<Item = &ResolvedJoint> + '_ {
        self.joints.iter().filter(move |j| j.parent == link)
    }
}

/// Resolves `robot` for export, or every reason it cannot be. Hulls are
/// computed here, once per referenced mesh however many links share it,
/// and written as `<stem>_hull.stl` beside the mesh (ADR-0008).
pub fn resolve(
    robot: &Robot,
    meshes: &impl MeshLookup,
    options: &ExportOptions,
) -> Result<ResolvedRobot, Vec<ExportError>> {
    let mut errors: Vec<ExportError> = validation_errors(robot)
        .into_iter()
        .map(ExportError::Invalid)
        .collect();
    if !errors.is_empty() {
        return Err(errors);
    }

    let order = robot.subtree(robot.root);
    let index: BTreeMap<LinkId, usize> = order.iter().enumerate().map(|(i, &l)| (l, i)).collect();

    let names = mesh_names(robot);
    let mut files = BTreeMap::new();
    let mut hulls: BTreeMap<MeshId, Result<Arc<TriMesh>, ExportError>> = BTreeMap::new();
    let mut links = Vec::with_capacity(order.len());
    let mut joints = Vec::with_capacity(order.len().saturating_sub(1));

    for (i, &lid) in order.iter().enumerate() {
        let link = &robot.links[&lid];
        let parent_joint = robot.parent_joint(lid).map(|j| &robot.joints[&j]);
        if let Some(joint) = parent_joint {
            joints.push(ResolvedJoint {
                name: joint.name.clone(),
                kind: joint.kind,
                parent: index[&joint.parent],
                child: i,
                origin: joint.origin,
                axis: joint.axis.normalize_or_zero(),
                limits: joint.limits,
                dynamics: joint.dynamics,
            });
        }

        let mut resolve_geoms = |geoms: &[Geom]| -> Vec<ResolvedGeom> {
            geoms
                .iter()
                .filter_map(|g| {
                    let name = names[&g.mesh].clone();
                    match meshes.mesh(g.mesh) {
                        Some(mesh) => {
                            let mesh = files
                                .entry(name.clone())
                                .or_insert_with(|| Arc::new(mesh.clone()))
                                .clone();
                            Some(ResolvedGeom::Mesh {
                                name,
                                mesh,
                                pose: g.pose,
                            })
                        }
                        None => {
                            errors.push(ExportError::UnloadableMesh {
                                mesh: g.mesh,
                                path: robot.assets[&g.mesh].path.clone(),
                                reason: "not loaded".into(),
                            });
                            None
                        }
                    }
                })
                .collect()
        };
        let visuals = resolve_geoms(&link.visuals);
        let collisions = match &link.collision {
            CollisionPolicy::None => Vec::new(),
            CollisionPolicy::SameAsVisual => visuals.clone(),
            CollisionPolicy::Primitives(ps) => {
                ps.iter().cloned().map(ResolvedGeom::Primitive).collect()
            }
            CollisionPolicy::Meshes(geoms) => resolve_geoms(geoms),
            CollisionPolicy::ConvexHull => link
                .visuals
                .iter()
                .filter_map(|g| {
                    let hull = hulls.entry(g.mesh).or_insert_with(|| {
                        let mesh = meshes.mesh(g.mesh).ok_or(ExportError::UnloadableMesh {
                            mesh: g.mesh,
                            path: robot.assets[&g.mesh].path.clone(),
                            reason: "not loaded".into(),
                        })?;
                        riggen_mesh::convex_hull(&mesh.positions)
                            .map(Arc::new)
                            .map_err(|e| ExportError::DegenerateHull {
                                mesh: g.mesh,
                                path: robot.assets[&g.mesh].path.clone(),
                                reason: e.to_string(),
                            })
                    });
                    match hull {
                        Ok(hull) => {
                            let name = format!("{}_hull", names[&g.mesh]);
                            files.entry(name.clone()).or_insert_with(|| hull.clone());
                            Some(ResolvedGeom::Mesh {
                                name,
                                mesh: hull.clone(),
                                pose: g.pose,
                            })
                        }
                        Err(e) => {
                            // Reported once per mesh, on its first use.
                            if !errors.contains(e) {
                                errors.push(e.clone());
                            }
                            None
                        }
                    }
                })
                .collect(),
            CollisionPolicy::ConvexDecomposition { .. } => {
                errors.push(ExportError::Unsupported {
                    link: lid,
                    name: link.name.clone(),
                    what: "convex decomposition",
                });
                Vec::new()
            }
        };

        let moving = match parent_joint {
            Some(j) => j.kind.is_movable(),
            None => options.floating_base,
        };
        let inertial = match inertial::compose_inertial(link, meshes, &robot.materials) {
            Ok(composed) => {
                let value = composed.inertial;
                if value.mass <= 0.0 && !moving {
                    None
                } else if value.mass <= 0.0 {
                    errors.push(ExportError::ZeroMassMovableLink {
                        link: lid,
                        name: link.name.clone(),
                    });
                    None
                } else {
                    for error in inertial::check(&value) {
                        errors.push(ExportError::Inertial {
                            link: lid,
                            name: link.name.clone(),
                            error,
                        });
                    }
                    Some(value)
                }
            }
            // A geometry-less body has no mass whatever its density: fine
            // when static, and the clearer of the two errors when moving.
            Err(InertialError::NoDensity) if link.visuals.is_empty() => {
                if moving {
                    errors.push(ExportError::ZeroMassMovableLink {
                        link: lid,
                        name: link.name.clone(),
                    });
                }
                None
            }
            Err(error) => {
                errors.push(ExportError::Inertial {
                    link: lid,
                    name: link.name.clone(),
                    error,
                });
                None
            }
        };

        links.push(ResolvedLink {
            name: link.name.clone(),
            visuals,
            collisions,
            inertial,
        });
    }

    if !errors.is_empty() {
        return Err(errors);
    }
    Ok(ResolvedRobot {
        name: robot.name.clone(),
        links,
        joints,
        meshes: files,
        floating_base: options.floating_base,
    })
}

/// A file stem per referenced mesh: the asset's own stem made into a valid
/// identifier, disambiguated with `_2`, `_3`, … in `MeshId` order.
fn mesh_names(robot: &Robot) -> BTreeMap<MeshId, String> {
    let mut taken: BTreeSet<String> = BTreeSet::new();
    let mut names = BTreeMap::new();
    for id in robot.referenced_assets() {
        let Some(asset) = robot.assets.get(&id) else {
            continue;
        };
        let base = stem_name(
            asset
                .path
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or(""),
        );
        let mut name = base.clone();
        let mut n = 2;
        while !taken.insert(name.clone()) {
            name = format!("{base}_{n}");
            n += 1;
        }
        names.insert(id, name);
    }
    names
}

/// `[A-Za-z_][A-Za-z0-9_.-]*`: MJCF asset names are identifiers, and the
/// same string is the file stem.
fn stem_name(stem: &str) -> String {
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
        None => name = "mesh".into(),
        Some(c) if !(c.is_ascii_alphabetic() || c == '_') => name.insert(0, '_'),
        _ => {}
    }
    name
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MeshStore;
    use crate::test_util::{Builder, fixtures};
    use riggen_core::glam::DMat3;
    use riggen_core::{Command, InertialSpec, MeshAsset};
    use std::f64::consts::FRAC_PI_2;

    #[test]
    fn pendulum_resolves_in_order_with_computed_inertials() {
        let (robot, warnings) = riggen_core::load(&fixtures().join("pendulum.riggen")).unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");
        let (store, errors) = MeshStore::load(&robot);
        assert!(errors.is_empty(), "{errors:?}");
        let resolved = resolve(&robot, &store, &ExportOptions::default()).unwrap();

        assert_eq!(resolved.name, "pendulum");
        let names: Vec<&str> = resolved.links.iter().map(|l| l.name.as_str()).collect();
        assert_eq!(names, ["base_link", "arm"]);
        assert_eq!(resolved.joints.len(), 1);
        let hinge = &resolved.joints[0];
        assert_eq!(
            (hinge.name.as_str(), hinge.parent, hinge.child),
            ("hinge", 0, 1)
        );
        assert_eq!(hinge.kind, JointKind::Revolute);
        assert_eq!(hinge.origin.t, DVec3::new(0.0, 0.0, 0.5));
        assert_eq!(resolved.parent_joint(0), None);
        assert_eq!(resolved.parent_joint(1).unwrap().name, "hinge");
        assert_eq!(resolved.child_joints(0).count(), 1);

        // Two distinct mesh files, named by stem; collision is the visual.
        let stems: Vec<&String> = resolved.meshes.keys().collect();
        assert_eq!(stems, ["cube_ascii", "cube_binary"]);
        for link in &resolved.links {
            assert_eq!(link.visuals.len(), 1);
            assert_eq!(link.collisions, link.visuals);
            let inertial = link.inertial.expect("both links have a cube");
            assert!(inertial.mass > 0.0);
            assert!(inertial::check(&inertial).is_empty());
        }
        // The arm is PLA (1240 kg/m³), a unit cube from the fixture.
        let arm = resolved.links[1].inertial.unwrap();
        let ResolvedGeom::Mesh { mesh, .. } = &resolved.links[1].visuals[0] else {
            panic!()
        };
        let volume = riggen_mesh::mass_properties(mesh, 1.0).volume;
        assert!((arm.mass - 1240.0 * volume).abs() < 1e-9, "{}", arm.mass);
        assert!(!resolved.floating_base);
    }

    #[test]
    fn topological_order_is_parent_before_child_with_joint_indices() {
        let mut b = Builder::new();
        let cube = b.mesh("cube", TriMesh::cube(0.05));
        let root = b.robot.root;
        // root ─ a ─ b, root ─ c; added out of order so id order ≠ tree order
        // is at least exercised for the joint index mapping.
        let a = b.link("a", root, JointKind::Revolute, Some(cube));
        let c = b.link("c", root, JointKind::Fixed, Some(cube));
        let bb = b.link("b", a, JointKind::Prismatic, Some(cube));
        let resolved = b.resolve().unwrap();
        let names: Vec<&str> = resolved.links.iter().map(|l| l.name.as_str()).collect();
        assert_eq!(names, ["base_link", "a", "b", "c"]);
        let edges: Vec<(usize, usize)> = resolved
            .joints
            .iter()
            .map(|j| (j.parent, j.child))
            .collect();
        assert_eq!(edges, [(0, 1), (1, 2), (0, 3)]);
        assert_eq!(b.robot.subtree(root), [root, a, bb, c]);
        // One file for the one mesh, however many geoms use it.
        assert_eq!(resolved.meshes.len(), 1);
        // The static root has no mesh and no inertial; nothing complains.
        assert_eq!(resolved.links[0].inertial, None);
    }

    #[test]
    fn empty_static_root_needs_no_density_but_a_moving_empty_link_is_an_error() {
        let mut b = Builder::new();
        b.robot.links.get_mut(&b.robot.root).unwrap().material = None;
        let cube = b.mesh("cube", TriMesh::cube(0.05));
        let root = b.robot.root;
        b.link("arm", root, JointKind::Continuous, Some(cube));
        let resolved = b.resolve().unwrap();
        assert_eq!(resolved.links[0].inertial, None);
        assert!(resolved.links[1].inertial.is_some());

        let ghost = b.link("ghost", root, JointKind::Revolute, None);
        let errors = b.resolve().unwrap_err();
        assert_eq!(
            errors,
            vec![ExportError::ZeroMassMovableLink {
                link: ghost,
                name: "ghost".into()
            }]
        );
        assert!(errors[0].to_string().contains("\"ghost\" moves"));

        // Welded onto its parent, the same empty link is fine.
        b.robot.joints.values_mut().for_each(|j| {
            if j.child == ghost {
                j.kind = JointKind::Fixed;
                j.limits = None;
            }
        });
        assert!(b.resolve().is_ok());
    }

    #[test]
    fn floating_base_makes_the_root_a_moving_body() {
        let mut b = Builder::new();
        let cube = b.mesh("cube", TriMesh::cube(0.05));
        let root = b.robot.root;
        b.link("arm", root, JointKind::Fixed, Some(cube));
        let options = ExportOptions {
            floating_base: true,
            ..Default::default()
        };
        let errors = resolve(&b.robot, &b.store, &options).unwrap_err();
        assert!(matches!(
            errors[..],
            [ExportError::ZeroMassMovableLink { link, .. }] if link == root
        ));
        let g = b.geom(cube, Pose::IDENTITY);
        Command::AddGeom(root, g).apply(&mut b.robot).unwrap();
        let resolved = resolve(&b.robot, &b.store, &options).unwrap();
        assert!(resolved.floating_base);
        assert!(resolved.links[0].inertial.is_some());
    }

    #[test]
    fn every_error_is_collected_in_one_pass() {
        let mut b = Builder::new();
        let cube = b.mesh("cube", TriMesh::cube(0.05));
        let mut open = TriMesh::cube(0.05);
        open.indices.truncate(30);
        let open_mesh = b.mesh("open", open);
        let unloaded = b.mesh("missing", TriMesh::cube(0.05));
        b.store.0.remove(&unloaded);
        let root = b.robot.root;

        let a = b.link("a", root, JointKind::Revolute, Some(open_mesh));
        let bad = b.link("bad", root, JointKind::Revolute, Some(cube));
        b.robot.links.get_mut(&bad).unwrap().inertial = InertialSpec::Override {
            mass: 1.0,
            com: DVec3::ZERO,
            inertia: DMat3::from_diagonal(DVec3::new(1.0, 1.0, 3.0)),
        };
        let decomposed = b.link("decomposed", root, JointKind::Fixed, Some(cube));
        b.robot.links.get_mut(&decomposed).unwrap().collision =
            CollisionPolicy::ConvexDecomposition { max_hulls: 4 };
        let lost = b.link("lost", root, JointKind::Fixed, Some(unloaded));
        let empty = b.link("empty", root, JointKind::Prismatic, None);

        let errors = b.resolve().unwrap_err();
        let kinds: Vec<String> = errors
            .iter()
            .map(|e| match e {
                ExportError::Inertial { link, error, .. } => format!("inertial {link} {error:?}"),
                ExportError::ZeroMassMovableLink { link, .. } => format!("zero {link}"),
                ExportError::UnloadableMesh { mesh, .. } => format!("unloadable {mesh}"),
                ExportError::Unsupported { link, what, .. } => format!("unsupported {link} {what}"),
                ExportError::DegenerateHull { mesh, .. } => format!("degenerate {mesh}"),
                ExportError::Invalid(e) => format!("invalid {e}"),
            })
            .collect();
        let open_geom = b.robot.links[&a].visuals[0].id;
        assert_eq!(
            kinds,
            [
                format!("inertial {a} OpenMesh {{ geom: {open_geom:?} }}"),
                format!("inertial {bad} TriangleInequality {{ moments: [1.0, 1.0, 3.0] }}"),
                format!("unsupported {decomposed} convex decomposition"),
                format!("unloadable {unloaded}"),
                format!(
                    "inertial {lost} MissingMesh {{ geom: {:?}, mesh: {unloaded:?} }}",
                    b.robot.links[&lost].visuals[0].id
                ),
                format!("zero {empty}"),
            ]
        );
    }

    #[test]
    fn an_invalid_document_reports_validation_and_nothing_else() {
        let mut b = Builder::new();
        let cube = b.mesh("cube", TriMesh::cube(0.05));
        let root = b.robot.root;
        let arm = b.link("arm", root, JointKind::Revolute, Some(cube));
        b.robot.links.get_mut(&arm).unwrap().material = Some("unobtainium".into());
        let errors = b.resolve().unwrap_err();
        assert_eq!(
            errors,
            vec![ExportError::Invalid(ValidationError::DanglingMaterial {
                link: arm,
                material: "unobtainium".into()
            })]
        );
    }

    #[test]
    fn collision_policies_resolve_to_their_geoms() {
        let mut b = Builder::new();
        let cube = b.mesh("cube", TriMesh::cube(0.05));
        let coarse = b.mesh("cube-coarse", TriMesh::cube(0.06));
        let root = b.robot.root;
        let none = b.link("none", root, JointKind::Fixed, Some(cube));
        let prims = b.link("prims", root, JointKind::Fixed, Some(cube));
        let meshes = b.link("meshes", root, JointKind::Fixed, Some(cube));
        b.robot.links.get_mut(&none).unwrap().collision = CollisionPolicy::None;
        let sphere = Primitive::Sphere {
            pose: Pose::IDENTITY,
            radius: 0.1,
        };
        b.robot.links.get_mut(&prims).unwrap().collision =
            CollisionPolicy::Primitives(vec![sphere.clone()]);
        let pose = Pose::from_xyz_rpy(DVec3::X, DVec3::new(FRAC_PI_2, 0.0, 0.0));
        let g = b.geom(coarse, pose);
        b.robot.links.get_mut(&meshes).unwrap().collision = CollisionPolicy::Meshes(vec![g]);
        // The collision-only mesh counts as referenced, so save keeps it.
        assert!(b.robot.referenced_assets().contains(&coarse));

        let resolved = b.resolve().unwrap();
        let by_name = |n: &str| resolved.links.iter().find(|l| l.name == n).unwrap();
        assert!(by_name("none").collisions.is_empty());
        assert_eq!(
            by_name("prims").collisions,
            vec![ResolvedGeom::Primitive(sphere)]
        );
        match &by_name("meshes").collisions[..] {
            [ResolvedGeom::Mesh { name, pose: p, .. }] => {
                assert_eq!(name, "cube-coarse");
                assert_eq!(*p, pose);
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(resolved.meshes.len(), 2);
    }

    #[test]
    fn convex_hull_is_one_hull_per_visual_cached_per_mesh() {
        let mut b = Builder::new();
        // A cube with an extra interior vertex: the hull is the cube.
        let mut dented = TriMesh::cube(0.05);
        dented.positions.push(DVec3::ZERO);
        let cube = b.mesh("cube", dented);
        let root = b.robot.root;
        let a = b.link("a", root, JointKind::Fixed, Some(cube));
        let c = b.link("c", root, JointKind::Fixed, Some(cube));
        for l in [a, c] {
            let link = b.robot.links.get_mut(&l).unwrap();
            link.collision = CollisionPolicy::ConvexHull;
            let g = Geom {
                id: b.robot.next_id.alloc(),
                mesh: cube,
                pose: Pose::from_translation(DVec3::X),
                color: None,
            };
            b.robot.links.get_mut(&l).unwrap().visuals.push(g);
        }
        let resolved = b.resolve().unwrap();
        let stems: Vec<&String> = resolved.meshes.keys().collect();
        assert_eq!(stems, ["cube", "cube_hull"]);
        let hull = &resolved.meshes["cube_hull"];
        assert_eq!((hull.positions.len(), hull.triangle_count()), (8, 12));
        for name in ["a", "c"] {
            let link = resolved.links.iter().find(|l| l.name == name).unwrap();
            assert_eq!(link.collisions.len(), 2, "one hull per visual");
            for (col, vis) in link.collisions.iter().zip(&link.visuals) {
                let (ResolvedGeom::Mesh { name, mesh, pose }, ResolvedGeom::Mesh { pose: vp, .. }) =
                    (col, vis)
                else {
                    panic!("{col:?}");
                };
                assert_eq!(name, "cube_hull");
                assert_eq!(pose, vp, "the hull sits where the visual sits");
                assert!(Arc::ptr_eq(mesh, hull), "one hull, shared");
            }
        }

        // A flat plate has no hull: one error, however many geoms use it.
        let plate = b.mesh(
            "plate",
            TriMesh {
                positions: vec![DVec3::ZERO, DVec3::X, DVec3::Y, DVec3::ONE - DVec3::Z],
                normals: vec![],
                indices: vec![0, 1, 2, 1, 3, 2],
            },
        );
        let flat = b.link("flat", root, JointKind::Fixed, Some(plate));
        let link = b.robot.links.get_mut(&flat).unwrap();
        link.collision = CollisionPolicy::ConvexHull;
        link.inertial = InertialSpec::Override {
            mass: 1.0,
            com: DVec3::ZERO,
            inertia: DMat3::IDENTITY,
        };
        let g = Geom {
            id: b.robot.next_id.alloc(),
            mesh: plate,
            pose: Pose::IDENTITY,
            color: None,
        };
        b.robot.links.get_mut(&flat).unwrap().visuals.push(g);
        let errors = b.resolve().unwrap_err();
        assert!(
            matches!(&errors[..], [ExportError::DegenerateHull { mesh, .. }] if *mesh == plate),
            "{errors:?}"
        );
        assert!(errors[0].to_string().contains("coplanar"), "{}", errors[0]);
    }

    #[test]
    fn mesh_names_are_identifiers_and_unique() {
        assert_eq!(stem_name("Upper Arm v2"), "Upper_Arm_v2");
        assert_eq!(stem_name("3dprint"), "_3dprint");
        assert_eq!(stem_name(""), "mesh");

        let mut b = Builder::new();
        let root = b.robot.root;
        // Two different files with the same stem in different directories.
        let a = b.mesh("part", TriMesh::cube(0.05));
        let c = b.robot.add_asset(MeshAsset {
            path: PathBuf::from("/elsewhere/part.stl"),
            content_hash: 1,
            scale: 1.0,
            fix_up: None,
        });
        b.store.insert(c, TriMesh::cube(0.07));
        b.link("x", root, JointKind::Fixed, Some(a));
        b.link("y", root, JointKind::Fixed, Some(c));
        let resolved = b.resolve().unwrap();
        let stems: Vec<&String> = resolved.meshes.keys().collect();
        assert_eq!(stems, ["part", "part_2"]);
    }
}
