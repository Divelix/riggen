//! URDF → `Robot` (docs/02-data-model.md §URDF import), over `urdf-rs`.
//! Links and joints map directly — URDF's joint-frame convention is ours
//! (ADR-0004) — `<inertial>` becomes `InertialSpec::Override`, a uniform
//! `<mesh scale>` becomes `MeshAsset::scale`, a `<collision>` mesh that is
//! not the visual becomes `CollisionPolicy::Meshes` (OPEN 1), primitives
//! become `Primitives`. What the document cannot hold — `<mimic>`,
//! `<safety_controller>`, a primitive visual, a non-uniform scale — is
//! dropped with an [`ImportWarning`] that names it, never silently.
//!
//! `package://` is resolved through a [`PackageMap`], else by looking for
//! the rest of the path beside the file and up its ancestors — `urdf-rs`'s
//! own resolution shells out to `rospack`, which a riggen user does not
//! have.

use std::collections::BTreeMap;
use std::fmt;
use std::path::{Path, PathBuf};

use riggen_core::glam::{DMat3, DVec3};
use riggen_core::{
    CollisionPolicy, Dynamics, Geom, GeomId, InertialSpec, Joint, JointId, JointKind, Limits, Link,
    LinkId, MeshAsset, MeshId, Pose, Primitive, Robot, ValidationError, validate,
};

/// `package name → directory` for `package://name/...` mesh paths.
#[derive(Debug, Clone, Default)]
pub struct PackageMap(pub BTreeMap<String, PathBuf>);

/// Something the URDF held that the document does not; the import went on
/// without it.
#[derive(Debug, Clone, PartialEq)]
pub enum ImportWarning {
    MimicDropped {
        joint: String,
        mimics: String,
    },
    SafetyControllerDropped {
        joint: String,
    },
    /// `<mesh scale>` with unequal components: the largest was used.
    NonUniformScale {
        link: String,
        file: String,
        used: f64,
    },
    /// A `<visual>` with a box / cylinder / sphere / capsule: visuals are
    /// meshes here.
    PrimitiveVisualDropped {
        link: String,
        kind: &'static str,
    },
    /// A `<collision>` primitive beside collision meshes in one link: the
    /// document holds one policy per link, and the meshes won.
    MixedCollisionDropped {
        link: String,
        kind: &'static str,
    },
    /// A link with geometry but no `<inertial>`: `Computed`, which needs a
    /// material before it exports.
    NoInertial {
        link: String,
    },
    /// `package://name` matched nothing; the path beside the file was used.
    PackageUnresolved {
        package: String,
        used: PathBuf,
    },
    /// The resolved mesh file does not exist (registered anyway; the
    /// document opens and the status bar names it).
    MeshNotFound {
        link: String,
        file: String,
        tried: PathBuf,
    },
}

impl fmt::Display for ImportWarning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MimicDropped { joint, mimics } => {
                write!(
                    f,
                    "joint \"{joint}\": <mimic joint=\"{mimics}\"> dropped (no mimic joints yet)"
                )
            }
            Self::SafetyControllerDropped { joint } => {
                write!(f, "joint \"{joint}\": <safety_controller> dropped")
            }
            Self::NonUniformScale { link, file, used } => write!(
                f,
                "link \"{link}\": {file} has a non-uniform scale; {used} used for every axis"
            ),
            Self::PrimitiveVisualDropped { link, kind } => {
                write!(
                    f,
                    "link \"{link}\": a {kind} visual was dropped (visuals are meshes)"
                )
            }
            Self::MixedCollisionDropped { link, kind } => write!(
                f,
                "link \"{link}\": a {kind} collision was dropped beside its collision meshes"
            ),
            Self::NoInertial { link } => write!(
                f,
                "link \"{link}\" has no <inertial>; assign a material so one can be computed"
            ),
            Self::PackageUnresolved { package, used } => {
                write!(f, "package://{package} not found; used {}", used.display())
            }
            Self::MeshNotFound { link, file, tried } => write!(
                f,
                "link \"{link}\": {file} not found at {}",
                tried.display()
            ),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ImportError {
    Io {
        path: PathBuf,
        message: String,
    },
    Parse {
        path: PathBuf,
        message: String,
    },
    /// `floating`, `planar` or `spherical`.
    UnsupportedJoint {
        joint: String,
        kind: String,
    },
    /// A joint names a link the file does not have.
    UnknownLink {
        joint: String,
        link: String,
    },
    /// Every link is some joint's child.
    NoRoot,
    /// More than one link is nobody's child.
    MultipleRoots(Vec<String>),
    /// The result breaks a document invariant (a bad name, a loop).
    Invalid(ValidationError),
}

impl fmt::Display for ImportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, message } | Self::Parse { path, message } => {
                write!(f, "{}: {message}", path.display())
            }
            Self::UnsupportedJoint { joint, kind } => {
                write!(f, "joint \"{joint}\": {kind} joints are not supported")
            }
            Self::UnknownLink { joint, link } => {
                write!(f, "joint \"{joint}\" refers to missing link \"{link}\"")
            }
            Self::NoRoot => write!(f, "no root link: every link is a child"),
            Self::MultipleRoots(names) => {
                write!(f, "more than one root link: {}", names.join(", "))
            }
            Self::Invalid(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for ImportError {}

/// Reads `path` and builds the document; mesh paths are resolved against
/// the file's directory and `packages`.
pub fn load(
    path: &Path,
    packages: &PackageMap,
) -> Result<(Robot, Vec<ImportWarning>), ImportError> {
    let text = std::fs::read_to_string(path).map_err(|e| ImportError::Io {
        path: path.to_owned(),
        message: e.to_string(),
    })?;
    let urdf = urdf_rs::read_from_string(&text).map_err(|e| ImportError::Parse {
        path: path.to_owned(),
        message: e.to_string(),
    })?;
    let abs = riggen_core::absolute(path).map_err(|e| ImportError::Io {
        path: path.to_owned(),
        message: e.to_string(),
    })?;
    let base_dir = abs.parent().unwrap_or(Path::new("/"));
    from_urdf(&urdf, base_dir, packages)
}

/// The conversion itself, for a parsed file.
pub fn from_urdf(
    urdf: &urdf_rs::Robot,
    base_dir: &Path,
    packages: &PackageMap,
) -> Result<(Robot, Vec<ImportWarning>), ImportError> {
    let mut warnings = Vec::new();
    let mut robot = Robot::new(if urdf.name.is_empty() {
        "robot".to_owned()
    } else {
        urdf.name.clone()
    });
    robot.links.clear();

    let mut ids: BTreeMap<&str, LinkId> = BTreeMap::new();
    let mut assets: BTreeMap<(PathBuf, u64), MeshId> = BTreeMap::new();
    for link in &urdf.links {
        let id: LinkId = robot.next_id.alloc();
        ids.insert(link.name.as_str(), id);
        let converted = convert_link(
            link,
            base_dir,
            packages,
            &mut robot,
            &mut assets,
            &mut warnings,
        );
        robot.links.insert(id, converted);
    }

    let mut children = std::collections::BTreeSet::new();
    for joint in &urdf.joints {
        let kind = match joint.joint_type {
            urdf_rs::JointType::Fixed => JointKind::Fixed,
            urdf_rs::JointType::Revolute => JointKind::Revolute,
            urdf_rs::JointType::Continuous => JointKind::Continuous,
            urdf_rs::JointType::Prismatic => JointKind::Prismatic,
            ref other => {
                return Err(ImportError::UnsupportedJoint {
                    joint: joint.name.clone(),
                    kind: format!("{other:?}").to_lowercase(),
                });
            }
        };
        let link_id = |name: &str| {
            ids.get(name)
                .copied()
                .ok_or_else(|| ImportError::UnknownLink {
                    joint: joint.name.clone(),
                    link: name.to_owned(),
                })
        };
        let parent = link_id(&joint.parent.link)?;
        let child = link_id(&joint.child.link)?;
        children.insert(child);
        if let Some(mimic) = &joint.mimic {
            warnings.push(ImportWarning::MimicDropped {
                joint: joint.name.clone(),
                mimics: mimic.joint.clone(),
            });
        }
        if joint.safety_controller.is_some() {
            warnings.push(ImportWarning::SafetyControllerDropped {
                joint: joint.name.clone(),
            });
        }
        let limits = kind.requires_limits().then_some(Limits {
            lower: joint.limit.lower,
            upper: joint.limit.upper,
            effort: joint.limit.effort,
            velocity: joint.limit.velocity,
        });
        let dynamics = joint
            .dynamics
            .as_ref()
            .map(|d| Dynamics {
                damping: d.damping,
                friction: d.friction,
                armature: 0.0,
            })
            .unwrap_or_default();
        let id: JointId = robot.next_id.alloc();
        robot.joints.insert(
            id,
            Joint {
                name: joint.name.clone(),
                kind,
                parent,
                child,
                origin: pose_of(&joint.origin),
                axis: DVec3::from_array(*joint.axis.xyz),
                limits,
                dynamics,
            },
        );
    }

    let roots: Vec<(&str, LinkId)> = urdf
        .links
        .iter()
        .map(|l| (l.name.as_str(), ids[l.name.as_str()]))
        .filter(|(_, id)| !children.contains(id))
        .collect();
    robot.root = match roots[..] {
        [] => return Err(ImportError::NoRoot),
        [(_, id)] => id,
        _ => {
            return Err(ImportError::MultipleRoots(
                roots.iter().map(|(n, _)| (*n).to_owned()).collect(),
            ));
        }
    };
    validate(&robot).map_err(ImportError::Invalid)?;
    Ok((robot, warnings))
}

fn pose_of(p: &urdf_rs::Pose) -> Pose {
    Pose::from_xyz_rpy(DVec3::from_array(*p.xyz), DVec3::from_array(*p.rpy))
}

fn convert_link(
    link: &urdf_rs::Link,
    base_dir: &Path,
    packages: &PackageMap,
    robot: &mut Robot,
    assets: &mut BTreeMap<(PathBuf, u64), MeshId>,
    warnings: &mut Vec<ImportWarning>,
) -> Link {
    let mut out = Link::new(link.name.clone());
    // (resolved path, scale, pose) per visual mesh, to tell a collision
    // that repeats the visuals from one that differs.
    let mut visual_keys: Vec<(PathBuf, f64, Pose)> = Vec::new();

    let mut mesh_geom = |filename: &str,
                         scale: Option<urdf_rs::Vec3>,
                         origin: &urdf_rs::Pose,
                         robot: &mut Robot,
                         warnings: &mut Vec<ImportWarning>|
     -> (Geom, (PathBuf, f64, Pose)) {
        let (path, package_warning) = resolve_mesh_path(filename, base_dir, packages);
        if let Some(w) = package_warning {
            warnings.push(w);
        }
        let scale = match scale {
            Some(s) => {
                let [x, y, z] = *s;
                let used = x.max(y).max(z);
                if (x - y).abs() > 1e-12 || (x - z).abs() > 1e-12 {
                    warnings.push(ImportWarning::NonUniformScale {
                        link: link.name.clone(),
                        file: filename.to_owned(),
                        used,
                    });
                }
                used
            }
            None => 1.0,
        };
        let content_hash = match riggen_core::hash_file(&path) {
            Ok(h) => h,
            Err(_) => {
                warnings.push(ImportWarning::MeshNotFound {
                    link: link.name.clone(),
                    file: filename.to_owned(),
                    tried: path.clone(),
                });
                0
            }
        };
        let key = (path.clone(), scale.to_bits());
        let mesh = *assets.entry(key).or_insert_with(|| {
            robot.add_asset(MeshAsset {
                path: path.clone(),
                content_hash,
                scale,
                fix_up: None,
            })
        });
        let pose = pose_of(origin);
        let id: GeomId = robot.next_id.alloc();
        (
            Geom {
                id,
                mesh,
                pose,
                color: None,
            },
            (path, scale, pose),
        )
    };

    for visual in &link.visual {
        match &visual.geometry {
            urdf_rs::Geometry::Mesh { filename, scale } => {
                let (mut geom, key) = mesh_geom(filename, *scale, &visual.origin, robot, warnings);
                geom.color = visual
                    .material
                    .as_ref()
                    .and_then(|m| m.color.as_ref())
                    .map(|c| (*c.rgba).map(|v| v as f32));
                out.visuals.push(geom);
                visual_keys.push(key);
            }
            other => warnings.push(ImportWarning::PrimitiveVisualDropped {
                link: link.name.clone(),
                kind: geometry_kind(other),
            }),
        }
    }

    let mut primitives = Vec::new();
    let mut meshes: Vec<(Geom, (PathBuf, f64, Pose))> = Vec::new();
    for collision in &link.collision {
        match &collision.geometry {
            urdf_rs::Geometry::Mesh { filename, scale } => {
                meshes.push(mesh_geom(
                    filename,
                    *scale,
                    &collision.origin,
                    robot,
                    warnings,
                ));
            }
            other => {
                primitives.push((primitive_of(other, &collision.origin), geometry_kind(other)))
            }
        }
    }
    out.collision = if !meshes.is_empty() {
        for (_, kind) in &primitives {
            warnings.push(ImportWarning::MixedCollisionDropped {
                link: link.name.clone(),
                kind,
            });
        }
        let same_as_visual = meshes.len() == visual_keys.len()
            && meshes.iter().all(|(_, k)| visual_keys.contains(k));
        if same_as_visual {
            CollisionPolicy::SameAsVisual
        } else {
            CollisionPolicy::Meshes(meshes.into_iter().map(|(g, _)| g).collect())
        }
    } else if !primitives.is_empty() {
        CollisionPolicy::Primitives(primitives.into_iter().map(|(p, _)| p).collect())
    } else {
        CollisionPolicy::None
    };

    let inertial = &link.inertial;
    out.inertial = if inertial.mass.value > 0.0 {
        let i = &inertial.inertia;
        let tensor = DMat3::from_cols(
            DVec3::new(i.ixx, i.ixy, i.ixz),
            DVec3::new(i.ixy, i.iyy, i.iyz),
            DVec3::new(i.ixz, i.iyz, i.izz),
        );
        // The tensor is given in the inertial frame; the document wants it
        // in link axes about the CoM.
        let r = DMat3::from_quat(pose_of(&inertial.origin).r);
        InertialSpec::Override {
            mass: inertial.mass.value,
            com: DVec3::from_array(*inertial.origin.xyz),
            inertia: r * tensor * r.transpose(),
        }
    } else {
        if !out.visuals.is_empty() {
            warnings.push(ImportWarning::NoInertial {
                link: link.name.clone(),
            });
        }
        InertialSpec::Computed {
            density_override: None,
        }
    };
    out
}

fn geometry_kind(g: &urdf_rs::Geometry) -> &'static str {
    match g {
        urdf_rs::Geometry::Box { .. } => "box",
        urdf_rs::Geometry::Cylinder { .. } => "cylinder",
        urdf_rs::Geometry::Capsule { .. } => "capsule",
        urdf_rs::Geometry::Sphere { .. } => "sphere",
        urdf_rs::Geometry::Mesh { .. } => "mesh",
    }
}

fn primitive_of(g: &urdf_rs::Geometry, origin: &urdf_rs::Pose) -> Primitive {
    let pose = pose_of(origin);
    match *g {
        urdf_rs::Geometry::Box { size } => Primitive::Box {
            pose,
            size: DVec3::from_array(*size),
        },
        urdf_rs::Geometry::Cylinder { radius, length } => Primitive::Cylinder {
            pose,
            radius,
            length,
        },
        urdf_rs::Geometry::Capsule { radius, length } => Primitive::Capsule {
            pose,
            radius,
            length,
        },
        urdf_rs::Geometry::Sphere { radius } => Primitive::Sphere { pose, radius },
        urdf_rs::Geometry::Mesh { .. } => unreachable!("meshes are handled by the caller"),
    }
}

/// Where a `<mesh filename>` points. `package://name/rest`: the map, else
/// `rest` beside the file, else `name/rest` under an ancestor of the file's
/// directory; `file://` and absolute paths as they are; anything else
/// relative to the file.
pub fn resolve_mesh_path(
    filename: &str,
    base_dir: &Path,
    packages: &PackageMap,
) -> (PathBuf, Option<ImportWarning>) {
    if let Some(rest) = filename.strip_prefix("package://") {
        let (package, rest) = rest.split_once('/').unwrap_or((rest, ""));
        if let Some(dir) = packages.0.get(package) {
            return (dir.join(rest), None);
        }
        let beside = base_dir.join(rest);
        if beside.exists() {
            return (beside, None);
        }
        for ancestor in base_dir.ancestors() {
            let candidate = ancestor.join(package).join(rest);
            if candidate.exists() {
                return (candidate, None);
            }
        }
        return (
            beside.clone(),
            Some(ImportWarning::PackageUnresolved {
                package: package.to_owned(),
                used: beside,
            }),
        );
    }
    if let Some(abs) = filename.strip_prefix("file://") {
        return (PathBuf::from(abs), None);
    }
    let path = Path::new(filename);
    if path.is_absolute() {
        (path.to_owned(), None)
    } else {
        (base_dir.join(path), None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::fixtures;
    use riggen_core::{JointState, fk};

    fn arm() -> (Robot, Vec<ImportWarning>) {
        load(&fixtures().join("arm/arm.urdf"), &PackageMap::default()).unwrap()
    }

    #[test]
    fn the_sample_urdf_imports_with_the_expected_warnings() {
        let (robot, warnings) = arm();
        validate(&robot).unwrap();
        assert_eq!(robot.name, "arm");
        assert_eq!(robot.links.len(), 5);
        assert_eq!(robot.joints.len(), 4);
        assert_eq!(robot.links[&robot.root].name, "base_link");
        assert_eq!(
            warnings,
            vec![
                ImportWarning::SafetyControllerDropped {
                    joint: "upper_joint".into()
                },
                ImportWarning::MimicDropped {
                    joint: "fore_joint".into(),
                    mimics: "upper_joint".into()
                },
            ]
        );
        let by_name = |n: &str| {
            robot
                .links
                .values()
                .find(|l| l.name == n)
                .unwrap_or_else(|| panic!("{n}"))
        };
        // Every mesh resolved beside the file (package://arm/… with no map).
        for asset in robot.assets.values() {
            assert!(asset.path.exists(), "{}", asset.path.display());
            assert_eq!(asset.scale, 0.001);
        }
        // <inertial> → Override on every part.
        for name in ["base", "shoulder", "upper", "fore"] {
            assert!(
                matches!(by_name(name).inertial, InertialSpec::Override { .. }),
                "{name}"
            );
        }
        assert!(matches!(
            by_name("base_link").inertial,
            InertialSpec::Computed { .. }
        ));
        // The four collision shapes the file holds.
        assert!(matches!(
            by_name("base").collision,
            CollisionPolicy::Primitives(ref p) if matches!(p[..], [Primitive::Box { .. }])
        ));
        assert_eq!(by_name("shoulder").collision, CollisionPolicy::SameAsVisual);
        assert_eq!(by_name("upper").collision, CollisionPolicy::SameAsVisual);
        let CollisionPolicy::Meshes(geoms) = &by_name("fore").collision else {
            panic!("{:?}", by_name("fore").collision);
        };
        assert_eq!(geoms.len(), 1);
        assert!(robot.assets[&geoms[0].mesh].path.ends_with("fore_hull.stl"));
        // Kinds: fixed, revolute, revolute, continuous.
        let mut kinds: Vec<(String, JointKind)> = robot
            .joints
            .values()
            .map(|j| (j.name.clone(), j.kind))
            .collect();
        kinds.sort_by(|a, b| a.0.cmp(&b.0));
        assert_eq!(
            kinds,
            [
                ("base_joint".to_owned(), JointKind::Fixed),
                ("fore_joint".to_owned(), JointKind::Continuous),
                ("shoulder_joint".to_owned(), JointKind::Revolute),
                ("upper_joint".to_owned(), JointKind::Revolute),
            ]
        );
    }

    #[test]
    fn the_imported_arm_has_the_samples_fk() {
        let (imported, _) = arm();
        let (sample, _) = riggen_core::load(&fixtures().join("arm/arm.riggen")).unwrap();
        let joint_id = |robot: &Robot, name: &str| {
            *robot.joints.iter().find(|(_, j)| j.name == name).unwrap().0
        };
        for q in [[0.0, 0.0, 0.0], [0.5, -0.7, 1.2], [-2.0, 1.0, 3.0]] {
            let mut qi = JointState::new();
            let mut qs = JointState::new();
            for (name, v) in ["shoulder_joint", "upper_joint", "fore_joint"]
                .iter()
                .zip(q)
            {
                qi.set(joint_id(&imported, name), v);
                qs.set(joint_id(&sample, name), v);
            }
            let wi = fk(&imported, &qi);
            let ws = fk(&sample, &qs);
            for (id, link) in &imported.links {
                let other = sample
                    .links
                    .iter()
                    .find(|(_, l)| l.name == link.name)
                    .unwrap()
                    .0;
                let (a, b) = (wi[id], ws[other]);
                assert!(
                    (a.t - b.t).length() < 1e-9,
                    "{} at {q:?}: {a:?} vs {b:?}",
                    link.name
                );
                assert!(
                    a.r.abs_diff_eq(b.r, 1e-9) || a.r.abs_diff_eq(-b.r, 1e-9),
                    "{} at {q:?}",
                    link.name
                );
            }
        }
    }

    #[test]
    fn the_imported_arm_exports_to_mjcf() {
        let (robot, _) = arm();
        let (store, errors) = crate::MeshStore::load(&robot);
        assert!(errors.is_empty(), "{errors:?}");
        let resolved = crate::resolve(
            &robot,
            &store,
            &crate::ComputeNow,
            &crate::ExportOptions::default(),
        )
        .unwrap();
        let dir = std::env::temp_dir().join(format!("riggen-urdf-in-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let written = crate::export(&resolved, &crate::ExportOptions::default(), &dir).unwrap();
        assert!(dir.join("arm.xml").is_file());
        assert!(dir.join("meshes/fore_hull.stl").is_file(), "{written:?}");
        let xml = std::fs::read_to_string(dir.join("arm.xml")).unwrap();
        assert!(xml.contains("type=\"box\""), "{xml}");
        assert!(xml.contains("mesh=\"fore_hull\""), "{xml}");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn package_paths_resolve_through_the_map_beside_the_file_or_up_the_tree() {
        let dir = fixtures().join("arm");
        let none = PackageMap::default();
        let (p, w) = resolve_mesh_path("package://arm/base.stl", &dir, &none);
        assert_eq!(p, dir.join("base.stl"));
        assert!(w.is_none());
        // Up the tree: fixtures/arm/base.stl from fixtures/.
        let (p, w) = resolve_mesh_path("package://arm/base.stl", &fixtures(), &none);
        assert_eq!(p, dir.join("base.stl"));
        assert!(w.is_none());
        // The map wins.
        let mut map = PackageMap::default();
        map.0.insert("arm".into(), PathBuf::from("/elsewhere"));
        let (p, _) = resolve_mesh_path("package://arm/base.stl", &dir, &map);
        assert_eq!(p, PathBuf::from("/elsewhere/base.stl"));
        // Unknown: beside the file, with a warning.
        let (p, w) = resolve_mesh_path("package://nope/x.stl", &dir, &none);
        assert_eq!(p, dir.join("x.stl"));
        assert!(matches!(w, Some(ImportWarning::PackageUnresolved { .. })));
        assert_eq!(
            resolve_mesh_path("file:///abs/x.stl", &dir, &none).0,
            PathBuf::from("/abs/x.stl")
        );
        assert_eq!(
            resolve_mesh_path("rel/x.stl", &dir, &none).0,
            dir.join("rel/x.stl")
        );
    }

    #[test]
    fn what_the_document_cannot_hold_is_a_warning_or_an_error() {
        let text = r#"
<robot name="odd">
  <link name="a">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <visual><origin xyz="0 0 1"/><geometry><mesh filename="cube_binary.stl" scale="0.001 0.002 0.001"/></geometry></visual>
    <collision><geometry><mesh filename="cube_ascii.stl"/></geometry></collision>
    <collision><geometry><sphere radius="1"/></geometry></collision>
  </link>
  <link name="b"/>
  <joint name="j" type="prismatic">
    <parent link="a"/><child link="b"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="0.1" effort="2" velocity="3"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>
</robot>"#;
        let urdf = urdf_rs::read_from_string(text).unwrap();
        let (robot, warnings) = from_urdf(&urdf, &fixtures(), &PackageMap::default()).unwrap();
        let a = robot.links.values().find(|l| l.name == "a").unwrap();
        assert_eq!(a.visuals.len(), 1, "the box visual was dropped");
        assert_eq!(robot.assets[&a.visuals[0].mesh].scale, 0.002);
        assert!(matches!(a.collision, CollisionPolicy::Meshes(ref g) if g.len() == 1));
        assert!(matches!(a.inertial, InertialSpec::Computed { .. }));
        let kinds: Vec<&str> = warnings
            .iter()
            .map(|w| match w {
                ImportWarning::PrimitiveVisualDropped { kind, .. } => *kind,
                ImportWarning::NonUniformScale { .. } => "scale",
                ImportWarning::MixedCollisionDropped { kind, .. } => *kind,
                ImportWarning::NoInertial { .. } => "inertial",
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(kinds, ["box", "scale", "sphere", "inertial"]);
        let j = robot.joints.values().next().unwrap();
        assert_eq!(j.kind, JointKind::Prismatic);
        assert_eq!(j.limits.unwrap().effort, 2.0);
        assert_eq!(j.dynamics.damping, 0.5);

        let floating = text.replace("type=\"prismatic\"", "type=\"floating\"");
        let urdf = urdf_rs::read_from_string(&floating).unwrap();
        assert!(matches!(
            from_urdf(&urdf, &fixtures(), &PackageMap::default()),
            Err(ImportError::UnsupportedJoint { .. })
        ));
        let two_roots = text
            .replace("<joint name=\"j\"", "<joint name=\"j\" xx=\"1\"")
            .replace(
                "<parent link=\"a\"/><child link=\"b\"/>",
                "<parent link=\"a\"/><child link=\"c\"/>",
            );
        let urdf = urdf_rs::read_from_string(&two_roots).unwrap();
        assert!(matches!(
            from_urdf(&urdf, &fixtures(), &PackageMap::default()),
            Err(ImportError::UnknownLink { .. })
        ));
        assert!(matches!(
            load(Path::new("/nowhere/none.urdf"), &PackageMap::default()),
            Err(ImportError::Io { .. })
        ));
    }
}
