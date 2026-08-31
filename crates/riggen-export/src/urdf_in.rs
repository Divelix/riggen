//! URDF → `Robot` (docs/02-data-model.md §URDF import), over `urdf-rs`.
//! Links and joints map directly — URDF's joint-frame convention is ours
//! (ADR-0004) — `<inertial>` becomes `InertialSpec::Override`, a uniform
//! `<mesh scale>` becomes `MeshAsset::scale`, a `<collision>` mesh that is
//! not the visual becomes `CollisionPolicy::Meshes` (OPEN 1), primitives
//! become `Primitives`, `<mimic>` becomes a `Mimic` (ADR-0013). What the
//! document cannot hold — `<safety_controller>`, a primitive visual, a
//! non-uniform scale, a coupling `validate` refuses — is dropped with an
//! [`ImportWarning`] that names it, never silently.
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
    LinkId, MeshAsset, MeshId, Mimic, Pose, Primitive, Robot, ValidationError, validate,
};

/// `package name → directory` for `package://name/...` mesh paths.
#[derive(Debug, Clone, Default)]
pub struct PackageMap(pub BTreeMap<String, PathBuf>);

/// Something the URDF held that the document does not; the import went on
/// without it.
#[derive(Debug, Clone, PartialEq)]
pub enum ImportWarning {
    /// A `<mimic>` the document cannot hold. `reason` says which rule it
    /// broke — a chain, a fixed leader, a leader not in the file, a reach
    /// outside the follower's limits (ADR-0013).
    MimicDropped {
        joint: String,
        mimics: String,
        reason: String,
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
            Self::MimicDropped {
                joint,
                mimics,
                reason,
            } => {
                write!(
                    f,
                    "joint \"{joint}\": <mimic joint=\"{mimics}\"> dropped, {reason}"
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
    let mut joint_ids: BTreeMap<&str, JointId> = BTreeMap::new();
    let mut pending_mimics: Vec<(JointId, &urdf_rs::Mimic)> = Vec::new();
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
                // A `<mimic>` may name a joint further down the file, so
                // the couplings are resolved in a second pass below.
                mimic: None,
            },
        );
        joint_ids.insert(joint.name.as_str(), id);
        if let Some(m) = &joint.mimic {
            pending_mimics.push((id, m));
        }
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
    // The couplings, now that every joint has an id and the tree is whole
    // (ADR-0013). URDF's defaults are multiplier 1, offset 0.
    for (follower, m) in pending_mimics {
        let multiplier = m.multiplier.unwrap_or(1.0);
        let offset = m.offset.unwrap_or(0.0);
        let refused = if !multiplier.is_finite() || !offset.is_finite() {
            Some("its multiplier or offset is not a number".to_owned())
        } else {
            match joint_ids.get(m.joint.as_str()) {
                None => Some("no joint of that name is in the file".to_owned()),
                Some(&leader) => {
                    robot
                        .joints
                        .get_mut(&follower)
                        .expect("just inserted")
                        .mimic = Some(Mimic {
                        joint: leader,
                        multiplier,
                        offset,
                    });
                    None
                }
            }
        };
        if let Some(reason) = refused {
            warnings.push(ImportWarning::MimicDropped {
                joint: robot.joints[&follower].name.clone(),
                mimics: m.joint.clone(),
                reason,
            });
        }
    }
    // Whatever `validate` refuses about them is dropped with its reason
    // rather than failing the whole import: a gripper whose fingers move
    // independently still opens, and the user is told why.
    for (follower, reason) in mimic_refusals(&robot) {
        let leader = robot
            .joints
            .get_mut(&follower)
            .and_then(|j| j.mimic.take())
            .map(|m| m.joint);
        let mimics = leader
            .and_then(|l| robot.joints.get(&l))
            .map(|j| j.name.clone())
            .unwrap_or_default();
        warnings.push(ImportWarning::MimicDropped {
            joint: robot.joints[&follower].name.clone(),
            mimics,
            reason,
        });
    }

    validate(&robot).map_err(ImportError::Invalid)?;
    Ok((robot, warnings))
}

/// What `validate` refuses about a coupling, per follower. `validate` owns
/// the rules (ADR-0013); this only phrases its verdict for the status bar.
/// Every other error it reports is left alone and still fails the import.
fn mimic_refusals(robot: &Robot) -> Vec<(JointId, String)> {
    riggen_core::validation_errors(robot)
        .into_iter()
        .filter_map(|e| match e {
            ValidationError::SelfMimic(j) => Some((j, "a joint cannot follow itself".to_owned())),
            ValidationError::MimicOnFixedJoint(j) => {
                Some((j, "a fixed joint has no value to drive".to_owned()))
            }
            ValidationError::ZeroMimicMultiplier(j) => {
                Some((j, "its multiplier is zero".to_owned()))
            }
            ValidationError::DanglingMimicJoint { joint, .. } => {
                Some((joint, "its leader is not a joint in this file".to_owned()))
            }
            ValidationError::MimicLeaderFixed { joint, .. } => {
                Some((joint, "its leader is a fixed joint".to_owned()))
            }
            ValidationError::MimicChain { joint, .. } => Some((
                joint,
                "its leader is itself a mimic, and chains are not supported".to_owned(),
            )),
            ValidationError::MimicExceedsLimits {
                joint,
                lower,
                upper,
                ..
            } => Some((
                joint,
                format!("it would reach {lower}..{upper}, outside its own limits"),
            )),
            _ => None,
        })
        .collect()
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
        // Five real links and joints, plus the two dummy links the file's
        // named frames are written as: the import keeps them as links, on
        // purpose (ADR-0012).
        assert_eq!(robot.links.len(), 7);
        assert_eq!(robot.joints.len(), 6);
        assert!(robot.frames.is_empty(), "no massless link became a frame");
        assert_eq!(robot.links[&robot.root].name, "base_link");
        assert_eq!(
            warnings,
            vec![ImportWarning::SafetyControllerDropped {
                joint: "upper_joint".into()
            }],
            "the file's <mimic> is kept now, so nothing is dropped for it"
        );
        // …and it is kept as the coupling it is (ADR-0013).
        let joint = |n: &str| robot.joints.values().find(|j| j.name == n).unwrap();
        let upper = *robot
            .joints
            .iter()
            .find(|(_, j)| j.name == "upper_joint")
            .unwrap()
            .0;
        assert_eq!(
            joint("fore_joint").mimic,
            Some(riggen_core::Mimic {
                joint: upper,
                multiplier: -0.5,
                offset: 0.1
            })
        );
        assert_eq!(joint("upper_joint").mimic, None);
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
                ("camera_mount_fixed".to_owned(), JointKind::Fixed),
                ("fore_joint".to_owned(), JointKind::Continuous),
                ("shoulder_joint".to_owned(), JointKind::Revolute),
                ("tcp_fixed".to_owned(), JointKind::Fixed),
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
        const NAMES: [&str; 3] = ["shoulder_joint", "upper_joint", "fore_joint"];
        for q in [[0.0, 0.0, 0.0], [0.5, -0.7, 1.2], [-2.0, 1.0, 3.0]] {
            let mut qi = JointState::new();
            for (name, v) in NAMES.iter().zip(q) {
                qi.set(joint_id(&imported, name), v);
            }
            // The URDF carries a `<mimic>` on `fore_joint` that the import
            // now keeps (ADR-0013), so the two documents are compared at
            // one configuration: the imported one's, couplings resolved.
            let qi = riggen_core::resolve_q(&imported, &qi);
            let mut qs = JointState::new();
            for name in NAMES {
                qs.set(joint_id(&sample, name), qi.get(joint_id(&imported, name)));
            }
            let wi = fk(&imported, &qi);
            let ws = fk(&sample, &qs);
            for (id, link) in &imported.links {
                // The sample has the same name either as a link or — for
                // the two the URDF spells as dummy links — as a frame,
                // whose world pose is `world(parent) ∘ pose` (ADR-0012).
                let b = match sample.links.iter().find(|(_, l)| l.name == link.name) {
                    Some((other, _)) => ws[other],
                    None => {
                        let frame = sample
                            .frames
                            .values()
                            .find(|f| f.name == link.name)
                            .unwrap_or_else(|| panic!("{} is neither link nor frame", link.name));
                        ws[&frame.parent].compose(&frame.pose)
                    }
                };
                let a = wi[id];
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

    /// A three-joint chain the tests bend into each refused shape. `mimic`
    /// is spliced into `j2` unless it is given for another joint.
    fn coupled(mimic_on_j2: &str, extra: &str) -> String {
        format!(
            r#"
<robot name="grip">
  <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <joint name="j1" type="revolute">
    <parent link="a"/><child link="b"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="b"/><child link="c"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
    {mimic_on_j2}
  </joint>
  <joint name="j3" type="fixed">
    <parent link="c"/><child link="d"/>
    {extra}
  </joint>
</robot>"#
        )
    }

    fn import(text: &str) -> (Robot, Vec<ImportWarning>) {
        let urdf = urdf_rs::read_from_string(text).unwrap();
        from_urdf(&urdf, &fixtures(), &PackageMap::default()).unwrap()
    }

    /// A `<mimic>` naming a joint further down the file is still resolved:
    /// the couplings are a second pass over the whole tree (ADR-0013).
    #[test]
    fn a_mimic_is_kept_whatever_order_the_file_names_it_in() {
        let (robot, warnings) = import(&coupled(
            r#"<mimic joint="j1" multiplier="-0.5" offset="0.1"/>"#,
            "",
        ));
        assert_eq!(warnings, vec![]);
        let id = |n: &str| *robot.joints.iter().find(|(_, j)| j.name == n).unwrap().0;
        assert_eq!(
            robot.joints[&id("j2")].mimic,
            Some(Mimic {
                joint: id("j1"),
                multiplier: -0.5,
                offset: 0.1
            })
        );
        // URDF's own defaults, when the attributes are left off.
        let (robot, _) = import(&coupled(r#"<mimic joint="j1"/>"#, ""));
        let m = robot
            .joints
            .values()
            .find(|j| j.name == "j2")
            .unwrap()
            .mimic
            .unwrap();
        assert_eq!((m.multiplier, m.offset), (1.0, 0.0));
    }

    /// Everything `validate` refuses is dropped with a reason, and the
    /// document still opens (ADR-0013) — the import never fails over a
    /// coupling.
    #[test]
    fn a_refused_mimic_is_dropped_with_its_reason_not_an_error() {
        let cases = [
            (
                r#"<mimic joint="nope"/>"#,
                "",
                "no joint of that name is in the file",
            ),
            (r#"<mimic joint="j3"/>"#, "", "its leader is a fixed joint"),
            (r#"<mimic joint="j2"/>"#, "", "a joint cannot follow itself"),
            (
                r#"<mimic joint="j1" multiplier="0"/>"#,
                "",
                "its multiplier is zero",
            ),
            (
                r#"<mimic joint="j1" multiplier="3"/>"#,
                "",
                "it would reach -3..3, outside its own limits",
            ),
            (
                r#"<mimic joint="j1" multiplier="nan"/>"#,
                "",
                "its multiplier or offset is not a number",
            ),
        ];
        for (mimic, extra, reason) in cases {
            let (robot, warnings) = import(&coupled(mimic, extra));
            assert!(
                robot.joints.values().all(|j| j.mimic.is_none()),
                "{mimic}: {robot:?}"
            );
            let dropped: Vec<&ImportWarning> = warnings
                .iter()
                .filter(|w| matches!(w, ImportWarning::MimicDropped { .. }))
                .collect();
            assert_eq!(dropped.len(), 1, "{mimic}: {warnings:?}");
            let ImportWarning::MimicDropped {
                joint, reason: r, ..
            } = dropped[0]
            else {
                unreachable!()
            };
            assert_eq!(joint, "j2");
            assert_eq!(r, reason, "{mimic}");
            assert!(dropped[0].to_string().contains(reason));
        }

        // A chain: j2 follows j1, and j3 — made movable — follows j2.
        let text = coupled(r#"<mimic joint="j1" multiplier="0.5"/>"#, "")
            .replace(
                r#"<joint name="j3" type="fixed">"#,
                r#"<joint name="j3" type="continuous">"#,
            )
            .replace(
                r#"<parent link="c"/><child link="d"/>"#,
                r#"<parent link="c"/><child link="d"/><axis xyz="0 0 1"/><mimic joint="j2"/>"#,
            );
        let (robot, warnings) = import(&text);
        let id = |n: &str| *robot.joints.iter().find(|(_, j)| j.name == n).unwrap().0;
        assert!(robot.joints[&id("j2")].mimic.is_some(), "the leader stays");
        assert_eq!(robot.joints[&id("j3")].mimic, None);
        assert_eq!(
            warnings,
            vec![ImportWarning::MimicDropped {
                joint: "j3".into(),
                mimics: "j2".into(),
                reason: "its leader is itself a mimic, and chains are not supported".into(),
            }]
        );

        // And a `<mimic>` on a fixed joint, which has nothing to drive.
        let text = coupled("", r#"<mimic joint="j1"/>"#);
        let (robot, warnings) = import(&text);
        assert!(robot.joints.values().all(|j| j.mimic.is_none()));
        assert_eq!(
            warnings,
            vec![ImportWarning::MimicDropped {
                joint: "j3".into(),
                mimics: "j1".into(),
                reason: "a fixed joint has no value to drive".into(),
            }]
        );
    }
}
