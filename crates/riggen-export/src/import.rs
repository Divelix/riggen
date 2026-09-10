//! The vocabulary both imports speak (ADR-0015 §4).
//!
//! [`ImportWarning`] is something the file held that the document has no
//! field for — the import went on without it. [`ImportError`] is a file
//! whose *shape* the document cannot represent, where importing anyway
//! would silently change the robot. There is one pair of these for URDF
//! and MJCF together, because a `MeshNotFound` is the same event whichever
//! file it came out of, and the three places that phrase them — the app's
//! status bar, `RiggenWarning` in the SDK, the CLI's stderr — should not
//! have to match on two types to say one sentence.

use std::fmt;
use std::path::{Path, PathBuf};

use riggen_core::{JointId, Robot, ValidationError};

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
    // ---------------------------------------------------------------
    // MJCF (ADR-0015 §4). Everything above is raised by the URDF import,
    // and `MimicDropped`, `NonUniformScale`, `PrimitiveVisualDropped`,
    // `NoInertial` and `MeshNotFound` are raised by both: one event, one
    // name, whichever file it came out of.
    // ---------------------------------------------------------------
    /// An element — or a robot-changing attribute — the document has no
    /// field for: `<tendon>`, `<sensor>`, `<contact>`, `<general actdim>`.
    /// One warning per name with a count, not one per occurrence (ADR-0015
    /// §1).
    ElementDropped {
        element: String,
        count: usize,
    },
    /// A `<geom>` whose shape the document has none of: `plane`,
    /// `ellipsoid`, `sdf`, `hfield`, or a mesh it could not read.
    GeomDropped {
        link: String,
        kind: String,
    },
    /// A `<freejoint>` on the root body: `floating_base` is an export
    /// option, not a document field, so the robot imports fixed to the
    /// world and exports floating again when asked to (ADR-0015 §5).
    FreeJointDropped {
        body: String,
    },
    /// An actuator that is not one of the three presets, or that drives
    /// neither a joint nor a tendon (ADR-0014, ADR-0015 §1, ADR-0025 §4).
    ActuatorDropped {
        actuator: String,
        reason: String,
    },
    /// A `<tendon>` the document cannot hold: a `<spatial>`, which routes
    /// over sites and wrapping geoms, or a `<fixed>` whose joints it has
    /// no room for — one that is not in the file, one that is fixed, one
    /// twice, a zero coefficient, none at all (ADR-0025 §4). A tendon is
    /// dropped whole: half a linear combination is a different tendon.
    TendonDropped {
        tendon: String,
        reason: String,
    },
    /// A `<site>` that cannot become a `Frame`: it has no name, or its
    /// name is already a link's or another frame's — they are one
    /// namespace (ADR-0012).
    FrameDropped {
        site: String,
        reason: String,
    },
    /// A body whose `<geom mass|density>` was its only mass: MuJoCo's
    /// `inertiafromgeom`, which the import deliberately does not do
    /// (ADR-0015 §7).
    MassFromGeomIgnored {
        link: String,
    },
    /// A `slide` joint with no range. The document has no unlimited
    /// prismatic, so a range was invented rather than the joint dropped.
    LimitsInvented {
        joint: String,
        lower: f64,
        upper: f64,
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
            Self::ElementDropped { element, count } => write!(
                f,
                "{element} × {count}: nothing in the document holds it; not read"
            ),
            Self::GeomDropped { link, kind } => {
                write!(f, "link \"{link}\": {kind} was dropped")
            }
            Self::FreeJointDropped { body } => write!(
                f,
                "body \"{body}\": <freejoint> dropped; export with a floating base to get it back"
            ),
            Self::ActuatorDropped { actuator, reason } => {
                write!(f, "actuator \"{actuator}\" dropped, {reason}")
            }
            Self::TendonDropped { tendon, reason } => {
                write!(f, "tendon \"{tendon}\" dropped, {reason}")
            }
            Self::FrameDropped { site, reason } => {
                write!(f, "site \"{site}\" dropped, {reason}")
            }
            Self::MassFromGeomIgnored { link } => write!(
                f,
                "link \"{link}\": the mass on its geoms was not read; assign a material so one can be computed"
            ),
            Self::LimitsInvented {
                joint,
                lower,
                upper,
            } => write!(
                f,
                "joint \"{joint}\" has no range and the document has no unlimited prismatic; {lower}..{upper} used"
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
    /// Several `<joint>`s in one `<body>` — MuJoCo's way of spelling a ball
    /// or planar DoF, against a document whose joints are the edges of the
    /// link tree (ADR-0005, ADR-0015 §5, re-affirmed with the corpus in
    /// ADR-0022). The message says how to split the body, because this is
    /// the refusal a real Menagerie file hits.
    CompositeJoint {
        body: String,
        joints: Vec<String>,
    },
    /// A `<joint>` on the body directly under `<worldbody>`: the document's
    /// root link has no parent joint to carry it, and dropping it would
    /// weld the robot to the world.
    JointOnRoot {
        body: String,
        joint: String,
    },
    /// An element that re-shapes the tree — `<replicate>`, `<attach>` — or
    /// a `<compiler coordinate="global">`. `<include>` and `<frame>` are
    /// not among them: they are resolved before the reader looks
    /// (ADR-0026). `meaning` says what the element does, so the refusal
    /// leaves the user able to act (ADR-0022 §2).
    UnsupportedElement {
        element: String,
        meaning: String,
    },
    /// An `<include file>` that neither the model's directory nor the
    /// including file's holds. Its own variant rather than `Io` because the
    /// message has to say what else to bring: on the web a `scene.xml`
    /// dropped without its `robot.xml` is the common case (ADR-0017,
    /// ADR-0026 §2). `file` is the path as written; `from` is the file
    /// that asked for it.
    IncludeNotFound {
        file: PathBuf,
        from: PathBuf,
    },
    /// The same `<include file>` — as written, normalised — a second time,
    /// which is also how a cycle and a self-include present. MuJoCo makes
    /// it a hard error, and a file MuJoCo refuses should not open here
    /// (ADR-0026 §2).
    DuplicateInclude {
        file: PathBuf,
    },
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
            Self::CompositeJoint { body, joints } => write!(
                f,
                "body \"{body}\" has {} joints ({}); MuJoCo spells a multi-DoF \
                 joint this way, and the link tree holds one joint per body \
                 — split it into nested bodies with one joint each, the same \
                 model, and import that",
                joints.len(),
                joints.join(", ")
            ),
            Self::JointOnRoot { body, joint } => write!(
                f,
                "root body \"{body}\" carries joint \"{joint}\"; the root link has no parent joint"
            ),
            Self::UnsupportedElement { element, meaning } => {
                write!(f, "{element} is not supported: {meaning}")
            }
            // File names, not paths, and short enough to read in the
            // status bar: on the web `from` is a synthetic `/dropped/…`
            // (ADR-0017), and the two directories that were searched are
            // detail beside the one thing to do about it.
            Self::IncludeNotFound { file, from } => write!(
                f,
                "{} includes {}, which is not with it; open or drop both files together",
                name_of(from),
                file.display()
            ),
            Self::DuplicateInclude { file } => write!(
                f,
                "{} is included twice; MuJoCo refuses that, and so does the import",
                file.display()
            ),
            Self::Invalid(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for ImportError {}

/// A path as a message names it: its file name. Every path an import
/// error carries is either absolute or synthetic, and neither is what a
/// status bar has room for.
fn name_of(path: &Path) -> std::path::Display<'_> {
    path.file_name().map_or(path, Path::new).display()
}

/// What `validate` refuses about a coupling, per follower. `validate` owns
/// the rules (ADR-0013); this only phrases its verdict for the status bar.
/// Every other error it reports is left alone and still fails the import.
/// Both imports phrase it the same way (ADR-0015 §4).
pub(crate) fn mimic_refusals(robot: &Robot) -> Vec<(JointId, String)> {
    riggen_core::validation_errors(robot)
        .into_iter()
        .flat_map(|e| match e {
            ValidationError::SelfMimic(j) => vec![(j, "a joint cannot follow itself".to_owned())],
            ValidationError::MimicOnFixedJoint(j) => {
                vec![(j, "a fixed joint has no value to drive".to_owned())]
            }
            ValidationError::ZeroMimicMultiplier(j) => {
                vec![(j, "its multiplier is zero".to_owned())]
            }
            ValidationError::DanglingMimicJoint { joint, .. } => {
                vec![(joint, "its leader is not a joint in this file".to_owned())]
            }
            ValidationError::MimicLeaderFixed { joint, .. } => {
                vec![(joint, "its leader is a fixed joint".to_owned())]
            }
            // A chain resolves (ADR-0025); a ring of followers has no free
            // leader at its head, so every joint in it loses its coupling.
            ValidationError::MimicCycle { joints } => joints
                .into_iter()
                .map(|j| (j, "it is part of a cycle of followers".to_owned()))
                .collect(),
            ValidationError::MimicExceedsLimits {
                joint,
                lower,
                upper,
                ..
            } => vec![(
                joint,
                format!("it would reach {lower}..{upper}, outside its own limits"),
            )],
            _ => Vec::new(),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use crate::MeshStore;
    use crate::test_util::fixtures;
    use riggen_core::{Disk, FileSource, MemorySource};
    use std::path::Path;

    /// Every fixture file the arm needs, keyed under a root that does not
    /// exist, so a read that slipped through to the filesystem fails
    /// loudly instead of quietly agreeing with the on-disk load.
    fn dropped(root: &Path, files: &[(&str, &str)]) -> MemorySource {
        let mut memory = MemorySource::default();
        for (at, from) in files {
            memory.insert(root.join(at), std::fs::read(fixtures().join(from)).unwrap());
        }
        memory
    }

    /// The arm's five meshes, `(where the reader will look, what to put
    /// there)`. URDF's `package://arm/x.stl` falls back to "beside the
    /// file"; MJCF's `meshdir="arm"` makes a subdirectory of it.
    const URDF_SET: [(&str, &str); 6] = [
        ("arm.urdf", "arm/arm.urdf"),
        ("base.stl", "arm/base.stl"),
        ("shoulder.stl", "arm/shoulder.stl"),
        ("upper.stl", "arm/upper.stl"),
        ("fore.stl", "arm/fore.stl"),
        ("fore_hull.stl", "arm/fore_hull.stl"),
    ];

    const MJCF_SET: [(&str, &str); 8] = [
        ("menagerie_style.xml", "menagerie_style.xml"),
        // The half the main file `<include>`s (ADR-0026): a dropped set
        // has to carry it, exactly as it carries the meshes.
        ("menagerie_style_arm.xml", "menagerie_style_arm.xml"),
        ("arm/base.stl", "arm/base.stl"),
        ("arm/shoulder.stl", "arm/shoulder.stl"),
        ("arm/upper.stl", "arm/upper.stl"),
        ("arm/fore.stl", "arm/fore.stl"),
        ("arm/fore_hull.stl", "arm/fore_hull.stl"),
        ("arm/thing.msh", "arm/thing.msh"),
    ];

    /// The URDF import over bytes (ADR-0017): `package://arm/base.stl`
    /// resolves against the dropped set the same way it resolves beside the
    /// file on disk, and the document that comes out is the same one.
    #[test]
    fn urdf_import_from_memory_matches_disk() {
        let root = Path::new("/dropped");
        assert!(!root.exists(), "the synthetic root must not exist");
        let memory = dropped(root, &URDF_SET);

        let (from_memory, memory_warnings) = crate::urdf_in::load(
            &root.join("arm.urdf"),
            &crate::PackageMap::default(),
            &memory,
        )
        .unwrap();
        let (mut from_disk, disk_warnings) = crate::urdf_in::load(
            &fixtures().join("arm/arm.urdf"),
            &crate::PackageMap::default(),
            &Disk,
        )
        .unwrap();
        assert_eq!(memory_warnings, disk_warnings);
        for asset in from_disk.assets.values_mut() {
            asset.path = root.join(asset.path.file_name().unwrap());
        }
        assert_eq!(
            serde_json::to_value(&from_memory).unwrap(),
            serde_json::to_value(&from_disk).unwrap()
        );

        // And the meshes those paths name load out of the same set.
        let (store, errors) = MeshStore::load(&from_memory, &memory);
        assert_eq!(errors, Vec::new());
        assert_eq!(store.0.len(), from_memory.referenced_assets().len());
    }

    /// The MJCF import over bytes, `<compiler meshdir="arm">` and all: the
    /// mesh directory is a path inside the dropped set, not on any disk.
    #[test]
    fn mjcf_import_from_memory_matches_disk() {
        let root = Path::new("/dropped");
        let memory = dropped(root, &MJCF_SET);

        let (from_memory, memory_warnings, memory_inline) =
            crate::mjcf_in::load(&root.join("menagerie_style.xml"), &memory).unwrap();
        let (mut from_disk, disk_warnings, disk_inline) =
            crate::mjcf_in::load(&fixtures().join("menagerie_style.xml"), &Disk).unwrap();
        assert_eq!(memory_warnings, disk_warnings);
        assert_eq!(memory_inline, disk_inline);
        // Every asset's path rebases the same way `dropped()` laid the set
        // out — most of them under `arm/` (`<compiler meshdir>`), but the
        // one inline mesh sits beside the source file itself, same as on
        // disk (docs/DATA-MODEL.md §Geometry).
        let fixtures_abs = riggen_core::absolute(&fixtures()).unwrap();
        for asset in from_disk.assets.values_mut() {
            let rel = asset.path.strip_prefix(&fixtures_abs).unwrap();
            asset.path = root.join(rel);
        }
        assert_eq!(
            serde_json::to_value(&from_memory).unwrap(),
            serde_json::to_value(&from_disk).unwrap()
        );

        // The inline mesh's bytes never touched `memory` — `load` only
        // returns them, placing them is the caller's job — so they are
        // added here exactly as `open_mjcf` would.
        let mut memory = memory;
        for (name, bytes) in memory_inline {
            memory.insert(root.join(name), bytes);
        }
        let (store, errors) = MeshStore::load(&from_memory, &memory);
        assert_eq!(errors, Vec::new());
        assert_eq!(store.0.len(), from_memory.referenced_assets().len());
    }

    /// A mesh missing from the set is an `UnloadableMesh` that names it,
    /// not a panic and not a silent hole in the store.
    #[test]
    fn mesh_store_reports_what_the_set_does_not_carry() {
        let root = Path::new("/dropped");
        let memory = dropped(root, &[("arm.riggen", "arm/arm.riggen")]);
        let text = String::from_utf8(memory.read(&root.join("arm.riggen")).unwrap()).unwrap();
        let (robot, _) = riggen_core::load_from(&text, &root.join("arm.riggen"), &memory).unwrap();
        let (store, errors) = MeshStore::load(&robot, &memory);
        assert!(store.0.is_empty());
        assert_eq!(errors.len(), 4, "{errors:?}");
    }
}
