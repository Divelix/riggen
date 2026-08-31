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
use std::path::PathBuf;

use riggen_core::ValidationError;

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
    /// field for: `<tendon>`, `<sensor>`, `<contact>`, `<joint ref>`. One
    /// warning per name with a count, not one per occurrence (ADR-0015 §1).
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
    /// An actuator that is not one of the three presets, or that does not
    /// drive a joint (ADR-0014, ADR-0015 §1).
    ActuatorDropped {
        actuator: String,
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
                write!(f, "link \"{link}\": a {kind} geom was dropped")
            }
            Self::FreeJointDropped { body } => write!(
                f,
                "body \"{body}\": <freejoint> dropped; export with a floating base to get it back"
            ),
            Self::ActuatorDropped { actuator, reason } => {
                write!(f, "actuator \"{actuator}\" dropped, {reason}")
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
    /// link tree (ADR-0005, ADR-0015 §5).
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
    /// An element that composes files or re-shapes the tree —
    /// `<include>`, `<replicate>`, `<attach>`, `<frame>` — or a
    /// `<compiler coordinate="global">`.
    UnsupportedElement {
        element: String,
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
                "body \"{body}\" has {} joints ({}); the link tree holds one per body",
                joints.len(),
                joints.join(", ")
            ),
            Self::JointOnRoot { body, joint } => write!(
                f,
                "root body \"{body}\" carries joint \"{joint}\"; the root link has no parent joint"
            ),
            Self::UnsupportedElement { element } => {
                write!(f, "{element} is not supported")
            }
            Self::Invalid(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for ImportError {}
