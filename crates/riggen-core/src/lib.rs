//! The `Robot` document, commands, history and kinematics
//! (docs/02-data-model.md). Never depends on egui or wgpu
//! (docs/01-architecture.md §Crates).
//!
//! Meters, radians, right-handed, Z-up, `f64` everywhere. Ids are
//! per-document counters and joints are the edges of the link tree
//! (ADR-0005).

pub mod command;
pub mod file;
pub mod fk;
pub mod history;
pub mod ids;
pub mod pose;
pub mod robot;
pub mod validate;

pub use command::{Command, EditError};
pub use file::{FileError, Warning, absolute, content_hash, hash_file, load, save};
pub use fk::{JointState, fk, motion};
pub use history::History;
pub use ids::{FrameId, GeomId, Id, IdGen, JointId, LinkId, MeshId};
pub use pose::Pose;
pub use robot::{
    CollisionPolicy, Dynamics, Frame, Geom, InertialSpec, Joint, JointKind, Limits, Link, Material,
    MeshAsset, Primitive, Robot,
};
pub use validate::{ValidationError, validate, validation_errors};

/// Re-export so downstream crates spell the math library the same way
/// `riggen-mesh` does (ADR-0001).
pub use riggen_mesh::glam;
