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
pub mod inertial;
pub mod pose;
pub mod robot;
pub mod validate;

pub use command::{Command, Created, EditError};
pub use file::{
    Disk, FileError, FileSource, MemorySource, Warning, absolute, content_hash, hash_file, load,
    load_from, save, to_json,
};
pub use fk::{JointState, fk, frames, motion, origin_for_world, resolve_q};
pub use history::{GestureId, History};
pub use ids::{FrameId, GeomId, Id, IdGen, JointId, LinkId, MeshId};
pub use inertial::{Inertial, InertialError, LinkInertial, MeshLookup, compose_inertial};
pub use pose::Pose;
pub use robot::{
    ActuatorSpec, CollisionPolicy, Dynamics, Frame, Geom, InertialSpec, Joint, JointKind, Limits,
    Link, Material, MeshAsset, Mimic, Primitive, Robot,
};
pub use validate::{ValidationError, validate, validation_errors};

/// Re-export so downstream crates spell the math library the same way
/// `riggen-mesh` does (ADR-0001).
pub use riggen_mesh::glam;
