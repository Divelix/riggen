//! MJCF and URDF export, and URDF import (docs/02-data-model.md
//! §`ResolvedRobot`, §Format mapping, §URDF import; ADR-0004, ADR-0008).
//! Never depends on egui or wgpu (docs/01-architecture.md §Crates).
//!
//! The writers never see `Robot`: [`resolve`] turns the document plus its
//! loaded meshes into a pure-numeric, convention-fixed [`ResolvedRobot`],
//! and each writer is a dumb serialiser of that.

mod export;
pub mod fk_samples;
pub mod mesh_store;
pub mod mjcf;
pub mod resolve;
#[cfg(test)]
pub(crate) mod test_util;
pub mod urdf;
pub mod xml;

pub use export::{ExportIoError, export};
pub use mesh_store::MeshStore;
pub use resolve::{
    ExportError, ExportOptions, Format, MeshPathStyle, ResolvedGeom, ResolvedJoint, ResolvedLink,
    ResolvedRobot, resolve,
};
