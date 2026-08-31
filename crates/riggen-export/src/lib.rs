//! MJCF, URDF and SDF export, and MJCF and URDF import
//! (docs/02-data-model.md §`ResolvedRobot`, §Format mapping, §URDF import,
//! §MJCF import; ADR-0004, ADR-0008, ADR-0016).
//! Never depends on egui or wgpu (docs/01-architecture.md §Crates).
//!
//! The writers never see `Robot`: [`resolve`] turns the document plus its
//! loaded meshes into a pure-numeric, convention-fixed [`ResolvedRobot`],
//! and each writer is a dumb serialiser of that.

mod export;
pub mod fk_samples;
pub mod import;
pub mod mesh_store;
pub mod mjcf;
pub mod mjcf_in;
pub mod resolve;
pub mod sdf;
#[cfg(test)]
pub(crate) mod test_util;
pub mod urdf;
pub mod urdf_in;
pub mod xml;

pub use export::{ExportIoError, export};
pub use import::{ImportError, ImportWarning};
pub use mesh_store::MeshStore;
pub use resolve::{
    ComputeNow, DecompMiss, DecompSource, ExportError, ExportOptions, Format, MeshPathStyle,
    ResolvedGeom, ResolvedJoint, ResolvedLink, ResolvedMimic, ResolvedRobot, ResolvedSite, resolve,
};
pub use urdf_in::PackageMap;
