//! The egui panels around the viewport. Each draws from the document and
//! turns what the user did into commands after drawing; none of them holds
//! document state of its own, only transient UI state (an inline rename in
//! progress).

mod joints;
mod materials;
mod properties;
mod tree;

pub(crate) use joints::JointsWindow;
pub(crate) use materials::MaterialsWindow;
pub(crate) use properties::PropertiesState;
pub use properties::{DECOMP_CONSENT_BUTTON, DECOMP_FREEZE_WARNING, fmt_num};
pub(crate) use tree::{RenameTarget, TreeState};
