//! The egui panels around the viewport. Each draws from the document and
//! turns what the user did into commands after drawing; none of them holds
//! document state of its own, only transient UI state (an inline rename in
//! progress).

mod joint_tree;
mod materials;
mod properties;
mod tree;

pub(crate) use joint_tree::JointTreeState;
pub use joint_tree::NOTHING_TO_POSE;
pub(crate) use materials::MaterialsWindow;
pub use properties::{DECOMP_CONSENT_BUTTON, DECOMP_FREEZE_WARNING, fmt_num};
pub(crate) use properties::{PropertiesState, STEP_M};
pub(crate) use tree::{RenameTarget, TreeState};
