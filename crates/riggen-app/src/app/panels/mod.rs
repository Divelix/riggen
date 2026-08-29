//! The egui panels around the viewport. Each draws from the document and
//! turns what the user did into commands after drawing; none of them holds
//! document state of its own, only transient UI state (an inline rename in
//! progress).

mod tree;

pub(crate) use tree::TreeState;
