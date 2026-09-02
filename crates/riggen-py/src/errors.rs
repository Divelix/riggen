//! Rust errors → the exception classes of `python/riggen/errors.py`. The
//! classes live in Python (docstrings, `pyright`, a plain `except
//! riggen.EditError`); this side only looks them up by name and raises
//! them with the Rust message.

use pyo3::prelude::*;
use riggen_core::EditError;

/// Raises `riggen.errors.<class>(message)`. If the package cannot be
/// imported — the module loaded outside its wheel — that error is raised
/// instead, so nothing is silently swallowed.
pub fn raise(py: Python<'_>, class: &str, message: impl Into<String>) -> PyErr {
    let message: String = message.into();
    let exc = py
        .import("riggen.errors")
        .and_then(|m| m.getattr(class))
        .and_then(|c| c.call1((message,)));
    match exc {
        Ok(exc) => PyErr::from_value(exc),
        Err(e) => e,
    }
}

/// Every `EditError` variant is its own subclass of `riggen.EditError`
/// (docs/01-architecture.md §Python SDK).
pub fn edit_error(py: Python<'_>, e: EditError) -> PyErr {
    let class = match &e {
        EditError::Invalid(_) => "InvalidDocument",
        EditError::UnknownId { .. } => "UnknownId",
        EditError::UnknownMaterial(_) => "UnknownMaterial",
        EditError::WouldCreateCycle { .. } => "WouldCreateCycle",
        EditError::CannotRemoveRoot => "CannotRemoveRoot",
        EditError::CannotReparentRoot => "CannotReparentRoot",
        EditError::MaterialInUse { .. } => "MaterialInUse",
        EditError::MaterialExists(_) => "MaterialExists",
        EditError::MovableJointOnRootPath(_) => "MovableJointOnRootPath",
    };
    raise(py, class, e.to_string())
}
