//! `riggen._riggen`, the extension module behind the Python SDK
//! (docs/01-architecture.md §Python SDK, ADR-0009). A thin, typed layer over
//! `riggen-core` and `riggen-export`: one method per `Command`, no sugar —
//! the public API lives in `python/riggen/`. Never depends on egui or wgpu.
//!
//! The wheel carries this module beside the `riggen` binary
//! (`pyproject.toml`, `python/build_wheel.py`); stubs in
//! `python/riggen/_riggen.pyi` mirror everything exposed here.

use pyo3::prelude::*;

/// The module. `__version__` is `CARGO_PKG_VERSION` — the workspace
/// version, the same number `importlib.metadata.version("riggen")` reports.
#[pymodule]
fn _riggen(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
