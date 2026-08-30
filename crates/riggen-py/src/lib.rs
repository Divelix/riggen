//! `riggen._riggen`, the extension module behind the Python SDK
//! (docs/01-architecture.md §Python SDK, ADR-0009). A thin, typed layer over
//! `riggen-core` and `riggen-export`: one method per `Command`, no sugar —
//! the public API lives in `python/riggen/`. Never depends on egui or wgpu.
//!
//! The wheel carries this module beside the `riggen` binary
//! (`pyproject.toml`, `python/build_wheel.py`); stubs in
//! `python/riggen/_riggen.pyi` mirror everything exposed here.

use pyo3::prelude::*;

/// The module. `__version__` is `CARGO_PKG_VERSION` in PEP 440 spelling —
/// the workspace version, the same string `importlib.metadata.version
/// ("riggen")` reports (`test_wheel.py` checks the two agree).
#[pymodule]
fn _riggen(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", pep440(env!("CARGO_PKG_VERSION")))?;
    Ok(())
}

/// Cargo's pre-release spelling to PEP 440's, the way maturin writes the
/// wheel's version: `0.2.0-dev` → `0.2.0.dev0`, `-alpha.1` → `a1`,
/// `-beta.2` → `b2`, `-rc.1` → `rc1`. A release version passes through; an
/// unknown tag is left as Cargo spelled it.
fn pep440(cargo: &str) -> String {
    let Some((base, pre)) = cargo.split_once('-') else {
        return cargo.to_string();
    };
    let (tag, n) = pre.split_once('.').unwrap_or((pre, "0"));
    let tag = match tag {
        "dev" => ".dev",
        "alpha" | "a" => "a",
        "beta" | "b" => "b",
        "rc" | "pre" => "rc",
        _ => return cargo.to_string(),
    };
    format!("{base}{tag}{n}")
}
