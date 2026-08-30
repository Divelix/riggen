//! `riggen._riggen`, the extension module behind the Python SDK
//! (docs/01-architecture.md §Python SDK, ADR-0009). A thin, typed layer over
//! `riggen-core` and `riggen-export`: one method per `Command`, no sugar —
//! the public API lives in `python/riggen/`. Never depends on egui or wgpu.
//!
//! Values cross the boundary in the document's own serde shape — the v1
//! schema of docs/02-data-model.md §Schema, with ids as ints ([`doc`]) —
//! so the mapping table is the schema. Errors are the exception classes of
//! `python/riggen/errors.py`, raised by name ([`errors`]).
//!
//! The wheel carries this module beside the `riggen` binary
//! (`pyproject.toml`, `python/build_wheel.py`); stubs in
//! `python/riggen/_riggen.pyi` mirror everything exposed here.

mod doc;
mod errors;
mod robot;

use pyo3::prelude::*;
use riggen_core::Pose;
use riggen_core::glam::{DQuat, DVec3};

/// `(roll, pitch, yaw)` in radians → the quaternion `[x, y, z, w]` of
/// `Pose::from_xyz_rpy` (URDF's `Rz·Ry·Rx`), so Python never re-derives
/// the convention.
#[pyfunction]
fn rpy_to_quat(rpy: [f64; 3]) -> [f64; 4] {
    Pose::from_xyz_rpy(DVec3::ZERO, DVec3::from_array(rpy))
        .r
        .to_array()
}

/// The inverse, `Pose::to_xyz_rpy`: pitch in `[-π/2, π/2]`, roll folded
/// into yaw at gimbal lock.
#[pyfunction]
fn quat_to_rpy(quat: [f64; 4]) -> [f64; 3] {
    Pose::from_rotation(DQuat::from_array(quat))
        .to_xyz_rpy()
        .1
        .to_array()
}

/// The module. `__version__` is `CARGO_PKG_VERSION` in PEP 440 spelling —
/// the workspace version, the same string `importlib.metadata.version
/// ("riggen")` reports (`test_wheel.py` checks the two agree).
#[pymodule]
fn _riggen(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", pep440(env!("CARGO_PKG_VERSION")))?;
    m.add_class::<robot::PyRobot>()?;
    m.add_function(wrap_pyfunction!(rpy_to_quat, m)?)?;
    m.add_function(wrap_pyfunction!(quat_to_rpy, m)?)?;
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
