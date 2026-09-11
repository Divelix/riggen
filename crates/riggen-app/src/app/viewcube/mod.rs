//! The ViewCube: the chamfered cube's geometry, its projection, and the
//! widget that paints it in the viewport's bottom-right corner.
//!
//! App-side on purpose (ADR-0028 §3). The cube is interactive chrome that
//! *writes* the camera — it registers a rect in `chrome_rects` like every
//! other corner widget and reads the pointer — where the viewport owns the
//! camera itself. It is painter-drawn rather than a wgpu pass: picking a
//! facet needs a screen-space hit test regardless, and a third pass is one
//! more thing a later MSAA change would have to be taught.
//!
//! Ported from RoboCAD's `robocad-ui/src/viewcube/` (ADR-0001 names it the
//! ancestor). The port is cgmath to glam and `robocad_viewport::{Projection,
//! ViewOrientation}` to `riggen_viewport`'s — the same enums with the same
//! variants — plus RoboCAD's `is_sketch_mode` parameter dropped, riggen
//! having no sketch mode to dim for.

pub mod facets;
pub mod projection;
pub mod widget;

#[cfg(test)]
mod tests;

pub use projection::project_viewcube;
pub use widget::{ViewCubeAction, projection_button_rect, viewcube};
