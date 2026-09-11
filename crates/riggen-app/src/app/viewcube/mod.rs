//! The ViewCube: the chamfered cube's geometry, its projection, and (step 5)
//! the widget that paints it in the viewport's bottom-right corner.
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

#[allow(
    dead_code,
    reason = "the widget that calls all of this lands in step 5; until then \
              the ported tests are its only caller"
)]
pub mod facets;
#[allow(
    dead_code,
    reason = "the widget that calls all of this lands in step 5; until then \
              the ported tests are its only caller"
)]
pub mod projection;

#[cfg(test)]
mod tests;
