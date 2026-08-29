//! World-space primitives drawn over the rendered scene with egui's painter
//! (docs/01-architecture.md §Layer map).
//!
//! The viewport owns the projection, so it owns the overlay: everything
//! drawn on top of the scene — joint glyphs, snap markers, readouts —
//! arrives as a list of points in **world** coordinates and is projected
//! here, through the same `camera.view_proj` the wgpu pass rasterized with.
//! An overlay therefore cannot disagree with the geometry about where a
//! point is.
//!
//! The viewport never sees a `Joint`: the app builds the items, the viewport
//! draws them (`riggen-app/src/app/glyphs.rs`).
//!
//! **Not depth-tested.** egui's painter has no depth buffer, so an overlay
//! is always on top. For a glyph that is the wanted behaviour — a joint
//! inside a part still has to be reachable — and a depth-tested overlay is
//! a backlog item, not an oversight.

use riggen_mesh::glam::DVec3;

/// One primitive, in world coordinates.
#[derive(Debug, Clone, PartialEq)]
pub enum OverlayItem {
    Segment {
        from: DVec3,
        to: DVec3,
        color: egui::Color32,
        width: f32,
    },
    /// An open polyline; two points are a segment.
    Polyline {
        points: Vec<DVec3>,
        color: egui::Color32,
        width: f32,
    },
    /// A circular arc of `sweep` radians about `axis`, starting at
    /// `center + start * radius` and turning right-handed about `axis`.
    /// Tessellated here so no caller repeats the trigonometry.
    Arc {
        center: DVec3,
        /// Unit; the arc lies in the plane perpendicular to it.
        axis: DVec3,
        /// Unit, perpendicular to `axis`: where the arc begins.
        start: DVec3,
        radius: f64,
        sweep: f64,
        color: egui::Color32,
        width: f32,
    },
    /// A filled dot of `radius` **screen** points.
    Point {
        at: DVec3,
        radius: f32,
        color: egui::Color32,
    },
    /// Text anchored at a world point, offset by `offset` screen points.
    Label {
        at: DVec3,
        text: String,
        color: egui::Color32,
        offset: egui::Vec2,
    },
}

/// Everything drawn over the scene this frame, in draw order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Overlay {
    pub items: Vec<OverlayItem>,
}

/// Points per tessellated arc segment: fine enough that a limit arc reads as
/// a curve at any size a glyph is drawn at, cheap enough to rebuild every
/// frame.
const ARC_STEP: f64 = std::f64::consts::PI / 32.0;

impl Overlay {
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    pub fn push(&mut self, item: OverlayItem) {
        self.items.push(item);
    }

    pub fn segment(&mut self, from: DVec3, to: DVec3, color: egui::Color32, width: f32) {
        self.push(OverlayItem::Segment {
            from,
            to,
            color,
            width,
        });
    }

    pub fn point(&mut self, at: DVec3, radius: f32, color: egui::Color32) {
        self.push(OverlayItem::Point { at, radius, color });
    }

    pub fn label(
        &mut self,
        at: DVec3,
        text: impl Into<String>,
        color: egui::Color32,
        offset: egui::Vec2,
    ) {
        self.push(OverlayItem::Label {
            at,
            text: text.into(),
            color,
            offset,
        });
    }
}

impl OverlayItem {
    /// The world points of an [`OverlayItem::Arc`], including both ends.
    pub fn arc_points(
        center: DVec3,
        axis: DVec3,
        start: DVec3,
        radius: f64,
        sweep: f64,
    ) -> Vec<DVec3> {
        let steps = ((sweep.abs() / ARC_STEP).ceil() as usize).max(1);
        let axis = axis.normalize_or_zero();
        let start = start.normalize_or_zero();
        (0..=steps)
            .map(|i| {
                let angle = sweep * i as f64 / steps as f64;
                let dir = riggen_mesh::glam::DQuat::from_axis_angle(axis, angle) * start;
                center + dir * radius
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn an_arc_starts_and_ends_where_it_says() {
        let points = OverlayItem::arc_points(DVec3::Z, DVec3::Z, DVec3::X, 2.0, FRAC_PI_2);
        assert!(points.len() > 8, "tessellated: {}", points.len());
        assert!((points[0] - DVec3::new(2.0, 0.0, 1.0)).length() < 1e-12);
        assert!((points[points.len() - 1] - DVec3::new(0.0, 2.0, 1.0)).length() < 1e-12);
        // Every point is on the circle.
        for p in &points {
            assert!(((*p - DVec3::Z).length() - 2.0).abs() < 1e-12, "{p}");
        }
    }

    #[test]
    fn a_negative_sweep_turns_the_other_way_and_a_zero_one_is_a_point() {
        let back = OverlayItem::arc_points(DVec3::ZERO, DVec3::Z, DVec3::X, 1.0, -FRAC_PI_2);
        assert!((back[back.len() - 1] - DVec3::NEG_Y).length() < 1e-12);
        let none = OverlayItem::arc_points(DVec3::ZERO, DVec3::Z, DVec3::X, 1.0, 0.0);
        assert_eq!(none.len(), 2);
        assert_eq!(none[0], none[1]);
        // A half turn is still one arc, not two.
        let half = OverlayItem::arc_points(DVec3::ZERO, DVec3::Z, DVec3::X, 1.0, PI);
        assert!((half[half.len() - 1] - DVec3::NEG_X).length() < 1e-12);
    }
}
