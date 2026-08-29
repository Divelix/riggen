//! Joint glyphs: the document turned into overlay primitives
//! (docs/01-architecture.md §Panels and menus).
//!
//! A joint has no geometry, so without a glyph it is invisible in the
//! viewport — the tree is the only place it exists, and "which way does this
//! hinge turn?" has to be read off two number fields. The glyph answers it
//! in the picture: an **axis segment** through the pivot, an **origin triad**
//! in the axes triad's colours, and a **limit arc** (revolute) or **limit
//! segment** (prismatic) with a tick at the current `q`.
//!
//! Drawn for every movable joint plus the selected one, whatever its kind
//! (plans/m2-placement-ux OPEN 4): an unselected `Fixed` joint has nothing
//! to show and every weld in a big assembly would be noise.
//!
//! The anchor is the **pivot** — `world(parent) ∘ origin` — not the child
//! link frame, which for a prismatic joint has already slid away by `q`.

use riggen_core::glam::{DQuat, DVec3};
use riggen_core::{JointId, JointKind, LinkId, Pose};
use riggen_viewport::{Overlay, OverlayItem};

use super::{RiggenApp, Selection};

/// Colour of the axis segment and the limit arc: amber, which nothing in
/// the scene or the triad already means.
const AXIS_COLOR: egui::Color32 = egui::Color32::from_rgb(255, 183, 77);
/// The same, for the joint the user is pointing at or has selected.
const AXIS_COLOR_ACTIVE: egui::Color32 = egui::Color32::from_rgb(255, 236, 179);
/// The axes triad's colours (`gpu_mesh::AxesTriadMesh`), so a frame reads
/// the same in the corner and on a joint.
const TRIAD_COLORS: [egui::Color32; 3] = [
    egui::Color32::from_rgb(230, 64, 64),
    egui::Color32::from_rgb(89, 217, 89),
    egui::Color32::from_rgb(77, 140, 242),
];
/// The tick at the current `q`.
const TICK_COLOR: egui::Color32 = egui::Color32::from_rgb(245, 245, 245);

/// Fractions of the glyph's size (§`glyph_size`).
const AXIS_HALF_LENGTH: f64 = 1.15;
const TRIAD_LENGTH: f64 = 0.4;
const ARC_RADIUS: f64 = 0.6;
/// How far past the arc the current-`q` tick sticks out.
const TICK_OVERSHOOT: f64 = 1.25;

/// One joint's glyph, already placed in the world: what the overlay draws
/// and what a hover hit-test measures against.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointGlyph {
    pub joint: JointId,
    /// The pivot: `world(parent) ∘ origin`.
    pub pivot: Pose,
    /// Unit, in world coordinates.
    pub axis: DVec3,
    /// Half-length of the axis segment; every other measure is a fraction
    /// of the same size.
    pub size: f64,
    pub kind: JointKind,
    pub q: f64,
    pub limits: Option<(f64, f64)>,
}

impl JointGlyph {
    /// The ends of the axis segment — the line a hover hit-test measures
    /// the cursor's distance to.
    pub fn axis_ends(&self) -> (DVec3, DVec3) {
        let half = self.axis * self.size * AXIS_HALF_LENGTH;
        (self.pivot.t - half, self.pivot.t + half)
    }

    /// A unit direction perpendicular to the axis, in world coordinates:
    /// where a limit arc begins measuring from. Derived from the pivot's
    /// own frame so it turns with the joint instead of flipping when the
    /// camera moves.
    fn reference(&self) -> DVec3 {
        let local = DVec3::new(0.0, 0.0, 1.0);
        let axis_local = self.pivot.r.inverse() * self.axis;
        let reference = if axis_local.cross(local).length_squared() < 1e-12 {
            DVec3::X
        } else {
            local
        };
        let perpendicular = (reference - axis_local * reference.dot(axis_local)).normalize();
        (self.pivot.r * perpendicular).normalize()
    }
}

impl RiggenApp {
    /// Every glyph the viewport should draw this frame.
    pub fn joint_glyphs(&self) -> Vec<JointGlyph> {
        let world = riggen_core::fk(&self.robot, &self.q);
        let selected = match self.selection {
            Selection::Joint(j) => Some(j),
            _ => None,
        };
        self.robot
            .joints
            .iter()
            .filter(|(id, joint)| joint.kind.is_movable() || selected == Some(**id))
            .filter_map(|(&id, joint)| {
                let pivot = world.get(&joint.parent)?.compose(&joint.origin);
                let axis = (pivot.r * joint.axis).normalize_or_zero();
                if axis == DVec3::ZERO {
                    return None; // validate refuses these; draw nothing rather than NaN
                }
                Some(JointGlyph {
                    joint: id,
                    pivot,
                    axis,
                    size: self.glyph_size(joint.child),
                    kind: joint.kind,
                    q: self.q.get(id),
                    limits: joint.limits.map(|l| (l.lower, l.upper)),
                })
            })
            .collect()
    }

    /// How big a glyph on `child` is: the half-diagonal of the child link's
    /// world bounds, so a glyph is the size of the part it belongs to.
    /// A link with no geometry yet falls back to the scene radius, and an
    /// empty scene to one metre.
    fn glyph_size(&self, child: LinkId) -> f64 {
        let own = self
            .instances
            .iter()
            .filter(|((link, _), _)| *link == child)
            .filter_map(|(_, id)| {
                let state = self.viewport.instance_states().find(|s| s.id == *id)?;
                Some(state.bounds?.transformed(&state.model))
            })
            .reduce(|a, b| a.union(&b));
        if let Some(bounds) = own
            && bounds.half_diagonal() > 1e-9
        {
            return bounds.half_diagonal();
        }
        self.viewport
            .scene_bounds()
            .map(|(_, radius)| radius)
            .filter(|r| *r > 1e-9)
            .unwrap_or(1.0)
    }

    /// The glyphs as overlay primitives. `active` is the joint the user is
    /// pointing at or has selected, drawn brighter and thicker.
    pub(crate) fn glyph_overlay(&self, glyphs: &[JointGlyph], active: Option<JointId>) -> Overlay {
        let mut overlay = Overlay::default();
        for glyph in glyphs {
            let hot = active == Some(glyph.joint);
            let color = if hot { AXIS_COLOR_ACTIVE } else { AXIS_COLOR };
            let width = if hot { 3.0 } else { 1.5 };

            let (from, to) = glyph.axis_ends();
            overlay.segment(from, to, color, width);
            overlay.point(glyph.pivot.t, if hot { 5.0 } else { 3.5 }, color);

            // The pivot's own frame, in the triad's colours.
            for (i, local) in [DVec3::X, DVec3::Y, DVec3::Z].into_iter().enumerate() {
                overlay.segment(
                    glyph.pivot.t,
                    glyph.pivot.t + glyph.pivot.r * local * glyph.size * TRIAD_LENGTH,
                    TRIAD_COLORS[i],
                    width,
                );
            }

            match glyph.kind {
                JointKind::Revolute | JointKind::Continuous => {
                    self.push_arc(&mut overlay, glyph, color, width)
                }
                JointKind::Prismatic => self.push_slide(&mut overlay, glyph, color, width),
                JointKind::Fixed => {}
            }
        }
        overlay
    }

    /// The joint a glyph is drawn hot for: the selected one for now, and
    /// from step 6 the hovered one too.
    pub fn active_joint(&self) -> Option<JointId> {
        match self.selection {
            Selection::Joint(j) => Some(j),
            _ => None,
        }
    }

    /// The swept range of a revolute joint, with a tick at the current `q`.
    /// A `Continuous` joint has no limits and gets the full circle.
    fn push_arc(
        &self,
        overlay: &mut Overlay,
        glyph: &JointGlyph,
        color: egui::Color32,
        width: f32,
    ) {
        let radius = glyph.size * ARC_RADIUS;
        let reference = glyph.reference();
        let (lower, sweep) = match glyph.limits {
            Some((lower, upper)) => (lower, upper - lower),
            None => (0.0, std::f64::consts::TAU),
        };
        let start = DQuat::from_axis_angle(glyph.axis, lower) * reference;
        overlay.push(OverlayItem::Arc {
            center: glyph.pivot.t,
            axis: glyph.axis,
            start,
            radius,
            sweep,
            color,
            width,
        });
        // The tick: a spoke from the pivot through the arc at the current
        // angle, so "where is this joint now" is one glance.
        let at = DQuat::from_axis_angle(glyph.axis, glyph.q) * reference;
        overlay.segment(
            glyph.pivot.t,
            glyph.pivot.t + at * radius * TICK_OVERSHOOT,
            TICK_COLOR,
            width,
        );
    }

    /// The travel of a prismatic joint along its axis, with a tick at `q`.
    fn push_slide(
        &self,
        overlay: &mut Overlay,
        glyph: &JointGlyph,
        color: egui::Color32,
        width: f32,
    ) {
        let (lower, upper) = glyph.limits.unwrap_or((0.0, 0.0));
        let reference = glyph.reference() * glyph.size * ARC_RADIUS * 0.5;
        // Offset off the axis line so the travel is readable beside it
        // rather than drawn on top of the axis segment.
        let base = glyph.pivot.t + reference;
        overlay.segment(
            base + glyph.axis * lower,
            base + glyph.axis * upper,
            color,
            width,
        );
        for end in [lower, upper] {
            let at = base + glyph.axis * end;
            overlay.segment(at - reference * 0.4, at + reference * 0.4, color, width);
        }
        let at = base + glyph.axis * glyph.q;
        overlay.segment(
            at - reference * 0.7,
            at + reference * 0.7,
            TICK_COLOR,
            width,
        );
    }
}
