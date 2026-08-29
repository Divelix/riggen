//! The Align tool: two clicks put a part exported out of place *in* place
//! (docs/01-architecture.md §Panels and menus).
//!
//! First click a feature on the **selected link**, second click a feature
//! anywhere. Two circles are made **concentric** — the minimal rotation
//! that takes the first axis onto the second, then the first centre onto
//! the second — and anything else is a plain **point → point** translation.
//! The result is one `SetJoint` on the link's parent joint, through
//! `fk::origin_for_world`: the link and its whole subtree move, and the
//! gesture costs exactly one history entry.
//!
//! This is the mouse-only route to a part that came out of CAD at the
//! wrong origin. Snapping *during* a gizmo drag would be the other one; it
//! is a backlog line, not part of M2.

use riggen_core::glam::{DQuat, DVec3};
use riggen_core::{Command, LinkId, Pose, origin_for_world};
use riggen_viewport::Overlay;

use super::{RiggenApp, Selection, SnapCandidate};

/// The pending first pick's marker: magenta, so it is not confused with the
/// cyan of the live snap under the cursor.
const SOURCE_COLOR: egui::Color32 = egui::Color32::from_rgb(236, 122, 236);

/// What the status bar asks for after the first click.
pub const ALIGN_PROMPT: &str = "align: now pick what to bring it onto";

/// What it says when the first click landed on the wrong part.
pub const ALIGN_WRONG_LINK: &str = "align: pick a feature on the selected link first";

/// The rigid transform that brings `source` onto `target`.
///
/// Two circles: the minimal rotation taking the source axis onto the target
/// axis — the target axis is flipped first when that is the shorter way
/// round, since a circle's axis has no preferred direction — about the
/// source centre, then the centres together. Anything else: the translation
/// that takes one point to the other, because a vertex or a face point says
/// nothing about orientation.
pub fn align_transform(source: &SnapCandidate, target: &SnapCandidate) -> Pose {
    match (source.circle, target.circle) {
        (Some(a), Some(b)) => {
            let from = a.axis.normalize_or_zero();
            let mut to = b.axis.normalize_or_zero();
            if from == DVec3::ZERO || to == DVec3::ZERO {
                return Pose::from_translation(b.center - a.center);
            }
            if from.dot(to) < 0.0 {
                to = -to;
            }
            let rotation = DQuat::from_rotation_arc(from, to);
            Pose::new(b.center - rotation * a.center, rotation)
        }
        _ => Pose::from_translation(target.point - source.point),
    }
}

/// What the status bar says once the second click has landed.
pub fn aligned_status(link: &str, target: &SnapCandidate) -> String {
    format!("aligned {link} to {}", target.readout())
}

impl RiggenApp {
    /// The first pick of an align gesture, if one is waiting for its second.
    pub fn align_source(&self) -> Option<SnapCandidate> {
        self.align_source
    }

    /// Forgets a half-finished align (a tool change, `Esc`, a new
    /// selection).
    pub(crate) fn cancel_align(&mut self) {
        self.align_source = None;
    }

    /// One click of the Align tool.
    pub(crate) fn align_click(&mut self, snap: &SnapCandidate) {
        let Selection::Link(link) = self.selection else {
            return;
        };
        match self.align_source.take() {
            None => {
                // The first pick has to be *on the part being moved*: it is
                // the thing the second pick brings somewhere.
                if snap.link != link {
                    self.status = Some(ALIGN_WRONG_LINK.to_owned());
                    return;
                }
                self.align_source = Some(*snap);
                self.status = Some(ALIGN_PROMPT.to_owned());
            }
            Some(source) => self.commit_align(link, &source, snap),
        }
    }

    /// The one `SetJoint` the gesture is worth.
    fn commit_align(&mut self, link: LinkId, source: &SnapCandidate, target: &SnapCandidate) {
        let Some(joint_id) = self.robot.parent_joint(link) else {
            return;
        };
        let world = riggen_core::fk(&self.robot, &self.q);
        let Some(current) = world.get(&link).copied() else {
            return;
        };
        let moved = align_transform(source, target).compose(&current);
        let Some(origin) = origin_for_world(&self.robot, link, moved) else {
            return;
        };
        let mut joint = self.robot.joints[&joint_id].clone();
        joint.origin = origin;
        if self.apply(Command::SetJoint(joint_id, joint)).is_ok() {
            let name = self.robot.links[&link].name.clone();
            self.status = Some(aligned_status(&name, target));
        }
    }

    /// The pending first pick, so a half-finished gesture is visible rather
    /// than remembered.
    pub(crate) fn push_align_overlay(&self, overlay: &mut Overlay) {
        let Some(source) = self.align_source else {
            return;
        };
        overlay.point(source.point, 6.0, SOURCE_COLOR);
        overlay.label(
            source.point,
            format!("align from {}", source.readout()),
            SOURCE_COLOR,
            egui::vec2(10.0, 8.0),
        );
        if let Some(fit) = source.circle {
            overlay.push(riggen_viewport::OverlayItem::Arc {
                center: fit.center,
                axis: fit.axis,
                start: fit.axis.any_orthonormal_vector(),
                radius: fit.radius,
                sweep: std::f64::consts::TAU,
                color: SOURCE_COLOR,
                width: 2.0,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::SnapKind;
    use riggen_core::Id;
    use riggen_mesh::feature::CircleFit;
    use riggen_viewport::InstanceId;

    fn candidate(point: DVec3, circle: Option<CircleFit>) -> SnapCandidate {
        SnapCandidate {
            kind: if circle.is_some() {
                SnapKind::Circle
            } else {
                SnapKind::Point
            },
            point: circle.map_or(point, |c| c.center),
            normal: DVec3::Z,
            circle,
            hit: point,
            link: LinkId::from_raw(1),
            instance: InstanceId(0),
            triangle: 0,
        }
    }

    fn circle(center: DVec3, axis: DVec3) -> CircleFit {
        CircleFit {
            center,
            axis: axis.normalize(),
            radius: 0.012,
            residual: 0.0,
            segments: 24,
        }
    }

    #[test]
    fn two_circles_become_concentric() {
        let source = candidate(
            DVec3::ZERO,
            Some(circle(DVec3::new(1.0, 2.0, 3.0), DVec3::Y)),
        );
        let target = candidate(
            DVec3::ZERO,
            Some(circle(DVec3::new(-1.0, 0.5, 0.0), DVec3::Z)),
        );
        let t = align_transform(&source, &target);

        // The centre lands on the target's…
        let moved_center = t.transform_point(DVec3::new(1.0, 2.0, 3.0));
        assert!((moved_center - DVec3::new(-1.0, 0.5, 0.0)).length() < 1e-12);
        // …and the axis with it.
        assert!((t.r * DVec3::Y - DVec3::Z).length() < 1e-12);
    }

    #[test]
    fn the_shorter_way_round_wins() {
        // A target axis pointing the other way is the same circle: the
        // rotation taken is the small one, not the 173° one.
        let source = candidate(DVec3::ZERO, Some(circle(DVec3::ZERO, DVec3::Z)));
        let target = candidate(
            DVec3::ZERO,
            Some(circle(DVec3::ZERO, DVec3::new(0.1, 0.0, -1.0))),
        );
        let t = align_transform(&source, &target);
        let turned = t.r * DVec3::Z;
        assert!(
            turned.dot(DVec3::Z) > 0.9,
            "turned {turned} — took the long way"
        );
        assert!((t.r.angle_between(DQuat::IDENTITY)) < 0.2, "{}", t.r);
    }

    #[test]
    fn anything_that_is_not_two_circles_only_translates() {
        let source = candidate(DVec3::new(1.0, 1.0, 1.0), None);
        let target = candidate(DVec3::new(4.0, -1.0, 0.5), None);
        let t = align_transform(&source, &target);
        assert_eq!(t.r, DQuat::IDENTITY);
        assert!((t.t - DVec3::new(3.0, -2.0, -0.5)).length() < 1e-12);

        // One circle and one point is still a translation: a point has no
        // axis to rotate onto.
        let half = candidate(DVec3::ZERO, Some(circle(DVec3::X, DVec3::Y)));
        let t = align_transform(&half, &target);
        assert_eq!(t.r, DQuat::IDENTITY);
        assert!((t.transform_point(DVec3::X) - target.point).length() < 1e-12);
    }

    #[test]
    fn a_degenerate_axis_falls_back_to_a_translation() {
        let source = candidate(DVec3::ZERO, Some(circle(DVec3::ZERO, DVec3::Z)));
        let mut broken = circle(DVec3::X, DVec3::Z);
        broken.axis = DVec3::ZERO;
        let target = candidate(DVec3::ZERO, Some(broken));
        let t = align_transform(&source, &target);
        assert_eq!(t.r, DQuat::IDENTITY);
        assert!((t.t - DVec3::X).length() < 1e-12);
    }
}
