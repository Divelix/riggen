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
//! **Depth is per item.** egui's painter has no depth buffer, so an
//! overlay is on top by default ([`Occlusion::Always`]) and that is what
//! cursor feedback wants: a snap marker, an align pick or a readout label
//! answers "where is the pointer", and hiding it behind the part the
//! pointer is aiming at would answer nothing. A glyph asks for
//! [`Occlusion::Test`] instead: it claims to be somewhere in the scene, so
//! it has to look it (ADR-0020). A stroke is split at its depth crossings;
//! a fill ([`OverlayItem::Strip`]) is dimmed quad by quad ([`split_strip`]).

use riggen_mesh::glam::DVec3;

/// Whether an item is drawn against the scene's depth (ADR-0020).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Occlusion {
    /// Full strength wherever it lands. The default, so nothing becomes
    /// depth-tested by accident.
    #[default]
    Always,
    /// Split at depth crossings; the runs behind geometry are dimmed
    /// rather than dropped, because a glyph inside a part still has to be
    /// visible and aimable.
    Test,
}

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
    /// A filled quad strip: each pair is one **rung** (inner, outer) and
    /// consecutive rungs bound one quad. The fill carries its alpha in
    /// `color`, so a translucent band is a strip and nothing more. A
    /// two-rung strip is a bar; [`Overlay::sector`] tessellates an annulus
    /// sector into one.
    Strip {
        pairs: Vec<(DVec3, DVec3)>,
        color: egui::Color32,
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

/// One item and how it meets the scene's depth.
#[derive(Debug, Clone, PartialEq)]
pub struct OverlayEntry {
    pub item: OverlayItem,
    pub occlusion: Occlusion,
}

/// Everything drawn over the scene this frame, in draw order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Overlay {
    pub items: Vec<OverlayEntry>,
}

/// How strongly a run behind geometry is dimmed: the same colour and the
/// same width at roughly a third of the strength (ADR-0020 §5).
///
/// Dimmed, not dropped, and not thinned: a glyph inside a part still has to
/// be visible, and `glyph_at` hit-tests the whole screen-space line — a run
/// drawn narrower or not at all would be a target the user can hit but
/// cannot see.
pub const HIDDEN_STRENGTH: f32 = 0.35;

/// How far apart, in screen points, a depth-tested path is sampled.
///
/// A path's own points are far too coarse to classify against depth: an
/// axis segment is two points, and both ends of one running through a link
/// are outside it. The path is walked in world space and classified per
/// sample, so the split lands where the line actually enters the geometry.
const DEPTH_SAMPLE_SPACING: f32 = 4.0;

/// Samples one segment of a depth-tested path may be cut into. Bounds the
/// work for a segment that runs off to the horizon, or one whose ends do
/// not project at all.
const MAX_DEPTH_SAMPLES: usize = 64;

/// How many samples a segment whose ends land at `a` and `b` is cut into.
/// An end that does not project cannot be measured, so such a segment is
/// sampled at the cap — it is exactly the case where part of it is on
/// screen and subdividing is what finds the part.
pub fn depth_samples(a: Option<egui::Pos2>, b: Option<egui::Pos2>) -> usize {
    match (a, b) {
        (Some(a), Some(b)) => {
            (((a - b).length() / DEPTH_SAMPLE_SPACING).ceil() as usize).clamp(1, MAX_DEPTH_SAMPLES)
        }
        _ => MAX_DEPTH_SAMPLES,
    }
}

/// Splits a classified, projected path into runs of constant visibility.
///
/// `None` is a sample that does not project — behind the camera, outside
/// the depth range — and ends the run it is in, as an unsplit path already
/// did. A sample whose classification differs from the run it arrives in
/// **ends that run and starts the next**, so the two strokes meet at it
/// instead of leaving a gap. A run of one point is dropped: there is no
/// line to draw.
///
/// Pure, and the reason the split is testable without a GPU (ADR-0020).
pub fn split_runs(samples: &[Option<(egui::Pos2, bool)>]) -> Vec<(bool, Vec<egui::Pos2>)> {
    let mut runs: Vec<(bool, Vec<egui::Pos2>)> = Vec::new();
    let mut current: Vec<egui::Pos2> = Vec::new();
    let mut hidden = false;
    for sample in samples {
        match sample {
            None => {
                if current.len() > 1 {
                    runs.push((hidden, std::mem::take(&mut current)));
                } else {
                    current.clear();
                }
            }
            Some((pos, sample_hidden)) => {
                if current.is_empty() {
                    hidden = *sample_hidden;
                } else if *sample_hidden != hidden {
                    current.push(*pos);
                    runs.push((hidden, std::mem::take(&mut current)));
                    hidden = *sample_hidden;
                }
                current.push(*pos);
            }
        }
    }
    if current.len() > 1 {
        runs.push((hidden, current));
    }
    runs
}

/// Splits a classified, projected strip into runs of constant visibility —
/// the fill's analogue of [`split_runs`] (ADR-0020 §5, extended).
///
/// A rung is `(inner, outer, hidden)`, or `None` when it does not project.
/// Visibility belongs to a **quad**, not a rung: the quad between two rungs
/// is hidden when **both** are, so a crossing lands on a rung and the
/// visible and the dimmed fill meet at it without a gap — that rung ends
/// the run it is in and begins the next. A rung that does not project
/// ends its run without joining the next; a run of one rung bounds no
/// quad and is dropped.
///
/// "Both", not "either": a fill's rungs are a few points apart, and a
/// single hidden rung between two visible ones is a part's silhouette
/// grazing the band, not the band going in — dimming the two quads either
/// side of it would flicker as the camera turns.
#[allow(clippy::type_complexity)]
pub fn split_strip(
    rungs: &[Option<(egui::Pos2, egui::Pos2, bool)>],
) -> Vec<(bool, Vec<(egui::Pos2, egui::Pos2)>)> {
    let mut runs: Vec<(bool, Vec<(egui::Pos2, egui::Pos2)>)> = Vec::new();
    let mut current: Vec<(egui::Pos2, egui::Pos2)> = Vec::new();
    let mut current_hidden = false;
    // The previous rung's own classification, while it is in `current`.
    let mut previous: Option<bool> = None;
    for rung in rungs {
        match rung {
            None => {
                if current.len() > 1 {
                    runs.push((current_hidden, std::mem::take(&mut current)));
                } else {
                    current.clear();
                }
                previous = None;
            }
            Some((inner, outer, hidden)) => {
                let pair = (*inner, *outer);
                match previous {
                    None => current.push(pair),
                    Some(previous_hidden) => {
                        let quad_hidden = previous_hidden && *hidden;
                        if current.len() == 1 {
                            current_hidden = quad_hidden;
                        } else if quad_hidden != current_hidden {
                            let crossing = current[current.len() - 1];
                            runs.push((current_hidden, std::mem::take(&mut current)));
                            current.push(crossing);
                            current_hidden = quad_hidden;
                        }
                        current.push(pair);
                    }
                }
                previous = Some(*hidden);
            }
        }
    }
    if current.len() > 1 {
        runs.push((current_hidden, current));
    }
    runs
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
        self.items.push(OverlayEntry {
            item,
            occlusion: Occlusion::Always,
        });
    }

    /// Everything `body` pushes is [`Occlusion::Test`].
    ///
    /// A scope rather than a mode on the builder: an overlay is assembled
    /// by several callers in turn (`glyphs.rs`, `snap.rs`, `align.rs`) and
    /// a sticky flag would leak from one into the next.
    pub fn depth_tested(&mut self, body: impl FnOnce(&mut Overlay)) {
        let mut inner = Overlay::default();
        body(&mut inner);
        self.items
            .extend(inner.items.into_iter().map(|entry| OverlayEntry {
                occlusion: Occlusion::Test,
                ..entry
            }));
    }

    /// Whether anything drawn this frame needs the scene's depth — the
    /// switch that decides whether the viewport reads the depth buffer back
    /// at all.
    pub fn wants_depth(&self) -> bool {
        self.items
            .iter()
            .any(|entry| entry.occlusion == Occlusion::Test)
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

    /// A filled quad strip; see [`OverlayItem::Strip`].
    pub fn strip(&mut self, pairs: Vec<(DVec3, DVec3)>, color: egui::Color32) {
        self.push(OverlayItem::Strip { pairs, color });
    }

    /// A filled annulus sector of `sweep` radians about `axis`, between
    /// radii `inner` and `outer`, beginning at `center + start * r` and
    /// turning right-handed about `axis` — the same arc as
    /// [`OverlayItem::Arc`], with a width. Tessellated here at the arc's
    /// own step, so no caller repeats the trigonometry.
    #[allow(clippy::too_many_arguments)]
    pub fn sector(
        &mut self,
        center: DVec3,
        axis: DVec3,
        start: DVec3,
        inner: f64,
        outer: f64,
        sweep: f64,
        color: egui::Color32,
    ) {
        self.strip(
            OverlayItem::sector_rungs(center, axis, start, inner, outer, sweep),
            color,
        );
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

    /// The rungs of an annulus sector, inner then outer, one per point of
    /// the arc [`Self::arc_points`] would draw at either radius — so a
    /// sector and the arc on its edge share their tessellation and their
    /// ends land on the same points.
    pub fn sector_rungs(
        center: DVec3,
        axis: DVec3,
        start: DVec3,
        inner: f64,
        outer: f64,
        sweep: f64,
    ) -> Vec<(DVec3, DVec3)> {
        let steps = ((sweep.abs() / ARC_STEP).ceil() as usize).max(1);
        let axis = axis.normalize_or_zero();
        let start = start.normalize_or_zero();
        (0..=steps)
            .map(|i| {
                let angle = sweep * i as f64 / steps as f64;
                let dir = riggen_mesh::glam::DQuat::from_axis_angle(axis, angle) * start;
                (center + dir * inner, center + dir * outer)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn visible(x: f32) -> Option<(egui::Pos2, bool)> {
        Some((egui::pos2(x, 0.0), false))
    }

    fn behind(x: f32) -> Option<(egui::Pos2, bool)> {
        Some((egui::pos2(x, 0.0), true))
    }

    fn xs(run: &[egui::Pos2]) -> Vec<f32> {
        run.iter().map(|p| p.x).collect()
    }

    #[test]
    fn an_unsplit_path_is_one_run() {
        let runs = split_runs(&[visible(0.0), visible(1.0), visible(2.0)]);
        assert_eq!(runs.len(), 1);
        assert!(!runs[0].0);
        assert_eq!(xs(&runs[0].1), vec![0.0, 1.0, 2.0]);
        // …whichever side of the geometry it is on.
        let runs = split_runs(&[behind(0.0), behind(1.0)]);
        assert_eq!(runs.len(), 1);
        assert!(runs[0].0);
    }

    #[test]
    fn a_path_through_a_part_splits_into_three_meeting_runs() {
        let runs = split_runs(&[
            visible(0.0),
            visible(1.0),
            behind(2.0),
            behind(3.0),
            visible(4.0),
            visible(5.0),
        ]);
        assert_eq!(runs.len(), 3);
        assert_eq!(
            runs.iter().map(|(hidden, _)| *hidden).collect::<Vec<_>>(),
            vec![false, true, false]
        );
        // The crossing samples belong to both runs, so the strokes meet.
        assert_eq!(xs(&runs[0].1), vec![0.0, 1.0, 2.0]);
        assert_eq!(xs(&runs[1].1), vec![2.0, 3.0, 4.0]);
        assert_eq!(xs(&runs[2].1), vec![4.0, 5.0]);
    }

    #[test]
    fn a_point_that_does_not_project_ends_the_run() {
        let runs = split_runs(&[visible(0.0), visible(1.0), None, visible(5.0), visible(6.0)]);
        assert_eq!(runs.len(), 2);
        assert_eq!(xs(&runs[0].1), vec![0.0, 1.0]);
        assert_eq!(xs(&runs[1].1), vec![5.0, 6.0]);
        // A single point either side of a gap is no line at all.
        assert!(split_runs(&[visible(0.0), None, behind(1.0)]).is_empty());
        assert!(split_runs(&[]).is_empty());
    }

    #[test]
    fn a_one_sample_crossing_meets_the_run_before_it_and_the_one_after() {
        let runs = split_runs(&[visible(0.0), behind(1.0), visible(2.0)]);
        // Two runs, not three: the hidden one reaches back to 0's side and
        // forward to 2, and what is left of the visible run past 2 is a
        // single point, which is no line.
        assert_eq!(runs.len(), 2);
        assert_eq!(xs(&runs[0].1), vec![0.0, 1.0]);
        assert!(!runs[0].0);
        assert_eq!(xs(&runs[1].1), vec![1.0, 2.0]);
        assert!(runs[1].0);
    }

    fn rung(x: f32, hidden: bool) -> Option<(egui::Pos2, egui::Pos2, bool)> {
        Some((egui::pos2(x, 0.0), egui::pos2(x, 1.0), hidden))
    }

    fn rung_xs(run: &[(egui::Pos2, egui::Pos2)]) -> Vec<f32> {
        run.iter().map(|(inner, _)| inner.x).collect()
    }

    #[test]
    fn an_unsplit_strip_is_one_run() {
        let runs = split_strip(&[rung(0.0, false), rung(1.0, false), rung(2.0, false)]);
        assert_eq!(runs.len(), 1);
        assert!(!runs[0].0);
        assert_eq!(rung_xs(&runs[0].1), vec![0.0, 1.0, 2.0]);
        let runs = split_strip(&[rung(0.0, true), rung(1.0, true)]);
        assert_eq!(runs.len(), 1);
        assert!(runs[0].0);
        // One rung bounds no quad.
        assert!(split_strip(&[rung(0.0, true)]).is_empty());
        assert!(split_strip(&[]).is_empty());
    }

    #[test]
    fn a_strip_through_a_part_splits_at_the_rungs_either_side_of_the_hidden_stretch() {
        let runs = split_strip(&[
            rung(0.0, false),
            rung(1.0, false),
            rung(2.0, true),
            rung(3.0, true),
            rung(4.0, true),
            rung(5.0, false),
            rung(6.0, false),
        ]);
        assert_eq!(
            runs.iter().map(|(hidden, _)| *hidden).collect::<Vec<_>>(),
            vec![false, true, false]
        );
        // A quad is hidden only when both its rungs are, so the quads
        // 1–2 and 4–5 stay visible and the crossings land on rungs 2 and
        // 4 — which belong to both runs, so the fills meet.
        assert_eq!(rung_xs(&runs[0].1), vec![0.0, 1.0, 2.0]);
        assert_eq!(rung_xs(&runs[1].1), vec![2.0, 3.0, 4.0]);
        assert_eq!(rung_xs(&runs[2].1), vec![4.0, 5.0, 6.0]);
        // Every rung keeps both its ends.
        for (_, run) in &runs {
            for (inner, outer) in run {
                assert_eq!(inner.y, 0.0);
                assert_eq!(outer.y, 1.0);
            }
        }
    }

    #[test]
    fn a_single_hidden_rung_dims_no_quad() {
        let runs = split_strip(&[rung(0.0, false), rung(1.0, true), rung(2.0, false)]);
        assert_eq!(runs.len(), 1);
        assert!(!runs[0].0);
        assert_eq!(rung_xs(&runs[0].1), vec![0.0, 1.0, 2.0]);
        // Two hidden rungs are one hidden quad, meeting both neighbours.
        let runs = split_strip(&[
            rung(0.0, false),
            rung(1.0, true),
            rung(2.0, true),
            rung(3.0, false),
        ]);
        assert_eq!(
            runs.iter().map(|(hidden, _)| *hidden).collect::<Vec<_>>(),
            vec![false, true, false]
        );
        assert_eq!(rung_xs(&runs[1].1), vec![1.0, 2.0]);
    }

    #[test]
    fn a_rung_that_does_not_project_ends_the_strip_run() {
        let runs = split_strip(&[
            rung(0.0, false),
            rung(1.0, false),
            None,
            rung(5.0, true),
            rung(6.0, true),
        ]);
        assert_eq!(runs.len(), 2);
        assert_eq!(rung_xs(&runs[0].1), vec![0.0, 1.0]);
        assert!(!runs[0].0);
        assert_eq!(rung_xs(&runs[1].1), vec![5.0, 6.0]);
        assert!(runs[1].0);
        // A lone rung either side of a gap bounds nothing.
        assert!(split_strip(&[rung(0.0, false), None, rung(1.0, true)]).is_empty());
    }

    #[test]
    fn a_sector_has_a_rung_per_arc_point_at_both_radii() {
        let arc = OverlayItem::arc_points(DVec3::Z, DVec3::Z, DVec3::X, 2.0, FRAC_PI_2);
        let rungs = OverlayItem::sector_rungs(DVec3::Z, DVec3::Z, DVec3::X, 1.0, 2.0, FRAC_PI_2);
        assert_eq!(rungs.len(), arc.len());
        for ((inner, outer), on_arc) in rungs.iter().zip(&arc) {
            assert!((*outer - *on_arc).length() < 1e-12, "outer edge is the arc");
            assert!(
                ((*inner - DVec3::Z).length() - 1.0).abs() < 1e-12,
                "{inner}"
            );
            assert!(
                ((*outer - DVec3::Z).length() - 2.0).abs() < 1e-12,
                "{outer}"
            );
            // The rung is radial: inner and outer share a direction.
            let (di, do_) = (
                (*inner - DVec3::Z).normalize(),
                (*outer - DVec3::Z).normalize(),
            );
            assert!((di - do_).length() < 1e-12);
        }
        assert!((rungs[0].1 - DVec3::new(2.0, 0.0, 1.0)).length() < 1e-12);
        assert!((rungs[rungs.len() - 1].1 - DVec3::new(0.0, 2.0, 1.0)).length() < 1e-12);
    }

    #[test]
    fn a_negative_sector_turns_the_other_way() {
        let rungs =
            OverlayItem::sector_rungs(DVec3::ZERO, DVec3::Z, DVec3::X, 0.5, 1.0, -FRAC_PI_2);
        assert!((rungs[rungs.len() - 1].1 - DVec3::NEG_Y).length() < 1e-12);
        assert!((rungs[rungs.len() - 1].0 - DVec3::NEG_Y * 0.5).length() < 1e-12);
        let none = OverlayItem::sector_rungs(DVec3::ZERO, DVec3::Z, DVec3::X, 0.5, 1.0, 0.0);
        assert_eq!(none.len(), 2);
        assert_eq!(none[0], none[1]);
    }

    #[test]
    fn a_segment_is_sampled_by_its_screen_length() {
        let at = |x: f32| Some(egui::pos2(x, 0.0));
        assert_eq!(depth_samples(at(0.0), at(0.0)), 1, "a degenerate segment");
        assert_eq!(depth_samples(at(0.0), at(4.0)), 1);
        assert_eq!(depth_samples(at(0.0), at(5.0)), 2);
        assert_eq!(depth_samples(at(0.0), at(200.0)), 50);
        assert_eq!(depth_samples(at(0.0), at(1e6)), MAX_DEPTH_SAMPLES, "capped");
        assert_eq!(depth_samples(at(0.0), None), MAX_DEPTH_SAMPLES);
        assert_eq!(depth_samples(None, None), MAX_DEPTH_SAMPLES);
    }

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
