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
//! it has to look it (ADR-0020).

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
