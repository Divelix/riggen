//! Joint glyphs: the document turned into overlay primitives
//! (docs/ARCHITECTURE.md §Panels and menus).
//!
//! A joint has no geometry, so without a glyph it is invisible in the
//! viewport — the tree is the only place it exists, and "which way does this
//! hinge turn?" has to be read off two number fields. The glyph answers it
//! in the picture: an **axis segment** through the pivot, an **origin triad**
//! in the axes triad's colours, and a **band** (revolute) or the same band
//! unrolled into **bars** (prismatic) with a tick at the current `q`. The
//! band is an annulus in the joint's plane drawn as three **opaque**
//! sectors, three shades of the one colour — the full circle darkest, the
//! limits over it, the run from zero to `q` on top — so which end of a
//! hinge is the lower limit, how much of the range is used and whether it
//! is near a stop read without finding the arc's start. Opaque because a
//! translucent sector foreshortened onto itself double-covers and shows a
//! seam the joint does not have (ADR-0027 §2); a slide's bars are the same
//! three shades, its range bar the axis segment's own extent.
//!
//! Drawn for every movable joint plus the selected one, whatever its kind
//! (plans/m2-placement-ux OPEN 4): an unselected `Fixed` joint has nothing
//! to show and every weld in a big assembly would be noise.
//!
//! A glyph also says whether its joint is free to move: a **mimic
//! follower** (ADR-0013) is drawn in a muted amber and labelled with its
//! leader, an **actuated** joint (ADR-0014) keeps the full amber and gains
//! a ring at the pivot labelled with the preset. Without that the viewport
//! draws a driven hinge exactly like a free one and only the joint tree
//! knows the difference (ADR-0020).
//!
//! The anchor is the **pivot** — `world(parent) ∘ origin` — not the child
//! link frame, which for a prismatic joint has already slid away by `q`.

use riggen_core::glam::{DQuat, DVec3};
use riggen_core::{FrameId, JointId, JointKind, JointState, LinkId, Pose};
use riggen_viewport::{Overlay, OverlayItem};

use super::{Mode, RiggenApp, Selection};

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
/// A mimic follower's amber, muted (ADR-0013): the joint cannot move on
/// its own, and the glyph says so before its label is read.
const AXIS_COLOR_MIMIC: egui::Color32 = egui::Color32::from_rgb(166, 133, 84);
/// The same, for the muted glyph the user is pointing at or has selected.
const AXIS_COLOR_MIMIC_ACTIVE: egui::Color32 = egui::Color32::from_rgb(214, 186, 143);
/// The tick at the current `q`, and a frame glyph's origin dot and label.
const TICK_COLOR: egui::Color32 = egui::Color32::from_rgb(245, 245, 245);
/// A frame's name and origin dot: the same near-white, so the label reads
/// against the scene without competing with the joints' amber.
const LABEL_COLOR: egui::Color32 = TICK_COLOR;

/// How near the cursor has to come to a glyph's axis segment, in screen
/// points, to count as pointing at it. Roughly a finger's worth of slop on
/// a line that is 1.5 points wide — a joint is a small target and missing it
/// by two pixels should not mean picking the part behind it instead.
pub const GLYPH_HOVER_RADIUS: f32 = 8.0;

/// Fractions of the glyph's size (§`glyph_size`).
const AXIS_HALF_LENGTH: f64 = 1.15;
/// A frame glyph's triad arms, as a fraction of its link's glyph size.
/// Longer than a joint's origin triad, which is one decoration among four:
/// the triad *is* the frame glyph, so it has to be aimable on its own.
const FRAME_TRIAD_LENGTH: f64 = 0.55;
const TRIAD_LENGTH: f64 = 0.4;
/// The band's outer edge — where the limit arc used to be stroked.
pub const ARC_RADIUS: f64 = 0.6;
/// The band's inner edge: inside [`ARC_RADIUS`], and outside
/// [`ACTUATOR_RING_RADIUS`] with room to spare, so the actuator ring and
/// the pivot triad stay in the clear bore and never read as part of the
/// band.
pub const BAND_INNER: f64 = 0.42;
/// The actuated-joint ring, well inside the band so the two never read as
/// one.
pub const ACTUATOR_RING_RADIUS: f64 = 0.16;
const _: () = assert!(ACTUATOR_RING_RADIUS < BAND_INNER && BAND_INNER < ARC_RADIUS);
/// How far past the band the current-`q` tick sticks out.
const TICK_OVERSHOOT: f64 = 1.25;

/// The band's three shades, as fractions of the glyph's own colour: the
/// full circle (a slide's whole travel), the limits over it, the run from
/// the zero position to `q` on top of that — which is the colour itself.
/// Each sector is **opaque** (ADR-0027 §2): a translucent one drawn over
/// itself double-covers, and at a grazing camera angle a foreshortened
/// annulus sector does exactly that, showing a lighter seam where nothing
/// about the joint changed. An opaque sector over itself is itself.
/// Settled by eye on the goldens, as their alphas were.
const RANGE_SHADE: f32 = 0.34;
const LIMIT_SHADE: f32 = 0.62;
const VALUE_SHADE: f32 = 1.0;
const _: () = assert!(RANGE_SHADE < LIMIT_SHADE && LIMIT_SHADE < VALUE_SHADE);

/// `color` scaled towards black by `factor`, at full alpha: the band's
/// ramp is three shades of one colour rather than one colour at three
/// alphas. Scaling the **RGB** and not the alpha is what makes the stack
/// order-independent, and it shades whatever colour it is handed, so a
/// mimic follower's muted amber (ADR-0013) and a hot glyph's bright one
/// keep their ramp without a constant each.
fn shade(color: egui::Color32, factor: f32) -> egui::Color32 {
    let scaled = |c: u8| (f32::from(c) * factor).round().clamp(0.0, 255.0) as u8;
    egui::Color32::from_rgb(scaled(color.r()), scaled(color.g()), scaled(color.b()))
}

/// One joint's glyph, already placed in the world: what the overlay draws
/// and what a hover hit-test measures against.
#[derive(Debug, Clone, PartialEq)]
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
    /// The joint this one follows, if any (ADR-0013). A follower is drawn
    /// muted: its `q` is somebody else's.
    pub mimic: Option<JointId>,
    /// The preset of every actuator driving it, in `ActuatorId` order —
    /// `"position"`, `"velocity"` or `"motor"` (ADR-0014). Presets, not
    /// gains and not names: the glyph says *that* the joint is driven and
    /// by how many, the panel says how hard and under what name. Several
    /// are legal (ADR-0023) and MuJoCo sums them.
    pub actuators: Vec<&'static str>,
}

impl JointGlyph {
    /// Whether something other than the user's hand moves this joint.
    pub fn driven(&self) -> bool {
        self.mimic.is_some() || !self.actuators.is_empty()
    }
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
    pub(crate) fn reference(&self) -> DVec3 {
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

    /// The band's inner and outer radius, in metres, for a joint that has
    /// one — a revolute or continuous joint. A prismatic joint's bars and
    /// a weld have no band.
    pub fn band(&self) -> Option<(f64, f64)> {
        match self.kind {
            JointKind::Revolute | JointKind::Continuous => {
                Some((self.size * BAND_INNER, self.size * ARC_RADIUS))
            }
            JointKind::Prismatic | JointKind::Fixed => None,
        }
    }

    /// The signed sweep of the value sector: from the zero position to `q`,
    /// in the joint's own unit. What the drawing sweeps, exposed so a
    /// scenario asserts the shape and not only the pixels.
    pub fn value_sweep(&self) -> f64 {
        match self.kind {
            JointKind::Fixed => 0.0,
            _ => self.q,
        }
    }

    /// The band's centreline as world points — the full circle midway
    /// between its edges, starting at the zero position. In View this is
    /// the hover target: the pointer inside the circle, or within
    /// [`GLYPH_HOVER_RADIUS`] of it, is on the joint (`glyph_at`,
    /// ADR-0021 §6). Empty for a joint without a band.
    pub fn band_points(&self) -> Vec<DVec3> {
        let Some((inner, outer)) = self.band() else {
            return Vec::new();
        };
        OverlayItem::arc_points(
            self.pivot.t,
            self.axis,
            self.reference(),
            (inner + outer) * 0.5,
            std::f64::consts::TAU,
        )
    }
}

/// One frame's glyph: a triad in the triad colours at the frame's world
/// pose, with its name beside it (ADR-0012). A frame has no geometry, so
/// like a joint it exists in the viewport only as this.
#[derive(Debug, Clone, PartialEq)]
pub struct FrameGlyph {
    pub frame: FrameId,
    pub name: String,
    /// `world(parent) ∘ frame.pose`.
    pub pose: Pose,
    /// Length of a triad arm.
    pub size: f64,
}

impl FrameGlyph {
    /// The far end of each triad arm, in world coordinates, X then Y then Z
    /// — what the overlay draws and a hover hit-test measures against.
    pub fn arms(&self) -> [DVec3; 3] {
        [DVec3::X, DVec3::Y, DVec3::Z].map(|local| self.pose.t + self.pose.r * local * self.size)
    }
}

impl RiggenApp {
    /// Every glyph the viewport should draw this frame.
    pub fn joint_glyphs(&self) -> Vec<JointGlyph> {
        // The visibility row's `joints` toggle empties the list, which
        // takes out the drawing *and* `glyph_at` in one place: a hidden
        // thing answers nothing (ADR-0021, amended).
        if !self.overlays().joints {
            return Vec::new();
        }
        let world = riggen_core::fk(&self.robot, &self.q);
        // Resolved, not raw: a mimic follower's own entry in `self.q` is
        // never written, so the tick of a driven joint would sit at zero
        // while its link is somewhere else entirely (ADR-0013). `fk` above
        // already resolves for the poses; this is the same answer for the
        // glyph's own numbers.
        let q = riggen_core::resolve_q(&self.robot, &self.q);
        let selected = match self.selection {
            Selection::Joint(j) => Some(j),
            _ => None,
        };
        self.robot
            .joints
            .iter()
            .filter(|(id, joint)| joint.kind.is_movable() || selected == Some(**id))
            .filter_map(|(&id, joint)| {
                // A gizmo drag previews on the glyph: a pivot move changes
                // `origin` and nothing else, so the geometry does not budge
                // and the glyph is the only thing that can show the gesture.
                let pivot = match self.dragged_pivot(id) {
                    Some(pose) => pose,
                    None => world.get(&joint.parent)?.compose(&joint.origin),
                };
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
                    q: q.get(id),
                    limits: joint.limits.map(|l| (l.lower, l.upper)),
                    mimic: joint.mimic.map(|m| m.joint),
                    actuators: self
                        .robot
                        .actuators_on(id)
                        .map(|(_, a)| a.spec.kind_name())
                        .collect(),
                })
            })
            .collect()
    }

    /// A glyph for every named frame, in `FrameId` order. Unlike joints,
    /// all of them are drawn all the time: a frame is a thing the user
    /// placed on purpose and there are a handful, not one per weld.
    pub fn frame_glyphs(&self) -> Vec<FrameGlyph> {
        // As for joints: hidden takes the triad and `frame_glyph_at`
        // together (ADR-0021, amended).
        if !self.overlays().frames {
            return Vec::new();
        }
        let world = riggen_core::frames(&self.robot, &self.q);
        self.robot
            .frames
            .iter()
            .filter_map(|(&id, frame)| {
                Some(FrameGlyph {
                    frame: id,
                    name: frame.name.clone(),
                    // A gizmo drag previews on the glyph: nothing else in
                    // the scene moves with a frame.
                    pose: self.dragged_frame(id).or_else(|| world.get(&id).copied())?,
                    size: self.glyph_size(frame.parent) * FRAME_TRIAD_LENGTH,
                })
            })
            .collect()
    }

    /// The frame glyphs as overlay primitives. `active` is the frame the
    /// user is pointing at or has selected: brighter, thicker, and its
    /// label in the active amber.
    pub(crate) fn push_frame_overlay(
        &self,
        overlay: &mut Overlay,
        glyphs: &[FrameGlyph],
        active: Option<FrameId>,
    ) {
        for glyph in glyphs {
            let hot = active == Some(glyph.frame);
            let width = if hot { 3.0 } else { 1.5 };
            // The triad is geometry and meets the scene's depth; the name
            // beside it is text, and text that fades behind a part is
            // unreadable rather than informative (ADR-0020).
            overlay.depth_tested(|overlay| {
                overlay.point(glyph.pose.t, if hot { 5.0 } else { 3.5 }, LABEL_COLOR);
                for (arm, color) in glyph.arms().into_iter().zip(TRIAD_COLORS) {
                    overlay.segment(glyph.pose.t, arm, color, width);
                }
            });
            // A frame's name is a name, so it goes with the row's
            // `joint names` toggle rather than with the triad.
            if self.overlays().joint_names {
                overlay.label(
                    glyph.pose.t,
                    glyph.name.clone(),
                    if hot { AXIS_COLOR_ACTIVE } else { LABEL_COLOR },
                    egui::vec2(8.0, -8.0),
                );
            }
        }
    }

    /// The frame a glyph is drawn hot for: the one under the pointer, else
    /// the selected one.
    pub fn active_frame(&self) -> Option<FrameId> {
        self.hovered_frame.or(match self.selection {
            Selection::Frame(f) => Some(f),
            _ => None,
        })
    }

    /// The frame the pointer is on, from the tree or from its glyph.
    pub fn hovered_frame(&self) -> Option<FrameId> {
        self.hovered_frame
    }

    /// The frame whose glyph is under `pos`: screen distance to the nearest
    /// of its three triad arms, within [`GLYPH_HOVER_RADIUS`], as for a
    /// joint's axis segment.
    pub fn frame_glyph_at(&self, glyphs: &[FrameGlyph], pos: egui::Pos2) -> Option<FrameId> {
        glyphs
            .iter()
            .filter_map(|glyph| {
                let origin = self.project_world(glyph.pose.t)?;
                let distance = glyph
                    .arms()
                    .into_iter()
                    .filter_map(|arm| {
                        Some(distance_to_segment(pos, origin, self.project_world(arm)?))
                    })
                    .fold(f32::INFINITY, f32::min);
                (distance <= GLYPH_HOVER_RADIUS).then_some((distance, glyph.frame))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, frame)| frame)
    }

    /// How big a glyph on `child` is: the half-diagonal of the child link's
    /// bounds **in its own frame** — every visual geom's box through its
    /// geom pose and nothing else — so a glyph is the size of the part it
    /// belongs to and stays that size as the joint moves. Measured through
    /// `world(child)` instead, the axis-aligned box of a turned part grows
    /// and shrinks with `q`, and the band would breathe under the
    /// scrubber; the part is the same part at either end of its travel.
    ///
    /// A link with no geometry yet falls back to
    /// [`Self::rest_scene_radius`] — the scene's radius at the zero
    /// configuration, which does not move either — and an empty scene to
    /// one metre.
    fn glyph_size(&self, child: LinkId) -> f64 {
        let own = self
            .robot
            .links
            .get(&child)
            .into_iter()
            .flat_map(|link| link.visuals.iter())
            .filter_map(|geom| {
                let id = self.instances.get(&(child, geom.id))?;
                let state = self.viewport.instance_states().find(|s| s.id == *id)?;
                Some(state.bounds?.transformed(&geom.pose.to_mat4()))
            })
            .reduce(|a, b| a.union(&b));
        if let Some(bounds) = own
            && bounds.half_diagonal() > 1e-9
        {
            return bounds.half_diagonal();
        }
        self.rest_scene_radius()
    }

    /// The scene's radius with every joint at zero, and one metre for a
    /// scene that has nothing in it. `Viewport::scene_bounds` measures
    /// where the parts are *now*, so it moves with `q` — a glyph on a
    /// geometry-less link sized off it would breathe under the scrubber
    /// exactly as the world-bounds measure did. One extra `fk` on a
    /// fallback nobody hits unless a link is empty.
    fn rest_scene_radius(&self) -> f64 {
        let rest = riggen_core::fk(&self.robot, &JointState::default());
        self.instances
            .iter()
            .filter_map(|(&(link, geom), id)| {
                let g = self
                    .robot
                    .links
                    .get(&link)?
                    .visuals
                    .iter()
                    .find(|g| g.id == geom)?;
                let state = self.viewport.instance_states().find(|s| s.id == *id)?;
                Some(
                    state
                        .bounds?
                        .transformed(&rest.get(&link)?.compose(&g.pose).to_mat4()),
                )
            })
            .reduce(|a, b| a.union(&b))
            .map(|bounds| bounds.half_diagonal())
            .filter(|r| *r > 1e-9)
            .unwrap_or(1.0)
    }

    /// The colour a glyph's amber parts are drawn in: brighter for the
    /// joint the user is pointing at, muted for one a mimic drives — its
    /// `q` is somebody else's, and the glyph says so before the label is
    /// read (ADR-0013).
    ///
    /// An **actuated** joint keeps the full amber on purpose: an actuator
    /// holds a joint the user can still pose, where a mimic takes the
    /// posing away. Its mark is the ring, not the colour.
    fn axis_color(glyph: &JointGlyph, hot: bool) -> egui::Color32 {
        match (glyph.mimic.is_some(), hot) {
            (false, false) => AXIS_COLOR,
            (false, true) => AXIS_COLOR_ACTIVE,
            (true, false) => AXIS_COLOR_MIMIC,
            (true, true) => AXIS_COLOR_MIMIC_ACTIVE,
        }
    }

    /// What a glyph says in words about not being free: `» <leader>` for a
    /// mimic follower, one preset name per actuator driving it. Both, in
    /// that order, for a joint that is somehow both — `validate` rejects
    /// that pairing (ADR-0014), and a glyph is not the place to hide it.
    ///
    /// `»` and not `↳`: egui's bundled fonts have no arrows, and a mark
    /// that renders as a tofu box says nothing at all. `»` reads as
    /// "follows" without claiming the equality `=` would — the multiplier
    /// and offset are the joint tree's to state
    /// (`panels/joint_tree.rs::mimic_rule`).
    fn driven_marks(&self, glyph: &JointGlyph) -> Vec<String> {
        let mut marks = Vec::new();
        if let Some(leader) = glyph.mimic {
            let name = self
                .robot
                .joints
                .get(&leader)
                .map_or_else(|| leader.to_string(), |j| j.name.clone());
            marks.push(format!("\u{bb} {name}"));
        }
        marks.extend(glyph.actuators.iter().map(|a| (*a).to_owned()));
        marks
    }

    /// The glyphs as overlay primitives. `active` is the joint the user is
    /// pointing at or has selected, drawn brighter and thicker.
    pub(crate) fn glyph_overlay(&self, glyphs: &[JointGlyph], active: Option<JointId>) -> Overlay {
        let mut overlay = Overlay::default();
        // Every part of a joint glyph claims to be somewhere in the scene,
        // so all of it meets the scene's depth (ADR-0020).
        overlay.depth_tested(|overlay| {
            for glyph in glyphs {
                let hot = active == Some(glyph.joint);
                let color = Self::axis_color(glyph, hot);
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

                // An actuated joint gets a ring round the pivot, in the
                // joint's own plane and well inside the limit arc
                // (ADR-0014).
                if !glyph.actuators.is_empty() {
                    overlay.push(OverlayItem::Arc {
                        center: glyph.pivot.t,
                        axis: glyph.axis,
                        start: glyph.reference(),
                        radius: glyph.size * ACTUATOR_RING_RADIUS,
                        sweep: std::f64::consts::TAU,
                        color,
                        width,
                    });
                }

                match glyph.kind {
                    JointKind::Revolute | JointKind::Continuous => {
                        self.push_arc(overlay, glyph, color, width)
                    }
                    JointKind::Prismatic => self.push_slide(overlay, glyph, color, width),
                    JointKind::Fixed => {}
                }
            }
        });
        // The labels last and undepthed: text that fades behind a part is
        // unreadable rather than informative (ADR-0020 §4). The row's
        // `joint names` toggle drops them and nothing else — a name is a
        // decoration on a glyph, not a target of its own.
        for glyph in glyphs.iter().filter(|_| self.overlays().joint_names) {
            let color = Self::axis_color(glyph, active == Some(glyph.joint));
            for (i, mark) in self.driven_marks(glyph).into_iter().enumerate() {
                overlay.label(
                    glyph.pivot.t,
                    mark,
                    color,
                    egui::vec2(11.0, 26.0 + i as f32 * 13.0),
                );
            }
        }
        overlay
    }

    /// The joint a glyph is drawn hot for: the one under the pointer, else
    /// the selected one.
    pub fn active_joint(&self) -> Option<JointId> {
        self.hovered_joint.or(match self.selection {
            Selection::Joint(j) => Some(j),
            _ => None,
        })
    }

    /// The joint the pointer is on, from the tree or from its glyph.
    pub fn hovered_joint(&self) -> Option<JointId> {
        self.hovered_joint
    }

    /// The joint whose glyph is under `pos`; the nearest wins.
    ///
    /// In **Edit** the target is the axis segment within
    /// [`GLYPH_HOVER_RADIUS`] screen points, and nothing more: the mesh
    /// under the cursor is what Edit's tools aim at, and a target that
    /// swallowed the part behind it would take the hover pick away. In
    /// **View** the mesh answers nothing (ADR-0021 §1), so the target grows
    /// to the **band and its interior** — the pointer inside the band's
    /// projected centreline, or within the same radius of it — because a
    /// joint is what the user came to pose and a thin line is a poor thing
    /// to aim a wheel at. A prismatic joint, having no band, keeps the
    /// axis in both.
    ///
    /// Screen space, not a ray cast: the glyph is drawn at a fixed pixel
    /// width and what the user is aiming at is what they can see, not a
    /// solid around it that shrinks with distance. The score is the
    /// distance to the axis or to the centreline, so where one band's disc
    /// contains a smaller glyph the nearer ring wins.
    pub fn glyph_at(&self, glyphs: &[JointGlyph], pos: egui::Pos2) -> Option<JointId> {
        glyphs
            .iter()
            .filter_map(|glyph| Some((self.glyph_distance(glyph, pos)?, glyph.joint)))
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, joint)| joint)
    }

    /// How far `pos` is from `glyph`'s hover target, or `None` when it is
    /// not on it — see [`Self::glyph_at`] for what the target is in each
    /// mode.
    fn glyph_distance(&self, glyph: &JointGlyph, pos: egui::Pos2) -> Option<f32> {
        let (from, to) = glyph.axis_ends();
        let axis = match (self.project_world(from), self.project_world(to)) {
            (Some(a), Some(b)) => Some(distance_to_segment(pos, a, b)),
            _ => None,
        }
        .filter(|d| *d <= GLYPH_HOVER_RADIUS);
        if self.mode != Mode::View {
            return axis;
        }
        let ring: Vec<egui::Pos2> = glyph
            .band_points()
            .into_iter()
            .filter_map(|p| self.project_world(p))
            .collect();
        let band = (ring.len() >= 3)
            .then(|| distance_to_polygon(pos, &ring))
            .filter(|d| *d <= GLYPH_HOVER_RADIUS || point_in_polygon(pos, &ring));
        match (axis, band) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// The band of a revolute joint: three filled sectors of the annulus
    /// between [`BAND_INNER`] and [`ARC_RADIUS`], each an opaque shade of
    /// the glyph's colour (ADR-0027 §2) — the full circle at
    /// [`RANGE_SHADE`], the limits over it at [`LIMIT_SHADE`], the run
    /// from the zero position to `q` on top at [`VALUE_SHADE`] — with the
    /// white spoke at `q` kept, so a joint at zero still points. A
    /// `Continuous` joint has no limits: its full circle *is* the limit
    /// band.
    ///
    /// The colour carries the mimic muting and the hot brightening as the
    /// stroke did; the width has no fill to change, so a hot band is the
    /// brighter amber alone.
    fn push_arc(
        &self,
        overlay: &mut Overlay,
        glyph: &JointGlyph,
        color: egui::Color32,
        width: f32,
    ) {
        let Some((inner, outer)) = glyph.band() else {
            return;
        };
        let reference = glyph.reference();
        let mut sector = |start: DVec3, sweep: f64, factor: f32| {
            overlay.sector(
                glyph.pivot.t,
                glyph.axis,
                start,
                inner,
                outer,
                sweep,
                shade(color, factor),
            );
        };
        match glyph.limits {
            Some((lower, upper)) => {
                sector(reference, std::f64::consts::TAU, RANGE_SHADE);
                let start = DQuat::from_axis_angle(glyph.axis, lower) * reference;
                sector(start, upper - lower, LIMIT_SHADE);
            }
            None => sector(reference, std::f64::consts::TAU, LIMIT_SHADE),
        }
        sector(reference, glyph.value_sweep(), VALUE_SHADE);
        // The tick: a spoke from the pivot through the band at the current
        // angle, so "where is this joint now" is one glance even at zero,
        // where the value sector has no width.
        let at = DQuat::from_axis_angle(glyph.axis, glyph.q) * reference;
        overlay.segment(
            glyph.pivot.t,
            glyph.pivot.t + at * outer * TICK_OVERSHOOT,
            TICK_COLOR,
            width,
        );
    }

    /// The travel of a prismatic joint: the band unrolled into **bars**
    /// beside the axis, between the same [`BAND_INNER`] and [`ARC_RADIUS`]
    /// offsets the revolute band spans, so a slide and a hinge are read
    /// the same way. Three bars in the three opaque shades, as the hinge
    /// has three sectors: the axis segment's own extent at
    /// [`RANGE_SHADE`], the limits over it at [`LIMIT_SHADE`], the run
    /// from zero to `q` on top at [`VALUE_SHADE`], with the end stops and
    /// the white tick at `q` kept as they were.
    ///
    /// The range bar is the hinge's full circle for a slide (ADR-0027 §4).
    /// A slide has no travel beyond its own limits to be faint over, so it
    /// borrows the length the glyph already claims on screen — and without
    /// it an **unlimited** slide, whose limit and value bars are
    /// zero-length, would draw a tick and nothing else in View, with
    /// nothing to aim at.
    fn push_slide(
        &self,
        overlay: &mut Overlay,
        glyph: &JointGlyph,
        color: egui::Color32,
        width: f32,
    ) {
        let (lower, upper) = glyph.limits.unwrap_or((0.0, 0.0));
        // Offset off the axis line so the travel is readable beside it
        // rather than drawn on top of the axis segment.
        let reference = glyph.reference();
        let inner = reference * glyph.size * BAND_INNER;
        let outer = reference * glyph.size * ARC_RADIUS;
        // One rung of a bar: the pair the strip spans at travel `t`.
        let rung = |t: f64| {
            let at = glyph.pivot.t + glyph.axis * t;
            (at + inner, at + outer)
        };
        let half = glyph.size * AXIS_HALF_LENGTH;
        overlay.strip(vec![rung(-half), rung(half)], shade(color, RANGE_SHADE));
        overlay.strip(vec![rung(lower), rung(upper)], shade(color, LIMIT_SHADE));
        overlay.strip(
            vec![rung(0.0), rung(glyph.value_sweep())],
            shade(color, VALUE_SHADE),
        );
        for end in [lower, upper] {
            let (from, to) = rung(end);
            overlay.segment(from, to, color, width);
        }
        // The tick, as the revolute's spoke: from the axis out through the
        // bar and past it, so `q` is one glance even where the value bar
        // has no length.
        let at = glyph.pivot.t + glyph.axis * glyph.q;
        overlay.segment(at, at + outer * TICK_OVERSHOOT, TICK_COLOR, width);
    }
}

/// Distance in screen points from `pos` to the closed polygon through
/// `ring` — the nearest of its edges, the closing one included.
pub(crate) fn distance_to_polygon(pos: egui::Pos2, ring: &[egui::Pos2]) -> f32 {
    ring.iter()
        .zip(ring.iter().cycle().skip(1))
        .map(|(a, b)| distance_to_segment(pos, *a, *b))
        .fold(f32::INFINITY, f32::min)
}

/// Whether `pos` is inside the closed polygon through `ring`, by the
/// even–odd rule: a ray cast to +x crosses an odd number of edges. A
/// projected circle is convex, but the rule needs no such promise.
pub(crate) fn point_in_polygon(pos: egui::Pos2, ring: &[egui::Pos2]) -> bool {
    let mut inside = false;
    for (a, b) in ring.iter().zip(ring.iter().cycle().skip(1)) {
        if (a.y > pos.y) != (b.y > pos.y) {
            let x = a.x + (pos.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if pos.x < x {
                inside = !inside;
            }
        }
    }
    inside
}

/// Distance in screen points from `pos` to the segment `a`–`b`; the
/// distance to the nearer end when the projection falls outside it.
pub(crate) fn distance_to_segment(pos: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let ab = b - a;
    let length_sq = ab.length_sq();
    if length_sq <= f32::EPSILON {
        return (pos - a).length();
    }
    let t = ((pos - a).dot(ab) / length_sq).clamp(0.0, 1.0);
    (pos - (a + ab * t)).length()
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::pos2;
    use riggen_core::Id;

    #[test]
    fn the_ramp_is_three_opaque_shades_of_whatever_colour_it_is_handed() {
        // Every shade is opaque: the stack cannot double-cover itself
        // however the camera foreshortens it (ADR-0027 §2).
        for factor in [RANGE_SHADE, LIMIT_SHADE, VALUE_SHADE] {
            assert_eq!(shade(AXIS_COLOR, factor).a(), 255);
        }
        // Ordered (the `const` assert beside them) and monotone on the
        // colour: darkest is the range, the value is the colour itself.
        assert_eq!(shade(AXIS_COLOR, VALUE_SHADE), AXIS_COLOR);
        assert!(shade(AXIS_COLOR, RANGE_SHADE).r() < shade(AXIS_COLOR, LIMIT_SHADE).r());
        // It shades whatever colour it is handed, so a follower's muted
        // amber (ADR-0013) keeps the ramp without a constant of its own.
        assert!(shade(AXIS_COLOR_MIMIC, LIMIT_SHADE).r() < shade(AXIS_COLOR, LIMIT_SHADE).r());
        assert_eq!(shade(AXIS_COLOR, 0.0), egui::Color32::BLACK);
    }

    #[test]
    fn the_band_sits_between_the_actuator_ring_and_the_old_arc() {
        let glyph = JointGlyph {
            joint: JointId::from_raw(1),
            pivot: Pose::IDENTITY,
            axis: DVec3::Z,
            size: 2.0,
            kind: JointKind::Revolute,
            q: 0.3,
            limits: Some((-1.0, 1.0)),
            mimic: None,
            actuators: Vec::new(),
        };
        assert_eq!(glyph.band(), Some((2.0 * BAND_INNER, 2.0 * ARC_RADIUS)));
        assert_eq!(glyph.value_sweep(), 0.3);
        let points = glyph.band_points();
        assert!(points.len() > 8);
        let mid = (BAND_INNER + ARC_RADIUS) * 0.5 * 2.0;
        for p in &points {
            assert!((p.length() - mid).abs() < 1e-12, "{p}");
        }
        // The zero position is where the circle starts.
        assert!((points[0] - glyph.reference() * mid).length() < 1e-12);
        let slide = JointGlyph {
            kind: JointKind::Prismatic,
            ..glyph.clone()
        };
        assert_eq!(slide.band(), None);
        assert!(slide.band_points().is_empty());
        let weld = JointGlyph {
            kind: JointKind::Fixed,
            ..glyph.clone()
        };
        assert_eq!(weld.value_sweep(), 0.0);
    }

    #[test]
    fn a_point_is_inside_a_polygon_or_near_its_edge() {
        // A unit square from (10, 10) to (20, 20).
        let square = [
            pos2(10.0, 10.0),
            pos2(20.0, 10.0),
            pos2(20.0, 20.0),
            pos2(10.0, 20.0),
        ];
        assert!(point_in_polygon(pos2(15.0, 15.0), &square));
        assert!(point_in_polygon(pos2(10.5, 19.5), &square));
        assert!(!point_in_polygon(pos2(25.0, 15.0), &square));
        assert!(!point_in_polygon(pos2(15.0, 5.0), &square));
        // The closing edge counts: a point just left of the square is
        // nearest to the (10, 20)–(10, 10) edge, not to a corner.
        assert!((distance_to_polygon(pos2(7.0, 15.0), &square) - 3.0).abs() < 1e-5);
        assert!((distance_to_polygon(pos2(15.0, 24.0), &square) - 4.0).abs() < 1e-5);
        // Inside, the distance is to the nearest edge.
        assert!((distance_to_polygon(pos2(12.0, 15.0), &square) - 2.0).abs() < 1e-5);
        // A degenerate ring is no target.
        assert!(!point_in_polygon(
            pos2(15.0, 15.0),
            &[pos2(10.0, 10.0), pos2(20.0, 20.0)]
        ));
    }

    #[test]
    fn distance_to_a_segment_clamps_to_its_ends() {
        let (a, b) = (pos2(10.0, 10.0), pos2(30.0, 10.0));
        // Beside the middle.
        assert!((distance_to_segment(pos2(20.0, 15.0), a, b) - 5.0).abs() < 1e-5);
        // On it.
        assert!(distance_to_segment(pos2(25.0, 10.0), a, b) < 1e-5);
        // Past an end: the distance to the end, not to the infinite line.
        assert!((distance_to_segment(pos2(40.0, 10.0), a, b) - 10.0).abs() < 1e-5);
        assert!((distance_to_segment(pos2(0.0, 10.0), a, b) - 10.0).abs() < 1e-5);
        // A degenerate segment is a point.
        assert!((distance_to_segment(pos2(13.0, 14.0), a, a) - 5.0).abs() < 1e-5);
    }
}
