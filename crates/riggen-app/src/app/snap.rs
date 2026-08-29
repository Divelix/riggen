//! Snapping: what the cursor is *really* pointing at
//! (docs/01-architecture.md §Picking and snapping).
//!
//! The ID buffer says which triangle is under the cursor. That is enough to
//! recover an exact point with `ray_triangle`, and — through
//! `riggen_mesh::feature` — the circle a bore or a shaft belongs to. The
//! candidates, in the order they beat each other:
//!
//! 1. **vertex** — a corner of the hit triangle within the pixel radius;
//! 2. **box** — a corner or face centre of the instance's own AABB, also
//!    within the pixel radius (a part with no modelled features still has
//!    somewhere obvious to grab);
//! 3. **circle** — the fitted circle of the smooth region around the hit
//!    triangle, whose centre and axis are what "click the bore, get the
//!    joint axis" is made of. No pixel radius: the centre of a bore is
//!    nowhere near the wall the user is pointing at, which is the point;
//! 4. **point** — the ray/triangle hit itself, which always exists.
//!
//! The ladder is a pure function ([`choose`]) so it is unit-tested without
//! a GPU, and the circle fit is memoised per `(instance, triangle)`: a
//! cursor resting on one facet fits once, not once per frame.

use riggen_core::glam::{DMat4, DVec3};
use riggen_core::{JointId, LinkId, Pose};
use riggen_mesh::feature::CircleFit;
use riggen_mesh::{Ray, ray_triangle};
use riggen_viewport::{InstanceId, Overlay, OverlayItem};

use super::{RiggenApp, Tool};

/// How near the cursor has to come to a vertex or a box corner, in screen
/// points, for it to win over the plain hit point. Wider than the glyph
/// radius: these are the targets a user is trying to hit, and the fallback
/// (the exact point on the surface) is never wrong, only less useful.
pub const SNAP_PIXEL_RADIUS: f32 = 12.0;

/// The colour of every snap marker: cyan, which no material, glyph or gizmo
/// axis uses.
const SNAP_COLOR: egui::Color32 = egui::Color32::from_rgb(64, 224, 224);

/// What kind of thing the cursor snapped to, most specific first — this
/// order *is* the priority ladder.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SnapKind {
    Vertex,
    BoxCorner,
    BoxFaceCenter,
    Circle,
    Point,
}

impl SnapKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Vertex => "vertex",
            Self::BoxCorner => "box corner",
            Self::BoxFaceCenter => "box face",
            Self::Circle => "circle",
            Self::Point => "point",
        }
    }
}

/// One resolved snap target, in world coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SnapCandidate {
    pub kind: SnapKind,
    /// Where to snap: the vertex, the box corner, the circle's centre, or
    /// the exact hit.
    pub point: DVec3,
    /// The hit triangle's normal — "axis = face normal" for a flat face.
    pub normal: DVec3,
    /// Present for [`SnapKind::Circle`], already in world coordinates.
    pub circle: Option<CircleFit>,
    /// The exact ray/triangle hit, whatever the chosen kind snapped to.
    pub hit: DVec3,
    pub link: LinkId,
    pub instance: InstanceId,
    pub triangle: u32,
}

impl SnapCandidate {
    /// The axis this candidate offers a joint: the circle's for a circle,
    /// the face normal otherwise.
    pub fn axis(&self) -> DVec3 {
        self.circle.map_or(self.normal, |fit| fit.axis)
    }

    /// The readout drawn beside the marker — `circle r 12.0 mm · 24 seg ·
    /// res 0.01 mm`. Millimetres, because that is what the parts are drawn
    /// in even though the document is metres, and because a residual in
    /// metres reads as a row of zeros.
    pub fn readout(&self) -> String {
        match self.circle {
            Some(fit) => format!(
                "circle r {:.1} mm · {} seg · res {:.2} mm",
                fit.radius * 1000.0,
                fit.segments,
                fit.residual * 1000.0
            ),
            None => self.kind.label().to_owned(),
        }
    }
}

/// The last circle fit, kept so a resting cursor does not refit every frame.
/// The fit is in **mesh-local** coordinates; the instance's model matrix
/// puts it in the world.
#[derive(Debug, Clone, Default)]
pub(crate) struct SnapCache {
    key: Option<(InstanceId, u32)>,
    circle: Option<CircleFit>,
}

/// The priority ladder, as a pure function: the first candidate that exists
/// wins, and `point` always does.
///
/// Split out from the geometry above it so the order can be tested without
/// a GPU, a mesh or a camera — the ladder is the part a change is most
/// likely to get wrong, and the least likely to notice.
pub(crate) fn choose(
    vertex: Option<DVec3>,
    boxed: Option<(SnapKind, DVec3)>,
    circle: Option<CircleFit>,
    point: DVec3,
) -> (SnapKind, DVec3, Option<CircleFit>) {
    if let Some(at) = vertex {
        return (SnapKind::Vertex, at, None);
    }
    if let Some((kind, at)) = boxed {
        return (kind, at, None);
    }
    if let Some(fit) = circle {
        return (SnapKind::Circle, fit.center, Some(fit));
    }
    (SnapKind::Point, point, None)
}

/// The nearest of `candidates` to `cursor` within `radius` screen points,
/// or `None` when none is close enough. `candidates` are already projected.
pub(crate) fn nearest_within(
    cursor: egui::Pos2,
    candidates: &[(egui::Pos2, DVec3, SnapKind)],
    radius: f32,
) -> Option<(SnapKind, DVec3)> {
    candidates
        .iter()
        .map(|(screen, world, kind)| ((*screen - cursor).length(), *kind, *world))
        .filter(|(distance, _, _)| *distance <= radius)
        // Ties break on the kind, so a box corner beats the face centre it
        // shares a plane with rather than depending on iteration order.
        .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
        .map(|(_, kind, world)| (kind, world))
}

impl Tool {
    /// Whether this tool snaps. Snapping is a placement affordance: markers
    /// under the cursor while merely selecting would be noise.
    pub fn snaps(self) -> bool {
        matches!(self, Tool::PlaceJoint | Tool::Align)
    }
}

impl RiggenApp {
    /// The snap target under the cursor, if any.
    pub fn snap(&self) -> Option<SnapCandidate> {
        self.snap_candidate
    }

    /// Recomputes the snap target for this frame. Called before the overlay
    /// is built, from the same place the glyph hover is resolved.
    pub(crate) fn update_snap(&mut self, ctx: &egui::Context) {
        self.snap_candidate = self.tool.snaps().then(|| self.compute_snap(ctx)).flatten();
        if self.snap_candidate.is_none() {
            self.snap_cache.key = None;
        }
    }

    fn compute_snap(&mut self, ctx: &egui::Context) -> Option<SnapCandidate> {
        // Nothing behind the toolbar, the gizmo or a modal is being pointed
        // at, and a click there must not place anything either.
        if self.gizmo_state.captured || self.pending.is_some() {
            return None;
        }
        let hit = self.viewport.hovered()?;
        let cursor = ctx.pointer_hover_pos()?;
        if self.toolbar_rect.is_some_and(|r| r.contains(cursor)) {
            return None;
        }
        let link = self.link_of_instance(hit.instance)?;
        let geom = self.geom_of_instance(hit.instance)?;
        let mesh_id = self
            .robot
            .links
            .get(&link)?
            .visuals
            .iter()
            .find(|g| g.id == geom)?
            .mesh;
        let model = self
            .viewport
            .instance_states()
            .find(|s| s.id == hit.instance)?
            .model;
        let triangle = hit.triangle as usize;

        // The exact point, from the one triangle the ID buffer chose: the
        // ray is taken into mesh space so no spatial index is needed.
        let ray = self.viewport.cursor_ray(cursor)?;
        let inverse = model.inverse();
        let local_ray = Ray {
            origin: inverse.transform_point3(ray.origin),
            dir: inverse.transform_vector3(ray.dir),
        };
        let corners = {
            let loaded = self.mesh_store.get(&mesh_id)?;
            (triangle < loaded.mesh.triangle_count()).then(|| loaded.mesh.triangle(triangle))?
        };
        let t = ray_triangle(&local_ray, &corners)?;
        let hit_point = model.transform_point3(local_ray.at(t));
        let normal = model
            .transform_vector3((corners[1] - corners[0]).cross(corners[2] - corners[0]))
            .normalize_or_zero();

        // Vertex: a corner of that triangle, in screen space.
        let vertex = {
            let projected: Vec<_> = corners
                .iter()
                .filter_map(|c| {
                    let world = model.transform_point3(*c);
                    Some((self.viewport.project(world)?, world, SnapKind::Vertex))
                })
                .collect();
            nearest_within(cursor, &projected, SNAP_PIXEL_RADIUS).map(|(_, at)| at)
        };

        // Box: the instance's own bounds, corners and face centres.
        let boxed = {
            let bounds = self
                .viewport
                .instance_states()
                .find(|s| s.id == hit.instance)
                .and_then(|s| s.bounds)
                .map(|b| b.transformed(&model));
            let projected: Vec<_> = bounds
                .iter()
                .flat_map(box_targets)
                .filter_map(|(world, kind)| Some((self.viewport.project(world)?, world, kind)))
                .collect();
            nearest_within(cursor, &projected, SNAP_PIXEL_RADIUS)
        };

        let circle = self
            .cached_circle(mesh_id, hit.instance, hit.triangle)
            .map(|fit| world_circle(&fit, &model));

        let (kind, point, circle) = choose(vertex, boxed, circle, hit_point);
        Some(SnapCandidate {
            kind,
            point,
            normal,
            circle,
            hit: hit_point,
            link,
            instance: hit.instance,
            triangle: hit.triangle,
        })
    }

    /// The mesh-local circle fit for one `(instance, triangle)`, computed
    /// once and kept until the cursor moves to another facet. The welded
    /// adjacency is cached beside the loaded mesh, so only the fit itself
    /// is repeated even when the triangle changes.
    fn cached_circle(
        &mut self,
        mesh: riggen_core::MeshId,
        instance: InstanceId,
        triangle: u32,
    ) -> Option<CircleFit> {
        if self.snap_cache.key == Some((instance, triangle)) {
            return self.snap_cache.circle;
        }
        let loaded = self.mesh_store.get_mut(&mesh)?;
        let fit = riggen_mesh::feature::fit_circle_with(
            &loaded.mesh.clone(),
            loaded.adjacency(),
            triangle as usize,
        );
        self.snap_cache.key = Some((instance, triangle));
        self.snap_cache.circle = fit;
        fit
    }

    /// The snap marker and its readout, appended to the glyph overlay.
    pub(crate) fn push_snap_overlay(&self, overlay: &mut Overlay) {
        let Some(snap) = self.snap_candidate else {
            return;
        };
        if let Some(fit) = snap.circle {
            let start = fit.axis.any_orthonormal_vector();
            overlay.push(OverlayItem::Arc {
                center: fit.center,
                axis: fit.axis,
                start,
                radius: fit.radius,
                sweep: std::f64::consts::TAU,
                color: SNAP_COLOR,
                width: 2.0,
            });
            // A stub of the axis, so which way "click the bore" would point
            // the joint is visible before committing to it.
            let half = fit.axis * fit.radius * 2.0;
            overlay.segment(fit.center - half, fit.center + half, SNAP_COLOR, 1.5);
        }
        overlay.point(snap.point, 5.0, SNAP_COLOR);
        overlay.label(
            snap.point,
            snap.readout(),
            SNAP_COLOR,
            egui::vec2(10.0, -6.0),
        );
    }
}

/// What the status bar says after a placement, so a test can assert on the
/// gesture's outcome rather than on prose.
pub fn placed_status(joint: &str, snap: &SnapCandidate) -> String {
    format!("placed {joint} on {}", snap.readout())
}

impl RiggenApp {
    /// Puts the selected joint's frame on the snapped feature: one
    /// `MoveJointFrame`, so nothing in the world moves and only the pivot
    /// does (plans/m2-placement-ux step 8).
    ///
    /// What the feature offers depends on what it is. A **circle** gives
    /// both an origin (its centre) and an axis (its own); a plain **point**
    /// on a face gives the hit and the face normal; a **vertex** or a box
    /// corner gives a position and nothing about direction, so the axis is
    /// left alone — inventing one from a corner would be a guess the user
    /// cannot see.
    pub fn place_joint(&mut self, joint: JointId, snap: &SnapCandidate) {
        let Some(current) = self.robot.joints.get(&joint).cloned() else {
            return;
        };
        let world = riggen_core::fk(&self.robot, &self.q);
        let Some(parent) = world.get(&current.parent).copied() else {
            return;
        };
        // The frame keeps its orientation and moves to the feature; the
        // axis is the feature's, re-expressed in that frame.
        let origin = Pose::new(
            parent.inverse().transform_point(snap.point),
            current.origin.r,
        );
        let axis = match snap.kind {
            SnapKind::Circle | SnapKind::Point => {
                let world_axis = snap.axis().normalize_or_zero();
                if world_axis == DVec3::ZERO {
                    current.axis
                } else {
                    (origin.r.inverse() * (parent.r.inverse() * world_axis)).normalize()
                }
            }
            SnapKind::Vertex | SnapKind::BoxCorner | SnapKind::BoxFaceCenter => current.axis,
        };
        if self
            .apply(riggen_core::Command::MoveJointFrame {
                joint,
                origin,
                axis,
            })
            .is_ok()
        {
            self.status = Some(placed_status(&current.name, snap));
        }
    }
}

/// The eight corners and six face centres of `bounds`, tagged with the kind
/// they snap as.
fn box_targets(bounds: &riggen_mesh::Aabb) -> Vec<(DVec3, SnapKind)> {
    let (min, max, center) = (bounds.min, bounds.max, bounds.center());
    let mut out = Vec::with_capacity(14);
    for i in 0..8 {
        out.push((
            DVec3::new(
                if i & 1 == 0 { min.x } else { max.x },
                if i & 2 == 0 { min.y } else { max.y },
                if i & 4 == 0 { min.z } else { max.z },
            ),
            SnapKind::BoxCorner,
        ));
    }
    for (axis, low, high) in [
        (DVec3::X, min.x, max.x),
        (DVec3::Y, min.y, max.y),
        (DVec3::Z, min.z, max.z),
    ] {
        for at in [low, high] {
            out.push((
                center + axis * (at - center.dot(axis)),
                SnapKind::BoxFaceCenter,
            ));
        }
    }
    out
}

/// A mesh-local fit put into the world. `model` is rigid — the asset's
/// scale was baked into the mesh at load — so the radius and the residual
/// carry over untouched.
fn world_circle(fit: &CircleFit, model: &DMat4) -> CircleFit {
    CircleFit {
        center: model.transform_point3(fit.center),
        axis: model.transform_vector3(fit.axis).normalize_or_zero(),
        ..*fit
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::pos2;
    use riggen_core::Id;

    fn fit(radius: f64) -> CircleFit {
        CircleFit {
            center: DVec3::new(1.0, 2.0, 3.0),
            axis: DVec3::Z,
            radius,
            residual: 1e-5,
            segments: 24,
        }
    }

    #[test]
    fn the_ladder_prefers_vertex_then_box_then_circle_then_point() {
        let point = DVec3::new(9.0, 9.0, 9.0);
        let vertex = DVec3::X;
        let corner = DVec3::Y;

        let (kind, at, circle) = choose(
            Some(vertex),
            Some((SnapKind::BoxCorner, corner)),
            Some(fit(0.5)),
            point,
        );
        assert_eq!((kind, at), (SnapKind::Vertex, vertex));
        assert_eq!(circle, None, "a vertex is not a circle");

        let (kind, at, _) = choose(
            None,
            Some((SnapKind::BoxFaceCenter, corner)),
            Some(fit(0.5)),
            point,
        );
        assert_eq!((kind, at), (SnapKind::BoxFaceCenter, corner));

        let (kind, at, circle) = choose(None, None, Some(fit(0.5)), point);
        assert_eq!(kind, SnapKind::Circle);
        assert_eq!(at, fit(0.5).center, "a circle snaps to its centre");
        assert_eq!(circle, Some(fit(0.5)));

        let (kind, at, circle) = choose(None, None, None, point);
        assert_eq!((kind, at, circle), (SnapKind::Point, point, None));
    }

    #[test]
    fn nearest_within_respects_the_pixel_radius_and_breaks_ties_by_kind() {
        let cursor = pos2(100.0, 100.0);
        let near = (pos2(104.0, 100.0), DVec3::X, SnapKind::BoxCorner);
        let far = (pos2(100.0, 130.0), DVec3::Y, SnapKind::BoxCorner);
        assert_eq!(
            nearest_within(cursor, &[far, near], SNAP_PIXEL_RADIUS),
            Some((SnapKind::BoxCorner, DVec3::X)),
            "the nearer one wins whatever the order"
        );
        assert_eq!(nearest_within(cursor, &[far], SNAP_PIXEL_RADIUS), None);
        // Exactly on the radius still counts; a hair past does not.
        let edge = (
            pos2(100.0 + SNAP_PIXEL_RADIUS, 100.0),
            DVec3::Z,
            SnapKind::Vertex,
        );
        assert!(nearest_within(cursor, &[edge], SNAP_PIXEL_RADIUS).is_some());
        assert!(nearest_within(cursor, &[edge], SNAP_PIXEL_RADIUS - 0.01).is_none());
        // A corner and a face centre at the same distance: the corner wins.
        let corner = (pos2(103.0, 100.0), DVec3::X, SnapKind::BoxCorner);
        let face = (pos2(97.0, 100.0), DVec3::Y, SnapKind::BoxFaceCenter);
        assert_eq!(
            nearest_within(cursor, &[face, corner], SNAP_PIXEL_RADIUS),
            Some((SnapKind::BoxCorner, DVec3::X))
        );
        assert_eq!(nearest_within(cursor, &[], SNAP_PIXEL_RADIUS), None);
    }

    #[test]
    fn a_box_offers_eight_corners_and_six_face_centres() {
        let bounds = riggen_mesh::Aabb {
            min: DVec3::splat(-1.0),
            max: DVec3::new(1.0, 3.0, 1.0),
        };
        let targets = box_targets(&bounds);
        assert_eq!(targets.len(), 14);
        assert_eq!(
            targets
                .iter()
                .filter(|(_, k)| *k == SnapKind::BoxCorner)
                .count(),
            8
        );
        assert!(targets.contains(&(DVec3::new(-1.0, -1.0, -1.0), SnapKind::BoxCorner)));
        assert!(targets.contains(&(DVec3::new(1.0, 3.0, 1.0), SnapKind::BoxCorner)));
        // Face centres sit on the face, centred in the other two axes.
        assert!(targets.contains(&(DVec3::new(1.0, 1.0, 0.0), SnapKind::BoxFaceCenter)));
        assert!(targets.contains(&(DVec3::new(0.0, 3.0, 0.0), SnapKind::BoxFaceCenter)));
    }

    #[test]
    fn the_readout_is_millimetres_and_names_the_kind() {
        let mut snap = SnapCandidate {
            kind: SnapKind::Circle,
            point: DVec3::ZERO,
            normal: DVec3::Z,
            circle: Some(CircleFit {
                radius: 0.012,
                residual: 1e-5,
                segments: 24,
                ..fit(0.012)
            }),
            hit: DVec3::ZERO,
            link: LinkId::from_raw(1),
            instance: InstanceId(0),
            triangle: 0,
        };
        assert_eq!(snap.readout(), "circle r 12.0 mm · 24 seg · res 0.01 mm");
        assert_eq!(snap.axis(), DVec3::Z);
        snap.kind = SnapKind::Vertex;
        snap.circle = None;
        assert_eq!(snap.readout(), "vertex");
        snap.normal = DVec3::Y;
        assert_eq!(snap.axis(), DVec3::Y, "no circle: the face normal");
    }

    #[test]
    fn only_the_placement_tools_snap() {
        assert!(Tool::PlaceJoint.snaps() && Tool::Align.snaps());
        for tool in [Tool::Select, Tool::Move, Tool::Rotate] {
            assert!(!tool.snaps(), "{}", tool.label());
        }
    }
}
