//! The chamfered cube's 26 facets — one per [`ViewOrientation`].
//!
//! Ported from RoboCAD's `robocad-ui/src/viewcube/facets.rs` (ADR-0028 §3,
//! ADR-0001): cgmath to glam, and `robocad_viewport::ViewOrientation` to
//! `riggen_viewport`'s, which is the same enum with the same variants —
//! `orientation.rs` has held all 26 since M0 with a comment saying they
//! were kept whole for exactly this.
//!
//! The cube is a unit cube with its edges and corners cut off, so every
//! canonical view is a *surface* the pointer can land on rather than a
//! keystroke: 6 face squares, 12 edge quads, 8 corner triangles. Geometry
//! only — nothing here projects, paints, or reads the pointer.

use riggen_core::glam::Vec3;
use riggen_viewport::ViewOrientation;

/// Inset parameter from the outer unit bounding cube [±1, ±1, ±1].
/// Controls the width of edge chamfers and corner triangles.
pub const CHAMFER_INSET: f32 = 0.28;

/// Half-extent of each primary face square: `1.0 - CHAMFER_INSET` (0.72).
pub const FACE_EXTENT: f32 = 1.0 - CHAMFER_INSET;

/// A 3D convex polygon facet on the chamfered cube with outward normal and vertices in CCW order.
#[derive(Debug, Clone)]
pub struct Facet3D {
    pub orientation: ViewOrientation,
    pub vertices: Vec<Vec3>,
    pub normal: Vec3,
    pub center: Vec3,
}

impl Facet3D {
    pub fn new(orientation: ViewOrientation, vertices: Vec<Vec3>) -> Self {
        let n = vertices.len() as f32;
        let mut sum = Vec3::ZERO;
        for v in &vertices {
            sum += *v;
        }
        let center = sum / n;
        let normal = orientation.normal();
        Self {
            orientation,
            vertices,
            normal,
            center,
        }
    }
}

/// Generates all 26 convex polygon facets (6 quad faces, 12 quad edges, 8 triangle corners)
/// for the 3D chamfered cube centered at the origin.
pub fn chamfered_cube_facets() -> Vec<Facet3D> {
    let a = FACE_EXTENT;
    vec![
        // ---------------------------------------------------------------------
        // 1. 6 Primary Face Squares (4 vertices each, CCW viewed from outside)
        // ---------------------------------------------------------------------

        // Front (+Y)
        Facet3D::new(
            ViewOrientation::Front,
            vec![
                Vec3::new(-a, 1.0, -a),
                Vec3::new(-a, 1.0, a),
                Vec3::new(a, 1.0, a),
                Vec3::new(a, 1.0, -a),
            ],
        ),
        // Back (-Y)
        Facet3D::new(
            ViewOrientation::Back,
            vec![
                Vec3::new(-a, -1.0, -a),
                Vec3::new(a, -1.0, -a),
                Vec3::new(a, -1.0, a),
                Vec3::new(-a, -1.0, a),
            ],
        ),
        // Left (-X)
        Facet3D::new(
            ViewOrientation::Left,
            vec![
                Vec3::new(-1.0, -a, -a),
                Vec3::new(-1.0, -a, a),
                Vec3::new(-1.0, a, a),
                Vec3::new(-1.0, a, -a),
            ],
        ),
        // Right (+X)
        Facet3D::new(
            ViewOrientation::Right,
            vec![
                Vec3::new(1.0, -a, -a),
                Vec3::new(1.0, a, -a),
                Vec3::new(1.0, a, a),
                Vec3::new(1.0, -a, a),
            ],
        ),
        // Top (+Z)
        Facet3D::new(
            ViewOrientation::Top,
            vec![
                Vec3::new(-a, -a, 1.0),
                Vec3::new(a, -a, 1.0),
                Vec3::new(a, a, 1.0),
                Vec3::new(-a, a, 1.0),
            ],
        ),
        // Bottom (-Z)
        Facet3D::new(
            ViewOrientation::Bottom,
            vec![
                Vec3::new(-a, -a, -1.0),
                Vec3::new(-a, a, -1.0),
                Vec3::new(a, a, -1.0),
                Vec3::new(a, -a, -1.0),
            ],
        ),
        // ---------------------------------------------------------------------
        // 2. 12 Chamfered Edge Quads (4 vertices each, CCW viewed from outside)
        // ---------------------------------------------------------------------

        // FrontTop (+Y, +Z)
        Facet3D::new(
            ViewOrientation::FrontTop,
            vec![
                Vec3::new(-a, 1.0, a),
                Vec3::new(-a, a, 1.0),
                Vec3::new(a, a, 1.0),
                Vec3::new(a, 1.0, a),
            ],
        ),
        // FrontBottom (+Y, -Z)
        Facet3D::new(
            ViewOrientation::FrontBottom,
            vec![
                Vec3::new(-a, 1.0, -a),
                Vec3::new(a, 1.0, -a),
                Vec3::new(a, a, -1.0),
                Vec3::new(-a, a, -1.0),
            ],
        ),
        // FrontLeft (-X, +Y)
        Facet3D::new(
            ViewOrientation::FrontLeft,
            vec![
                Vec3::new(-a, 1.0, -a),
                Vec3::new(-1.0, a, -a),
                Vec3::new(-1.0, a, a),
                Vec3::new(-a, 1.0, a),
            ],
        ),
        // FrontRight (+X, +Y)
        Facet3D::new(
            ViewOrientation::FrontRight,
            vec![
                Vec3::new(a, 1.0, -a),
                Vec3::new(a, 1.0, a),
                Vec3::new(1.0, a, a),
                Vec3::new(1.0, a, -a),
            ],
        ),
        // BackTop (-Y, +Z)
        Facet3D::new(
            ViewOrientation::BackTop,
            vec![
                Vec3::new(-a, -1.0, a),
                Vec3::new(a, -1.0, a),
                Vec3::new(a, -a, 1.0),
                Vec3::new(-a, -a, 1.0),
            ],
        ),
        // BackBottom (-Y, -Z)
        Facet3D::new(
            ViewOrientation::BackBottom,
            vec![
                Vec3::new(-a, -1.0, -a),
                Vec3::new(-a, -a, -1.0),
                Vec3::new(a, -a, -1.0),
                Vec3::new(a, -1.0, -a),
            ],
        ),
        // BackLeft (-X, -Y)
        Facet3D::new(
            ViewOrientation::BackLeft,
            vec![
                Vec3::new(-a, -1.0, -a),
                Vec3::new(-a, -1.0, a),
                Vec3::new(-1.0, -a, a),
                Vec3::new(-1.0, -a, -a),
            ],
        ),
        // BackRight (+X, -Y)
        Facet3D::new(
            ViewOrientation::BackRight,
            vec![
                Vec3::new(a, -1.0, -a),
                Vec3::new(1.0, -a, -a),
                Vec3::new(1.0, -a, a),
                Vec3::new(a, -1.0, a),
            ],
        ),
        // TopLeft (-X, +Z)
        Facet3D::new(
            ViewOrientation::TopLeft,
            vec![
                Vec3::new(-a, -a, 1.0),
                Vec3::new(-a, a, 1.0),
                Vec3::new(-1.0, a, a),
                Vec3::new(-1.0, -a, a),
            ],
        ),
        // TopRight (+X, +Z)
        Facet3D::new(
            ViewOrientation::TopRight,
            vec![
                Vec3::new(a, -a, 1.0),
                Vec3::new(1.0, -a, a),
                Vec3::new(1.0, a, a),
                Vec3::new(a, a, 1.0),
            ],
        ),
        // BottomLeft (-X, -Z)
        Facet3D::new(
            ViewOrientation::BottomLeft,
            vec![
                Vec3::new(-a, -a, -1.0),
                Vec3::new(-1.0, -a, -a),
                Vec3::new(-1.0, a, -a),
                Vec3::new(-a, a, -1.0),
            ],
        ),
        // BottomRight (+X, -Z)
        Facet3D::new(
            ViewOrientation::BottomRight,
            vec![
                Vec3::new(a, -a, -1.0),
                Vec3::new(a, a, -1.0),
                Vec3::new(1.0, a, -a),
                Vec3::new(1.0, -a, -a),
            ],
        ),
        // ---------------------------------------------------------------------
        // 3. 8 Chamfered Corner Triangles (3 vertices each, CCW viewed from outside)
        // ---------------------------------------------------------------------

        // FrontTopLeft (-X, +Y, +Z)
        Facet3D::new(
            ViewOrientation::FrontTopLeft,
            vec![
                Vec3::new(-a, a, 1.0),
                Vec3::new(-a, 1.0, a),
                Vec3::new(-1.0, a, a),
            ],
        ),
        // FrontTopRight (+X, +Y, +Z)
        Facet3D::new(
            ViewOrientation::FrontTopRight,
            vec![
                Vec3::new(a, a, 1.0),
                Vec3::new(1.0, a, a),
                Vec3::new(a, 1.0, a),
            ],
        ),
        // FrontBottomLeft (-X, +Y, -Z)
        Facet3D::new(
            ViewOrientation::FrontBottomLeft,
            vec![
                Vec3::new(-a, a, -1.0),
                Vec3::new(-1.0, a, -a),
                Vec3::new(-a, 1.0, -a),
            ],
        ),
        // FrontBottomRight (+X, +Y, -Z)
        Facet3D::new(
            ViewOrientation::FrontBottomRight,
            vec![
                Vec3::new(a, a, -1.0),
                Vec3::new(a, 1.0, -a),
                Vec3::new(1.0, a, -a),
            ],
        ),
        // BackTopLeft (-X, -Y, +Z)
        Facet3D::new(
            ViewOrientation::BackTopLeft,
            vec![
                Vec3::new(-a, -a, 1.0),
                Vec3::new(-1.0, -a, a),
                Vec3::new(-a, -1.0, a),
            ],
        ),
        // BackTopRight (+X, -Y, +Z)
        Facet3D::new(
            ViewOrientation::BackTopRight,
            vec![
                Vec3::new(a, -a, 1.0),
                Vec3::new(a, -1.0, a),
                Vec3::new(1.0, -a, a),
            ],
        ),
        // BackBottomLeft (-X, -Y, -Z)
        Facet3D::new(
            ViewOrientation::BackBottomLeft,
            vec![
                Vec3::new(-a, -a, -1.0),
                Vec3::new(-a, -1.0, -a),
                Vec3::new(-1.0, -a, -a),
            ],
        ),
        // BackBottomRight (+X, -Y, -Z)
        Facet3D::new(
            ViewOrientation::BackBottomRight,
            vec![
                Vec3::new(a, -a, -1.0),
                Vec3::new(1.0, -a, -a),
                Vec3::new(a, -1.0, -a),
            ],
        ),
    ]
}
