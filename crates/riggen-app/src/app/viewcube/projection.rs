//! Projecting the cube: the camera basis, the painter's-order sort, the
//! backface cull, the hit test, and the text embedded in a face's plane.
//!
//! Ported from RoboCAD (ADR-0028 §3), cgmath to glam. There is no
//! perspective divide anywhere here: the cube is drawn in a fixed
//! orthographic projection of its own, like the axes triad, so that it is
//! the same size in the corner whatever the scene's camera is doing — it
//! takes the scene camera's *orientation* and nothing else.

use riggen_core::glam::Vec3;
use riggen_viewport::{OrbitCamera, ViewOrientation};

use super::facets::{FACE_EXTENT, chamfered_cube_facets};

/// A 2D projected facet ready for sorting, culling, hit-testing, and rendering.
#[derive(Debug, Clone)]
pub struct ProjectedFacet {
    pub orientation: ViewOrientation,
    pub vertices_2d: Vec<egui::Pos2>,
    pub normal_3d: Vec3,
    pub center_2d: egui::Pos2,
    pub depth: f32,
}

/// The camera basis the cube is projected through: `(right, up, eye_dir)`,
/// where `eye_dir` points from the target *towards* the eye.
///
/// RoboCAD recomputed this from yaw and pitch; here it is
/// [`OrbitCamera::basis`] itself, so the pole heuristic that stands Y in
/// for Z at the exact top and bottom views has one home and the cube can
/// never disagree with the scene about which way is up. Target and
/// distance do not enter into a direction, so the borrowed camera's
/// defaults are irrelevant.
pub fn camera_basis(yaw: f32, pitch: f32) -> (Vec3, Vec3, Vec3) {
    let camera = OrbitCamera {
        yaw,
        pitch,
        ..Default::default()
    };
    let (forward, right, up) = camera.basis();
    (right, up, -forward)
}

/// Projects the 3D chamfered cube into 2D screen space inside `rect`,
/// performing backface culling and depth sorting.
///
/// Returns the visible facets sorted from back-to-front (painter's algorithm order).
pub fn project_viewcube(rect: egui::Rect, yaw: f32, pitch: f32) -> Vec<ProjectedFacet> {
    let (cam_right, cam_up, eye_dir) = camera_basis(yaw, pitch);
    let center = rect.center();
    // Fit comfortably inside the widget rect
    let radius = rect.width().min(rect.height()) * 0.5;
    let scale = radius / 1.75;

    let facets = chamfered_cube_facets();
    let mut projected = Vec::with_capacity(facets.len());

    for facet in facets {
        let view_dot = facet.normal.dot(eye_dir);
        // Backface culling: only facets facing the camera (positive dot product) are visible
        if view_dot <= 0.001 {
            continue;
        }

        let depth = facet.center.dot(eye_dir);
        let mut vertices_2d = Vec::with_capacity(facet.vertices.len());
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;

        for v in &facet.vertices {
            let x_cam = v.dot(cam_right);
            let y_cam = v.dot(cam_up);
            // In egui, +Y is DOWN on the screen
            let p2 = egui::pos2(center.x + x_cam * scale, center.y - y_cam * scale);
            sum_x += p2.x;
            sum_y += p2.y;
            vertices_2d.push(p2);
        }

        let n = vertices_2d.len() as f32;
        let center_2d = egui::pos2(sum_x / n, sum_y / n);

        projected.push(ProjectedFacet {
            orientation: facet.orientation,
            vertices_2d,
            normal_3d: facet.normal,
            center_2d,
            depth,
        });
    }

    // Sort back-to-front by depth (ascending order: lowest depth drawn first)
    projected.sort_by(|a, b| {
        a.depth
            .partial_cmp(&b.depth)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    projected
}

/// Tests whether a 2D point is inside a convex polygon defined by `vertices` (in CW or CCW order).
pub fn point_in_polygon_2d(point: egui::Pos2, vertices: &[egui::Pos2]) -> bool {
    if vertices.len() < 3 {
        return false;
    }
    let mut has_pos = false;
    let mut has_neg = false;
    let n = vertices.len();
    for i in 0..n {
        let v0 = vertices[i];
        let v1 = vertices[(i + 1) % n];
        let cross = (v1.x - v0.x) * (point.y - v0.y) - (v1.y - v0.y) * (point.x - v0.x);
        if cross > 1e-4 {
            has_pos = true;
        } else if cross < -1e-4 {
            has_neg = true;
        }
        if has_pos && has_neg {
            return false;
        }
    }
    true
}

/// Hit-tests a 2D screen coordinate against front-facing projected facets.
///
/// Facets in `facets` are expected in painter's order (ascending depth). Hit-testing
/// iterates in reverse order (highest depth / frontmost first) to pick the closest facet.
pub fn hit_test_viewcube(facets: &[ProjectedFacet], pos: egui::Pos2) -> Option<ViewOrientation> {
    for facet in facets.iter().rev() {
        if point_in_polygon_2d(pos, &facet.vertices_2d) {
            return Some(facet.orientation);
        }
    }
    None
}

/// Canonical in-plane reference directions (u = right, v = up) in 3D for each primary face.
///
/// Matches standard CAD face orientations (Z-up, +Y Front, +X Right) and viewport camera basis:
/// - Front (+Y):  U = -X (-1, 0, 0), V = +Z (0, 0, 1)
/// - Back (-Y):   U = +X (1, 0, 0),  V = +Z (0, 0, 1)
/// - Left (-X):   U = -Y (0, -1, 0), V = +Z (0, 0, 1)
/// - Right (+X):  U = +Y (0, 1, 0),  V = +Z (0, 0, 1)
/// - Top (+Z):    U = +X (1, 0, 0),  V = +Y (0, 1, 0)
/// - Bottom (-Z): U = -X (-1, 0, 0), V = +Y (0, 1, 0)
pub fn face_local_axes_3d(orientation: ViewOrientation) -> (Vec3, Vec3) {
    match orientation {
        ViewOrientation::Front => (Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        ViewOrientation::Back => (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        ViewOrientation::Left => (Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        ViewOrientation::Right => (Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        ViewOrientation::Top => (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
        ViewOrientation::Bottom => (Vec3::new(-1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
        _ => (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
    }
}

/// Projects 3D glyph vertices for a face label onto the 3D plane of the face
/// and transforms them directly to 2D screen space, emitting an `egui::Mesh`.
///
/// This provides true 3D embedded typography that naturally rotates, skews,
/// foreshortens, and scales with the cube's surfaces in real time.
#[allow(clippy::too_many_arguments)]
pub fn project_face_text_mesh(
    painter: &egui::Painter,
    orientation: ViewOrientation,
    label: &str,
    font_id: egui::FontId,
    color: egui::Color32,
    cam_right: Vec3,
    cam_up: Vec3,
    widget_center: egui::Pos2,
    scale: f32,
) -> Option<egui::Mesh> {
    let (u_axis, v_axis) = face_local_axes_3d(orientation);
    let normal_3d = orientation.normal();
    let face_center_3d = normal_3d; // Primary face centers are at distance 1.0 along normal

    let galley = painter.layout_no_wrap(label.to_string(), font_id, color);
    let galley_size = galley.size();
    if galley_size.x <= 0.0 || galley_size.y <= 0.0 {
        return None;
    }

    let font_image_size = painter.fonts(|f| f.font_image_size());
    let font_w = font_image_size[0] as f32;
    let font_h = font_image_size[1] as f32;
    if font_w <= 0.0 || font_h <= 0.0 {
        return None;
    }

    // Base scale factor mapping font points to 3D cube units
    let base_s = 1.0 / scale;
    // Ensure text fits comfortably within the face square extent
    let max_text_width_3d = FACE_EXTENT * 2.0 * 0.82;
    let text_width_3d = galley_size.x * base_s;
    let s = if text_width_3d > max_text_width_3d {
        base_s * (max_text_width_3d / text_width_3d)
    } else {
        base_s
    };

    let galley_center_x = galley_size.x * 0.5;
    let galley_center_y = galley_size.y * 0.5;

    // Small normal offset to prevent coplanar z-fighting with the face polygon
    let normal_offset = normal_3d * 0.005;
    let face_pos_3d = face_center_3d + normal_offset;

    let mut mesh = egui::Mesh {
        texture_id: egui::TextureId::default(),
        ..Default::default()
    };

    for row in &galley.rows {
        for glyph in &row.glyphs {
            if glyph.uv_rect.is_nothing() {
                continue;
            }

            let top_left_2d = glyph.pos + glyph.uv_rect.offset;
            let glyph_w = glyph.uv_rect.size.x;
            let glyph_h = glyph.uv_rect.size.y;

            let uv_min = glyph.uv_rect.min;
            let uv_max = glyph.uv_rect.max;
            let uv_tl = egui::pos2(uv_min[0] as f32 / font_w, uv_min[1] as f32 / font_h);
            let uv_tr = egui::pos2(uv_max[0] as f32 / font_w, uv_min[1] as f32 / font_h);
            let uv_br = egui::pos2(uv_max[0] as f32 / font_w, uv_max[1] as f32 / font_h);
            let uv_bl = egui::pos2(uv_min[0] as f32 / font_w, uv_max[1] as f32 / font_h);

            // 4 corners in 2D galley space
            let corners_galley = [
                egui::pos2(top_left_2d.x, top_left_2d.y),
                egui::pos2(top_left_2d.x + glyph_w, top_left_2d.y),
                egui::pos2(top_left_2d.x + glyph_w, top_left_2d.y + glyph_h),
                egui::pos2(top_left_2d.x, top_left_2d.y + glyph_h),
            ];
            let uvs = [uv_tl, uv_tr, uv_br, uv_bl];

            let idx_base = mesh.vertices.len() as u32;

            for i in 0..4 {
                let p_galley = corners_galley[i];
                let x_rel = p_galley.x - galley_center_x;
                let y_rel = p_galley.y - galley_center_y;

                // 3D point on face plane
                let p_3d = face_pos_3d + (x_rel * s) * u_axis - (y_rel * s) * v_axis;

                // Project to screen space
                let x_cam = p_3d.dot(cam_right);
                let y_cam = p_3d.dot(cam_up);
                let p_screen = egui::pos2(
                    widget_center.x + x_cam * scale,
                    widget_center.y - y_cam * scale,
                );

                mesh.vertices.push(egui::epaint::Vertex {
                    pos: p_screen,
                    uv: uvs[i],
                    color,
                });
            }

            mesh.indices.extend_from_slice(&[
                idx_base,
                idx_base + 1,
                idx_base + 2,
                idx_base,
                idx_base + 2,
                idx_base + 3,
            ]);
        }
    }

    if mesh.vertices.is_empty() {
        None
    } else {
        Some(mesh)
    }
}
