//! The interactive cube: what it paints, and what the pointer does to it.
//!
//! Ported from RoboCAD (ADR-0028 §3) with `is_sketch_mode` dropped — riggen
//! has no sketch mode to dim for — and the return widened to carry the
//! rect the widget actually occupies, which is what `chrome_rects` needs so
//! the camera holds still and the picks are suppressed under it (ADR-0021).

use riggen_core::glam::Vec3;
use riggen_viewport::{Projection, ViewOrientation};

use super::projection::{
    camera_basis, hit_test_viewcube, project_face_text_mesh, project_viewcube,
};

/// What the pointer asked the cube for. Every variant is a call the camera
/// already has (ADR-0028 §3); the cube invents no new camera operation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ViewCubeAction {
    /// Animate to a canonical orientation — a facet was clicked.
    Select(ViewOrientation),
    /// Orbit by these deltas, in radians — the cube itself was dragged.
    Orbit { delta_yaw: f32, delta_pitch: f32 },
    /// Re-frame the scene at the home orientation.
    Home,
    /// Perspective ↔ orthographic.
    ToggleProjection,
}

/// What one `viewcube` call did and where it drew.
pub struct ViewCubeOutput {
    pub action: Option<ViewCubeAction>,
    /// The cube and its projection button together: the rect to register in
    /// `chrome_rects`.
    pub rect: egui::Rect,
}

/// Draws the cube in `rect` with its projection button below it, and reads
/// the pointer over both.
///
/// `yaw` and `pitch` are the scene camera's; the cube shows the *same*
/// orientation, which is the whole of what it is for — the face towards you
/// on the cube is the face towards you in the viewport.
pub fn viewcube(
    ui: &mut egui::Ui,
    rect: egui::Rect,
    yaw: f32,
    pitch: f32,
    projection: Projection,
) -> ViewCubeOutput {
    let (cam_right, cam_up, eye_dir) = camera_basis(yaw, pitch);
    let visible_facets = project_viewcube(rect, yaw, pitch);
    let radius = rect.width().min(rect.height()) * 0.5;
    let scale = radius / 1.75;

    let home_size = (rect.width() * 0.16).clamp(18.0, 24.0);
    let home_rect = egui::Rect::from_min_size(
        egui::pos2(rect.min.x + 2.0, rect.min.y + 2.0),
        egui::vec2(home_size, home_size),
    );
    let home_id = ui.auto_id_with("riggen_viewcube_home");
    let home_response = ui.interact(home_rect, home_id, egui::Sense::click());

    let proj_rect = projection_button_rect(rect);
    let proj_id = ui.auto_id_with("riggen_viewcube_projection");
    let proj_response = ui.interact(proj_rect, proj_id, egui::Sense::click());

    let id = ui.auto_id_with("riggen_viewcube");
    let response = ui.interact(rect, id, egui::Sense::click_and_drag());

    home_response
        .clone()
        .on_hover_text("Home — default view, fit all (Home)");
    let proj_tooltip = match projection {
        Projection::Perspective => "Switch to orthographic (P)",
        Projection::Orthographic => "Switch to perspective (P)",
    };
    proj_response.clone().on_hover_text(proj_tooltip);

    let hover_pos = home_response
        .hover_pos()
        .or_else(|| proj_response.hover_pos())
        .or_else(|| response.hover_pos())
        .or_else(|| ui.input(|i| i.pointer.hover_pos()));
    let hovered_facet = hover_pos
        .filter(|p| rect.contains(*p) && !home_rect.contains(*p))
        .and_then(|pos| hit_test_viewcube(&visible_facets, pos));

    if response.dragged() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
    } else if home_response.hovered()
        || proj_response.hovered()
        || hover_pos.is_some_and(|p| home_rect.contains(p) || proj_rect.contains(p))
        || hovered_facet.is_some()
    {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }

    let clicked_pos = home_response
        .interact_pointer_pos()
        .or_else(|| proj_response.interact_pointer_pos())
        .or_else(|| response.interact_pointer_pos())
        .or(hover_pos);
    let is_home_click = home_response.clicked()
        || (response.clicked() && clicked_pos.is_some_and(|p| home_rect.contains(p)));
    let is_proj_click = proj_response.clicked()
        || (response.clicked() && clicked_pos.is_some_and(|p| proj_rect.contains(p)));

    let mut action = None;
    if is_proj_click {
        action = Some(ViewCubeAction::ToggleProjection);
    } else if is_home_click {
        action = Some(ViewCubeAction::Home);
    } else if response.dragged()
        && !hover_pos.is_some_and(|p| home_rect.contains(p) || proj_rect.contains(p))
    {
        let delta = response.drag_delta();
        if delta.x != 0.0 || delta.y != 0.0 {
            // The same radians-per-point the viewport's own orbit uses, so
            // dragging the cube and dragging the scene turn at one rate.
            action = Some(ViewCubeAction::Orbit {
                delta_yaw: -delta.x * 0.01,
                delta_pitch: delta.y * 0.01,
            });
        }
    } else if response.clicked()
        && let Some(pos) = clicked_pos
        && !home_rect.contains(pos)
        && !proj_rect.contains(pos)
        && let Some(orientation) = hit_test_viewcube(&visible_facets, pos)
    {
        action = Some(ViewCubeAction::Select(orientation));
    }

    let total_rect = rect.union(proj_rect);
    let painter = ui.painter_at(total_rect);
    // A fixed light in *camera* space, so the shading says which facet is
    // which and never changes as the cube turns.
    let light_dir_cam = Vec3::new(0.35, 0.55, 0.75).normalize();
    let dark_mode = ui.visuals().dark_mode;

    for facet in &visible_facets {
        let is_hovered = hovered_facet == Some(facet.orientation);

        let n_cam = Vec3::new(
            facet.normal_3d.dot(cam_right),
            facet.normal_3d.dot(cam_up),
            facet.normal_3d.dot(eye_dir),
        );
        let diffuse = n_cam.dot(light_dir_cam).max(0.0);
        let light_mult = 0.75 + 0.25 * diffuse;

        let fill_color = if is_hovered {
            if dark_mode {
                egui::Color32::from_rgb(55, 145, 240)
            } else {
                egui::Color32::from_rgb(65, 155, 250)
            }
        } else {
            let (base_r, base_g, base_b) = if dark_mode {
                if facet.orientation.is_face() {
                    (78.0, 86.0, 100.0)
                } else if facet.orientation.is_edge() {
                    (58.0, 65.0, 78.0)
                } else {
                    (48.0, 54.0, 66.0)
                }
            } else if facet.orientation.is_face() {
                (224.0, 230.0, 238.0)
            } else if facet.orientation.is_edge() {
                (205.0, 212.0, 222.0)
            } else {
                (192.0, 200.0, 210.0)
            };
            let channel = |base: f32| (base * light_mult).round().min(255.0) as u8;
            egui::Color32::from_rgb(channel(base_r), channel(base_g), channel(base_b))
        };

        // No stroke: the three shades are what separate a face from the
        // edge beside it, and an outline at this size reads as noise.
        painter.add(egui::epaint::PathShape::convex_polygon(
            facet.vertices_2d.clone(),
            fill_color,
            egui::Stroke::NONE,
        ));

        // The label lives in the face's own plane, so it foreshortens with
        // the face instead of floating in front of it.
        if facet.orientation.is_face()
            && let Some(label) = facet.orientation.label()
        {
            let label_color = if is_hovered {
                egui::Color32::WHITE
            } else if dark_mode {
                egui::Color32::from_rgb(225, 232, 242)
            } else {
                egui::Color32::from_rgb(45, 52, 64)
            };
            let font_size = (rect.width() * 0.115).clamp(9.0, 28.0);
            if let Some(mesh) = project_face_text_mesh(
                &painter,
                facet.orientation,
                label,
                egui::FontId::proportional(font_size),
                label_color,
                cam_right,
                cam_up,
                rect.center(),
                scale,
            ) {
                painter.add(egui::Shape::mesh(mesh));
            }
        }
    }

    paint_home_icon(
        &painter,
        home_rect,
        dark_mode,
        home_response.hovered() || hover_pos.is_some_and(|p| home_rect.contains(p)),
        response.hovered() || hover_pos.is_some_and(|p| rect.contains(p)),
    );
    paint_projection_button(
        &painter,
        proj_rect,
        projection,
        dark_mode,
        proj_response.hovered() || hover_pos.is_some_and(|p| proj_rect.contains(p)),
    );

    ViewCubeOutput {
        action,
        rect: total_rect,
    }
}

/// The projection button's rect for a cube drawn in `rect` — centred under
/// it. Public so the caller can reserve room for the whole block before it
/// decides where the cube goes.
pub fn projection_button_rect(rect: egui::Rect) -> egui::Rect {
    const MARGIN_TOP: f32 = 4.0;
    const HEIGHT: f32 = 20.0;
    let width = (rect.width() * 0.88).clamp(80.0, 116.0);
    egui::Rect::from_min_size(
        egui::pos2(rect.center().x - width * 0.5, rect.max.y + MARGIN_TOP),
        egui::vec2(width, HEIGHT),
    )
}

/// A filled house: roof, two jambs and a lintel, so the door is a cutout
/// rather than a second colour.
fn paint_home_icon(
    painter: &egui::Painter,
    home_rect: egui::Rect,
    dark_mode: bool,
    home_hovered: bool,
    cube_hovered: bool,
) {
    let color = if home_hovered {
        if dark_mode {
            egui::Color32::from_rgb(85, 175, 255)
        } else {
            egui::Color32::from_rgb(30, 120, 230)
        }
    } else if cube_hovered {
        if dark_mode {
            egui::Color32::from_rgb(225, 232, 242)
        } else {
            egui::Color32::from_rgb(50, 58, 70)
        }
    } else if dark_mode {
        egui::Color32::from_rgba_unmultiplied(165, 175, 190, 160)
    } else {
        egui::Color32::from_rgba_unmultiplied(100, 110, 125, 160)
    };

    let c = home_rect.center();
    painter.add(egui::epaint::PathShape::convex_polygon(
        vec![
            egui::pos2(c.x, c.y - 5.5),
            egui::pos2(c.x - 6.5, c.y - 0.2),
            egui::pos2(c.x + 6.5, c.y - 0.2),
        ],
        color,
        egui::Stroke::NONE,
    ));
    let body = |min: egui::Pos2, max: egui::Pos2| {
        painter.rect_filled(
            egui::Rect::from_min_max(min, max),
            egui::CornerRadius::ZERO,
            color,
        );
    };
    body(
        egui::pos2(c.x - 4.5, c.y - 0.2),
        egui::pos2(c.x - 1.4, c.y + 5.0),
    );
    body(
        egui::pos2(c.x + 1.4, c.y - 0.2),
        egui::pos2(c.x + 4.5, c.y + 5.0),
    );
    body(
        egui::pos2(c.x - 1.4, c.y - 0.2),
        egui::pos2(c.x + 1.4, c.y + 2.0),
    );
}

/// The projection readout, which is also its own switch — this is the text
/// the viewport used to paint in this corner (ADR-0028 §3).
fn paint_projection_button(
    painter: &egui::Painter,
    proj_rect: egui::Rect,
    projection: Projection,
    dark_mode: bool,
    hovered: bool,
) {
    let (bg, stroke, text) = if hovered {
        if dark_mode {
            (
                egui::Color32::from_rgba_unmultiplied(50, 60, 78, 245),
                egui::Color32::from_rgba_unmultiplied(85, 175, 255, 200),
                egui::Color32::from_rgb(85, 175, 255),
            )
        } else {
            (
                egui::Color32::from_rgba_unmultiplied(210, 218, 230, 255),
                egui::Color32::from_rgba_unmultiplied(30, 120, 230, 200),
                egui::Color32::from_rgb(30, 120, 230),
            )
        }
    } else if dark_mode {
        (
            egui::Color32::from_rgba_unmultiplied(35, 42, 54, 210),
            egui::Color32::from_rgba_unmultiplied(255, 255, 255, 30),
            egui::Color32::from_rgb(215, 222, 232),
        )
    } else {
        (
            egui::Color32::from_rgba_unmultiplied(225, 230, 238, 220),
            egui::Color32::from_rgba_unmultiplied(0, 0, 0, 30),
            egui::Color32::from_rgb(45, 52, 64),
        )
    };

    painter.rect_filled(proj_rect, egui::CornerRadius::same(4), bg);
    painter.rect_stroke(
        proj_rect,
        egui::CornerRadius::same(4),
        egui::Stroke::new(1.0, stroke),
        egui::StrokeKind::Inside,
    );
    painter.text(
        proj_rect.center(),
        egui::Align2::CENTER_CENTER,
        projection.label(),
        egui::FontId::proportional(11.0),
        text,
    );
}
