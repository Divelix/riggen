//! RoboCAD's ViewCube tests, ported with the math they cover (ADR-0028 §3).
//!
//! These are the step's evidence in place of a golden: nothing is painted
//! yet, so the facets, the cull, the sort and the hit test have to be
//! asserted as numbers. The widget's own tests arrive with the widget.

use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};

use riggen_core::glam::Vec3;
use riggen_viewport::ViewOrientation;

use super::facets::chamfered_cube_facets;
use super::projection::{
    camera_basis, face_local_axes_3d, hit_test_viewcube, point_in_polygon_2d,
    project_face_text_mesh, project_viewcube,
};

#[test]
fn chamfered_cube_facet_counts_and_shapes() {
    let facets = chamfered_cube_facets();
    assert_eq!(facets.len(), 26);

    let mut face_count = 0;
    let mut edge_count = 0;
    let mut corner_count = 0;

    for facet in &facets {
        if facet.orientation.is_face() {
            face_count += 1;
            assert_eq!(
                facet.vertices.len(),
                4,
                "{:?} must be a quad",
                facet.orientation
            );
        } else if facet.orientation.is_edge() {
            edge_count += 1;
            assert_eq!(
                facet.vertices.len(),
                4,
                "{:?} must be a quad",
                facet.orientation
            );
        } else if facet.orientation.is_corner() {
            corner_count += 1;
            assert_eq!(
                facet.vertices.len(),
                3,
                "{:?} must be a triangle",
                facet.orientation
            );
        }
    }

    assert_eq!(face_count, 6);
    assert_eq!(edge_count, 12);
    assert_eq!(corner_count, 8);
}

#[test]
fn all_facets_have_outward_ccw_winding() {
    let facets = chamfered_cube_facets();
    for facet in &facets {
        let v0 = facet.vertices[0];
        let v1 = facet.vertices[1];
        let v2 = facet.vertices[2];
        let cross = (v1 - v0).cross(v2 - v0);
        let dot = cross.dot(facet.normal);
        assert!(
            dot > 1e-4,
            "{:?}: polygon vertices are not winding CCW outward (cross·n = {})",
            facet.orientation,
            dot
        );
    }
}

#[test]
fn face_local_axes_orthonormality_and_canonical_orientations() {
    for orientation in ViewOrientation::FACES {
        let (u, v) = face_local_axes_3d(orientation);
        let n = orientation.normal();

        // Unit vectors
        assert!(
            (u.length_squared() - 1.0).abs() < 1e-5,
            "{:?} U axis is not a unit vector",
            orientation
        );
        assert!(
            (v.length_squared() - 1.0).abs() < 1e-5,
            "{:?} V axis is not a unit vector",
            orientation
        );

        // Orthogonal to each other and to face normal
        assert!(
            u.dot(v).abs() < 1e-5,
            "{:?} U and V axes are not orthogonal",
            orientation
        );
        assert!(
            u.dot(n).abs() < 1e-5,
            "{:?} U axis is not orthogonal to face normal",
            orientation
        );
        assert!(
            v.dot(n).abs() < 1e-5,
            "{:?} V axis is not orthogonal to face normal",
            orientation
        );

        // Check canonical directions
        match orientation {
            ViewOrientation::Front => {
                assert_eq!(u, Vec3::new(-1.0, 0.0, 0.0));
                assert_eq!(v, Vec3::new(0.0, 0.0, 1.0));
            }
            ViewOrientation::Back => {
                assert_eq!(u, Vec3::new(1.0, 0.0, 0.0));
                assert_eq!(v, Vec3::new(0.0, 0.0, 1.0));
            }
            ViewOrientation::Left => {
                assert_eq!(u, Vec3::new(0.0, -1.0, 0.0));
                assert_eq!(v, Vec3::new(0.0, 0.0, 1.0));
            }
            ViewOrientation::Right => {
                assert_eq!(u, Vec3::new(0.0, 1.0, 0.0));
                assert_eq!(v, Vec3::new(0.0, 0.0, 1.0));
            }
            ViewOrientation::Top => {
                assert_eq!(u, Vec3::new(1.0, 0.0, 0.0));
                assert_eq!(v, Vec3::new(0.0, 1.0, 0.0));
            }
            ViewOrientation::Bottom => {
                assert_eq!(u, Vec3::new(-1.0, 0.0, 0.0));
                assert_eq!(v, Vec3::new(0.0, 1.0, 0.0));
            }
            _ => {}
        }
    }
}

#[test]
fn front_view_culling_and_visibility() {
    let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(100.0, 100.0));
    // Front view: yaw = PI/2, pitch = 0.0
    let visible = project_viewcube(rect, FRAC_PI_2, 0.0);

    // Visible should contain Front face, FrontTop, FrontBottom, FrontLeft, FrontRight edges,
    // and 4 Front corners (total 1 + 4 + 4 = 9 facets).
    assert_eq!(visible.len(), 9);

    let visible_orientations: Vec<_> = visible.iter().map(|f| f.orientation).collect();
    assert!(visible_orientations.contains(&ViewOrientation::Front));
    assert!(visible_orientations.contains(&ViewOrientation::FrontTop));
    assert!(visible_orientations.contains(&ViewOrientation::FrontBottom));
    assert!(visible_orientations.contains(&ViewOrientation::FrontLeft));
    assert!(visible_orientations.contains(&ViewOrientation::FrontRight));
    assert!(visible_orientations.contains(&ViewOrientation::FrontTopLeft));
    assert!(visible_orientations.contains(&ViewOrientation::FrontTopRight));
    assert!(visible_orientations.contains(&ViewOrientation::FrontBottomLeft));
    assert!(visible_orientations.contains(&ViewOrientation::FrontBottomRight));

    // Back face must be culled
    assert!(!visible_orientations.contains(&ViewOrientation::Back));

    // Front face must have the highest depth (frontmost)
    let front_facet = visible
        .iter()
        .find(|f| f.orientation == ViewOrientation::Front)
        .unwrap();
    for f in &visible {
        assert!(
            front_facet.depth >= f.depth - 1e-5,
            "Front face should have the highest depth"
        );
    }
}

#[test]
fn isometric_view_culling_and_sorting() {
    let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(100.0, 100.0));
    // Front-Top-Right isometric view
    let (yaw, pitch) = ViewOrientation::FrontTopRight.yaw_pitch();
    let visible = project_viewcube(rect, yaw, pitch);

    let visible_orientations: Vec<_> = visible.iter().map(|f| f.orientation).collect();

    // Must see Front, Top, Right faces
    assert!(visible_orientations.contains(&ViewOrientation::Front));
    assert!(visible_orientations.contains(&ViewOrientation::Top));
    assert!(visible_orientations.contains(&ViewOrientation::Right));

    // Must see FrontTopRight corner as the closest facet
    assert!(visible_orientations.contains(&ViewOrientation::FrontTopRight));
    let corner = visible
        .iter()
        .find(|f| f.orientation == ViewOrientation::FrontTopRight)
        .unwrap();
    for f in &visible {
        assert!(
            corner.depth >= f.depth - 1e-5,
            "FrontTopRight corner should have the highest depth in its view"
        );
    }

    // Back, Bottom, Left faces must be culled
    assert!(!visible_orientations.contains(&ViewOrientation::Back));
    assert!(!visible_orientations.contains(&ViewOrientation::Bottom));
    assert!(!visible_orientations.contains(&ViewOrientation::Left));

    // Ensure facets are sorted in ascending depth order
    for window in visible.windows(2) {
        assert!(
            window[0].depth <= window[1].depth + 1e-5,
            "Facets must be sorted in painter's order (ascending depth)"
        );
    }
}

#[test]
fn screen_coordinates_lie_within_widget_bounds() {
    let rect = egui::Rect::from_min_size(egui::pos2(10.0, 10.0), egui::vec2(100.0, 100.0));
    // Sample several views
    for (yaw, pitch) in [
        (0.0, 0.0),
        (FRAC_PI_2, 0.0),
        (FRAC_PI_4, FRAC_PI_4),
        (0.0, FRAC_PI_2),
    ] {
        let visible = project_viewcube(rect, yaw, pitch);
        for f in visible {
            for p in f.vertices_2d {
                assert!(
                    rect.expand(5.0).contains(p),
                    "Projected vertex {:?} out of bounds {:?}",
                    p,
                    rect
                );
            }
        }
    }
}

#[test]
fn point_in_polygon_2d_basic_shapes_and_windings() {
    // CCW Square [0, 0] to [10, 10]
    let ccw_square = vec![
        egui::pos2(0.0, 0.0),
        egui::pos2(10.0, 0.0),
        egui::pos2(10.0, 10.0),
        egui::pos2(0.0, 10.0),
    ];
    // CW Square
    let cw_square = vec![
        egui::pos2(0.0, 0.0),
        egui::pos2(0.0, 10.0),
        egui::pos2(10.0, 10.0),
        egui::pos2(10.0, 0.0),
    ];

    for sq in [&ccw_square, &cw_square] {
        // Inside points
        assert!(point_in_polygon_2d(egui::pos2(5.0, 5.0), sq));
        assert!(point_in_polygon_2d(egui::pos2(1.0, 1.0), sq));
        assert!(point_in_polygon_2d(egui::pos2(9.0, 9.0), sq));

        // Vertices
        assert!(point_in_polygon_2d(egui::pos2(0.0, 0.0), sq));
        assert!(point_in_polygon_2d(egui::pos2(10.0, 10.0), sq));

        // Edges
        assert!(point_in_polygon_2d(egui::pos2(5.0, 0.0), sq));
        assert!(point_in_polygon_2d(egui::pos2(10.0, 5.0), sq));

        // Outside points
        assert!(!point_in_polygon_2d(egui::pos2(-1.0, 5.0), sq));
        assert!(!point_in_polygon_2d(egui::pos2(11.0, 5.0), sq));
        assert!(!point_in_polygon_2d(egui::pos2(5.0, -1.0), sq));
        assert!(!point_in_polygon_2d(egui::pos2(5.0, 11.0), sq));
        assert!(!point_in_polygon_2d(egui::pos2(12.0, 12.0), sq));
    }

    // Triangle
    let tri = vec![
        egui::pos2(0.0, 0.0),
        egui::pos2(6.0, 0.0),
        egui::pos2(3.0, 6.0),
    ];
    assert!(point_in_polygon_2d(egui::pos2(3.0, 2.0), &tri));
    assert!(!point_in_polygon_2d(egui::pos2(0.0, 5.0), &tri));

    // Degenerate (< 3 vertices)
    assert!(!point_in_polygon_2d(
        egui::pos2(0.0, 0.0),
        &[egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)]
    ));
}

#[test]
fn hit_test_viewcube_front_view() {
    let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(100.0, 100.0));
    let visible = project_viewcube(rect, FRAC_PI_2, 0.0);

    // Center of the widget is the center of the Front face
    let center_hit = hit_test_viewcube(&visible, rect.center());
    assert_eq!(center_hit, Some(ViewOrientation::Front));

    // Find each projected facet and verify clicking its 2D center hits that orientation
    for facet in &visible {
        let hit = hit_test_viewcube(&visible, facet.center_2d);
        assert_eq!(
            hit,
            Some(facet.orientation),
            "Hit testing center_2d failed for {:?}",
            facet.orientation
        );
    }

    // Position far outside the cube returns None
    assert_eq!(hit_test_viewcube(&visible, egui::pos2(1.0, 1.0)), None);
    assert_eq!(hit_test_viewcube(&visible, egui::pos2(99.0, 1.0)), None);
}

#[test]
fn hit_test_viewcube_isometric_view() {
    let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(100.0, 100.0));
    let (yaw, pitch) = ViewOrientation::FrontTopRight.yaw_pitch();
    let visible = project_viewcube(rect, yaw, pitch);

    // Verify that hit-testing the center of every visible facet returns its orientation
    for facet in &visible {
        let hit = hit_test_viewcube(&visible, facet.center_2d);
        assert_eq!(
            hit,
            Some(facet.orientation),
            "Isometric hit testing center_2d failed for {:?}",
            facet.orientation
        );
    }

    // The FrontTopRight corner is directly in the center of its view
    let hit_corner = hit_test_viewcube(&visible, rect.center());
    assert_eq!(hit_corner, Some(ViewOrientation::FrontTopRight));
}

#[test]
fn project_face_text_mesh_generates_valid_mesh() {
    let ctx = egui::Context::default();
    let mut output = ctx.run_ui(Default::default(), |ui| {
        let rect = egui::Rect::from_min_size(egui::pos2(10.0, 10.0), egui::vec2(192.0, 192.0));
        let painter = ui.painter_at(rect);
        let (cam_right, cam_up, _) = camera_basis(FRAC_PI_2, 0.0);
        let font_id = egui::FontId::proportional(20.0);
        let radius = rect.width() * 0.5;
        let scale = radius / 1.75;

        let mesh_opt = project_face_text_mesh(
            &painter,
            ViewOrientation::Front,
            "FRONT",
            font_id,
            egui::Color32::WHITE,
            cam_right,
            cam_up,
            rect.center(),
            scale,
        );

        assert!(mesh_opt.is_some(), "Mesh should be generated for FRONT");
        let mesh = mesh_opt.unwrap();
        assert!(!mesh.vertices.is_empty(), "Mesh vertices must not be empty");
        assert!(!mesh.indices.is_empty(), "Mesh indices must not be empty");
        assert_eq!(mesh.indices.len() % 3, 0, "Indices must form triangles");

        for idx in &mesh.indices {
            assert!((*idx as usize) < mesh.vertices.len());
        }

        for v in &mesh.vertices {
            assert!(
                rect.expand(20.0).contains(v.pos),
                "Projected vertex {:?} should be inside widget bounds",
                v.pos
            );
            assert!(
                v.uv.x >= 0.0 && v.uv.x <= 1.0 && v.uv.y >= 0.0 && v.uv.y <= 1.0,
                "UV coordinates {:?} must be in [0, 1] range",
                v.uv
            );
        }
    });
    output.textures_delta.clear();
}
