//! robocad's 19 camera tests, ported verbatim to glam, plus the one that
//! pins the `[0, 1]` clip depth the port relies on.

use std::time::Duration;
use web_time::Instant;

use riggen_mesh::glam::Vec3;

use super::animation::{CameraAnimation, CameraSample, shortest_angular_delta};
use super::orbit::OrbitCamera;
use super::orientation::{Projection, StandardView, ViewOrientation};

#[test]
fn standard_views_place_eye_on_the_expected_axis() {
    let mut cam = OrbitCamera::default();
    for (view, axis) in [
        (StandardView::Right, Vec3::X),
        (StandardView::Left, Vec3::NEG_X),
        (StandardView::Front, Vec3::Y),
        (StandardView::Back, Vec3::NEG_Y),
        (StandardView::Top, Vec3::Z),
        (StandardView::Bottom, Vec3::NEG_Z),
    ] {
        cam.set_standard_view(view);
        let dir = (cam.eye() - cam.target).normalize();
        assert!(
            (dir - axis).length() < 1e-4,
            "{view:?}: eye direction {dir:?} does not match {axis:?}"
        );
    }
}

#[test]
fn top_and_bottom_views_do_not_degenerate_the_view_matrix() {
    let mut cam = OrbitCamera::default();
    for view in [StandardView::Top, StandardView::Bottom] {
        cam.set_standard_view(view);
        let m = cam.view_matrix();
        assert!(
            m.to_cols_array().iter().all(|x| x.is_finite()),
            "{view:?} produced a degenerate view matrix: {m:?}"
        );
    }
}

#[test]
fn toggle_projection_round_trips() {
    let mut cam = OrbitCamera::default();
    assert_eq!(cam.projection, Projection::Perspective);
    assert_eq!(cam.projection.label(), "Perspective");
    assert_eq!(cam.projection.toggled(), Projection::Orthographic);

    cam.toggle_projection();
    assert_eq!(cam.projection, Projection::Orthographic);
    assert_eq!(cam.projection.label(), "Orthographic");
    assert_eq!(cam.projection.toggled(), Projection::Perspective);

    cam.toggle_projection();
    assert_eq!(cam.projection, Projection::Perspective);
}

#[test]
fn frame_bounds_centers_target_and_backs_off_to_cover_the_radius() {
    let mut cam = OrbitCamera::default();
    let center = Vec3::new(1.0, 2.0, 3.0);
    cam.frame_bounds(center, 2.0);
    assert_eq!(cam.target, center);
    // The bounding sphere must fit within the perspective frustum at the
    // chosen distance: radius <= distance * sin(fov_y / 2).
    let half_fov = cam.fov_y * 0.5;
    assert!(cam.distance * half_fov.sin() >= 2.0 - 1e-4);
}

#[test]
fn zoom_to_cursor_centered_on_target_matches_plain_zoom() {
    let mut cam = OrbitCamera::default();
    let mut plain = cam.clone();
    cam.zoom_to_cursor(-120.0, (0.0, 0.0), 1.0);
    plain.zoom(-120.0);
    assert!((cam.distance - plain.distance).abs() < 1e-5);
    assert!((cam.target - plain.target).length() < 1e-5);
}

#[test]
fn zoom_to_cursor_off_center_shifts_target_toward_the_cursor() {
    let mut cam = OrbitCamera::default();
    let before = cam.target;
    cam.zoom_to_cursor(-120.0, (0.5, 0.0), 1.0);
    assert!(cam.target != before);
}

#[test]
fn view_orientation_normals_are_unit_vectors() {
    for orientation in ViewOrientation::ALL {
        let n = orientation.normal();
        assert!(
            (n.length() - 1.0).abs() < 1e-6,
            "{orientation:?}: normal {n:?} has length {}",
            n.length()
        );
    }
}

#[test]
fn view_orientation_yaw_pitch_reproduces_normal() {
    for orientation in ViewOrientation::ALL {
        let (yaw, pitch) = orientation.yaw_pitch();
        let (sy, cy) = yaw.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let dir = Vec3::new(cp * cy, cp * sy, sp);
        let n = orientation.normal();
        assert!(
            (dir - n).length() < 1e-5,
            "{orientation:?}: spherical direction {dir:?} does not match normal {n:?}"
        );
    }
}

#[test]
fn set_orientation_sets_eye_direction_matching_normal() {
    let mut cam = OrbitCamera::default();
    for orientation in ViewOrientation::ALL {
        cam.set_orientation(orientation);
        let dir = (cam.eye() - cam.target).normalize();
        let n = orientation.normal();
        assert!(
            (dir - n).length() < 1e-5,
            "{orientation:?}: eye direction {dir:?} does not match normal {n:?}"
        );
    }
}

#[test]
fn from_direction_resolves_exact_and_perturbed_directions() {
    for orientation in ViewOrientation::ALL {
        let n = orientation.normal();
        // Exact normal
        assert_eq!(
            ViewOrientation::from_direction(n),
            orientation,
            "Failed to match exact normal for {orientation:?}"
        );

        // Scaled vector
        assert_eq!(
            ViewOrientation::from_direction(n * 5.0),
            orientation,
            "Failed to match scaled normal for {orientation:?}"
        );

        // Slightly perturbed vector (within ~5 degrees)
        let perp = if n.x.abs() < 0.9 {
            Vec3::X.cross(n).normalize()
        } else {
            Vec3::Y.cross(n).normalize()
        };
        let perturbed = (n + perp * 0.05).normalize();
        assert_eq!(
            ViewOrientation::from_direction(perturbed),
            orientation,
            "Failed to match perturbed normal for {orientation:?}"
        );
    }
}

#[test]
fn closest_orientation_matches_current_camera_state() {
    let mut cam = OrbitCamera::default();
    for orientation in ViewOrientation::ALL {
        cam.set_orientation(orientation);
        assert_eq!(
            cam.closest_orientation(),
            orientation,
            "closest_orientation mismatch for {orientation:?}"
        );
    }
}

#[test]
fn view_orientation_subsets_and_counts() {
    assert_eq!(ViewOrientation::ALL.len(), 26);
    assert_eq!(ViewOrientation::FACES.len(), 6);
    assert_eq!(ViewOrientation::EDGES.len(), 12);
    assert_eq!(ViewOrientation::CORNERS.len(), 8);

    let mut count_faces = 0;
    let mut count_edges = 0;
    let mut count_corners = 0;

    for orientation in ViewOrientation::ALL {
        if orientation.is_face() {
            count_faces += 1;
            assert!(orientation.label().is_some());
        } else if orientation.is_edge() {
            count_edges += 1;
            assert!(orientation.label().is_none());
        } else if orientation.is_corner() {
            count_corners += 1;
            assert!(orientation.label().is_none());
        } else {
            panic!("Orientation {orientation:?} is not face, edge, or corner");
        }
        assert!(!orientation.name().is_empty());
    }

    assert_eq!(count_faces, 6);
    assert_eq!(count_edges, 12);
    assert_eq!(count_corners, 8);
}

#[test]
fn shortest_angular_delta_across_pi_branch_cut() {
    use std::f32::consts::PI;

    // Small positive step
    assert!((shortest_angular_delta(0.1, 0.3) - 0.2).abs() < 1e-6);
    // Small negative step
    assert!((shortest_angular_delta(0.3, 0.1) - (-0.2)).abs() < 1e-6);

    // Across the +π / -π branch cut:
    // from +170 deg (2.967 rad) to -170 deg (-2.967 rad):
    // delta should be +20 deg (+0.349 rad), NOT -340 deg
    let from = 2.9670597;
    let to = -2.9670597;
    let delta = shortest_angular_delta(from, to);
    assert!(
        delta > 0.0,
        "Delta across boundary should take the short positive path"
    );
    assert!((delta - 0.3490659).abs() < 1e-4);

    // from -170 deg to +170 deg:
    // delta should be -20 deg (-0.349 rad)
    let delta_rev = shortest_angular_delta(to, from);
    assert!(
        delta_rev < 0.0,
        "Delta across boundary should take the short negative path"
    );
    assert!((delta_rev - (-0.3490659)).abs() < 1e-4);

    // Exactly opposite angles (PI difference)
    assert!((shortest_angular_delta(0.0, PI).abs() - PI).abs() < 1e-5);
}

#[test]
fn camera_animation_cubic_ease_out() {
    let t0 = Instant::now();
    let dur = Duration::from_millis(200);
    let anim = CameraAnimation::with_duration(0.0, 0.0, 1.0, 2.0, t0, dur);

    // At t = 0 (start)
    let (s0, finished0) = anim.sample(t0);
    assert!(!finished0);
    assert!((s0.yaw - 0.0).abs() < 1e-6);
    assert!((s0.pitch - 0.0).abs() < 1e-6);

    // At t = 0.5 (halfway in time): ease = 1 - (1 - 0.5)^3 = 1 - 0.125 = 0.875
    let t_half = t0 + Duration::from_millis(100);
    let (s_half, finished_half) = anim.sample(t_half);
    assert!(!finished_half);
    assert!((s_half.yaw - 0.875).abs() < 1e-3);
    assert!((s_half.pitch - 1.75).abs() < 1e-3);

    // At t = 1.0 (end)
    let t_end = t0 + Duration::from_millis(200);
    let (s_end, finished_end) = anim.sample(t_end);
    assert!(finished_end);
    assert!((s_end.yaw - 1.0).abs() < 1e-6);
    assert!((s_end.pitch - 2.0).abs() < 1e-6);

    // Past the end
    let t_past = t0 + Duration::from_millis(300);
    let (s_past, finished_past) = anim.sample(t_past);
    assert!(finished_past);
    assert!((s_past.yaw - 1.0).abs() < 1e-6);
    assert!((s_past.pitch - 2.0).abs() < 1e-6);
}

#[test]
fn camera_animation_interpolates_target_and_distance() {
    let t0 = Instant::now();
    let dur = Duration::from_millis(200);
    let anim = CameraAnimation::from_samples(
        CameraSample::new(Vec3::ZERO, 10.0, 0.0, 0.0),
        CameraSample::new(Vec3::new(4.0, 6.0, 8.0), 2.0, 1.0, 0.5),
        t0,
        dur,
    );

    // Halfway (t = 0.5, ease = 0.875)
    let t_half = t0 + Duration::from_millis(100);
    let (s_half, finished_half) = anim.sample(t_half);
    assert!(!finished_half);
    assert!((s_half.target.x - 3.5).abs() < 1e-3);
    assert!((s_half.target.y - 5.25).abs() < 1e-3);
    assert!((s_half.target.z - 7.0).abs() < 1e-3);
    assert!((s_half.distance - (10.0 + (2.0 - 10.0) * 0.875)).abs() < 1e-3);

    // End (t = 1.0)
    let t_end = t0 + Duration::from_millis(200);
    let (s_end, finished_end) = anim.sample(t_end);
    assert!(finished_end);
    assert!((s_end.target.x - 4.0).abs() < 1e-5);
    assert!((s_end.target.y - 6.0).abs() < 1e-5);
    assert!((s_end.target.z - 8.0).abs() < 1e-5);
    assert!((s_end.distance - 2.0).abs() < 1e-5);
}

#[test]
fn step_animation_advances_and_completes() {
    let mut cam = OrbitCamera {
        yaw: 0.0,
        pitch: 0.0,
        ..Default::default()
    };

    let t0 = Instant::now();
    let dur = Duration::from_millis(100);
    cam.animation = Some(CameraAnimation::with_duration(0.0, 0.0, 1.0, 0.5, t0, dur));

    assert!(cam.is_animating());

    // Step at halfway
    let still_animating = cam.step_animation(t0 + Duration::from_millis(50));
    assert!(still_animating);
    assert!(cam.is_animating());
    assert!(cam.yaw > 0.5); // Ease-out is > 50% at half time

    // Step at completion
    let still_animating_end = cam.step_animation(t0 + Duration::from_millis(100));
    assert!(!still_animating_end);
    assert!(!cam.is_animating());
    assert!((cam.yaw - 1.0).abs() < 1e-5);
    assert!((cam.pitch - 0.5).abs() < 1e-5);
}

#[test]
fn animate_frame_bounds_steps_smoothly_to_target() {
    let mut cam = OrbitCamera {
        target: Vec3::ZERO,
        distance: 10.0,
        yaw: 0.5,
        pitch: 0.2,
        ..Default::default()
    };

    let target_center = Vec3::new(5.0, 5.0, 5.0);
    cam.animate_frame_bounds(target_center, 2.0);
    assert!(cam.is_animating());

    let t0 = Instant::now();
    // Halfway step
    let animating = cam.step_animation(t0 + Duration::from_millis(90));
    assert!(animating);
    assert!(cam.target.x > 0.0 && cam.target.x < 5.0);
    assert!(cam.distance < 10.0);

    // Complete step (past 180ms)
    let animating_done = cam.step_animation(t0 + Duration::from_millis(250));
    assert!(!animating_done);
    assert!(!cam.is_animating());
    assert!((cam.target.x - 5.0).abs() < 1e-4);
    assert!((cam.target.y - 5.0).abs() < 1e-4);
    assert!((cam.target.z - 5.0).abs() < 1e-4);
    // Orientation should stay preserved
    assert!((cam.yaw - 0.5).abs() < 1e-4);
    assert!((cam.pitch - 0.2).abs() < 1e-4);
}

#[test]
fn user_interactions_cancel_active_animation() {
    let mut cam = OrbitCamera::default();

    // Orbit cancels
    cam.animate_to(1.0, 1.0);
    assert!(cam.is_animating());
    cam.orbit(0.1, 0.1);
    assert!(!cam.is_animating());

    // Pan cancels
    cam.animate_to(1.0, 1.0);
    assert!(cam.is_animating());
    cam.pan(10.0, 10.0);
    assert!(!cam.is_animating());

    // Zoom cancels
    cam.animate_to(1.0, 1.0);
    assert!(cam.is_animating());
    cam.zoom(5.0);
    assert!(!cam.is_animating());

    // Set orientation cancels
    cam.animate_to(1.0, 1.0);
    assert!(cam.is_animating());
    cam.set_orientation(ViewOrientation::Front);
    assert!(!cam.is_animating());

    // Set standard view cancels
    cam.animate_to(1.0, 1.0);
    assert!(cam.is_animating());
    cam.set_standard_view(StandardView::Top);
    assert!(!cam.is_animating());

    // Frame bounds cancels
    cam.animate_to(1.0, 1.0);
    assert!(cam.is_animating());
    cam.frame_bounds(Vec3::ZERO, 1.0);
    assert!(!cam.is_animating());
}

#[test]
fn animate_home_transitions_target_distance_and_orientation() {
    let mut cam = OrbitCamera {
        target: Vec3::new(10.0, -5.0, 3.0),
        distance: 15.0,
        yaw: 1.2,
        pitch: -0.4,
        ..Default::default()
    };

    let target_center = Vec3::new(2.0, 3.0, 4.0);
    let target_radius = 5.0;
    cam.animate_home(target_center, target_radius);
    assert!(cam.is_animating());

    // Advance to completion
    let start = Instant::now();
    let complete_time = start + CameraAnimation::DEFAULT_DURATION + Duration::from_millis(10);
    let still_running = cam.step_animation(complete_time);
    assert!(!still_running);
    assert!(!cam.is_animating());

    assert!((cam.target - target_center).length() < 1e-4);
    assert!((cam.yaw - OrbitCamera::DEFAULT_YAW).abs() < 1e-4);
    assert!((cam.pitch - OrbitCamera::DEFAULT_PITCH).abs() < 1e-4);
}

/// New with the glam port: robocad multiplied cgmath's OpenGL-convention
/// projections by `OPENGL_TO_WGPU_MATRIX`; glam's `_rh` constructors emit
/// wgpu's `[0, 1]` clip depth directly, and this is the test that would
/// catch the remap being reintroduced (near plane at −1, or everything
/// squashed into `[0.5, 1]`).
#[test]
fn projections_map_near_to_depth_0_and_far_to_depth_1() {
    let mut cam = OrbitCamera::default();
    let aspect = 1.5;
    for projection in [Projection::Perspective, Projection::Orthographic] {
        cam.projection = projection;
        let (forward, _, _) = cam.basis();
        let view_proj = cam.view_proj(aspect);
        // `project_point3` does the perspective divide.
        let near = view_proj.project_point3(cam.eye() + forward * cam.near);
        let far = view_proj.project_point3(cam.eye() + forward * cam.far);
        assert!(
            near.z.abs() < 1e-4,
            "{projection:?}: near plane lands at depth {}",
            near.z
        );
        assert!(
            (far.z - 1.0).abs() < 1e-4,
            "{projection:?}: far plane lands at depth {}",
            far.z
        );
        // Both on the view axis, so at the centre of the screen.
        assert!(
            near.x.abs() < 1e-4 && near.y.abs() < 1e-4,
            "{projection:?}: {near:?}"
        );
        assert!(
            far.x.abs() < 1e-4 && far.y.abs() < 1e-4,
            "{projection:?}: {far:?}"
        );
        // And a point past the target is deeper than one before it.
        let nearer = view_proj.project_point3(cam.eye() + forward * (cam.distance * 0.5));
        let farther = view_proj.project_point3(cam.eye() + forward * (cam.distance * 1.5));
        assert!(nearer.z < farther.z, "{projection:?}: depth not increasing");
    }
}

/// A fit sets the depth range from the radius, so a millimetre part sits
/// well inside `[near, far]` and the zoom range follows.
#[test]
fn fit_sets_the_depth_range_from_the_radius() {
    let mut cam = OrbitCamera::default();
    assert_eq!((cam.near, cam.far), (0.01, 100.0));
    cam.frame_bounds(Vec3::ZERO, 0.000_866);
    assert!(cam.near < 1e-4, "near {}", cam.near);
    assert!(
        cam.near * 2.0 <= cam.distance,
        "{} vs {}",
        cam.near,
        cam.distance
    );
    assert!(
        cam.distance - 0.000_866 > cam.near,
        "part in front of the near plane"
    );
    assert!(
        cam.distance + 0.000_866 < cam.far,
        "part inside the far plane"
    );
    // Zooming in stops short of the near plane, out short of the far one.
    for _ in 0..200 {
        cam.zoom(1000.0);
    }
    assert!(cam.distance >= cam.near * 2.0);
    for _ in 0..200 {
        cam.zoom(-1000.0);
    }
    assert!(cam.distance <= cam.far * 0.5);
    // A room-sized scene keeps a sane range too.
    cam.frame_bounds(Vec3::ZERO, 10.0);
    assert!(
        (cam.near - 0.1).abs() < 1e-6 && cam.far == 10_000.0,
        "{} {}",
        cam.near,
        cam.far
    );
    assert!((cam.distance - 10.0 / (cam.fov_y * 0.5).sin() * 1.2).abs() < 1e-3);
}
