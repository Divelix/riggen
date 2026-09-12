//! The world axes on the cube's corner: three arms, `X`, `Y` and `Z`, from
//! just outside the (−X, −Y, −Z) corner along the three edges that meet
//! there, each with its letter past the end (plans/viewcube-corner-axes).
//!
//! Geometry only, like `projection.rs`: nothing here paints or reads the
//! pointer. The arms go through the cube's own [`camera_basis`] and
//! [`cube_scale`], so they turn with the cube as a part of it and cannot
//! drift from the facets.

use std::f32::consts::{PI, SQRT_2};

use riggen_core::glam::Vec3;

use super::projection::{
    ProjectedFacet, camera_basis, cube_scale, point_in_polygon_2d, project_viewcube,
};

/// How far outside the bounding cube's corner the arms start, in cube units
/// along the (−1, −1, −1) diagonal: enough to lift them off the edges.
pub const AXES_GAP: f32 = 0.12;
/// An arm's length in cube units — 1.3 edges of the ±1 cube, so it clears
/// the far corner.
pub const ARM_LENGTH: f32 = 2.6;
/// The side of a letter's square, in points.
pub const LETTER_SIZE: f32 = 11.0;
/// Between an arm's end and its letter's square, in points.
pub const LETTER_GAP: f32 = 3.0;
/// Below this many points on screen an arm points at the eye, and its
/// letter turns from the arm's direction to the corner's.
pub const FORESHORTENED: f32 = 12.0;
/// Samples along an arm for the front/behind split.
const ARM_SAMPLES: usize = 48;

/// One arm, projected for a cube drawn in some rect.
#[derive(Debug, Clone, PartialEq)]
pub struct CornerAxis {
    /// 0, 1, 2 for X, Y, Z.
    pub axis: usize,
    pub tail: egui::Pos2,
    pub tip: egui::Pos2,
    /// Stretches the cube hides, from tail towards tip: painted before the
    /// facets, so they show through the fill.
    pub behind: Vec<[egui::Pos2; 2]>,
    /// Stretches nothing hides: painted after the facets.
    pub in_front: Vec<[egui::Pos2; 2]>,
    /// The tip's distance towards the eye, in cube units: which of two
    /// letters is nearer.
    pub depth: f32,
    /// The centre of the letter's square.
    pub letter: egui::Pos2,
    /// False while the tip is behind the cube, or while a nearer letter
    /// covers this one: then no letter is drawn at all.
    pub letter_visible: bool,
}

/// Where the three arms start, in cube units.
pub fn corner_origin() -> Vec3 {
    Vec3::NEG_ONE + Vec3::NEG_ONE.normalize() * AXES_GAP
}

/// The square a letter centred at `letter` occupies.
pub fn letter_rect(letter: egui::Pos2) -> egui::Rect {
    egui::Rect::from_center_size(letter, egui::Vec2::splat(LETTER_SIZE))
}

/// The three arms for a cube drawn in `rect` at `yaw` and `pitch`.
pub fn project_corner_axes(rect: egui::Rect, yaw: f32, pitch: f32) -> [CornerAxis; 3] {
    let (cam_right, cam_up, eye_dir) = camera_basis(yaw, pitch);
    let center = rect.center();
    let scale = cube_scale(rect);
    let facets = project_viewcube(rect, yaw, pitch);
    let project = |p: Vec3| {
        egui::pos2(
            center.x + p.dot(cam_right) * scale,
            center.y - p.dot(cam_up) * scale,
        )
    };

    let origin = corner_origin();
    let tail = project(origin);
    let mut axes: [CornerAxis; 3] = std::array::from_fn(|axis| {
        let dir = Vec3::AXES[axis];
        let end = origin + dir * ARM_LENGTH;
        let tip = project(end);
        let [behind, in_front] = split_runs(|t| {
            let p = origin + dir * (ARM_LENGTH * t);
            is_behind(&facets, p, project(p))
        })
        .map(|runs| {
            runs.into_iter()
                .map(|[a, b]| [tail + (tip - tail) * a, tail + (tip - tail) * b])
                .collect()
        });
        CornerAxis {
            axis,
            tail,
            tip,
            behind,
            in_front,
            depth: end.dot(eye_dir),
            letter: tip
                + letter_direction(tip - tail, tail - center) * (LETTER_GAP + LETTER_SIZE * 0.5),
            letter_visible: !is_behind(&facets, end, tip),
        }
    });

    // Nearest first: a letter behind a drawn one is covered by it, as the
    // cube covers one behind the cube.
    let mut order = [0, 1, 2];
    order.sort_by(|&a, &b| axes[b].depth.total_cmp(&axes[a].depth));
    let mut drawn: Vec<egui::Pos2> = Vec::with_capacity(3);
    for i in order {
        let axis = &mut axes[i];
        if !axis.letter_visible {
            continue;
        }
        if drawn
            .iter()
            .any(|&other| letters_overlap(other, axis.letter))
        {
            axis.letter_visible = false;
        } else {
            drawn.push(axis.letter);
        }
    }
    axes
}

/// The square around the cube's centre that every arm and letter stays in
/// at any orientation, for a cube drawn in `rect`.
///
/// A point `p` projects at most `|p|` from the centre, and the farthest
/// point of an arm is its tail or its tip; the letter adds its gap, half a
/// side, and the half-diagonal a square turned towards a corner reaches.
pub fn corner_axes_extent(rect: egui::Rect) -> egui::Rect {
    let origin = corner_origin();
    let reach = origin
        .length()
        .max((origin + Vec3::X * ARM_LENGTH).length())
        * cube_scale(rect)
        + LETTER_GAP
        + LETTER_SIZE * 0.5 * (1.0 + SQRT_2);
    egui::Rect::from_center_size(rect.center(), egui::Vec2::splat(2.0 * reach))
}

/// Whether the squares of two letters centred at `a` and `b` share area.
pub fn letters_overlap(a: egui::Pos2, b: egui::Pos2) -> bool {
    let d = a - b;
    d.x.abs() < LETTER_SIZE && d.y.abs() < LETTER_SIZE
}

/// The cube is convex, so `point` is hidden iff its projection falls in a
/// front-facing facet and it lies on the cube's side of that facet's plane:
/// the ray from it towards the eye then crosses the facet.
fn is_behind(facets: &[ProjectedFacet], point: Vec3, screen: egui::Pos2) -> bool {
    facets.iter().any(|facet| {
        facet.normal_3d.dot(point) < facet.plane_offset - 1e-4
            && point_in_polygon_2d(screen, &facet.vertices_2d)
    })
}

/// Samples `hidden` along `t ∈ [0, 1]` and returns the `(behind, in_front)`
/// runs as parameter intervals, a boundary halfway between two samples
/// that disagree.
fn split_runs(hidden: impl Fn(f32) -> bool) -> [Vec<[f32; 2]>; 2] {
    let mut runs = [Vec::new(), Vec::new()];
    let mut start = 0.0;
    let mut previous = hidden(0.0);
    for i in 1..=ARM_SAMPLES {
        let current = hidden(i as f32 / ARM_SAMPLES as f32);
        if current != previous {
            let boundary = (i as f32 - 0.5) / ARM_SAMPLES as f32;
            runs[usize::from(!previous)].push([start, boundary]);
            start = boundary;
            previous = current;
        }
    }
    runs[usize::from(!previous)].push([start, 1.0]);
    runs
}

/// The unit direction a letter sits in from its arm's tip: along the arm,
/// turning towards the corner's direction from the cube centre as the arm
/// shortens below [`FORESHORTENED`], so a letter never lands on the corner
/// its arm points out of. The turn is by angle, so it has no zero-length
/// midpoint when the two point opposite ways.
fn letter_direction(arm: egui::Vec2, corner: egui::Vec2) -> egui::Vec2 {
    let w = (arm.length() / FORESHORTENED).min(1.0);
    let from = corner.angle();
    let mut turn = arm.angle() - from;
    if turn > PI {
        turn -= 2.0 * PI;
    } else if turn <= -PI {
        turn += 2.0 * PI;
    }
    egui::Vec2::angled(from + turn * w)
}
