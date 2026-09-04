//! The scene's depth buffer, read back so an egui-painter overlay can be
//! depth-tested (ADR-0020, docs/01-architecture.md §Frame loop).
//!
//! egui's painter has no depth buffer of its own, so the only way an
//! overlay can know that a glyph runs behind a part is to be told what the
//! wgpu pass wrote. The copy is asynchronous exactly as the ID-buffer pick
//! is (`picking.rs`) — never a blocking poll — and the result is therefore
//! at best one frame old.
//!
//! **The matrix travels with the image.** A [`DepthImage`] carries the
//! `view_proj`, rect and scale it was rendered with, and a glyph is
//! classified by projecting it through *those*, not through this frame's
//! camera. A one-frame-stale image is then still self-consistent: the
//! answer is "was this point behind geometry when those texels were
//! written", which is off by at most one frame of camera motion instead of
//! being wrong by however far the camera moved.

use std::sync::{Arc, Mutex};

use egui_wgpu::wgpu;
use riggen_mesh::glam::{DVec3, Mat4};

/// Bytes per `Depth32Float` texel.
pub const DEPTH_TEXEL: u32 = 4;

/// How many frames a depth readback may stay in flight before it is
/// abandoned, for the reason [`super::picking::MAX_PICK_FRAMES`] gives: a
/// request nobody will ever answer must not wedge the overlay forever.
pub const MAX_DEPTH_FRAMES: u32 = 8;

/// Depth bias, as a fraction of the NDC distance left to the far plane.
///
/// A glyph drawn *on* a surface — a pivot placed by a snap, a frame triad
/// on a face — projects to the same depth the geometry wrote, and without
/// slack it would flicker between hidden and visible on rounding alone.
/// Scaling the slack by `1 - stored` makes it a constant *relative* margin
/// in eye space under a perspective projection (the NDC depth is
/// `A/z_eye + B`, so `1 - z_ndc` is proportional to `1/z_eye` near the
/// camera): about a part in a thousand of the distance to the camera,
/// wherever the part is.
pub const DEPTH_BIAS: f32 = 1e-3;

/// `bytes_per_row` for a `width`-wide `Depth32Float` copy: wgpu requires a
/// multiple of 256, so rows are padded and a lookup steps over the padding.
pub fn row_stride(width: u32) -> u32 {
    (width * DEPTH_TEXEL).div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
        * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT
}

/// Bytes a readback buffer for a `size` image needs.
pub fn readback_size(size: (u32, u32)) -> wgpu::BufferAddress {
    row_stride(size.0) as wgpu::BufferAddress * size.1 as wgpu::BufferAddress
}

/// Everything a depth image's answer depends on. Re-copying the buffer
/// while all three are unchanged would produce the image already held, so a
/// resting camera over an unchanging scene reads the depth buffer back
/// exactly once — the same memo the hover pick keeps
/// ([`super::picking::PickInputs`]), and the reason a 4 MB copy is not in
/// the steady-state frame budget.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepthInputs {
    pub view_proj: [[f32; 4]; 4],
    pub size: (u32, u32),
    /// [`crate::Scene`]'s revision: what is drawn and where it sits.
    pub revision: u64,
}

/// One depth readback in flight.
///
/// Two stages, because the copy is recorded on **egui's** encoder — after
/// the scene pass, so it sees this frame's depth and not the last one's —
/// and `map_async` is only legal once that encoder has been submitted.
/// `recorded` is set from inside the paint callback, which runs only on a
/// frame that actually renders; the next `ui()` sees it and maps.
pub struct PendingDepth {
    pub buffer: wgpu::Buffer,
    /// Set by the paint callback once the copy is on egui's encoder.
    pub recorded: Arc<std::sync::atomic::AtomicBool>,
    /// Filled by the `map_async` callback, whenever wgpu gets to it: the
    /// mapped bytes, verbatim.
    pub result: Arc<Mutex<Option<Vec<u8>>>>,
    /// `true` once `map_async` has been registered.
    pub mapping: bool,
    pub inputs: DepthInputs,
    pub rect: egui::Rect,
    pub pixels_per_point: f32,
    /// Frames this request has waited. See [`MAX_DEPTH_FRAMES`].
    pub age: u32,
}

/// The scene's depth buffer as the overlay reads it, with the camera it was
/// rendered with (see the module doc).
#[derive(Debug, Clone)]
pub struct DepthImage {
    /// The readback exactly as it was mapped: little-endian `f32` NDC
    /// depths, rows padded to [`row_stride`], `1.0` where nothing opaque
    /// was drawn.
    ///
    /// Kept packed rather than widened into a `Vec<f32>`: converting every
    /// texel cost about a millisecond a frame at a full-window viewport
    /// (ADR-0020), and an overlay reads a few hundred of them.
    pub bytes: Vec<u8>,
    /// Physical pixels.
    pub size: (u32, u32),
    pub view_proj: Mat4,
    /// The viewport rect, in egui logical points.
    pub rect: egui::Rect,
    pub pixels_per_point: f32,
    pub inputs: DepthInputs,
}

impl DepthImage {
    /// Where `world` landed on screen when this image was rendered, in
    /// logical points, and its own NDC depth. `None` behind the camera or
    /// outside the depth range — the same rejections [`crate::Viewport::project`]
    /// makes, through this image's own matrix.
    pub fn project(&self, world: DVec3) -> Option<(egui::Pos2, f32)> {
        let clip = self.view_proj * world.as_vec3().extend(1.0);
        if clip.w <= 0.0 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        if !(-1.0..=1.0).contains(&ndc.z) {
            return None;
        }
        Some((
            egui::pos2(
                self.rect.min.x + (ndc.x * 0.5 + 0.5) * self.rect.width(),
                self.rect.min.y + (0.5 - ndc.y * 0.5) * self.rect.height(),
            ),
            ndc.z,
        ))
    }

    /// The depth stored at a logical-point position, or `None` off the
    /// image.
    pub fn depth_at(&self, pos: egui::Pos2) -> Option<f32> {
        let local = (pos - self.rect.min) * self.pixels_per_point;
        let (x, y) = (local.x.floor(), local.y.floor());
        if x < 0.0 || y < 0.0 || x >= self.size.0 as f32 || y >= self.size.1 as f32 {
            return None;
        }
        let at = y as usize * row_stride(self.size.0) as usize + x as usize * DEPTH_TEXEL as usize;
        let texel = self.bytes.get(at..at + DEPTH_TEXEL as usize)?;
        Some(f32::from_le_bytes(texel.try_into().ok()?))
    }

    /// Whether opaque geometry stands between the camera and `world`.
    ///
    /// A point the image cannot answer for — behind the camera, off the
    /// rect — reads as **visible**: an overlay that is not sure must not
    /// dim, or a glyph would fade for reasons the user cannot see.
    pub fn hidden(&self, world: DVec3) -> bool {
        let Some((pos, own)) = self.project(world) else {
            return false;
        };
        let Some(stored) = self.depth_at(pos) else {
            return false;
        };
        stored < own - DEPTH_BIAS * (1.0 - stored)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_row_is_padded_to_the_copy_alignment() {
        assert_eq!(row_stride(64), 256);
        assert_eq!(row_stride(1), 256);
        assert_eq!(row_stride(65), 512);
        assert_eq!(row_stride(1440), 5888);
        assert_eq!(readback_size((1440, 900)), 5888 * 900);
    }

    #[test]
    fn a_lookup_steps_over_the_row_padding() {
        let size = (2, 2);
        let stride = row_stride(size.0) as usize;
        let mut bytes = vec![0u8; stride * size.1 as usize];
        for (row, values) in [[0.25f32, 0.5], [0.75, 1.0]].into_iter().enumerate() {
            for (col, value) in values.into_iter().enumerate() {
                let at = row * stride + col * 4;
                bytes[at..at + 4].copy_from_slice(&value.to_le_bytes());
            }
        }
        let image = DepthImage {
            bytes,
            ..image(0.0)
        };
        for (row, values) in [[0.25f32, 0.5], [0.75, 1.0]].into_iter().enumerate() {
            for (col, value) in values.into_iter().enumerate() {
                let pos = egui::pos2(col as f32 + 0.5, row as f32 + 0.5);
                assert_eq!(image.depth_at(pos), Some(value), "({col}, {row})");
            }
        }
    }

    /// An image whose "geometry" is a single depth value everywhere, at a
    /// camera looking down `-Z` with an orthographic-ish matrix built by
    /// hand: enough to exercise `project` / `depth_at` / `hidden` without a
    /// GPU.
    fn image(stored: f32) -> DepthImage {
        let size = (2, 2);
        let stride = row_stride(size.0) as usize;
        let mut bytes = vec![0u8; stride * size.1 as usize];
        for row in 0..size.1 as usize {
            for col in 0..size.0 as usize {
                let at = row * stride + col * 4;
                bytes[at..at + 4].copy_from_slice(&stored.to_le_bytes());
            }
        }
        let view_proj = Mat4::orthographic_rh(-1.0, 1.0, -1.0, 1.0, 0.0, 10.0)
            * Mat4::look_at_rh(
                riggen_mesh::glam::Vec3::new(0.0, 0.0, 5.0),
                riggen_mesh::glam::Vec3::ZERO,
                riggen_mesh::glam::Vec3::Y,
            );
        DepthImage {
            bytes,
            size,
            view_proj,
            rect: egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(2.0, 2.0)),
            pixels_per_point: 1.0,
            inputs: DepthInputs {
                view_proj: view_proj.to_cols_array_2d(),
                size,
                revision: 0,
            },
        }
    }

    #[test]
    fn a_point_behind_the_stored_depth_is_hidden_and_one_in_front_is_not() {
        // The origin sits 5 away from the eye, 10 deep: NDC depth 0.5.
        let centre = DVec3::ZERO;
        assert!((image(1.0).project(centre).unwrap().1 - 0.5).abs() < 1e-6);
        // Nothing drawn: the far plane hides nothing.
        assert!(!image(1.0).hidden(centre));
        // Geometry in front of it.
        assert!(image(0.25).hidden(centre));
        // Geometry behind it.
        assert!(!image(0.75).hidden(centre));
    }

    #[test]
    fn a_glyph_lying_on_the_surface_is_not_hidden() {
        // Exactly coplanar, plus a hair of rounding either way.
        for stored in [0.5, 0.5 - 1e-5, 0.5 + 1e-5] {
            assert!(!image(stored).hidden(DVec3::ZERO), "stored {stored}");
        }
    }

    #[test]
    fn a_point_the_image_cannot_answer_for_reads_as_visible() {
        let image = image(0.0);
        // Off the rect.
        assert!(!image.hidden(DVec3::new(100.0, 0.0, 0.0)));
        // Behind the camera.
        assert!(!image.hidden(DVec3::new(0.0, 0.0, 50.0)));
        assert_eq!(image.depth_at(egui::pos2(-1.0, 0.0)), None);
        assert_eq!(image.depth_at(egui::pos2(2.0, 0.0)), None);
        assert_eq!(image.depth_at(egui::pos2(0.0, 0.0)), Some(0.0));
    }
}
