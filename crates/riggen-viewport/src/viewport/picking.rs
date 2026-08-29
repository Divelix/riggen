//! The async ID-buffer pick: what is requested, what is read back, and how
//! the readback resolves (docs/01-architecture.md §Picking and snapping).

use std::sync::{Arc, Mutex};

use egui_wgpu::wgpu;

use crate::pick_id;

/// Physical-pixel side length of the square the pick readback samples
/// around the cursor. Scanning a small square instead of one pixel gives a
/// few pixels of forgiveness at silhouettes; the hit nearest the cursor
/// wins. Odd so the cursor pixel is exactly the center cell.
pub const PICK_REGION: u32 = 5;

/// ID-buffer pick target format: one packed [`pick_id`] per pixel.
pub const PICK_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::R32Uint;

/// wgpu requires `bytes_per_row` on a buffer-copy layout to be a multiple
/// of 256; a `PICK_REGION`-wide row is far under that even at `R32Uint`, so
/// `bytes_per_row` is just the alignment minimum and the readback buffer is
/// that minimum times the number of rows.
pub const PICK_ROW_STRIDE: wgpu::BufferAddress = 256;
pub const PICK_READBACK_SIZE: wgpu::BufferAddress =
    PICK_ROW_STRIDE * PICK_REGION as wgpu::BufferAddress;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickKind {
    Hover,
    Select,
}

/// The `PICK_REGION`-ish pixel square actually read back this frame —
/// clamped to the pick target's bounds, so it can be smaller near an edge
/// of a tiny viewport panel. `origin` is in physical pixels, top-left.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PickRegion {
    pub origin: (u32, u32),
    pub width: u32,
    pub height: u32,
    pub cursor: (u32, u32),
}

impl PickRegion {
    /// The region around `pixel` on a target of `size`, shifted inward
    /// rather than shrunk at the edges (except in a panel narrower than
    /// `PICK_REGION` pixels, which `ensure_offscreen` only guarantees to be
    /// at least 1×1).
    pub fn around(pixel: (u32, u32), size: (u32, u32)) -> Self {
        let width = PICK_REGION.min(size.0);
        let height = PICK_REGION.min(size.1);
        Self {
            origin: (
                (pixel.0.saturating_sub(PICK_REGION / 2)).min(size.0 - width),
                (pixel.1.saturating_sub(PICK_REGION / 2)).min(size.1 - height),
            ),
            width,
            height,
            cursor: pixel,
        }
    }
}

/// Everything a hover pick's answer depends on: which pixel was asked about
/// and the camera the ID buffer was rasterized with. Re-asking the same
/// question would decode to the id already in `Viewport::hovered`, so a
/// hover pick whose inputs are unchanged is skipped — otherwise a cursor
/// merely *resting* over the viewport re-renders the whole ID buffer,
/// copies it back, and requests the repaint that starts the next one, at
/// vsync rate forever.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PickInputs {
    pub pixel: (u32, u32),
    pub view_proj: [[f32; 4]; 4],
}

/// What `Viewport::ui` does about picking this frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickDecision {
    /// Nothing to do: a pick is in flight, the scene is empty, or the
    /// answer is already known.
    Nothing,
    /// The pointer left the viewport: forget the hover and the memo.
    ClearHover,
    Issue {
        kind: PickKind,
        pixel: (u32, u32),
    },
}

/// The per-frame pick policy, kept pure so the memo can be unit-tested:
/// at most one pick in flight; a completed click beats a plain hover; a
/// hover whose inputs match the last one issued is not re-issued.
pub fn decide_pick(
    busy: bool,
    clicked_at: Option<(u32, u32)>,
    hover_at: Option<(u32, u32)>,
    last: Option<PickInputs>,
    view_proj: [[f32; 4]; 4],
) -> PickDecision {
    if busy {
        return PickDecision::Nothing;
    }
    if let Some(pixel) = clicked_at {
        return PickDecision::Issue {
            kind: PickKind::Select,
            pixel,
        };
    }
    match hover_at {
        Some(pixel) if last != Some(PickInputs { pixel, view_proj }) => PickDecision::Issue {
            kind: PickKind::Hover,
            pixel,
        },
        Some(_) => PickDecision::Nothing,
        None => PickDecision::ClearHover,
    }
}

/// One in-flight ID-buffer readback. `result` is filled in by the
/// `map_async` callback whenever wgpu gets around to it — never blocked on.
/// Holds every pick id in `region`, row-major, so nearest-to-cursor can be
/// resolved once mapping completes.
pub struct PendingPick {
    pub kind: PickKind,
    pub region: PickRegion,
    pub result: Arc<Mutex<Option<Vec<u32>>>>,
}

/// Resolves a mapped-back pick region to the `(slot, triangle)` nearest
/// the cursor pixel, or `None` if every pixel was a miss.
pub fn resolve_pick_region(ids: &[u32], region: &PickRegion) -> Option<(u32, u32)> {
    let mut best: Option<(u32, (u32, u32))> = None; // (dist2, hit)
    for row in 0..region.height {
        for col in 0..region.width {
            let Some(hit) = pick_id::decode(ids[(row * region.width + col) as usize]) else {
                continue;
            };
            let px = region.origin.0 + col;
            let py = region.origin.1 + row;
            let dx = px.abs_diff(region.cursor.0);
            let dy = py.abs_diff(region.cursor.1);
            let dist2 = dx * dx + dy * dy;
            if best.is_none_or(|(best_dist2, _)| dist2 < best_dist2) {
                best = Some((dist2, hit));
            }
        }
    }
    best.map(|(_, hit)| hit)
}

#[cfg(test)]
mod tests {
    use super::*;

    const VP: [[f32; 4]; 4] = [[1.0; 4]; 4];
    const VP2: [[f32; 4]; 4] = [[2.0; 4]; 4];

    #[test]
    fn resting_cursor_does_not_reissue_the_hover_pick() {
        let pixel = (10, 20);
        let first = decide_pick(false, None, Some(pixel), None, VP);
        assert_eq!(
            first,
            PickDecision::Issue {
                kind: PickKind::Hover,
                pixel
            }
        );
        let memo = Some(PickInputs {
            pixel,
            view_proj: VP,
        });
        assert_eq!(
            decide_pick(false, None, Some(pixel), memo, VP),
            PickDecision::Nothing,
            "same pixel, same camera: the answer is already in `hovered`"
        );
        assert!(matches!(
            decide_pick(false, None, Some((11, 20)), memo, VP),
            PickDecision::Issue {
                kind: PickKind::Hover,
                ..
            }
        ));
        assert!(matches!(
            decide_pick(false, None, Some(pixel), memo, VP2),
            PickDecision::Issue {
                kind: PickKind::Hover,
                ..
            }
        ));
    }

    #[test]
    fn click_beats_hover_and_busy_beats_everything() {
        let memo = Some(PickInputs {
            pixel: (1, 1),
            view_proj: VP,
        });
        assert_eq!(
            decide_pick(false, Some((1, 1)), Some((1, 1)), memo, VP),
            PickDecision::Issue {
                kind: PickKind::Select,
                pixel: (1, 1)
            },
            "a click at the memoised pixel still issues a select pick"
        );
        assert_eq!(
            decide_pick(true, Some((1, 1)), Some((1, 1)), None, VP),
            PickDecision::Nothing
        );
        assert_eq!(
            decide_pick(false, None, None, memo, VP),
            PickDecision::ClearHover
        );
    }

    #[test]
    fn region_is_centred_and_shifted_inward_at_edges() {
        let r = PickRegion::around((50, 60), (100, 100));
        assert_eq!(r.origin, (48, 58));
        assert_eq!((r.width, r.height), (5, 5));
        assert_eq!(PickRegion::around((0, 0), (100, 100)).origin, (0, 0));
        assert_eq!(PickRegion::around((99, 99), (100, 100)).origin, (95, 95));
        let tiny = PickRegion::around((1, 0), (3, 1));
        assert_eq!((tiny.width, tiny.height), (3, 1));
        assert_eq!(tiny.origin, (0, 0));
    }

    #[test]
    fn nearest_hit_to_the_cursor_wins() {
        let region = PickRegion {
            origin: (10, 10),
            width: 5,
            height: 5,
            cursor: (12, 12),
        };
        let mut ids = vec![0u32; 25];
        assert_eq!(resolve_pick_region(&ids, &region), None);
        ids[0] = pick_id::encode(1, 7); // corner, dist² = 8
        assert_eq!(resolve_pick_region(&ids, &region), Some((1, 7)));
        ids[2 * 5 + 3] = pick_id::encode(2, 9); // one right of centre, dist² = 1
        assert_eq!(resolve_pick_region(&ids, &region), Some((2, 9)));
        ids[2 * 5 + 2] = pick_id::encode(3, 0); // the cursor pixel itself
        assert_eq!(resolve_pick_region(&ids, &region), Some((3, 0)));
    }
}
