//! Packs `(instance slot, triangle)` into the `u32` pixel values written by
//! the ID-buffer pick pass (docs/01-architecture.md §Picking and snapping).
//!
//! Layout: `slot` in the top 12 bits, `triangle + 1` in the low 20. `0` is
//! reserved for "nothing under the cursor" — the pick target's clear value —
//! which is why the triangle is stored offset by one: slot 0, triangle 0 is
//! a real hit and must not read as a miss. The 20-bit field is where
//! [`riggen_mesh::MAX_TRIANGLES`] comes from.
//!
//! The slot is the [`crate::Scene`]'s per-instance draw slot, not the
//! [`crate::InstanceId`]: ids grow without bound over a session, slots are
//! recycled.

pub const SLOT_BITS: u32 = 12;
pub const TRIANGLE_BITS: u32 = 20;
const SLOT_MASK: u32 = (1 << SLOT_BITS) - 1;
const TRIANGLE_MASK: u32 = (1 << TRIANGLE_BITS) - 1;

/// Packs a hit. Both fields are masked to their widths; callers keep them in
/// range ([`crate::MAX_INSTANCES`], [`riggen_mesh::MAX_TRIANGLES`]) and
/// debug builds assert it.
pub fn encode(slot: u32, triangle: u32) -> u32 {
    debug_assert!(
        slot <= SLOT_MASK,
        "instance slot {slot} does not fit {SLOT_BITS} bits"
    );
    debug_assert!(
        triangle < TRIANGLE_MASK,
        "triangle {triangle} does not fit {TRIANGLE_BITS} bits with the +1 offset"
    );
    ((slot & SLOT_MASK) << TRIANGLE_BITS) | ((triangle + 1) & TRIANGLE_MASK)
}

/// Unpacks a pixel value into `(slot, triangle)`; `None` for the clear
/// value.
pub fn decode(id: u32) -> Option<(u32, u32)> {
    let triangle_plus_one = id & TRIANGLE_MASK;
    if triangle_plus_one == 0 {
        return None;
    }
    Some((id >> TRIANGLE_BITS, triangle_plus_one - 1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn miss_is_zero_and_zero_is_a_miss() {
        assert_eq!(decode(0), None);
        assert_ne!(encode(0, 0), 0, "slot 0, triangle 0 is a real hit");
    }

    #[test]
    fn round_trips_across_the_ranges() {
        for slot in [0, 1, 7, 4095] {
            for triangle in [0, 1, 12, 65_535, riggen_mesh::MAX_TRIANGLES as u32 - 1] {
                let id = encode(slot, triangle);
                assert_eq!(
                    decode(id),
                    Some((slot, triangle)),
                    "slot {slot} tri {triangle}"
                );
            }
        }
    }

    #[test]
    fn widths_agree_with_the_scene_and_mesh_caps() {
        assert_eq!(1usize << SLOT_BITS, crate::MAX_INSTANCES);
        assert_eq!((1usize << TRIANGLE_BITS) - 1, riggen_mesh::MAX_TRIANGLES);
        // The largest legal hit uses every bit and is still not the miss value.
        let id = encode(
            crate::MAX_INSTANCES as u32 - 1,
            riggen_mesh::MAX_TRIANGLES as u32 - 1,
        );
        assert_eq!(id, u32::MAX);
        assert!(decode(id).is_some());
    }

    #[test]
    #[should_panic(expected = "does not fit")]
    fn out_of_range_triangle_is_caught_in_debug() {
        encode(0, riggen_mesh::MAX_TRIANGLES as u32);
    }
}
