//! A throwaway visual capture, for looking at the app rather than pinning it.
//!
//! Not run by `cargo test`: this target is `test = false` in `Cargo.toml`, so
//! it stays out of CI, out of `-- --ignored`, and out of everyone's way until
//! asked for by name.
//!
//! ```sh
//! cargo test -p riggen-app --test visual_scratch -- --nocapture
//! ```
//!
//! It writes `target/visual-scratch/scratch.{png,json}` and compares them
//! against nothing. Edit the body below to reach whatever state you want to
//! look at — the same helpers the real scenarios use are available — then read
//! the PNG, and the JSON when the question is "which coordinate is wrong".
//!
//! **Revert your edits before committing.** This file is tracked; its default
//! body is the startup state on purpose.

#[path = "visual/harness.rs"]
#[allow(dead_code, reason = "the scratch target uses only part of the harness")]
mod harness;

#[test]
fn scratch() {
    harness::scratch(|harness| {
        // ---- edit below ------------------------------------------------
        // Everything the real scenarios can do works here, e.g.:
        //
        //   harness.state_mut().open_path(std::path::Path::new("…"));
        //   harness::settle(harness);
        //   harness::click_at(harness, pos);         // needs rendered frames
        //   harness::pump_rendered(harness, 8);      // after any GPU-dependent input
        let _ = harness;
        // ---- edit above ------------------------------------------------
    });
}
