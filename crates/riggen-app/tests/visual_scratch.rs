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
//! against nothing. `RIGGEN_SCRATCH_OPEN=<path>` (relative to the workspace
//! root) opens a `.riggen` or a mesh and fits the view first, so a document
//! is looked at without editing anything:
//!
//! ```sh
//! RIGGEN_SCRATCH_OPEN=assets/fixtures/pendulum.riggen \
//!   cargo test -p riggen-app --test visual_scratch -- --nocapture
//! ```
//!
//! For any other state, edit the body below — the same helpers the real
//! scenarios use are available — then read the PNG, and the JSON when the
//! question is "which coordinate is wrong".
//!
//! **Revert your edits before committing.** This file is tracked; its default
//! body is the startup state on purpose. The `visual-debug` skill has the
//! recipes.

#[path = "visual/harness.rs"]
#[allow(dead_code, reason = "the scratch target uses only part of the harness")]
mod harness;

#[test]
fn scratch() {
    harness::scratch(|harness| {
        if let Some(path) = std::env::var_os("RIGGEN_SCRATCH_OPEN") {
            // The test binary runs in the crate directory; the variable is
            // typed at the workspace root.
            let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
            let path = root.join(path);
            harness
                .state_mut()
                .open_path(&path)
                .unwrap_or_else(|err| panic!("RIGGEN_SCRATCH_OPEN {}: {err}", path.display()));
            harness.state_mut().fit_view_now();
            harness::settle(harness);
            harness::pump_rendered(harness, 4);
        }
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
