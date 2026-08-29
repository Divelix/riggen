//! Visual snapshot scenarios (ADR-0003, docs/01-architecture.md §Testing).
//!
//! Each scenario drives the real `RiggenApp` headlessly and captures two
//! things: the rendered frame, compared against a committed PNG, and
//! `debug_state()`, compared against a committed JSON. The pair is the point —
//! the picture shows that something is wrong, the JSON says which number is.
//!
//! Run `UPDATE_SNAPSHOTS=1 cargo test -p riggen-app --test visual` after an
//! intentional UI change, and *look at the `.diff.png`* before committing.
//! Updating reflexively turns the suite into something that looks like
//! coverage without being any.
//!
//! Deliberately small. Nothing here should snapshot what an ordinary unit test
//! already covers.

mod harness;

use harness::{scenario, settle};

fn fixture(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/fixtures")
        .join(name)
}

/// The empty app: gradient background, axes triad, status bar. The broadest
/// regression net in the suite: almost any layout or render change moves
/// this frame.
#[test]
fn startup() {
    scenario("startup", |_harness| {});
}

/// One STL, fitted: the shaded cube, its bounds and the fitted camera.
#[test]
fn cube() {
    scenario("cube", |harness| {
        harness
            .state_mut()
            .open_path(&fixture("cube_binary.stl"))
            .expect("open cube fixture");
        harness.state_mut().fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        assert_eq!(state.instances.len(), 1);
        assert_eq!(state.instances[0].triangles, 12);
        assert_eq!(
            state.instances[0].bounds,
            Some([[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]])
        );
    });
}
