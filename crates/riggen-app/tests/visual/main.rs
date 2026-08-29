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

use harness::{click_at, pump_rendered, scenario, settle, with_app};

use riggen_mesh::glam::DVec3;

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

/// Hover restyle on the cube: the whole-instance tint and the `hover:`
/// readout in the status bar.
///
/// The rendered pump after `hover_at` is load-bearing — the pick is an
/// ID-buffer pass with an async readback, so the restyle appears several
/// *rendered* frames after the cursor moves.
#[test]
fn hover_cube() {
    scenario("hover_cube", |harness| {
        harness
            .state_mut()
            .open_path(&fixture("cube_binary.stl"))
            .expect("open cube fixture");
        harness.state_mut().fit_view_now();
        settle(harness);

        let center = harness
            .state()
            .viewport_center()
            .expect("viewport laid out");
        harness.hover_at(center);
        pump_rendered(harness, 8);

        let hovered = harness.state().debug_state().selection.hovered;
        assert_eq!(
            hovered.map(|h| h.instance),
            Some(0),
            "hovering the middle of a fitted view should hit the cube: {hovered:?}"
        );
        assert!(hovered.unwrap().triangle < 12);
    });
}

/// Selection restyle: click = select, and `PointerGone` leaves the frame
/// showing selection alone rather than selection under a hover.
#[test]
fn select_cube() {
    scenario("select_cube", |harness| {
        harness
            .state_mut()
            .open_path(&fixture("cube_binary.stl"))
            .expect("open cube fixture");
        harness.state_mut().fit_view_now();
        settle(harness);

        let center = harness
            .state()
            .viewport_center()
            .expect("viewport laid out");
        click_at(harness, center);

        let selection = harness.state().debug_state().selection;
        assert_eq!(
            selection.selected.map(|h| h.instance),
            Some(0),
            "clicking the middle of a fitted view should select the cube: {selection:?}"
        );
        assert_eq!(selection.hovered, None, "the pointer left the viewport");
    });
}

/// The three fixtures — binary STL, ASCII STL, OBJ — side by side: every
/// loader through `open_path`, three instances with their own model
/// matrices, fitted together.
#[test]
fn three_parts() {
    scenario("three_parts", |harness| {
        let app = harness.state_mut();
        for (i, name) in ["cube_binary.stl", "cube_ascii.stl", "cube.obj"]
            .iter()
            .enumerate()
        {
            let id = app.open_path(&fixture(name)).expect(name);
            assert!(app.place_instance(id, DVec3::new(1.5 * i as f64, 0.0, 0.0)));
        }
        app.fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        assert_eq!(state.instances.len(), 3);
        assert!(state.instances.iter().all(|i| i.triangles == 12));
        assert_eq!(state.instances[2].position, [3.0, 0.0, 0.0]);
    });
}

/// A file that does not load leaves the scene alone and says why in the
/// status bar; a batch with one bad file still opens the others.
#[test]
fn bad_path_reports_and_adds_nothing() {
    with_app(|harness| {
        let app = harness.state_mut();
        let missing = fixture("does_not_exist.stl");
        let err = app.open_path(&missing).unwrap_err();
        assert!(err.contains("does_not_exist.stl"), "{err}");
        assert_eq!(app.debug_state().status.as_deref(), Some(err.as_str()));
        assert!(app.debug_state().instances.is_empty());

        let err = app
            .open_path(&fixture("cube.obj").with_extension("ply"))
            .unwrap_err();
        assert!(err.contains("unsupported format"), "{err}");
        assert!(app.debug_state().instances.is_empty());

        app.load_files(&[missing, fixture("cube.obj")]);
        assert_eq!(app.debug_state().instances.len(), 1);
        let status = app.debug_state().status.unwrap();
        assert!(status.starts_with("opened 1 of 2 files"), "{status}");

        app.load_files(&[fixture("cube_binary.stl")]);
        assert_eq!(app.debug_state().status.as_deref(), Some("opened 1 file"));
        assert_eq!(app.debug_state().instances.len(), 2);
        assert!(app.debug_state().camera.animating, "loads fit the view");
    });
}
