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

use egui_kittest::kittest::Queryable;
use harness::{click_at, pump_rendered, scenario, settle, with_app};

use riggen_app::Selection;
use riggen_core::glam::DVec3;
use riggen_core::{Command, Link, LinkId, Pose};

fn fixture(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/fixtures")
        .join(name)
}

/// Opens a mesh as a new link, or panics with the loader's message.
fn open_link(app: &mut riggen_app::RiggenApp, name: &str) -> LinkId {
    app.open_path(&fixture(name))
        .expect(name)
        .expect("a mesh opens as a link")
}

/// The empty app: gradient background, axes triad, status bar. The broadest
/// regression net in the suite: almost any layout or render change moves
/// this frame.
#[test]
fn startup() {
    scenario("startup", |harness| {
        let state = harness.state().debug_state();
        assert_eq!(state.document.links.len(), 1, "a new document has its root");
        assert!(!state.document.dirty);
        assert_eq!(state.document.file, None);
    });
}

/// One STL, fitted: the shaded cube, its bounds and the fitted camera. The
/// cube is a link under the root with a fixed joint at identity.
#[test]
fn cube() {
    scenario("cube", |harness| {
        let cube = open_link(harness.state_mut(), "cube_binary.stl");
        harness.state_mut().fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        assert_eq!(state.instances.len(), 1);
        assert_eq!(state.instances[0].triangles, 12);
        assert_eq!(
            state.instances[0].link.as_deref(),
            Some(cube.to_string().as_str())
        );
        assert_eq!(
            state.instances[0].bounds,
            Some([[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]])
        );
        assert_eq!(state.document.links.len(), 2);
        assert_eq!(state.document.links[1].name, "cube_binary");
        assert_eq!(state.document.joints[0].kind, "Fixed");
        assert!(state.document.dirty, "a dropped mesh is an edit");
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
        open_link(harness.state_mut(), "cube_binary.stl");
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
/// showing selection alone rather than selection under a hover. The click
/// selects the *link* owning the hit instance in the document.
#[test]
fn select_cube() {
    scenario("select_cube", |harness| {
        let cube = open_link(harness.state_mut(), "cube_binary.stl");
        harness.state_mut().fit_view_now();
        settle(harness);

        let center = harness
            .state()
            .viewport_center()
            .expect("viewport laid out");
        click_at(harness, center);

        let state = harness.state().debug_state();
        assert_eq!(
            state.selection.selected.map(|h| h.instance),
            Some(0),
            "clicking the middle of a fitted view should select the cube: {:?}",
            state.selection
        );
        assert_eq!(
            state.selection.hovered, None,
            "the pointer left the viewport"
        );
        assert_eq!(state.document.selection, Some(format!("link {cube}")));
        assert_eq!(
            harness.state().selection(),
            riggen_app::Selection::Link(cube)
        );
    });
}

/// The three fixtures — binary STL, ASCII STL, OBJ — side by side: every
/// loader through `open_path`, three links under the root, placed by
/// editing their fixed joints' origins, fitted together.
#[test]
fn three_parts() {
    scenario("three_parts", |harness| {
        let app = harness.state_mut();
        for (i, name) in ["cube_binary.stl", "cube_ascii.stl", "cube.obj"]
            .iter()
            .enumerate()
        {
            let link = open_link(app, name);
            let joint = app.robot().parent_joint(link).expect("a parent joint");
            let mut edited = app.robot().joints[&joint].clone();
            edited.origin = Pose::from_translation(DVec3::new(1.5 * i as f64, 0.0, 0.0));
            app.apply(Command::SetJoint(joint, edited))
                .expect("SetJoint");
        }
        app.fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        assert_eq!(state.instances.len(), 3);
        assert!(state.instances.iter().all(|i| i.triangles == 12));
        assert_eq!(state.instances[2].position, [3.0, 0.0, 0.0]);
        let names: Vec<&str> = state
            .document
            .links
            .iter()
            .map(|l| l.name.as_str())
            .collect();
        assert_eq!(names, ["base_link", "cube_binary", "cube_ascii", "cube"]);
    });
}

/// The corpus file: base and arm, one revolute hinge. At `q = 0` the arm
/// cube sits on the base cube: the arm's instance is at the FK pose
/// `hinge.origin ∘ geom.pose = (0, 0, 0.5) + (0, 0, 0.5)`.
#[test]
fn pendulum() {
    scenario("pendulum", |harness| {
        let opened = harness
            .state_mut()
            .open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        assert_eq!(opened, None, "a document opens as a document, not a link");
        harness.state_mut().fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        assert_eq!(state.document.file.as_deref(), Some("pendulum.riggen"));
        assert!(!state.document.dirty);
        assert_eq!(state.status, None, "no warnings: the fixture hashes match");
        assert_eq!(state.document.links.len(), 2);
        assert_eq!(state.document.joints.len(), 1);
        assert_eq!(state.document.joints[0].kind, "Revolute");
        assert_eq!(state.instances.len(), 2);
        assert_eq!(state.instances[0].position, [0.0, 0.0, 0.0]);
        assert_eq!(state.instances[1].position, [0.0, 0.0, 1.0]);
    });
}

/// A millimetre part: the cube fixture at `scale = 0.001` is a 1 mm cube.
/// The fit sets the camera's depth range from the radius, so it is drawn
/// rather than clipped by M0's fixed 1 cm near plane.
#[test]
fn mm_scale_part() {
    scenario("mm_scale_part", |harness| {
        let app = harness.state_mut();
        app.set_import_scale(riggen_app::RiggenApp::DEFAULT_IMPORT_SCALE);
        let link = open_link(app, "cube_binary.stl");
        let mesh = app.robot().links[&link].visuals[0].mesh;
        assert_eq!(app.robot().assets[&mesh].scale, 0.001);
        app.fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        assert_eq!(
            state.instances[0].bounds,
            Some([[-0.0005, -0.0005, -0.0005], [0.0005, 0.0005, 0.0005]])
        );
        let camera = &state.camera;
        let radius = 0.0005f64 * 3f64.sqrt();
        assert!(
            camera.near < camera.distance - radius,
            "near {} clips a part at distance {}",
            camera.near,
            camera.distance
        );
        assert!(camera.far > camera.distance + radius);
        assert!(camera.distance < 0.02, "fitted close: {}", camera.distance);
    });
}

/// A file that does not load leaves the document alone and says why in the
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
        assert_eq!(app.robot().links.len(), 1);
        assert!(!app.history().is_dirty());

        let err = app
            .open_path(&fixture("cube.obj").with_extension("ply"))
            .unwrap_err();
        assert!(err.contains("unsupported format"), "{err}");
        assert!(app.debug_state().instances.is_empty());

        let err = app
            .open_path(&fixture("does_not_exist.riggen"))
            .unwrap_err();
        assert!(err.contains("does_not_exist.riggen"), "{err}");
        assert_eq!(app.debug_state().document.file, None);

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

/// Dropping a mesh is one undoable edit: undo removes the link and its
/// instance, redo brings both back without reloading (the asset stayed
/// registered), and the dirty flag follows the history.
#[test]
fn drop_undo_redo() {
    with_app(|harness| {
        let app = harness.state_mut();
        let cube = open_link(app, "cube_binary.stl");
        assert_eq!(app.robot().links.len(), 2);
        assert_eq!(app.debug_state().instances.len(), 1);
        assert!(app.history().is_dirty());

        assert!(app.undo());
        assert_eq!(app.robot().links.len(), 1);
        assert!(app.debug_state().instances.is_empty());
        assert!(!app.history().is_dirty());
        assert_eq!(app.robot().assets.len(), 1, "the asset stays registered");

        assert!(app.redo());
        assert_eq!(app.robot().links.len(), 2);
        assert!(app.robot().links.contains_key(&cube));
        let state = app.debug_state();
        assert_eq!(state.instances.len(), 1);
        assert_eq!(
            state.instances[0].link.as_deref(),
            Some(cube.to_string().as_str())
        );

        // A second drop with the first selected lands under it.
        app.select(riggen_app::Selection::Link(cube));
        let child = open_link(app, "cube_ascii.stl");
        let joint = app.robot().parent_joint(child).unwrap();
        assert_eq!(app.robot().joints[&joint].parent, cube);
        assert_eq!(app.robot().links[&child].name, "cube_ascii");
        // And the same file again gets a deduplicated name.
        let again = open_link(app, "cube_ascii.stl");
        assert_eq!(app.robot().links[&again].name, "cube_ascii_2");
    });
}

/// Opening a document replaces everything: the old links, instances,
/// history and selection are gone, and the file is clean.
#[test]
fn open_document_replaces_the_scene() {
    with_app(|harness| {
        let app = harness.state_mut();
        let cube = open_link(app, "cube_binary.stl");
        app.select(riggen_app::Selection::Link(cube));
        assert!(app.history().is_dirty());

        app.load_files(&[fixture("pendulum.riggen")]);
        let state = app.debug_state();
        assert_eq!(state.document.file.as_deref(), Some("pendulum.riggen"));
        assert!(!state.document.dirty);
        assert_eq!(state.document.selection, None);
        assert_eq!(state.document.links.len(), 2);
        assert_eq!(state.instances.len(), 2);
        assert_eq!(state.status.as_deref(), Some("opened 1 file"));
        assert!(!app.undo(), "a fresh document has no history");

        // Swinging the hinge moves the arm's instance, not the base's.
        let hinge = *app.robot().joints.keys().next().unwrap();
        app.set_joint_value(hinge, std::f64::consts::FRAC_PI_2);
        let state = app.debug_state();
        assert_eq!(state.instances[0].position, [0.0, 0.0, 0.0]);
        // Rotating (0, 0, 0.5) about +Y by 90° gives (0.5, 0, 0), plus the
        // hinge origin (0, 0, 0.5).
        assert_eq!(state.instances[1].position, [0.5, 0.0, 0.5]);
        // Values are clamped to the limits (±90°).
        app.set_joint_value(hinge, 10.0);
        assert!((app.joint_value(hinge) - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    });
}

/// The tree panel with the pendulum open and `arm` selected *through the
/// tree*: the row is highlighted, the arm's instance is tinted in the
/// viewport, and the status bar names it.
#[test]
fn tree_pendulum() {
    scenario("tree_pendulum", |harness| {
        harness
            .state_mut()
            .open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        harness.state_mut().fit_view_now();
        settle(harness);

        harness.get_by_label("arm").click();
        pump_rendered(harness, 4);

        let state = harness.state().debug_state();
        let arm = state
            .document
            .links
            .iter()
            .find(|l| l.name == "arm")
            .expect("arm link");
        assert_eq!(state.document.selection, Some(format!("link {}", arm.id)));
        let arm_instance = state
            .instances
            .iter()
            .find(|i| i.link.as_deref() == Some(arm.id.as_str()))
            .expect("arm instance");
        assert_eq!(
            state.selection.selected.map(|h| h.instance),
            Some(arm_instance.id),
            "the tree selection reached the viewport"
        );
    });
}

/// Reparenting keeps the world pose: `arm` is hung under a cube placed at
/// x = 1.5 (through the command API — kittest cannot drag), the tree
/// shows it there, and the arm's instance has not moved. The hinge origin
/// absorbed the difference.
#[test]
fn tree_reparent() {
    scenario("tree_reparent", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let cube = open_link(app, "cube.obj");
        let cube_joint = app.robot().parent_joint(cube).unwrap();
        let mut edited = app.robot().joints[&cube_joint].clone();
        edited.origin = Pose::from_translation(DVec3::new(1.5, 0.0, 0.0));
        app.apply(Command::SetJoint(cube_joint, edited)).unwrap();
        let arm = *app
            .robot()
            .links
            .iter()
            .find(|(_, l)| l.name == "arm")
            .map(|(id, _)| id)
            .unwrap();
        let hinge = app.robot().parent_joint(arm).unwrap();
        let before = app.debug_state();

        app.apply(Command::Reparent {
            link: arm,
            new_parent: cube,
            keep_world_pose: true,
        })
        .unwrap();
        app.select(Selection::Link(arm));
        app.fit_view_now();
        settle(harness);

        let app = harness.state();
        let state = app.debug_state();
        assert_eq!(app.robot().joints[&hinge].parent, cube);
        let origin = app.robot().joints[&hinge].origin.t;
        assert!(
            (origin - DVec3::new(-1.5, 0.0, 0.5)).length() < 1e-9,
            "{origin}"
        );
        // Every instance is where it was.
        for (was, is) in before.instances.iter().zip(&state.instances) {
            assert_eq!(
                (was.link.as_ref(), was.position),
                (is.link.as_ref(), is.position)
            );
        }
        let hinge_debug = state
            .document
            .joints
            .iter()
            .find(|j| j.name == "hinge")
            .unwrap();
        assert_eq!(hinge_debug.parent, cube.to_string());
    });
}

/// The tree's own edits: "+ Link" adds an empty link under the selection
/// and starts a rename, typing + Enter commits it, Delete removes the
/// selection, the root refuses to go, and a joint selection is a drop
/// target too (under the joint's child).
#[test]
fn tree_add_rename_delete() {
    with_app(|harness| {
        harness
            .state_mut()
            .open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        settle(harness);
        harness.get_by_label("arm").click();
        harness.step();
        let arm = match harness.state().selection() {
            Selection::Link(l) => l,
            other => panic!("arm should be selected: {other:?}"),
        };

        harness.get_by_label("+ Link").click();
        harness.step();
        let app = harness.state();
        assert_eq!(app.robot().links.len(), 3);
        let new = match app.selection() {
            Selection::Link(l) => l,
            other => panic!("the new link should be selected: {other:?}"),
        };
        assert_eq!(app.robot().links[&new].name, "link");
        let new_joint = app.robot().parent_joint(new).unwrap();
        assert_eq!(app.robot().joints[&new_joint].parent, arm);
        assert_eq!(
            app.debug_state().ui.renaming,
            Some((new.to_string(), "link".into())),
            "+ Link starts an inline rename"
        );

        // Replace the text and commit with Enter. Delete must not fire
        // while the field has focus.
        harness.step();
        let field = harness.get_by_role(egui::accesskit::Role::TextInput);
        field.focus();
        harness.step();
        harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
        harness.step();
        harness
            .get_by_role(egui::accesskit::Role::TextInput)
            .type_text("hand");
        harness.step();
        harness.key_press(egui::Key::Delete);
        harness.step();
        assert_eq!(
            harness.state().robot().links.len(),
            3,
            "Delete in a text field edits text"
        );
        harness.key_press(egui::Key::Enter);
        harness.step();
        harness.step();
        let app = harness.state();
        assert_eq!(app.debug_state().ui.renaming, None);
        assert_eq!(app.robot().links[&new].name, "hand");
        assert!(app.history().is_dirty());

        // Delete removes the selected link (still `hand`).
        harness.key_press(egui::Key::Delete);
        harness.step();
        harness.step();
        let app = harness.state();
        assert_eq!(app.robot().links.len(), 2);
        assert_eq!(app.selection(), Selection::None);

        // Selecting the root and pressing Delete is refused with a reason.
        harness.get_by_label("base_link").click();
        harness.step();
        harness.key_press(egui::Key::Delete);
        harness.step();
        harness.step();
        let app = harness.state();
        assert_eq!(app.robot().links.len(), 2);
        assert_eq!(
            app.debug_state().status.as_deref(),
            Some("the root link cannot be removed")
        );

        // A dropped mesh with a joint selected lands under that joint's child.
        let hinge = *app.robot().joints.keys().next().unwrap();
        harness.state_mut().select(Selection::Joint(hinge));
        let cube = open_link(harness.state_mut(), "cube.obj");
        let app = harness.state();
        let cube_joint = app.robot().parent_joint(cube).unwrap();
        assert_eq!(app.robot().joints[&cube_joint].parent, arm);
        let _ = Link::new("unused");
    });
}
