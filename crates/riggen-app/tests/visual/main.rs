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

use egui_kittest::kittest::{NodeT, Queryable};
use harness::{click_at, click_widget, pump_rendered, scenario, settle, synthetic_drag, with_app};

use riggen_app::{Selection, Tool, ZERO_CONFIG_STATUS};
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

        // The rename field took focus when it appeared. Replace the text
        // and commit with Enter. Delete must not fire while it has focus.
        harness.step();
        harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
        harness.step();
        harness.event(egui::Event::Text("hand".to_owned()));
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

/// The properties panel for a link: name, material, the one mesh with
/// its pose (z = 0.5 m), scale and fix-up.
#[test]
fn properties_link() {
    scenario("properties_link", |harness| {
        harness
            .state_mut()
            .open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        harness.state_mut().fit_view_now();
        settle(harness);
        harness.get_by_label("arm").click();
        pump_rendered(harness, 4);

        let state = harness.state().debug_state();
        assert!(
            state
                .document
                .selection
                .as_deref()
                .unwrap()
                .starts_with("link ")
        );
        // The name field shows the link name; the golden pins the rest.
        assert_eq!(harness.get_by_label("name").value().as_deref(), Some("arm"));
    });
}

/// The properties panel for a joint: kind, origin, axis, limits in
/// degrees, dynamics.
#[test]
fn properties_joint() {
    scenario("properties_joint", |harness| {
        harness
            .state_mut()
            .open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        harness.state_mut().fit_view_now();
        settle(harness);
        harness.get_by_label("hinge · revolute").click();
        pump_rendered(harness, 4);

        let state = harness.state().debug_state();
        assert!(
            state
                .document
                .selection
                .as_deref()
                .unwrap()
                .starts_with("joint ")
        );
        assert_eq!(
            harness.get_by_label("lower °").value().as_deref(),
            Some("-90")
        );
        assert_eq!(
            harness.get_by_label("upper °").value().as_deref(),
            Some("90")
        );
        assert_eq!(
            harness.get_by_label("damping").value().as_deref(),
            Some("0.1")
        );
    });
}

/// Replaces a field's text and commits it with Enter.
fn type_into(
    harness: &mut egui_kittest::Harness<'_, riggen_app::RiggenApp>,
    node_label: &str,
    nth: usize,
    text: &str,
) {
    let node = harness
        .get_all_by_label(node_label)
        .nth(nth)
        .unwrap_or_else(|| panic!("field {node_label:?} #{nth}"));
    node.focus();
    harness.step();
    harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
    harness.step();
    harness
        .get_all_by_label(node_label)
        .nth(nth)
        .unwrap()
        .type_text(text);
    harness.step();
    harness.key_press(egui::Key::Enter);
    harness.step();
    harness.step();
}

/// Typing an origin and a yaw into the hinge's fields moves the arm's
/// instance to the FK pose: origin (0.25, 0, 0.5), Rz(90°) applied to the
/// geom offset (0, 0, 0.5) stays (0, 0, 0.5), so the cube sits at
/// (0.25, 0, 1). Two commits, two history entries.
#[test]
fn typing_origin_and_rpy_moves_the_arm() {
    with_app(|harness| {
        harness
            .state_mut()
            .open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        settle(harness);
        harness.get_by_label("hinge · revolute").click();
        harness.step();
        let depth = harness.state().history().undo_depth();

        // The origin row is the first "x" / "yaw" in the panel; the axis
        // row is the second "x".
        type_into(harness, "x", 0, "0.25");
        type_into(harness, "yaw", 0, "90");

        let app = harness.state();
        let state = app.debug_state();
        let hinge = app
            .robot()
            .joints
            .values()
            .find(|j| j.name == "hinge")
            .unwrap();
        assert!(
            (hinge.origin.t - DVec3::new(0.25, 0.0, 0.5)).length() < 1e-9,
            "{:?}",
            hinge.origin
        );
        let (_, rpy) = hinge.origin.to_xyz_rpy();
        assert!((rpy.z - std::f64::consts::FRAC_PI_2).abs() < 1e-9, "{rpy}");
        assert_eq!(state.instances[1].position, [0.25, 0.0, 1.0]);
        assert_eq!(
            app.history().undo_depth(),
            depth + 2,
            "one commit per field"
        );

        // The axis row: normalised on commit.
        type_into(harness, "x", 1, "3");
        let app = harness.state();
        let hinge = app
            .robot()
            .joints
            .values()
            .find(|j| j.name == "hinge")
            .unwrap();
        assert!(
            (hinge.axis - DVec3::new(3.0, 1.0, 0.0).normalize()).length() < 1e-9,
            "{}",
            hinge.axis
        );

        // A limit typed in degrees lands in radians.
        type_into(harness, "upper °", 0, "45");
        let app = harness.state();
        let hinge = app
            .robot()
            .joints
            .values()
            .find(|j| j.name == "hinge")
            .unwrap();
        assert!((hinge.limits.unwrap().upper - std::f64::consts::FRAC_PI_4).abs() < 1e-9);
        assert_eq!(app.history().undo_depth(), depth + 4);
    });
}

/// Focusing every field in turn and leaving it commits its unchanged
/// value, which is a no-op: the history does not grow. Same for the link
/// panel, plus a real rename and a second mesh through the API.
#[test]
fn clicking_through_every_field_adds_no_history_entry() {
    with_app(|harness| {
        harness
            .state_mut()
            .open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        settle(harness);
        for row in ["hinge · revolute", "arm"] {
            harness.get_by_label(row).click();
            harness.step();
            let depth = harness.state().history().undo_depth();
            let fields: Vec<egui::Rect> = harness
                .get_all_by_role(egui::accesskit::Role::TextInput)
                .map(|n| n.rect())
                .collect();
            assert!(fields.len() >= 5, "{row}: {} fields", fields.len());
            for rect in fields {
                let pos = rect.center();
                harness.event(egui::Event::PointerMoved(pos));
                harness.event(egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: Default::default(),
                });
                harness.event(egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: Default::default(),
                });
                harness.step();
            }
            harness.key_press(egui::Key::Enter);
            harness.step();
            harness.step();
            assert_eq!(harness.state().history().undo_depth(), depth, "{row}");
        }

        // Renaming through the panel is one entry; a second mesh on the
        // link is another and shows up as an instance at the link's pose.
        let depth = harness.state().history().undo_depth();
        type_into(harness, "name", 0, "upper_arm");
        let app = harness.state();
        assert!(app.robot().links.values().any(|l| l.name == "upper_arm"));
        assert_eq!(app.history().undo_depth(), depth + 1);
        let arm = match app.selection() {
            Selection::Link(l) => l,
            other => panic!("{other:?}"),
        };
        let geom = harness
            .state_mut()
            .add_mesh_to_link(arm, &fixture("cube.obj"))
            .expect("add mesh");
        let app = harness.state();
        assert_eq!(app.robot().links[&arm].visuals.len(), 2);
        assert_eq!(app.robot().links[&arm].visuals[1].id, geom);
        let state = app.debug_state();
        assert_eq!(state.instances.len(), 3);
        assert_eq!(
            state.instances[2].position,
            [0.0, 0.0, 0.5],
            "at the link frame"
        );
        assert_eq!(app.history().undo_depth(), depth + 2);
    });
}

/// The joint sliders window with the hinge at 45°: the arm cube swings
/// about +Y through the hinge at (0, 0, 0.5), so its offset (0, 0, 0.5)
/// becomes (sin 45°, 0, cos 45°) · 0.5 and the instance sits at
/// (0.353553, 0, 0.853553).
#[test]
fn pendulum_swing() {
    scenario("pendulum_swing", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        app.set_joints_window_open(true);
        let hinge = *app.robot().joints.keys().next().unwrap();
        app.set_joint_value(hinge, std::f64::consts::FRAC_PI_4);
        app.fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        assert_eq!(state.ui.windows, vec!["joints"]);
        assert_eq!(
            state.document.joints[0].q,
            riggen_app::debug::round(std::f64::consts::FRAC_PI_4)
        );
        assert_eq!(state.instances[0].position, [0.0, 0.0, 0.0]);
        assert_eq!(state.instances[1].position, [0.353553, 0.0, 0.853553]);
        // The slider reads 45°.
        let slider = harness.get_by_role(egui::accesskit::Role::Slider);
        let value = slider.accesskit_node().numeric_value();
        assert!(value.is_some_and(|v| (v - 45.0).abs() < 1e-6), "{value:?}");
    });
}

/// `q` follows the limits: editing the upper limit below the current
/// value clamps it and moves the instance; "Reset all" zeroes it; the
/// Window menu toggles the window.
#[test]
fn joint_value_clamps_to_edited_limits() {
    with_app(|harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let hinge = *app.robot().joints.keys().next().unwrap();
        app.set_joint_value(hinge, std::f64::consts::FRAC_PI_4);
        let mut edited = app.robot().joints[&hinge].clone();
        edited.limits.as_mut().unwrap().upper = 30f64.to_radians();
        app.apply(Command::SetJoint(hinge, edited)).unwrap();
        assert!((app.joint_value(hinge) - 30f64.to_radians()).abs() < 1e-12);
        let state = app.debug_state();
        assert_eq!(
            state.document.joints[0].q,
            riggen_app::debug::round(std::f64::consts::FRAC_PI_6)
        );
        assert_eq!(state.instances[1].position, [0.25, 0.0, 0.933013]);
        // Setting past the limit clamps too.
        app.set_joint_value(hinge, 10.0);
        assert!((app.joint_value(hinge) - 30f64.to_radians()).abs() < 1e-12);

        // Window menu → Joints opens the window; Reset all zeroes q.
        let depth = app.history().undo_depth();
        assert!(!app.joints_window_open());
        harness.get_by_label("Window").click();
        harness.step();
        harness.get_by_label("Joints").click();
        harness.step();
        assert!(harness.state().joints_window_open());
        assert_eq!(harness.state().debug_state().ui.windows, vec!["joints"]);
        harness.get_by_label("Reset all").click();
        harness.step();
        let app = harness.state();
        assert_eq!(app.joint_value(hinge), 0.0);
        assert_eq!(app.debug_state().instances[1].position, [0.0, 0.0, 1.0]);
        assert_eq!(
            app.history().undo_depth(),
            depth,
            "joint values are not edits"
        );
    });
}

/// The tool toolbar over the pendulum: Select active, the other four
/// waiting. It floats in the viewport's top-left corner, which is why every
/// other golden moved with this step.
#[test]
fn toolbar() {
    scenario("toolbar", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        app.fit_view_now();
        settle(harness);
        assert_eq!(harness.state().debug_state().ui.tool, "Select");

        click_widget(harness, "Rotate");
        settle(harness);
        assert_eq!(harness.state().tool(), Tool::Rotate);
        assert_eq!(harness.state().debug_state().ui.tool, "Rotate");
    });
}

/// Clicking through the toolbar, Esc back to Select, and the
/// zero-configuration rule: an editing tool rewinds the sliders and says so
/// (plans/m2-placement-ux OPEN 1).
#[test]
fn tools_switch_and_reset_the_configuration() {
    with_app(|harness| {
        for tool in Tool::ALL {
            click_widget(harness, tool.label());
            assert_eq!(harness.state().tool(), tool, "clicking {}", tool.label());
        }

        // Esc leaves an editing tool; from Select it is nobody's business.
        harness.key_press(egui::Key::Escape);
        harness.step();
        assert_eq!(harness.state().tool(), Tool::Select);

        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let hinge = *app.robot().joints.keys().next().unwrap();
        app.set_joint_value(hinge, std::f64::consts::FRAC_PI_4);
        let depth = app.history().undo_depth();

        // Select does not disturb a posed document…
        app.set_tool(Tool::Select);
        assert_eq!(app.joint_value(hinge), std::f64::consts::FRAC_PI_4);
        assert_eq!(app.debug_state().status, None);

        // …an editing tool rewinds it, and says why.
        app.set_tool(Tool::PlaceJoint);
        assert_eq!(app.joint_value(hinge), 0.0);
        assert_eq!(
            app.debug_state().status.as_deref(),
            Some(ZERO_CONFIG_STATUS)
        );
        assert_eq!(app.debug_state().instances[1].position, [0.0, 0.0, 1.0]);
        assert_eq!(
            app.history().undo_depth(),
            depth,
            "resetting q is not an edit"
        );
    });
}

/// The gizmo's own origin on screen, which is where its view-plane handle
/// sits: a drag from there translates in the plane of the screen, and is the
/// one handle a script can aim at without knowing the gizmo's geometry.
fn gizmo_handle(harness: &egui_kittest::Harness<'_, riggen_app::RiggenApp>) -> egui::Pos2 {
    let screen = harness
        .state()
        .debug_state()
        .gizmo
        .expect("a gizmo is drawn")
        .screen
        .expect("its origin is on screen");
    egui::pos2(screen[0] as f32, screen[1] as f32)
}

/// The translate gizmo on a selected link (ADR-0007): local axes at the
/// arm's own frame, the pointer away so nothing is highlighted.
#[test]
fn gizmo_move_link() {
    scenario("gizmo_move_link", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let arm = *app
            .robot()
            .links
            .iter()
            .find(|(_, l)| l.name == "arm")
            .map(|(id, _)| id)
            .unwrap();
        app.fit_view_now();
        app.set_tool(Tool::Move);
        app.select(Selection::Link(arm));
        settle(harness);

        let gizmo = harness.state().debug_state().gizmo.expect("a gizmo");
        assert_eq!(gizmo.target, format!("link {arm}"));
        assert_eq!(gizmo.mode, "translate");
        // The *link frame*, not the cube it draws: the geom sits half a
        // metre above it (the pendulum's arm geom pose).
        assert_eq!(gizmo.origin, [0.0, 0.0, 0.5]);
        assert!(!gizmo.dragging && !gizmo.captured);
    });
}

/// The rotate gizmo on a selected joint: it sits on the joint frame, which
/// is the child link frame, and moving it moves the pivot alone (OPEN 2).
#[test]
fn gizmo_rotate_joint() {
    scenario("gizmo_rotate_joint", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let hinge = *app.robot().joints.keys().next().unwrap();
        app.fit_view_now();
        app.set_tool(Tool::Rotate);
        app.select(Selection::Joint(hinge));
        settle(harness);

        let gizmo = harness.state().debug_state().gizmo.expect("a gizmo");
        assert_eq!(gizmo.target, format!("joint {hinge}"));
        assert_eq!(gizmo.mode, "rotate");
        // The joint frame is the child link frame.
        assert_eq!(gizmo.origin, [0.0, 0.0, 0.5]);
    });
}

/// The whole gesture: drag the gizmo's view-plane handle, the part follows
/// live, the release is **one** command, undo puts it back. The spike that
/// ADR-0007 rests on.
#[test]
fn gizmo_drag_moves_the_link_in_one_command() {
    with_app(|harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let arm = *app
            .robot()
            .links
            .iter()
            .find(|(_, l)| l.name == "arm")
            .map(|(id, _)| id)
            .unwrap();
        let joint = app.robot().parent_joint(arm).unwrap();
        let before = app.robot().joints[&joint].origin;
        app.fit_view_now();
        app.set_tool(Tool::Move);
        app.select(Selection::Link(arm));
        settle(harness);

        let depth = harness.state().history().undo_depth();
        // The gizmo's origin on screen is its view-plane handle: a drag
        // from there translates in the plane of the screen.
        let from = gizmo_handle(harness);
        synthetic_drag(harness, from, from + egui::vec2(120.0, 0.0), 6);

        let app = harness.state();
        assert_eq!(
            app.history().undo_depth(),
            depth + 1,
            "one gesture is one command"
        );
        assert!(!app.gizmo_dragging(), "the drag ended");
        let after = app.robot().joints[&joint].origin;
        assert!(
            (after.t - before.t).length() > 0.05,
            "the arm moved: {:?} → {:?}",
            before.t,
            after.t
        );
        assert!(
            after.r.abs_diff_eq(before.r, 1e-9),
            "a translate gizmo does not rotate"
        );
        // The instance is where the document says, not where the preview
        // left it: the arm's geom sits half a metre above its link frame.
        let state = app.debug_state();
        assert_eq!(
            state.instances[1].position,
            [
                riggen_app::debug::round(after.t.x),
                riggen_app::debug::round(after.t.y),
                riggen_app::debug::round(after.t.z + 0.5),
            ]
        );

        harness.state_mut().undo();
        let app = harness.state();
        assert_eq!(app.robot().joints[&joint].origin.t, before.t);
        assert_eq!(app.history().undo_depth(), depth);
    });
}

/// A gizmo on a joint moves the pivot and leaves the geometry alone
/// (OPEN 2): one `MoveJointFrame`, every instance where it was.
#[test]
fn gizmo_drag_on_a_joint_moves_only_the_pivot() {
    with_app(|harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let hinge = *app.robot().joints.keys().next().unwrap();
        app.fit_view_now();
        app.set_tool(Tool::Move);
        app.select(Selection::Joint(hinge));
        settle(harness);

        let depth = harness.state().history().undo_depth();
        let positions: Vec<_> = harness
            .state()
            .debug_state()
            .instances
            .iter()
            .map(|i| i.position)
            .collect();
        let from = gizmo_handle(harness);
        synthetic_drag(harness, from, from + egui::vec2(100.0, 0.0), 6);

        let app = harness.state();
        assert_eq!(app.history().undo_depth(), depth + 1);
        let origin = app.robot().joints[&hinge].origin;
        assert!(
            (origin.t - DVec3::new(0.0, 0.0, 0.5)).length() > 0.05,
            "the pivot moved: {:?}",
            origin.t
        );
        let now: Vec<_> = app
            .debug_state()
            .instances
            .iter()
            .map(|i| i.position)
            .collect();
        assert_eq!(now, positions, "nothing in the world moved");
    });
}

/// A revolute joint's glyph: axis segment through the pivot, origin triad,
/// the limit arc and the tick at the current `q`. The hinge is selected, so
/// it is drawn hot.
#[test]
fn glyph_revolute() {
    scenario("glyph_revolute", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let hinge = *app.robot().joints.keys().next().unwrap();
        app.set_joint_value(hinge, 30f64.to_radians());
        app.select(Selection::Joint(hinge));
        app.fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        let glyph = &state.glyphs[0];
        assert_eq!(glyph.name, "hinge");
        assert_eq!(glyph.kind, "Revolute");
        // The pivot is the parent's frame composed with the origin, and it
        // does not move with `q`.
        assert_eq!(glyph.origin, [0.0, 0.0, 0.5]);
        assert_eq!(glyph.axis, [0.0, 1.0, 0.0]);
        assert!(glyph.active);
        assert!(glyph.screen.is_some());
    });
}

/// The same joint made prismatic: the arc becomes a travel segment with end
/// stops and a tick at `q`, and the arm has slid a quarter metre up.
#[test]
fn glyph_prismatic() {
    scenario("glyph_prismatic", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let hinge = *app.robot().joints.keys().next().unwrap();
        let mut joint = app.robot().joints[&hinge].clone();
        joint.kind = riggen_core::JointKind::Prismatic;
        joint.axis = DVec3::Z;
        joint.limits = Some(riggen_core::Limits {
            lower: -0.5,
            upper: 0.5,
            effort: 10.0,
            velocity: 3.0,
        });
        app.apply(Command::SetJoint(hinge, joint)).unwrap();
        app.set_joint_value(hinge, 0.25);
        app.fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        let glyph = &state.glyphs[0];
        assert_eq!(glyph.kind, "Prismatic");
        assert_eq!(glyph.axis, [0.0, 0.0, 1.0]);
        assert_eq!(glyph.q, 0.25);
        assert!(!glyph.active, "nothing is selected");
        // The arm rode along: its geom was 0.5 above the link frame.
        assert_eq!(state.instances[1].position, [0.0, 0.0, 1.25]);
    });
}

/// Which joints get a glyph (plans/m2-placement-ux OPEN 4) and how big it
/// is: every movable one plus the selected one, sized from the child link's
/// own bounds.
#[test]
fn glyphs_cover_movable_joints_and_the_selection() {
    with_app(|harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        app.fit_view_now();
        let hinge = *app.robot().joints.keys().next().unwrap();

        // One movable joint, one glyph, and its size is the arm cube's
        // half-diagonal.
        let glyphs = app.joint_glyphs();
        assert_eq!(glyphs.len(), 1);
        assert!(
            (glyphs[0].size - 0.75f64.sqrt()).abs() < 1e-9,
            "sized from the child's bounds: {}",
            glyphs[0].size
        );
        assert!((glyphs[0].axis.length() - 1.0).abs() < 1e-12);

        // Fixed and unselected: no glyph.
        let mut joint = app.robot().joints[&hinge].clone();
        joint.kind = riggen_core::JointKind::Fixed;
        joint.limits = None;
        app.apply(Command::SetJoint(hinge, joint)).unwrap();
        assert!(app.joint_glyphs().is_empty());

        // Fixed and selected: a glyph, drawn hot, without a limit arc.
        app.select(Selection::Joint(hinge));
        let glyphs = app.joint_glyphs();
        assert_eq!(glyphs.len(), 1);
        assert_eq!(app.active_joint(), Some(hinge));
        assert!(app.debug_state().glyphs[0].active);
    });
}

/// Pointing at a joint's glyph in the viewport: the glyph is drawn hot, the
/// tree row brightens, and the status bar names the joint instead of the
/// part behind it.
#[test]
fn glyph_hover() {
    scenario("glyph_hover", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        app.fit_view_now();
        settle(harness);

        let at = glyph_axis_point(harness, 0.8);
        harness.hover_at(at);
        pump_rendered(harness, 6);
        settle(harness);

        let state = harness.state().debug_state();
        assert!(state.glyphs[0].hovered && state.glyphs[0].active);
        // The viewport's own pick is suppressed while a glyph is hovered, so
        // the part behind it is not highlighted as well.
        assert_eq!(state.selection.hovered, None);
    });
}

/// A point on the hinge glyph's axis segment, `t` of the way out from the
/// pivot: what a hover has to land on.
fn glyph_axis_point(
    harness: &egui_kittest::Harness<'_, riggen_app::RiggenApp>,
    t: f64,
) -> egui::Pos2 {
    let glyph = harness.state().joint_glyphs()[0];
    harness
        .state()
        .project_world(glyph.pivot.t + glyph.axis * glyph.size * t)
        .expect("the glyph is on screen")
}

/// Hover both ways, and the click: the tree row lights its glyph, the glyph
/// lights its row and names itself, and clicking it selects the joint.
#[test]
fn hover_runs_both_ways_and_a_glyph_click_selects_the_joint() {
    with_app(|harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        app.fit_view_now();
        let hinge = *app.robot().joints.keys().next().unwrap();
        settle(harness);
        assert_eq!(harness.state().hovered_joint(), None);

        // Tree → glyph: hovering the joint's row makes its glyph hot.
        let row = harness
            .get_by_label("hinge · revolute")
            .accesskit_node()
            .bounding_box()
            .expect("the joint row has bounds");
        harness.hover_at(egui::pos2(
            ((row.x0 + row.x1) / 2.0) as f32,
            ((row.y0 + row.y1) / 2.0) as f32,
        ));
        pump_rendered(harness, 3);
        assert_eq!(harness.state().hovered_joint(), Some(hinge));
        assert!(harness.state().debug_state().glyphs[0].active);

        // Glyph → tree: pointing at the axis segment in the viewport.
        let at = glyph_axis_point(harness, 0.8);
        harness.hover_at(at);
        pump_rendered(harness, 3);
        assert_eq!(harness.state().hovered_joint(), Some(hinge));

        // Missing it by more than the pixel radius is not a hover.
        harness.hover_at(at + egui::vec2(0.0, riggen_app::GLYPH_HOVER_RADIUS + 6.0));
        pump_rendered(harness, 3);
        assert_eq!(harness.state().hovered_joint(), None);

        // And a click on it selects the joint, not the part behind it.
        harness.hover_at(at);
        pump_rendered(harness, 3);
        click_at(harness, at);
        assert_eq!(harness.state().selection(), Selection::Joint(hinge));
    });
}

/// Writes a cylinder as binary STL and returns its path: a bore to point
/// at, without committing a fixture the M2 arm folder will supply anyway.
fn cylinder_stl(name: &str, radius: f64, height: f64, segments: usize) -> std::path::PathBuf {
    let path = scratch_dir(name).join("boss.stl");
    let mesh = riggen_mesh::TriMesh::cylinder(radius, height, segments);
    std::fs::write(&path, riggen_mesh::write_binary(&mesh)).unwrap();
    path
}

/// Hovers `at` until the ID buffer has resolved and the snap is computed.
fn hover_until_snapped(
    harness: &mut egui_kittest::Harness<'_, riggen_app::RiggenApp>,
    at: egui::Pos2,
) {
    harness.hover_at(at);
    pump_rendered(harness, 8);
}

/// Pointing near a cube's corner with Place joint active: the vertex beats
/// the plain surface hit, and the marker says so.
#[test]
fn snap_vertex() {
    scenario("snap_vertex", |harness| {
        let app = harness.state_mut();
        open_link(app, "cube_binary.stl");
        app.fit_view_now();
        app.set_tool(Tool::PlaceJoint);
        settle(harness);

        // The top corner facing the camera, nudged inward so the pick lands
        // on a triangle rather than on the silhouette.
        let corner = DVec3::new(0.5, -0.5, 0.5);
        let at = harness.state().project_world(corner).expect("on screen");
        let centre = harness.state().viewport_center().unwrap();
        let at = at + (centre - at).normalized() * 6.0;
        hover_until_snapped(harness, at);

        let snap = harness.state().debug_state().snap.expect("a snap target");
        assert_eq!(snap.kind, "vertex");
        assert_eq!(snap.point, [0.5, -0.5, 0.5]);
        assert_eq!(snap.readout, "vertex");
        assert_eq!(snap.radius_mm, None);
    });
}

/// Pointing at a bore's wall: the circle fit wins, and the readout carries
/// its radius, segment count and residual so a bad fit is obvious.
#[test]
fn snap_circle() {
    scenario("snap_circle", |harness| {
        let path = cylinder_stl("snap_circle", 0.012, 0.05, 24);
        let app = harness.state_mut();
        app.open_path(&path).expect("open the boss").unwrap();
        app.fit_view_now();
        app.set_tool(Tool::PlaceJoint);
        settle(harness);

        // The middle of the silhouette is the wall, whatever the camera.
        let at = harness.state().viewport_center().unwrap();
        hover_until_snapped(harness, at);

        let snap = harness.state().debug_state().snap.expect("a snap target");
        assert_eq!(snap.kind, "circle");
        assert_eq!(snap.radius_mm, Some(12.0));
        assert_eq!(snap.segments, Some(24));
        assert_eq!(snap.residual_mm, Some(0.0));
        assert_eq!(snap.readout, "circle r 12.0 mm · 24 seg · res 0.00 mm");
        // The centre is on the axis, at the region's mean height: the
        // cylinder's own centre.
        assert_eq!(snap.point, [0.0, 0.0, 0.0]);
        assert_eq!(snap.axis, [0.0, 0.0, 1.0]);
    });
}

/// Snapping is a placement affordance: no tool, no markers — and the memo
/// is per `(instance, triangle)`, so a resting cursor keeps the same fit.
#[test]
fn snapping_is_off_outside_the_placement_tools() {
    with_app(|harness| {
        let path = cylinder_stl("snap_off", 0.012, 0.05, 24);
        let app = harness.state_mut();
        app.open_path(&path).expect("open the boss").unwrap();
        app.fit_view_now();
        settle(harness);

        let at = harness.state().viewport_center().unwrap();
        hover_until_snapped(harness, at);
        assert_eq!(harness.state().snap(), None, "Select does not snap");

        harness.state_mut().set_tool(Tool::Align);
        hover_until_snapped(harness, at);
        let first = harness.state().snap().expect("Align snaps");
        assert_eq!(first.kind, riggen_app::SnapKind::Circle);

        // The cursor has not moved: the same triangle, the same fit.
        pump_rendered(harness, 4);
        assert_eq!(harness.state().snap(), Some(first));

        // The pointer leaving takes the marker with it.
        harness.event(egui::Event::PointerGone);
        pump_rendered(harness, 6);
        assert_eq!(harness.state().snap(), None);
    });
}

/// The materials table over the pendulum: base_link is aluminium and arm
/// is PLA, and the viewport tints each cube with its material colour.
#[test]
fn materials() {
    scenario("materials", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        app.set_materials_window_open(true);
        app.fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        assert_eq!(state.ui.windows, vec!["materials"]);
        let robot = harness.state().robot();
        let expect = |name: &str| robot.materials[name].color.map(riggen_app::debug::round32);
        assert_eq!(state.instances[0].color, expect("aluminium"));
        assert_eq!(state.instances[1].color, expect("PLA"));
        assert_eq!(state.document.links[1].material.as_deref(), Some("PLA"));
    });
}

/// Table edits are commands: removing a material a link uses is refused
/// with the reason, freeing the link lets it go, "Add" makes a new row,
/// a density typed into the table lands in the document, and the tint
/// follows a material change on the link.
#[test]
fn materials_table_edits() {
    with_app(|harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        app.set_materials_window_open(true);
        settle(harness);
        let app = harness.state();
        let arm = *app
            .robot()
            .links
            .iter()
            .find(|(_, l)| l.name == "arm")
            .map(|(id, _)| id)
            .unwrap();
        let depth = app.history().undo_depth();

        // The second "Remove" row is PLA (rows are in name order: ABS, PLA, …).
        harness.get_all_by_label("Remove").nth(1).unwrap().click();
        harness.step();
        let app = harness.state();
        assert!(app.robot().materials.contains_key("PLA"));
        assert_eq!(
            app.debug_state().status.as_deref(),
            Some(format!("material \"PLA\" is used by link {arm}").as_str())
        );
        assert_eq!(
            app.history().undo_depth(),
            depth,
            "a refused command is not an entry"
        );

        // Free the link, then the removal goes through and the tint falls
        // back to the default.
        harness
            .state_mut()
            .apply(Command::SetLinkMaterial(arm, None))
            .unwrap();
        harness.get_all_by_label("Remove").nth(1).unwrap().click();
        harness.step();
        let app = harness.state();
        assert!(!app.robot().materials.contains_key("PLA"));
        assert_eq!(
            app.debug_state().instances[1].color,
            riggen_viewport::DEFAULT_INSTANCE_COLOR.map(riggen_app::debug::round32)
        );

        // Add a material by name.
        let new_name = harness
            .get_all_by_role(egui::accesskit::Role::TextInput)
            .last()
            .expect("the new-material field is the last text input");
        new_name.focus();
        harness.step();
        harness.event(egui::Event::Text("foam".to_owned()));
        harness.step();
        harness.get_by_label("Add").click();
        harness.step();
        let app = harness.state();
        assert_eq!(app.robot().materials["foam"].density, 1000.0);

        // Type a density into foam's row (rows in name order: ABS, aluminium,
        // foam, …; foam is the third density field).
        let index = app
            .robot()
            .materials
            .keys()
            .position(|k| k == "foam")
            .unwrap();
        let rect = harness
            .get_all_by_role(egui::accesskit::Role::TextInput)
            .nth(index)
            .unwrap()
            .rect();
        let pos = rect.center();
        harness.event(egui::Event::PointerMoved(pos));
        harness.event(egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: Default::default(),
        });
        harness.event(egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        });
        harness.step();
        harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
        harness.step();
        harness.event(egui::Event::Text("120".to_owned()));
        harness.step();
        harness.key_press(egui::Key::Enter);
        harness.step();
        harness.step();
        let app = harness.state();
        assert_eq!(app.robot().materials["foam"].density, 120.0);

        // The link combo reads the same table: give arm the new material
        // and the tint follows.
        harness
            .state_mut()
            .apply(Command::SetLinkMaterial(arm, Some("foam".into())))
            .unwrap();
        let app = harness.state();
        assert_eq!(
            app.debug_state().instances[1].color,
            [0.6, 0.6, 0.6, 1.0].map(riggen_app::debug::round32)
        );
        assert!(
            app.robot()
                .links
                .values()
                .any(|l| l.material.as_deref() == Some("foam"))
        );
    });
}

/// A fresh, empty directory under the OS temp dir for a save test.
fn scratch_dir(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("riggen-app-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// The dirty marker: after an edit the title and the status bar both
/// carry the `*`.
#[test]
fn dirty_title() {
    scenario("dirty_title", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        assert_eq!(app.window_title(), "pendulum.riggen — riggen");
        let arm = *app
            .robot()
            .links
            .iter()
            .find(|(_, l)| l.name == "arm")
            .map(|(id, _)| id)
            .unwrap();
        app.apply(Command::RenameLink(arm, "upper_arm".into()))
            .unwrap();
        app.fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        assert!(state.document.dirty);
        assert_eq!(state.ui.title, "pendulum.riggen* — riggen");
        assert_eq!(state.ui.modal, None);
    });
}

/// The unsaved-changes confirm: New on a dirty document shows the modal
/// with Save / Don't save / Cancel and touches nothing yet.
#[test]
fn unsaved_confirm() {
    scenario("unsaved_confirm", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        open_link(app, "cube.obj");
        app.request_new();
        app.fit_view_now();
        settle(harness);

        let state = harness.state().debug_state();
        assert_eq!(state.ui.modal, Some("unsaved_changes"));
        assert_eq!(
            harness.state().pending_action(),
            Some(&riggen_app::PendingAction::New)
        );
        assert_eq!(state.document.links.len(), 3, "nothing happened yet");
        harness.get_by_label("Save");
        harness.get_by_label("Don't save");
        harness.get_by_label("Cancel");
    });
}

/// Save → reopen → the same document, clean; then the confirm's three
/// answers: Cancel keeps everything, Don't save runs the action, Save
/// writes the file first. The OS close request takes the same path.
#[test]
fn save_reopen_and_confirm_answers() {
    with_app(|harness| {
        let dir = scratch_dir("save_reopen");
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let arm = *app
            .robot()
            .links
            .iter()
            .find(|(_, l)| l.name == "arm")
            .map(|(id, _)| id)
            .unwrap();
        app.apply(Command::RenameLink(arm, "upper_arm".into()))
            .unwrap();
        assert!(app.history().is_dirty());

        // Save As (through the API; the dialog is the only other route).
        let file = dir.join("copy");
        assert!(app.save_to(&file), "{:?}", app.debug_state().status);
        let file = dir.join("copy.riggen");
        assert!(file.exists(), "the extension is added");
        assert!(!app.history().is_dirty());
        assert_eq!(app.window_title(), "copy.riggen — riggen");
        assert_eq!(
            app.debug_state().status.as_deref(),
            Some("saved copy.riggen")
        );
        let saved = app.robot().clone();

        // New (clean, no confirm) then reopen: equal and clean.
        app.request_new();
        assert_eq!(app.robot().links.len(), 1);
        assert_eq!(app.window_title(), "untitled — riggen");
        app.open_path(&file).expect("reopen");
        assert_eq!(app.robot(), &saved);
        assert!(!app.history().is_dirty());
        assert_eq!(app.debug_state().instances.len(), 2);

        // Dirty again; Cancel keeps the document and the pending action.
        app.apply(Command::RenameLink(arm, "arm".into())).unwrap();
        app.request_new();
        assert_eq!(app.pending_action(), Some(&riggen_app::PendingAction::New));
        harness.step();
        harness.get_by_label("Cancel").click();
        harness.step();
        let app = harness.state();
        assert_eq!(app.pending_action(), None);
        assert_eq!(app.robot().links.len(), 2);
        assert!(app.history().is_dirty());

        // Save answer: the file is rewritten, then the action runs.
        harness.state_mut().request_new();
        harness.step();
        harness.get_by_label("Save").click();
        harness.step();
        let app = harness.state();
        assert_eq!(app.pending_action(), None);
        assert_eq!(app.robot().links.len(), 1, "New ran after the save");
        let (reloaded, _) = riggen_core::load(&file).unwrap();
        assert_eq!(reloaded.links[&arm].name, "arm");

        // Don't save: a dropped .riggen replaces the dirty document.
        let app = harness.state_mut();
        open_link(app, "cube.obj");
        assert!(app.history().is_dirty());
        app.request_open(vec![file.clone()]);
        assert!(matches!(
            app.pending_action(),
            Some(riggen_app::PendingAction::Open(Some(_)))
        ));
        harness.step();
        harness.get_by_label("Don't save").click();
        harness.step();
        let app = harness.state();
        assert_eq!(app.file(), Some(&file));
        assert_eq!(app.robot().links.len(), 2);
        assert!(!app.history().is_dirty());

        // Meshes alone never ask.
        let app = harness.state_mut();
        app.request_open(vec![fixture("cube.obj")]);
        assert_eq!(app.pending_action(), None);
        assert_eq!(app.robot().links.len(), 3);

        // Quit on a dirty document waits for the answer; Don't save agrees
        // to close.
        app.request_quit();
        assert!(!app.quit_confirmed());
        assert_eq!(app.pending_action(), Some(&riggen_app::PendingAction::Quit));
        harness.step();
        harness.get_by_label("Don't save").click();
        harness.step();
        assert!(harness.state().quit_confirmed());
    });
}

/// Ctrl+S saves a titled document even while a text field has focus;
/// Ctrl+N asks first when dirty.
#[test]
fn file_shortcuts() {
    with_app(|harness| {
        let dir = scratch_dir("shortcuts");
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        let file = dir.join("keys.riggen");
        assert!(app.save_to(&file));
        open_link(app, "cube.obj");
        assert!(app.history().is_dirty());
        settle(harness);

        // Focus the name field of the selected link (a text field).
        harness.get_by_label("cube").click();
        harness.step();
        harness.get_by_label("name").focus();
        harness.step();
        harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::S);
        harness.step();
        let app = harness.state();
        assert!(
            !app.history().is_dirty(),
            "Ctrl+S saved with a text field focused"
        );
        assert_eq!(riggen_core::load(&file).unwrap().0.links.len(), 3);

        let last = *harness.state().robot().links.keys().last().unwrap();
        harness
            .state_mut()
            .apply(Command::RenameLink(last, "block".into()))
            .unwrap();
        harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::N);
        harness.step();
        assert_eq!(
            harness.state().pending_action(),
            Some(&riggen_app::PendingAction::New)
        );
    });
}

/// Undo / redo shortcuts: Ctrl+Shift+Z redoes and does not undo (the
/// shifted pattern is matched first), Ctrl+Y redoes too, Ctrl+Z undoes;
/// inside a focused text field Ctrl+Z leaves the document alone. The
/// Edit menu does the same.
#[test]
fn undo_redo_shortcuts() {
    with_app(|harness| {
        let app = harness.state_mut();
        let a = open_link(app, "cube_binary.stl");
        let b = open_link(app, "cube_ascii.stl");
        assert_eq!(app.history().undo_depth(), 2);
        settle(harness);

        harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::Z);
        harness.step();
        let app = harness.state();
        assert_eq!(app.history().undo_depth(), 1);
        assert!(!app.robot().links.contains_key(&b));
        assert!(app.robot().links.contains_key(&a));

        harness.key_press_modifiers(
            egui::Modifiers::COMMAND | egui::Modifiers::SHIFT,
            egui::Key::Z,
        );
        harness.step();
        let app = harness.state();
        assert_eq!(
            app.history().undo_depth(),
            2,
            "Ctrl+Shift+Z redid, it did not undo"
        );
        assert!(app.robot().links.contains_key(&b));

        harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::Z);
        harness.step();
        harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::Y);
        harness.step();
        assert_eq!(harness.state().history().undo_depth(), 2, "Ctrl+Y redid");

        // A focused text field keeps Ctrl+Z for itself.
        harness.get_by_label("cube_ascii").click();
        harness.step();
        harness.get_by_label("name").focus();
        harness.step();
        harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::Z);
        harness.step();
        assert_eq!(
            harness.state().history().undo_depth(),
            2,
            "text field owns Ctrl+Z"
        );
        harness.key_press(egui::Key::Escape);
        harness.step();

        // Edit › Undo through the menu.
        harness.get_by_label("Edit").click();
        harness.step();
        harness.get_by_label("Undo").click();
        harness.step();
        assert_eq!(harness.state().history().undo_depth(), 1);
        harness.get_by_label("Edit").click();
        harness.step();
        harness.get_by_label("Redo").click();
        harness.step();
        assert_eq!(harness.state().history().undo_depth(), 2);
    });
}

/// Sets the (only) slider through its AccessKit `SetValue` action — the
/// path a screen reader takes, which egui routes through the same `set`
/// as a drag. A pointer click on the rail is not exact: the slider's
/// accessibility rect also spans its value box.
fn set_slider_value(harness: &mut egui_kittest::Harness<'_, riggen_app::RiggenApp>, value: f64) {
    let (target_node, target_tree) = harness
        .get_by_role(egui::accesskit::Role::Slider)
        .accesskit_node()
        .locate();
    harness.event(egui::Event::AccessKitActionRequest(
        egui::accesskit::ActionRequest {
            action: egui::accesskit::Action::SetValue,
            target_node,
            target_tree,
            data: Some(egui::accesskit::ActionData::NumericValue(value)),
        },
    ));
    harness.step();
    harness.step();
}

/// The M1 acceptance (docs/03-roadmap.md §M1): two cube fixtures dropped
/// as base and arm, the joint typed numerically in the properties panel
/// (kind, origin, axis, limits), the slider swung to 45° within its
/// limits, undo twice / redo twice back to the same document, saved to a
/// temp dir and reopened equal and clean.
#[test]
fn build_pendulum_numerically() {
    with_app(|harness| {
        let dir = scratch_dir("acceptance");
        let app = harness.state_mut();
        let base = open_link(app, "cube_binary.stl");
        app.select(Selection::Link(base));
        let arm = open_link(app, "cube_ascii.stl");
        let hinge = app.robot().parent_joint(arm).unwrap();
        assert_eq!(app.robot().joints[&hinge].parent, base);
        settle(harness);

        // The joint: kind through the combo, then the numbers.
        harness.get_by_label("cube_ascii_joint · fixed").click();
        harness.step();
        harness.get_by_role(egui::accesskit::Role::ComboBox).click();
        harness.step();
        harness.get_by_label("Revolute").click();
        harness.step();
        harness.step();
        let app = harness.state();
        assert_eq!(
            app.robot().joints[&hinge].kind,
            riggen_core::JointKind::Revolute
        );
        assert!(
            app.robot().joints[&hinge].limits.is_some(),
            "defaults arrive with the kind"
        );
        // Origin z, axis (0, 1, 0), limits ±90°.
        type_into(harness, "z", 0, "0.5");
        type_into(harness, "y", 1, "1");
        type_into(harness, "z", 1, "0");
        type_into(harness, "lower °", 0, "-90");
        type_into(harness, "upper °", 0, "90");
        let app = harness.state();
        let j = &app.robot().joints[&hinge];
        assert!(
            (j.origin.t - DVec3::new(0.0, 0.0, 0.5)).length() < 1e-9,
            "{:?}",
            j.origin
        );
        assert!((j.axis - DVec3::Y).length() < 1e-9, "{}", j.axis);
        let limits = j.limits.unwrap();
        assert!((limits.lower + std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        assert!((limits.upper - std::f64::consts::FRAC_PI_2).abs() < 1e-9);

        // The arm's mesh sits half a cube above the hinge.
        harness.get_by_label("cube_ascii").click();
        harness.step();
        type_into(harness, "z", 0, "0.5");
        let app = harness.state();
        assert_eq!(app.debug_state().instances[1].position, [0.0, 0.0, 1.0]);
        let built = app.robot().clone();
        let depth = app.history().undo_depth();

        // The slider at 45°, inside the ±90° limits: the arm swings and
        // the document is untouched. 120° is clamped by the slider range.
        harness.state_mut().set_joints_window_open(true);
        harness.step();
        set_slider_value(harness, 45.0);
        let app = harness.state();
        let q = app.joint_value(hinge);
        assert!((q - std::f64::consts::FRAC_PI_4).abs() < 1e-9, "q = {q}");
        assert_eq!(
            app.debug_state().instances[1].position,
            [0.353553, 0.0, 0.853553]
        );
        assert_eq!(app.robot(), &built, "the slider is not an edit");
        set_slider_value(harness, 120.0);
        let q = harness.state().joint_value(hinge);
        assert!(
            (q - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "clamped: q = {q}"
        );

        // Undo twice, redo twice: the same document.
        let app = harness.state_mut();
        assert!(app.undo());
        assert!(app.undo());
        assert_eq!(app.history().undo_depth(), depth - 2);
        assert_ne!(app.robot(), &built);
        assert!(app.redo());
        assert!(app.redo());
        assert_eq!(app.robot(), &built);

        // Save, reopen: equal and clean.
        let file = dir.join("pendulum.riggen");
        assert!(app.save_to(&file), "{:?}", app.debug_state().status);
        assert!(!app.history().is_dirty());
        app.new_document();
        assert_eq!(app.robot().links.len(), 1);
        app.open_path(&file).expect("reopen");
        assert_eq!(app.robot(), &built);
        assert!(!app.history().is_dirty());
        assert_eq!(app.debug_state().instances.len(), 2);
        assert_eq!(app.window_title(), "pendulum.riggen — riggen");
    });
}

/// The Debug menu open over the pendulum: egui's layout overlays and the
/// Copy / Save state items — the runtime route to `debug_state()`.
#[test]
fn debug_menu() {
    scenario("debug_menu", |harness| {
        let app = harness.state_mut();
        app.open_path(&fixture("pendulum.riggen"))
            .expect("open the corpus file");
        app.fit_view_now();
        settle(harness);

        harness.get_by_label("Debug").click();
        harness.step();
        // A new popup area is laid out invisibly on its first frame, so the
        // capture needs another one to actually show the menu.
        settle(harness);
        assert!(harness.query_by_label("Copy state (JSON)").is_some());
        assert!(harness.query_by_label("Show widget hits").is_some());
    });
}

/// An overlay toggle lands in both themes' styles. No golden: the overlay
/// is egui's, and a picture of it would churn with every egui upgrade
/// while showing nothing about riggen.
#[test]
fn debug_overlay_toggle_sets_both_themes() {
    with_app(|harness| {
        harness.get_by_label("Debug").click();
        harness.step();
        harness.get_by_label("Show widget hits").click();
        harness.step();
        for theme in [egui::Theme::Dark, egui::Theme::Light] {
            assert!(
                harness.ctx.style_of(theme).debug.show_widget_hits,
                "{theme:?}"
            );
        }
    });
}

/// Copy state (JSON) closes the menu and reports in the status bar; the
/// clipboard itself is egui's and not observable here.
#[test]
fn debug_copy_state_reports() {
    with_app(|harness| {
        harness.get_by_label("Debug").click();
        harness.step();
        harness.get_by_label("Copy state (JSON)").click();
        harness.step();
        settle(harness);

        let state = harness.state().debug_state();
        assert_eq!(state.status.as_deref(), Some(riggen_app::COPIED_STATUS));
        assert!(
            harness.query_by_label("Copy state (JSON)").is_none(),
            "the menu closed"
        );
    });
}
