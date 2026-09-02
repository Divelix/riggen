//! The application state and its per-frame `ui`: menu bar on top, status
//! bar on the bottom, the viewport in the central panel.

mod align;
mod debug_menu;
mod document;
mod export_dialog;
mod file_io;
mod file_menu;
mod gizmo;
mod glyphs;
mod panels;
mod shortcuts;
mod snap;
mod status_bar;
mod tool;

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;

use riggen_core::{GeomId, History, JointId, JointState, LinkId, MeshId, Pose, Robot};
use riggen_mesh::DecompParams;

use crate::jobs::Jobs;
use riggen_viewport::{InstanceId, PickHit, Viewport};

/// eframe storage key for View › Collision geometry.
pub(crate) const SHOW_COLLISION_KEY: &str = "riggen.show_collision";
use web_time::Instant;

pub use align::{ALIGN_PROMPT, ALIGN_WRONG_LINK, align_transform, aligned_status};
pub use debug_menu::COPIED_STATUS;
pub use document::Selection;
pub(crate) use document::{CollisionSource, DecompState, LoadedMesh};
pub use export_dialog::ExportDialog;
pub use file_io::{DroppedSet, Files};
pub use file_menu::PendingAction;
use file_menu::{IMPORT_SCALE_KEY, IMPORT_UNITS};
use gizmo::GizmoState;
pub use gizmo::GizmoTarget;
pub use glyphs::{FrameGlyph, GLYPH_HOVER_RADIUS, JointGlyph};
pub use panels::{DECOMP_CONSENT_BUTTON, DECOMP_FREEZE_WARNING, fmt_num};
use panels::{JointsWindow, MaterialsWindow, PropertiesState, TreeState};
use snap::SnapCache;
pub use snap::{SNAP_PIXEL_RADIUS, SnapCandidate, SnapKind, placed_status};
pub use tool::{Tool, ZERO_CONFIG_STATUS};

/// The eframe app: one `Robot` and what is derived from it
/// (docs/01-architecture.md §The document is the only state).
pub struct RiggenApp {
    robot: Robot,
    history: History,
    /// Where the document lives on disk; `None` until saved or opened.
    file: Option<PathBuf>,
    /// Mesh geometry beside the document, keyed by asset, loaded once per
    /// file and shared across history snapshots.
    mesh_store: HashMap<MeshId, LoadedMesh>,
    /// The viewport's instance per visual geom (docs/02-data-model.md
    /// §Geom): the only map between document and scene.
    instances: BTreeMap<(LinkId, GeomId), InstanceId>,
    /// The translucent instance per collision shape, keyed by the link and
    /// the shape's index in its resolved collision list; empty while View ›
    /// Collision geometry is off. Beside each, what was uploaded for it.
    collision_instances: BTreeMap<(LinkId, usize), (InstanceId, CollisionSource)>,
    /// View › Collision geometry. Off by default, remembered through eframe
    /// storage.
    show_collision: bool,
    /// The job thread (`crate::jobs`, docs/01-architecture.md §Jobs and
    /// threads). Drained once per frame.
    jobs: Jobs,
    /// Convex decompositions by `(mesh, parameters)`: derived state, never
    /// saved — the document holds the parameters and never the pieces
    /// (ADR-0011). Filled by the job thread, read by the properties panel,
    /// the collision view and the export.
    pub(crate) decomp: HashMap<(MeshId, DecompParams), DecompState>,
    /// Current joint values — slider state, never saved.
    q: JointState,
    selection: Selection,
    /// What a viewport gesture means (`tool.rs`).
    tool: Tool,
    /// The transform gizmo and the drag it is in the middle of
    /// (`gizmo.rs`, ADR-0007).
    gizmo_state: GizmoState,
    /// The joint under the pointer: hovered in the tree, or its glyph
    /// hovered in the viewport. Highlights both, both ways (`glyphs.rs`).
    hovered_joint: Option<JointId>,
    /// The frame under the pointer, from its row or its glyph, and whether
    /// that hover came from the glyph — the same pair as for joints.
    hovered_frame: Option<riggen_core::FrameId>,
    frame_glyph_hover: Option<riggen_core::FrameId>,
    /// Set when that hover came from the *glyph* rather than the tree: a
    /// click then selects the joint, and the viewport's own pick is
    /// suppressed so it does not select the part behind it instead.
    glyph_hover: Option<JointId>,
    /// The toolbar's rect, so a glyph behind it is not "hovered" through it.
    toolbar_rect: Option<egui::Rect>,
    /// What the cursor is really pointing at, for the placement tools
    /// (`snap.rs`). Rebuilt every frame from the hovered pick.
    snap_candidate: Option<SnapCandidate>,
    /// The last circle fit, so a resting cursor fits once and not per frame.
    snap_cache: SnapCache,
    /// The Align tool's first pick, waiting for its second (`align.rs`).
    align_source: Option<SnapCandidate>,
    /// A link's world pose while a gizmo drag previews it: `sync_scene`
    /// puts the link and its subtree there instead of at the FK pose, and
    /// the document is untouched until the release commits.
    preview_world: Option<(LinkId, Pose)>,
    /// What the viewport reported selected last frame, to notice a click
    /// resolving without mistaking a programmatic selection for one.
    last_viewport_selected: Option<PickHit>,
    /// `MeshAsset::scale` for a dropped mesh. Millimetres by default: that
    /// is what most STL exporters write.
    import_scale: f64,
    /// Transient state of the tree panel (an inline rename in progress).
    pub(crate) tree: TreeState,
    /// Transient state of the properties panel (fields being typed into).
    pub(crate) props: PropertiesState,
    /// The joint sliders window: open or not.
    pub(crate) joints_window: JointsWindow,
    /// The materials table window and its in-progress edits.
    pub(crate) materials_window: MaterialsWindow,
    /// Whether a V-HACD run may start (ADR-0011, docs/01-architecture.md
    /// §Jobs and threads). Always true on the desktop, where the job has a
    /// thread and nothing to consent to. In a browser `jobs` has no thread
    /// and the run happens inline, freezing the tab for a few seconds, so
    /// the properties panel asks once and this is the answer.
    pub(crate) decomp_consent: bool,
    /// Where the app reads bytes from: the filesystem on the desktop, the
    /// dropped files in a browser (`file_io.rs`, ADR-0017).
    pub(crate) files: Files,
    /// Drop gestures the browser is still reading, filled by the futures
    /// `handle_file_drops` spawns and drained once per frame. Never more
    /// than a handful of files; wasm is single-threaded, so an `Rc` and a
    /// `RefCell` are the whole synchronisation story.
    #[cfg(target_arch = "wasm32")]
    pub(crate) inbox: std::rc::Rc<std::cell::RefCell<Vec<Vec<(PathBuf, Vec<u8>)>>>>,
    /// New / Open / Quit waiting on the unsaved-changes answer.
    pub(crate) pending: Option<PendingAction>,
    /// File › Export… (`export_dialog.rs`).
    pub(crate) export_dialog: ExportDialog,
    /// Set once Quit (or the OS close button) passed the dirty check.
    pub(crate) quit_confirmed: bool,
    /// The title last pushed to the OS window.
    pub(crate) last_title: Option<String>,
    pub(crate) viewport: Viewport,
    /// The next [`InstanceId`] to hand out. Never reused within a session.
    next_instance: u32,
    /// A one-off message for the status bar — a load error, an export
    /// destination. `None` reads as "idle".
    pub(crate) status: Option<String>,
    /// Whether the status bar shows the frame-time readout. The snapshot
    /// suite turns it off: it reads the wall clock, so it differs on every
    /// frame.
    pub(crate) show_frame_hud: bool,
    last_frame_instant: Option<Instant>,
    /// Seconds between the last two frames, for the frame-time readout.
    pub(crate) last_frame_dt: Option<f32>,
    /// When the process started (`main`'s first line), or when `new` ran
    /// when nobody handed one in — the startup budget's clock
    /// (docs/03-roadmap.md §M4).
    started: Instant,
    /// Milliseconds from `started` to the end of the first `ui` pass, i.e.
    /// the first frame that is painted. `None` until then.
    pub(crate) first_frame_ms: Option<f64>,
    /// `--timing`: print `first_frame_ms` to stderr once it is known.
    print_timing: bool,
}

impl RiggenApp {
    /// The import scale a fresh app starts with: millimetres.
    pub const DEFAULT_IMPORT_SCALE: f64 = 0.001;

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        Self::with_start(cc, Instant::now())
    }

    /// [`Self::new`] with the startup clock already running: `main` takes
    /// `Instant::now()` before eframe creates the window and the wgpu
    /// device, so those are inside the first-frame number of the real app.
    /// The test harness has no earlier point and starts it here.
    pub fn with_start(cc: &eframe::CreationContext<'_>, started: Instant) -> Self {
        let render_state = cc
            .wgpu_render_state
            .as_ref()
            .expect("riggen-app requires eframe's wgpu backend");
        let viewport = Viewport::new(&render_state.device, render_state.target_format);
        let import_scale = cc
            .storage
            .and_then(|s| s.get_string(IMPORT_SCALE_KEY))
            .and_then(|s| s.parse::<f64>().ok())
            .filter(|s| {
                IMPORT_UNITS
                    .iter()
                    .any(|(_, known)| (known - s).abs() < 1e-12)
            })
            .unwrap_or(Self::DEFAULT_IMPORT_SCALE);
        let show_collision = cc
            .storage
            .and_then(|s| s.get_string(SHOW_COLLISION_KEY))
            .is_some_and(|s| s == "true");

        Self {
            robot: Robot::new("robot"),
            history: History::new(),
            file: None,
            mesh_store: HashMap::new(),
            instances: BTreeMap::new(),
            collision_instances: BTreeMap::new(),
            show_collision,
            jobs: Jobs::new({
                let ctx = cc.egui_ctx.clone();
                move || ctx.request_repaint()
            }),
            decomp: HashMap::new(),
            q: JointState::default(),
            selection: Selection::None,
            tool: Tool::default(),
            gizmo_state: GizmoState::default(),
            hovered_joint: None,
            glyph_hover: None,
            hovered_frame: None,
            frame_glyph_hover: None,
            toolbar_rect: None,
            snap_candidate: None,
            snap_cache: SnapCache::default(),
            align_source: None,
            preview_world: None,
            last_viewport_selected: None,
            import_scale,
            tree: TreeState::default(),
            props: PropertiesState::default(),
            joints_window: JointsWindow::default(),
            materials_window: MaterialsWindow::default(),
            decomp_consent: !cfg!(target_arch = "wasm32"),
            files: if cfg!(target_arch = "wasm32") {
                // Nothing has been dropped yet, and there is no filesystem
                // to fall back to (ADR-0017).
                Files::Dropped(file_io::DroppedSet::default())
            } else {
                Files::Disk
            },
            #[cfg(target_arch = "wasm32")]
            inbox: Default::default(),
            pending: None,
            export_dialog: ExportDialog::default(),
            quit_confirmed: false,
            last_title: None,
            viewport,
            next_instance: 0,
            status: None,
            show_frame_hud: true,
            last_frame_instant: None,
            last_frame_dt: None,
            started,
            first_frame_ms: None,
            print_timing: false,
        }
    }

    /// `--timing`: report the first frame on stderr when it lands, and
    /// right now how long the window and the device took before `new`.
    pub fn set_print_timing(&mut self, print: bool) {
        self.print_timing = print;
        if print {
            eprintln!(
                "startup: app created after {:.0} ms (window and wgpu device before it)",
                self.started.elapsed().as_secs_f64() * 1000.0
            );
        }
    }

    /// Milliseconds from the start clock to the first painted frame, once
    /// there has been one.
    pub fn first_frame_ms(&self) -> Option<f64> {
        self.first_frame_ms
    }

    /// Called at the end of every `ui` pass; only the first one counts.
    fn record_first_frame(&mut self) {
        if self.first_frame_ms.is_some() {
            return;
        }
        let ms = self.started.elapsed().as_secs_f64() * 1000.0;
        self.first_frame_ms = Some(ms);
        if self.print_timing {
            eprintln!("startup: first frame after {ms:.0} ms");
        }
    }

    fn tick_frame_clock(&mut self) {
        let now = Instant::now();
        if let Some(last) = self.last_frame_instant.replace(now) {
            let dt = now.duration_since(last).as_secs_f32();
            self.last_frame_dt = (dt > 0.0).then_some(dt);
        }
    }

    fn menu_bar(&mut self, ui: &mut egui::Ui) {
        egui::Panel::top("menu_bar").show(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button("File", |ui| self.file_menu(ui));
                ui.menu_button("Edit", |ui| self.edit_menu(ui));
                ui.menu_button("View", |ui| {
                    let mut show = self.show_collision;
                    if ui.checkbox(&mut show, "Collision geometry").changed() {
                        self.set_show_collision(show);
                    }
                });
                ui.menu_button("Window", |ui| {
                    let mut joints = self.joints_window.open;
                    if ui.checkbox(&mut joints, "Joints").changed() {
                        self.joints_window.set_open(joints);
                    }
                    ui.checkbox(&mut self.materials_window.open, "Materials");
                });
                ui.menu_button("Debug", |ui| self.debug_menu(ui));
            });
        });
    }

    /// Undo / Redo / Delete, greyed out when there is nothing to do.
    fn edit_menu(&mut self, ui: &mut egui::Ui) {
        if ui
            .add_enabled(self.history.can_undo(), egui::Button::new("Undo"))
            .clicked()
        {
            ui.close();
            self.undo();
        }
        if ui
            .add_enabled(self.history.can_redo(), egui::Button::new("Redo"))
            .clicked()
        {
            ui.close();
            self.redo();
        }
        ui.separator();
        if ui
            .add_enabled(
                self.selection != Selection::None,
                egui::Button::new("Delete"),
            )
            .clicked()
        {
            ui.close();
            self.remove_selected();
        }
    }

    /// Resolves what the pointer is on: a tree row hovered while this frame
    /// was drawn wins, otherwise a glyph under the cursor in the viewport.
    ///
    /// The toolbar and the gizmo are cut out first — both float *over* the
    /// viewport, and a glyph behind them is not something the user is
    /// pointing at.
    fn update_glyph_hover(
        &mut self,
        ctx: &egui::Context,
        glyphs: &[JointGlyph],
        frames: &[FrameGlyph],
    ) {
        let joint_from_tree = self.tree.hovered_joint.take();
        let frame_from_tree = self.tree.hovered_frame.take();
        self.glyph_hover = None;
        self.frame_glyph_hover = None;
        // A placement tool never lets a glyph take the pointer: the whole
        // gesture is "point at that feature", and the selected joint's own
        // glyph sits exactly where the user is aiming.
        if joint_from_tree.is_none()
            && frame_from_tree.is_none()
            && !self.snapping()
            && self.pending.is_none()
            && !self.gizmo_state.captured
            && let Some(pos) = ctx.pointer_hover_pos()
            && self
                .viewport
                .viewport_rect()
                .is_some_and(|r| r.contains(pos))
            && !self.toolbar_rect.is_some_and(|r| r.contains(pos))
        {
            // A frame glyph is a small triad the user placed on purpose; a
            // joint glyph is a long axis line that often runs through it.
            // The frame wins the pointer where both are in reach.
            self.frame_glyph_hover = self.frame_glyph_at(frames, pos);
            if self.frame_glyph_hover.is_none() {
                self.glyph_hover = self.glyph_at(glyphs, pos);
            }
        }
        self.hovered_joint = joint_from_tree.or(self.glyph_hover);
        self.hovered_frame = frame_from_tree.or(self.frame_glyph_hover);
    }

    /// `arm (i1/t120)`: the link and the instance/triangle the ID buffer
    /// resolved.
    fn describe_hit(&self, hit: PickHit) -> String {
        let where_ = format!("i{}/t{}", hit.instance.0, hit.triangle);
        match self.link_name_of_instance(hit.instance) {
            Some(name) => format!("{name} ({where_})"),
            None => where_,
        }
    }
}

impl eframe::App for RiggenApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        storage.set_string(IMPORT_SCALE_KEY, self.import_scale.to_string());
        storage.set_string(SHOW_COLLISION_KEY, self.show_collision.to_string());
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.tick_frame_clock();
        // Once per frame, before anything reads the cache: what the thread
        // finished, then what the document has started wanting.
        self.drain_jobs();
        self.request_decompositions();
        self.handle_close_request(ui.ctx());
        self.handle_file_drops(ui.ctx());
        // A browser drop is read asynchronously; whatever finished lands here.
        self.drain_dropped();
        self.handle_shortcuts(ui.ctx());
        self.update_title(ui.ctx());

        self.menu_bar(ui);

        // A hovered glyph names its joint; otherwise the ID buffer's hit.
        // Both are last frame's — this panel is drawn before the viewport.
        let hovered = match self.hovered_frame.and_then(|f| self.robot.frames.get(&f)) {
            Some(frame) => Some(format!("{} (frame)", frame.name)),
            None => match self.hovered_joint.and_then(|j| self.robot.joints.get(&j)) {
                Some(joint) => Some(format!("{} (joint)", joint.name)),
                None => self.viewport.hovered().map(|h| self.describe_hit(h)),
            },
        };
        let selected = self.viewport.selected().map(|h| self.describe_hit(h));
        let document = format!(
            "{}{}",
            self.document_label(),
            if self.history.is_dirty() { "*" } else { "" }
        );
        status_bar::status_bar(
            ui,
            &status_bar::StatusView {
                document: &document,
                import_units: &status_bar::import_units_label(self.import_scale),
                hovered: hovered.as_deref(),
                selected: selected.as_deref(),
                instance_count: self.viewport.instance_count(),
                message: self.status.as_deref(),
                frame_dt: self.show_frame_hud.then_some(self.last_frame_dt).flatten(),
            },
        );

        self.tree_panel(ui);
        self.properties_panel(ui);

        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ui, |ui| {
                // Glyphs are derived from the document and the current `q`,
                // so they are rebuilt every frame and handed to the viewport
                // before it paints (docs/01-architecture.md §Frame loop).
                // The hover has to be resolved first: it decides both what is
                // drawn hot and whether the viewport gets the pointer at all.
                let glyphs = self.joint_glyphs();
                let frame_glyphs = self.frame_glyphs();
                self.update_glyph_hover(ui.ctx(), &glyphs, &frame_glyphs);
                self.update_snap(ui.ctx());
                let mut overlay = self.glyph_overlay(&glyphs, self.active_joint());
                self.push_frame_overlay(&mut overlay, &frame_glyphs, self.active_frame());
                self.push_align_overlay(&mut overlay);
                self.push_snap_overlay(&mut overlay);
                self.viewport.set_overlay(overlay);
                // One frame behind for the gizmo, which cannot say whether it
                // owns the cursor until it has run, and the viewport runs
                // first. Picking only: a handle or a glyph under the cursor
                // hides the geometry that would answer for it, but the camera
                // has no reason to stop (ADR-0010).
                self.viewport.set_pick_suppressed(
                    self.gizmo_state.captured
                        || self.glyph_hover.is_some()
                        || self.frame_glyph_hover.is_some(),
                );
                // The whole pointer, on the other hand, belongs to the
                // toolbar while the cursor is on it — it is drawn in the
                // viewport's own layer, which `contains_pointer` cannot see
                // through — and to a gizmo drag in flight, which is solved
                // against the projection it started in and would make the
                // part jump if the camera moved under it.
                let over_toolbar = ui
                    .ctx()
                    .pointer_hover_pos()
                    .is_some_and(|pos| self.toolbar_rect.is_some_and(|rect| rect.contains(pos)));
                self.viewport
                    .set_pointer_blocked(over_toolbar || self.gizmo_dragging());
                // A placement click means "put it here", not "select what is
                // under the cursor" — but the hover pick has to keep running,
                // because it is what the snap is computed from.
                self.viewport.set_select_suppressed(self.snapping());

                let response = self.viewport.ui(ui);
                let rect = response.rect;
                // After the viewport, in registration order: egui's hit
                // test prefers the widget registered last, so the gizmo
                // takes the pointer from the viewport and the toolbar from
                // the gizmo — but the gizmo only registers a widget at all
                // on the frames a handle is under the cursor, which is what
                // `contains_pointer` is for (ADR-0010).
                self.gizmo_ui(ui, rect, response.contains_pointer());
                self.tool_bar(ui, rect);
                // The viewport's pick is suppressed while a glyph is
                // hovered, so these clicks are unambiguous, and a hovered
                // glyph and a snap are mutually exclusive.
                let clicked = ui.input(|i| i.pointer.primary_clicked());
                if let Some(frame) = self.frame_glyph_hover.filter(|_| clicked) {
                    self.select(Selection::Frame(frame));
                } else if let Some(joint) = self.glyph_hover.filter(|_| clicked) {
                    self.select(Selection::Joint(joint));
                } else if let Some(snap) = self.snap_candidate.filter(|_| clicked) {
                    match self.tool {
                        Tool::PlaceJoint => {
                            if let Selection::Joint(joint) = self.selection {
                                self.place_joint(joint, &snap);
                            }
                        }
                        Tool::Align => self.align_click(&snap),
                        Tool::Move | Tool::Rotate => {
                            if let Some(frame) = self.placing_frame() {
                                self.place_frame(frame, &snap);
                            }
                        }
                        Tool::Select => {}
                    }
                }
            });
        self.sync_selection_from_viewport();
        // Windows float over everything, so they go last.
        self.joints_window(ui.ctx());
        self.materials_window(ui.ctx());
        self.unsaved_changes_modal(ui.ctx());
        self.export_modal(ui.ctx());
        if self.quit_confirmed {
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        self.record_first_frame();
    }
}
