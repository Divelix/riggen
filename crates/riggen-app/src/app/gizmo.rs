//! The transform gizmo: `transform-gizmo-egui` behind a thin adapter
//! (ADR-0007, docs/ARCHITECTURE.md §Frame loop).
//!
//! What the gizmo edits follows the selection (plans/m2-placement-ux
//! OPEN 2):
//!
//! - a **link** → its parent joint's `origin`, so the link and its whole
//!   subtree move; committed as one `SetJoint` through
//!   `fk::origin_for_world`;
//! - a **frame** → the frame's own pose on its link, committed as one
//!   `SetFrame`; nothing else moves, because nothing hangs off a frame;
//! - a **joint** → the pivot itself, committed as one `MoveJointFrame`.
//!   The joint frame *is* the child link frame, and the axis is expressed
//!   in it, so the axis rides along with the gizmo and is written back
//!   unchanged: a rotation of the gizmo rotates the axis in the world, a
//!   translation leaves it pointing the same way. Nothing in the world
//!   moves, which is the point — only the pivot does.
//!
//! Drag previews, release commits (AGENTS.md: one gesture = one command).
//! During a link drag `preview_world` overrides the FK pose in `sync_scene`
//! and no command exists yet; the single command is applied when the crate
//! stops reporting an interaction.
//!
//! The crate speaks `mint`, which is how its glam 0.32 and our glam 0.30
//! meet without either crate naming the other's types (ADR-0007).
//!
//! The egui half — registering an interaction widget, feeding the crate a
//! `GizmoInteraction`, painting its mesh — is [`interact`] below rather
//! than the crate's own `GizmoExt::interact`, because that one takes the
//! pointer away from the viewport on *every* frame a gizmo is on screen
//! (ADR-0010).

use riggen_core::glam::{DMat4, DQuat, DVec3};
use riggen_core::{
    Command, FrameId, GestureId, JointId, JointState, LinkId, Pose, origin_for_world,
};
use transform_gizmo_egui::{
    Gizmo, GizmoConfig, GizmoInteraction, GizmoMode, GizmoOrientation, GizmoResult, GizmoVisuals,
    math::Transform,
};

use super::{RiggenApp, Selection, Tool};

/// What the gizmo is attached to this frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GizmoTarget {
    /// Moves the link (its parent joint's origin); the subtree follows.
    Link(LinkId),
    /// Moves the pivot; the geometry stays.
    Joint(JointId),
    /// Moves the frame on its link; nothing else moves (ADR-0012).
    Frame(FrameId),
}

impl GizmoTarget {
    /// `"link l3"` / `"joint j7"`, the same spelling `Selection::describe`
    /// uses, for `debug_state`.
    pub fn describe(self) -> String {
        match self {
            Self::Link(l) => format!("link {l}"),
            Self::Joint(j) => format!("joint {j}"),
            Self::Frame(f) => format!("frame {f}"),
        }
    }
}

/// The gizmo and the drag it is in the middle of.
#[derive(Default)]
pub(crate) struct GizmoState {
    gizmo: Gizmo,
    /// The target and the world pose the drag is currently showing. `Some`
    /// exactly while a drag is in flight.
    pub(crate) drag: Option<(GizmoTarget, Pose)>,
    /// Whether the gizmo owns the cursor: a handle is under it, or a drag
    /// it started is still in flight. Fed to
    /// `Viewport::set_pick_suppressed` *before* the viewport runs, so it
    /// is one frame behind — the same lag egui's own interaction has.
    pub(crate) captured: bool,
    /// Which rotate ring the cursor is on, while one is
    /// ([`ring_under_cursor`]). `None` outside the Rotate tool, off the
    /// handles, and on the crate's fourth — view-axis — ring.
    pub(crate) hovered_ring: Option<RingAxis>,
    /// The wheel gesture the ring steps under, and when its last notch
    /// was: notches closer together than [`WHEEL_BURST`] coalesce into one
    /// history entry (ADR-0019 §2).
    wheel: Option<(GestureId, f64)>,
}

impl RiggenApp {
    /// The gizmo's target for the current tool and selection, or `None`
    /// when there is nothing to draw.
    pub fn gizmo_target(&self) -> Option<GizmoTarget> {
        if !matches!(self.tool, Tool::Move | Tool::Rotate) {
            return None;
        }
        match self.selection {
            // The root has no parent joint to write.
            Selection::Link(l) if l != self.robot.root => Some(GizmoTarget::Link(l)),
            Selection::Joint(j) if self.robot.joints.contains_key(&j) => {
                Some(GizmoTarget::Joint(j))
            }
            Selection::Frame(f) if self.robot.frames.contains_key(&f) => {
                Some(GizmoTarget::Frame(f))
            }
            _ => None,
        }
    }

    /// The pose a frame's glyph is drawn at: the drag in flight, if this is
    /// the frame being dragged, else its FK pose.
    pub(crate) fn dragged_frame(&self, frame: FrameId) -> Option<Pose> {
        match self.gizmo_state.drag {
            Some((GizmoTarget::Frame(dragged), pose)) if dragged == frame => Some(pose),
            _ => None,
        }
    }

    /// The pivot a joint's glyph is drawn at while its gizmo is being
    /// dragged, the way [`Self::dragged_frame`] does for a frame.
    ///
    /// Nothing else in the scene moves with a joint — the geometry stays
    /// exactly where it is, because a pivot move is a change of `origin`
    /// and not of the link — so without this the one gesture that moves a
    /// joint shows nothing at all until the release
    /// (`docs/BACKLOG.md`, retired here). The drag's pose is the child link
    /// frame, which *is* the joint frame (AGENTS.md), and `commit_gizmo`
    /// turns it back into an `origin` against the parent, so the release
    /// puts the glyph where the drag already had it.
    pub(crate) fn dragged_pivot(&self, joint: JointId) -> Option<Pose> {
        match self.gizmo_state.drag {
            Some((GizmoTarget::Joint(dragged), pose)) if dragged == joint => Some(pose),
            _ => None,
        }
    }

    /// Where the gizmo sits: a link's own frame, or — for a joint — the
    /// child link frame, which *is* the joint frame.
    pub fn gizmo_world(&self, target: GizmoTarget) -> Option<Pose> {
        if let Some((dragged, pose)) = self.gizmo_state.drag
            && dragged == target
        {
            return Some(pose);
        }
        if let GizmoTarget::Frame(f) = target {
            return riggen_core::frames(&self.robot, &self.q).get(&f).copied();
        }
        let link = match target {
            GizmoTarget::Link(l) => l,
            GizmoTarget::Joint(j) => self.robot.joints.get(&j)?.child,
            GizmoTarget::Frame(_) => unreachable!("handled above"),
        };
        riggen_core::fk(&self.robot, &self.q).get(&link).copied()
    }

    /// Draws and drives the gizmo. Called inside the central panel *after*
    /// `Viewport::ui`, so the widget [`interact`] registers comes later in
    /// the same layer and therefore wins the pointer; the toolbar is drawn
    /// after it in turn.
    ///
    /// `viewport_has_pointer` is the viewport response's
    /// `contains_pointer()`: the pointer is inside the viewport's rect and
    /// no *other layer* — a window, a modal — is over it. Same-layer widgets
    /// drawn on top (the toolbar) do not clear it, so the toolbar's own rect
    /// is checked here.
    pub(crate) fn gizmo_ui(
        &mut self,
        ui: &mut egui::Ui,
        rect: egui::Rect,
        viewport_has_pointer: bool,
    ) {
        let Some(target) = self.gizmo_target() else {
            self.end_gizmo_drag(None);
            self.gizmo_state.captured = false;
            self.gizmo_state.hovered_ring = None;
            return;
        };
        let Some(world) = self.gizmo_world(target) else {
            self.gizmo_state.captured = false;
            self.gizmo_state.hovered_ring = None;
            return;
        };

        let aspect = rect.width().max(1.0) / rect.height().max(1.0);
        let modes = match self.tool {
            Tool::Rotate => GizmoMode::all_rotate(),
            _ => GizmoMode::all_translate(),
        };
        // Hoisted out of the config: `ring_under_cursor` rebuilds the
        // crate's own ring geometry from exactly these, and a second copy
        // of the numbers would be a second copy to keep in step.
        let view = self.viewport.camera.view_matrix().as_dmat4();
        let projection = self.viewport.camera.proj_matrix(aspect).as_dmat4();
        let visuals = gizmo_visuals();
        self.gizmo_state.gizmo.update_config(GizmoConfig {
            view_matrix: view.into(),
            projection_matrix: projection.into(),
            viewport: rect,
            modes,
            // Local: the handles follow the frame being edited, which is
            // what "put this joint's axis along that bore" needs.
            orientation: GizmoOrientation::Local,
            pixels_per_point: ui.ctx().pixels_per_point(),
            visuals,
            ..Default::default()
        });

        let transform =
            Transform::from_scale_rotation_translation(DVec3::ONE, world.r.normalize(), world.t);

        // Our own hit test, not the widget's `hovered()`: `pick_preview`
        // asks the subgizmos directly, so it answers this frame rather than
        // the next one, and — the point of the change — it answers *before* a
        // widget has been registered, which is what lets us not register one
        // at all. The gizmo's pose inside `config` is a frame old (the crate
        // refreshes it from the targets inside `update`), so the first frame
        // after the selection moves the gizmo aims at where it was; every
        // other frame is exact.
        let cursor = ui.ctx().pointer_hover_pos();
        let over_handle = viewport_has_pointer
            && cursor.is_some_and(|c| {
                !self.over_chrome(c) && self.gizmo_state.gizmo.pick_preview((c.x, c.y))
            });
        // Which ring, for the wheel. Gated on `over_handle`, so the crate
        // has already said a handle is there and this only says *which* —
        // and only under Rotate, where the three rings are the handles.
        self.gizmo_state.hovered_ring = (self.tool == Tool::Rotate && over_handle)
            .then(|| {
                ring_under_cursor(
                    view,
                    projection,
                    rect,
                    world,
                    visuals.gizmo_size,
                    visuals.stroke_width,
                    cursor?,
                )
            })
            .flatten();

        // A drag that has left its handle still owns the pointer.
        let active = self.gizmo_state.drag.is_some();
        let result = interact(
            &mut self.gizmo_state.gizmo,
            ui,
            rect,
            &[transform],
            cursor,
            over_handle,
            active,
        );

        match result {
            Some((_, transforms)) => {
                if let Some(next) = transforms.first() {
                    let mut pose = Pose::new(
                        DVec3::from(next.translation),
                        DQuat::from(next.rotation).normalize(),
                    );
                    // A translate drag lands on the feature under the
                    // cursor, if there is one: the same ladder, marker and
                    // readout the placement tools use, and the gizmo's own
                    // origin is what lands on it (ADR-0019 §5). The
                    // rotation is the drag's, untouched — a snap says where,
                    // never which way round.
                    if self.translate_dragging()
                        && let Some(snap) = self.snap_candidate
                    {
                        pose = Pose::new(snap.point, pose.r);
                    }
                    self.gizmo_state.drag = Some((target, pose));
                    // Only a link drag moves anything in the world; a pivot
                    // move leaves the geometry exactly where it is.
                    if let GizmoTarget::Link(link) = target {
                        self.preview_world = Some((link, pose));
                    }
                    self.sync_scene();
                    ui.ctx().request_repaint();
                }
            }
            None => self.end_gizmo_drag(Some(target)),
        }
        self.step_ring_with_wheel(ui, target);
        self.gizmo_state.captured = over_handle || self.gizmo_state.drag.is_some();
    }

    /// Ends a drag in flight: drops the preview and commits the one command
    /// the gesture is worth. `expected` guards against committing a drag of
    /// a target that is no longer the gizmo's (the selection changed
    /// mid-drag).
    fn end_gizmo_drag(&mut self, expected: Option<GizmoTarget>) {
        let Some((target, pose)) = self.gizmo_state.drag.take() else {
            return;
        };
        self.preview_world = None;
        if expected.is_some_and(|e| e != target) {
            self.sync_scene();
            return;
        }
        self.commit_gizmo(target, pose, None);
    }

    /// One gesture, one command. `gesture` coalesces a burst of wheel
    /// notches into a single undo entry; a drag commits outside one,
    /// because a drag is already exactly one command.
    fn commit_gizmo(&mut self, target: GizmoTarget, world: Pose, gesture: Option<GestureId>) {
        match target {
            GizmoTarget::Link(link) => {
                let Some(joint_id) = self.robot.parent_joint(link) else {
                    return;
                };
                let Some(origin) = origin_for_world(&self.robot, link, world) else {
                    return;
                };
                let mut joint = self.robot.joints[&joint_id].clone();
                joint.origin = origin;
                self.commit(gesture, Command::SetJoint(joint_id, joint));
            }
            GizmoTarget::Joint(joint_id) => {
                let Some(joint) = self.robot.joints.get(&joint_id).cloned() else {
                    return;
                };
                let Some(origin) = origin_for_world(&self.robot, joint.child, world) else {
                    return;
                };
                self.commit(
                    gesture,
                    Command::MoveJointFrame {
                        joint: joint_id,
                        origin,
                        // In the child frame, which is the frame the gizmo just
                        // moved: the axis rides along unchanged.
                        axis: joint.axis,
                    },
                );
            }
            GizmoTarget::Frame(id) => {
                let Some(edited) = self.frame_at_world(id, world) else {
                    return;
                };
                self.commit(gesture, Command::SetFrame(id, edited));
            }
        }
    }

    /// One command, inside `gesture` when there is one.
    fn commit(&mut self, gesture: Option<GestureId>, command: Command) {
        let _ = match gesture {
            Some(gesture) => self.apply_in_gesture(command, gesture),
            None => self.apply(command),
        };
    }

    /// The wheel over a rotate ring steps it: 5° a notch, 1° with shift,
    /// about the ring's own local axis (ADR-0019 §2). The viewport has
    /// already been told not to zoom (`set_wheel_claimed`), so the notches
    /// are ours to read.
    ///
    /// A burst of notches is one gesture and therefore one undo entry, on
    /// the same [`WHEEL_BURST`] rule the Properties scrubbers use; a pause,
    /// or a move to another ring or another target, starts a new one.
    fn step_ring_with_wheel(&mut self, ui: &egui::Ui, target: GizmoTarget) {
        let Some(ring) = self.gizmo_state.hovered_ring else {
            return;
        };
        // A drag owns the gesture while it is in flight, and the wheel is
        // blocked outright then anyway.
        if self.gizmo_state.drag.is_some() {
            return;
        }
        let (notches, fine) = wheel_notches(ui);
        if notches == 0 {
            return;
        }
        let Some(world) = self.gizmo_world(target) else {
            return;
        };
        let step = if fine { WHEEL_STEP_FINE } else { WHEEL_STEP };
        let now = ui.input(|i| i.time);
        // About the ring's *own* axis, which is the local one: the gizmo is
        // configured `GizmoOrientation::Local`, so the ring the user is
        // pointing at is an axis of the frame being edited.
        let turn = DQuat::from_axis_angle(ring.local(), f64::from(notches) * step.to_radians());
        let pose = Pose::new(world.t, (world.r * turn).normalize());

        let gesture = wheel_gesture(target, ring);
        let burst = matches!(self.gizmo_state.wheel, Some((open, last))
            if open == gesture && now - last < WHEEL_BURST);
        if !burst {
            self.end_gesture();
        }
        self.gizmo_state.wheel = Some((gesture, now));
        self.commit_gizmo(target, pose, Some(gesture));
        ui.ctx().request_repaint();
    }

    /// The frame `id` would be, with its pose re-expressed so it sits at
    /// `world`. Like every frame-rewriting edit this reads the **zero
    /// configuration** — which is what the tool is in, since `set_tool`
    /// resets `q` before an editing tool runs.
    pub(crate) fn frame_at_world(&self, id: FrameId, world: Pose) -> Option<riggen_core::Frame> {
        let frame = self.robot.frames.get(&id)?;
        let parent = riggen_core::fk(&self.robot, &JointState::default())
            .get(&frame.parent)?
            .inverse();
        Some(riggen_core::Frame {
            pose: parent.compose(&world),
            ..frame.clone()
        })
    }

    /// Whether a gizmo drag is in flight — `debug_state` and the repaint
    /// policy.
    pub fn gizmo_dragging(&self) -> bool {
        self.gizmo_state.drag.is_some()
    }

    /// Whether the gizmo owns the cursor (hovered or dragging).
    pub fn gizmo_captured(&self) -> bool {
        self.gizmo_state.captured
    }

    /// Which rotate ring the cursor is on, if any — what the wheel will
    /// step (`debug_state`, and step 3 of plans/viewport-answers-the-mouse).
    pub fn hovered_ring(&self) -> Option<RingAxis> {
        self.gizmo_state.hovered_ring
    }

    /// Where `world` lands on screen, in egui logical points — what aims a
    /// scripted click at a part or at the gizmo.
    pub fn project_world(&self, world: DVec3) -> Option<egui::Pos2> {
        self.viewport.project(world)
    }
}

/// The egui half of the gizmo: what `GizmoExt::interact` does, minus the
/// part that broke the viewport (ADR-0010).
///
/// The crate's adapter registers a one-point click-and-drag widget at the
/// cursor on **every** frame. egui's hit test prefers the widget registered
/// last, and the gizmo is registered after the viewport — so while any gizmo
/// was on screen the viewport underneath saw no hover, no click and no wheel
/// event at all, which is the M2 exit gate's dead camera and its clicks that
/// only flickered the hover tint.
///
/// This registers that widget only on the frames the gizmo actually wants
/// the pointer: `over_handle` (a handle is under the cursor) or `active` (a
/// drag it started is still in flight, the cursor by then anywhere). Every
/// other frame the viewport keeps the pointer it has always had. And even
/// then it senses **clicks only**, so orbit and pan still start from a
/// handle — see the comment on the `ui.interact` call.
///
/// `hovered` is handed to the crate from our own hit test rather than from
/// the widget's `Response`, so it is not a frame behind — the crate only
/// needs to know whether a handle is under the cursor, and we had to answer
/// that before registering anything.
fn interact(
    gizmo: &mut Gizmo,
    ui: &egui::Ui,
    rect: egui::Rect,
    targets: &[Transform],
    cursor: Option<egui::Pos2>,
    over_handle: bool,
    active: bool,
) -> Option<(GizmoResult, Vec<Transform>)> {
    let cursor = cursor.unwrap_or_default();
    if over_handle || active {
        // `Sense::click()`, not `click_and_drag()`. The widget exists only to
        // deny the viewport the *click* under a handle; the gizmo itself
        // reads the raw pointer, never this response. Sensing drags as well
        // would take the middle-drag too: `hit_test` picks `hits.drag` from
        // the widgets that sense a drag, `interaction.rs` sets
        // `potential_drag_id` from it on a press of **any** button, and the
        // orbit would land on a widget that does not orbit. Click-only, the
        // hit test reports `click: gizmo, drag: viewport` — the exact split
        // this needs.
        ui.interact(
            egui::Rect::from_center_size(cursor, egui::Vec2::splat(1.0)),
            ui.id().with("riggen-gizmo-pointer"),
            egui::Sense::click(),
        );
    }

    let (drag_started, dragging) = ui.input(|i| {
        (
            i.pointer.button_pressed(egui::PointerButton::Primary),
            i.pointer.button_down(egui::PointerButton::Primary),
        )
    });
    let result = gizmo.update(
        GizmoInteraction {
            cursor_pos: (cursor.x, cursor.y),
            hovered: over_handle,
            drag_started,
            dragging,
        },
        targets,
    );

    // Drawn with egui's painter over the viewport, in the viewport's own
    // rect and layer, not depth-tested (ADR-0007).
    let draw = gizmo.draw();
    egui::Painter::new(ui.ctx().clone(), ui.layer_id(), rect).add(egui::Mesh {
        indices: draw.indices,
        vertices: draw
            .vertices
            .into_iter()
            .zip(draw.colors)
            .map(|(pos, [r, g, b, a])| egui::epaint::Vertex {
                pos: pos.into(),
                uv: egui::Pos2::default(),
                color: egui::Rgba::from_rgba_premultiplied(r, g, b, a).into(),
            })
            .collect(),
        ..Default::default()
    });

    result
}

/// The gizmo's visual style, in one place: [`RiggenApp::gizmo_ui`] hands it
/// to the crate and [`ring_under_cursor`] measures with it, so the ring the
/// wheel steps cannot drift from the ring that was drawn.
fn gizmo_visuals() -> GizmoVisuals {
    GizmoVisuals {
        // The axes triad's colours, so red/green/blue means the same thing
        // in the corner and under the cursor.
        x_color: egui::Color32::from_rgb(230, 64, 64),
        y_color: egui::Color32::from_rgb(89, 217, 89),
        z_color: egui::Color32::from_rgb(77, 140, 242),
        // 75 px (the crate's default) is a small target for a handle that
        // has to be hit on the first try.
        gizmo_size: 110.0,
        ..Default::default()
    }
}

/// Degrees a wheel notch turns a ring, and what shift makes of it
/// (ADR-0019 §2). Twelve notches to a quarter turn is fine enough that the
/// shifted step is for the last degree or two.
pub(crate) const WHEEL_STEP: f64 = 5.0;
pub(crate) const WHEEL_STEP_FINE: f64 = 1.0;

/// Seconds between notches that still count as one gesture — the same
/// number, for the same reason, as the Properties scrubbers' `WHEEL_BURST`.
const WHEEL_BURST: f64 = 0.4;

/// The gesture a burst of notches coalesces under: the target and the ring
/// together, so moving to another ring — or another joint — starts a new
/// undo entry rather than extending the last one.
fn wheel_gesture(target: GizmoTarget, ring: RingAxis) -> GestureId {
    use std::hash::{DefaultHasher, Hash, Hasher};
    let mut hasher = DefaultHasher::new();
    (target, ring.label()).hash(&mut hasher);
    GestureId(hasher.finish())
}

/// This frame's wheel notches over a ring, up positive, and whether shift
/// was held on them — the fine step.
///
/// Read from the raw events, like the viewport's own `raw_wheel_delta_y`
/// and the Properties panel's stepper. Events carrying egui's **zoom**
/// modifier are skipped — that gesture is egui's UI scale and never reached
/// the viewport's wheel either — but its **horizontal-scroll** modifier,
/// which is shift, is not: shift is the fine step here (ADR-0019 §2), and
/// there is nothing in the viewport for a horizontal scroll to move. The
/// raw event carries `delta.y` whatever the modifier; only egui's own
/// smoothing would have remapped it.
///
/// The modifier is read off the **event**, not off `InputState`: an event
/// carries the modifiers as they were when it happened, which is what a
/// gesture means by "with shift held", and it needs no key event to have
/// been seen first.
pub(crate) fn wheel_notches(ui: &egui::Ui) -> (i32, bool) {
    let options = ui.ctx().options(|o| o.input_options);
    let ignored = options.zoom_modifier;
    ui.input(|input| {
        input
            .raw
            .events
            .iter()
            .filter_map(|event| match event {
                egui::Event::MouseWheel {
                    unit,
                    delta,
                    modifiers,
                    ..
                } if !modifiers.matches_any(ignored) => {
                    let lines = match unit {
                        egui::MouseWheelUnit::Line => delta.y,
                        egui::MouseWheelUnit::Point => delta.y / options.line_scroll_speed,
                        egui::MouseWheelUnit::Page => delta.y.signum(),
                    };
                    // A notch is at least one, whatever the platform's
                    // lines-per-notch setting says.
                    Some((
                        lines.abs().max(1.0).round() as i32 * lines.signum() as i32,
                        modifiers.shift,
                    ))
                }
                _ => None,
            })
            .fold((0, false), |(sum, fine), (notches, shift)| {
                (sum + notches, fine || shift)
            })
    })
}

/// One of the rotate gizmo's three rings: the axis it turns about, in the
/// **local** frame the gizmo is drawn on (`GizmoOrientation::Local`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RingAxis {
    X,
    Y,
    Z,
}

impl RingAxis {
    /// The order [`ring_under_cursor`] tests them in; ties break on depth,
    /// not on this.
    pub const ALL: [RingAxis; 3] = [Self::X, Self::Y, Self::Z];

    /// The name `debug_state` reports.
    pub fn label(self) -> &'static str {
        match self {
            Self::X => "x",
            Self::Y => "y",
            Self::Z => "z",
        }
    }

    /// The axis in the gizmo's own frame.
    pub fn local(self) -> DVec3 {
        match self {
            Self::X => DVec3::X,
            Self::Y => DVec3::Y,
            Self::Z => DVec3::Z,
        }
    }
}

/// Which rotate ring the cursor is on, if any.
///
/// `Gizmo::pick_preview` answers only *whether* a handle is under the
/// cursor, and `transform-gizmo`'s subgizmos are private, so which ring it
/// is has to be recomputed here. This mirrors the crate's own geometry —
/// `subgizmo/rotation.rs::pick_preview` and `arc_angle`, `config.rs`'s
/// `scale_factor` / `focus_distance`, `math.rs`'s `ray_to_plane_origin` —
/// against the same matrices and the same [`gizmo_visuals`] the config was
/// built from, so the two agree by construction rather than by luck. The
/// caller gates it on the crate's own `pick_preview`, so this only ever
/// says *which*, never *whether*.
///
/// `None` on the crate's fourth ring — the view circle drawn outside the
/// other three, turning about the camera's own axis — which the wheel
/// deliberately does not claim (plans/viewport-answers-the-mouse: a step
/// about an axis the document has no name for is not one the user can
/// predict, and zoom keeps working there).
pub(crate) fn ring_under_cursor(
    view: DMat4,
    projection: DMat4,
    rect: egui::Rect,
    world: Pose,
    gizmo_size: f32,
    stroke_width: f32,
    cursor: egui::Pos2,
) -> Option<RingAxis> {
    let view_projection = projection * view;
    // The two radii and the tolerance the crate derives from its scale.
    let scale = ring_scale(view, projection, rect, world);
    if !scale.is_finite() || scale <= 0.0 {
        return None;
    }
    let radius = scale * gizmo_size as f64;
    let outer = scale * (gizmo_size + stroke_width + 5.0) as f64;
    let focus = scale * (stroke_width as f64 / 2.0 + 5.0);

    let inverse = view_projection.inverse();
    let origin = screen_to_world(rect, inverse, cursor, -1.0);
    let direction = (screen_to_world(rect, inverse, cursor, 1.0) - origin).normalize_or_zero();
    if direction == DVec3::ZERO {
        return None;
    }

    // The crate's `view_forward()`: the third *row* of the view matrix (its
    // config holds a `mint::RowMatrix4`, which glam transposes on the way
    // in). Its sign is what the arc test measures against, so the
    // handedness rule comes along with it.
    let mut forward = view.row(2).truncate();
    if left_handed(view, projection) {
        forward = -forward;
    }

    let best = RingAxis::ALL
        .iter()
        .filter_map(|&axis| {
            let normal = (world.r * axis.local()).normalize_or_zero();
            let (t, distance) = ray_to_ring(normal, world.t, origin, direction)?;
            if (distance - radius).abs() > focus {
                return None;
            }
            // The direction from the centre to the point of the ring
            // nearest the hit — the crate walks from the hit back toward
            // the centre, which lands on the same unit vector.
            let offset = (origin + direction * t - world.t).normalize_or_zero();
            let angle = f64::atan2(offset.cross(forward).dot(normal), offset.dot(forward));
            // The back half of a ring is not drawn and not pickable, unless
            // the ring is nearly face-on, where the arc closes into a full
            // circle.
            (angle.abs() < arc_angle(normal.dot(forward).abs())).then_some((t, axis))
        })
        .min_by(|(first, _), (second, _)| first.total_cmp(second));

    let (depth, axis) = best?;
    // The view ring is a full circle at `outer`, and it sits close enough to
    // the others that a cursor can be in both bands; when it is the nearer,
    // the wheel is not ours.
    let view_ring = ray_to_ring(forward, world.t, origin, direction)
        .filter(|(_, distance)| (distance - outer).abs() <= focus);
    match view_ring {
        Some((view_depth, _)) if view_depth < depth => None,
        _ => Some(axis),
    }
}

/// World units per screen point at the gizmo's depth
/// (`config.rs::update_for_config`), which is what turns the gizmo's size
/// in pixels into the radius of its rings in the world.
fn ring_scale(view: DMat4, projection: DMat4, rect: egui::Rect, world: Pose) -> f64 {
    let model = DMat4::from_rotation_translation(world.r.normalize(), world.t);
    let mvp = projection * view * model;
    mvp.w_axis.w / projection.x_axis.x / rect.width().max(1.0) as f64 * 2.0
}

/// `math.rs::screen_to_world`: a point on the near (`z = -1`) or far
/// (`z = 1`) plane, in world coordinates.
fn screen_to_world(rect: egui::Rect, inverse: DMat4, pos: egui::Pos2, z: f64) -> DVec3 {
    let x = (((pos.x - rect.min.x) / rect.width().max(1.0)) * 2.0 - 1.0) as f64;
    let y = (((pos.y - rect.min.y) / rect.height().max(1.0)) * 2.0 - 1.0) as f64;
    let mut world = inverse * riggen_core::glam::DVec4::new(x, -y, z, 1.0);
    // w is zero when the far plane is at infinity.
    if world.w.abs() < 1e-7 {
        world.w = 1e-7;
    }
    (world / world.w).truncate()
}

/// `math.rs::ray_to_plane_origin` on a ring's plane: the ray parameter and
/// the distance from the centre, or `None` when the ray runs parallel to
/// the plane or meets it behind the eye.
fn ray_to_ring(
    normal: DVec3,
    center: DVec3,
    ray_origin: DVec3,
    ray_dir: DVec3,
) -> Option<(f64, f64)> {
    let denominator = normal.dot(ray_dir);
    if denominator.abs() < 10e-8 {
        return None;
    }
    let t = (center - ray_origin).dot(normal) / denominator;
    (t >= 0.0).then(|| (t, (ray_origin + ray_dir * t - center).length()))
}

/// `rotation.rs::arc_angle`: how much of a ring is drawn, and therefore
/// pickable — half of it edge-on, all of it once it is within a few degrees
/// of facing the camera.
fn arc_angle(dot: f64) -> f64 {
    use std::f64::consts::{FRAC_PI_2, PI};
    const MIN_DOT: f64 = 0.990;
    const MAX_DOT: f64 = 0.995;
    let angle = ((dot - MIN_DOT).max(0.0) / (MAX_DOT - MIN_DOT)).min(1.0) * FRAC_PI_2 + FRAC_PI_2;
    if (angle - PI).abs() < 1e-2 { PI } else { angle }
}

/// `config.rs::update_for_config`'s handedness rule, which decides the sign
/// of the forward vector the arc test uses. Both of our projections —
/// `perspective_rh` and `orthographic_rh` — are right-handed, so this is
/// `false`; it is mirrored anyway, because a camera change should not
/// silently rotate the pickable half of every ring.
fn left_handed(view: DMat4, projection: DMat4) -> bool {
    if projection.z_axis.w == 0.0 {
        projection.z_axis.z > 0.0
            && view
                .x_axis
                .truncate()
                .cross(view.y_axis.truncate())
                .dot(view.z_axis.truncate())
                < 0.0
    } else {
        projection.z_axis.w > 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use riggen_core::glam::{DMat4, DVec3, DVec4};

    const RECT: egui::Rect = egui::Rect {
        min: egui::Pos2::ZERO,
        max: egui::Pos2::new(800.0, 600.0),
    };
    const SIZE: f32 = 110.0;
    const STROKE: f32 = 4.0;

    /// A camera five metres out along +Z, looking at the origin: the local
    /// Z ring is face-on, X and Y are edge-on.
    fn camera() -> (DMat4, DMat4) {
        let view = DMat4::look_at_rh(DVec3::new(0.0, 0.0, 5.0), DVec3::ZERO, DVec3::Y);
        let projection = DMat4::perspective_rh(
            45f64.to_radians(),
            RECT.width() as f64 / RECT.height() as f64,
            0.1,
            100.0,
        );
        (view, projection)
    }

    /// Where a world point lands, the inverse of [`screen_to_world`].
    fn project(view: DMat4, projection: DMat4, at: DVec3) -> egui::Pos2 {
        let clip = projection * view * DVec4::new(at.x, at.y, at.z, 1.0);
        let ndc = clip.truncate() / clip.w;
        egui::pos2(
            RECT.min.x + (ndc.x as f32 * 0.5 + 0.5) * RECT.width(),
            RECT.min.y + (0.5 - ndc.y as f32 * 0.5) * RECT.height(),
        )
    }

    /// A point on a ring of `world`, `turn` of the way around it.
    fn ring_point(view: DMat4, projection: DMat4, world: Pose, axis: RingAxis, turn: f64) -> DVec3 {
        let radius = ring_scale(view, projection, RECT, world) * SIZE as f64;
        let normal = world.r * axis.local();
        let (start, _) = normal.any_orthonormal_pair();
        let spoke = DQuat::from_axis_angle(normal, turn * std::f64::consts::TAU) * start;
        world.t + spoke * radius
    }

    /// The cursor on that point.
    fn on_ring(
        view: DMat4,
        projection: DMat4,
        world: Pose,
        axis: RingAxis,
        turn: f64,
    ) -> egui::Pos2 {
        project(
            view,
            projection,
            ring_point(view, projection, world, axis, turn),
        )
    }

    fn ring(world: Pose, cursor: egui::Pos2) -> Option<RingAxis> {
        let (view, projection) = camera();
        ring_under_cursor(view, projection, RECT, world, SIZE, STROKE, cursor)
    }

    #[test]
    fn the_face_on_ring_is_the_one_under_the_cursor() {
        let (view, projection) = camera();
        let world = Pose::IDENTITY;
        // Three points around the Z ring, none of them where it crosses the
        // two edge-on ones.
        for turn in [0.125, 0.375, 0.625] {
            let cursor = on_ring(view, projection, world, RingAxis::Z, turn);
            assert_eq!(
                ring(world, cursor),
                Some(RingAxis::Z),
                "at {turn} of a turn"
            );
        }
    }

    #[test]
    fn the_ring_follows_the_frame_it_is_drawn_on() {
        let (view, projection) = camera();
        // A quarter turn about Y puts the *local X* ring face-on.
        let world = Pose::new(
            DVec3::ZERO,
            DQuat::from_axis_angle(DVec3::Y, std::f64::consts::FRAC_PI_2),
        );
        let cursor = on_ring(view, projection, world, RingAxis::X, 0.125);
        assert_eq!(ring(world, cursor), Some(RingAxis::X));
    }

    #[test]
    fn the_middle_and_the_outside_are_not_a_ring() {
        let (view, projection) = camera();
        let world = Pose::IDENTITY;
        assert_eq!(ring(world, project(view, projection, DVec3::ZERO)), None);
        let radius = ring_scale(view, projection, RECT, world) * SIZE as f64;
        let far = project(view, projection, DVec3::new(radius * 3.0, 0.0, 0.0));
        assert_eq!(ring(world, far), None);
    }

    /// The crate draws a fourth ring outside the other three, turning about
    /// the camera's own axis. The wheel does not claim it, and it is close
    /// enough to the others that saying so takes a test.
    #[test]
    fn the_view_ring_is_not_ours() {
        let (view, projection) = camera();
        let world = Pose::IDENTITY;
        let scale = ring_scale(view, projection, RECT, world);
        let outer = scale * (SIZE + STROKE + 5.0) as f64;
        for turn in [0.125f64, 0.375, 0.625] {
            let spoke = DVec3::new(
                (turn * std::f64::consts::TAU).cos(),
                (turn * std::f64::consts::TAU).sin(),
                0.0,
            );
            let cursor = project(view, projection, spoke * outer);
            assert_eq!(ring(world, cursor), None, "at {turn} of a turn");
        }
    }

    /// An edge-on ring keeps its front half: the crate draws only the arc
    /// facing the camera, and picking it where nothing is drawn would step
    /// a ring the user cannot see.
    #[test]
    fn the_back_of_a_ring_is_not_pickable() {
        let (view, projection) = camera();
        // The Y ring seen edge-on from +Z: its front half is the +Z side.
        // Tilted 60° out of the screen, so the ring's arc is a half circle
        // rather than the full one a face-on ring gets.
        let world = Pose::new(
            DVec3::ZERO,
            DQuat::from_axis_angle(DVec3::X, 60f64.to_radians()),
        );
        let one = ring_point(view, projection, world, RingAxis::Z, 0.25);
        let other = ring_point(view, projection, world, RingAxis::Z, 0.75);
        // The camera is out along +Z, so the nearer of the two is the one
        // on the half that is drawn.
        let (near, far) = if one.z > other.z {
            (one, other)
        } else {
            (other, one)
        };
        assert_eq!(
            ring(world, project(view, projection, near)),
            Some(RingAxis::Z)
        );
        assert_eq!(ring(world, project(view, projection, far)), None);
    }
}
