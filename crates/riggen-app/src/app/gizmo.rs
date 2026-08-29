//! The transform gizmo: `transform-gizmo-egui` behind a thin adapter
//! (ADR-0007, docs/01-architecture.md §Frame loop).
//!
//! What the gizmo edits follows the selection (plans/m2-placement-ux
//! OPEN 2):
//!
//! - a **link** → its parent joint's `origin`, so the link and its whole
//!   subtree move; committed as one `SetJoint` through
//!   `fk::origin_for_world`;
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

use riggen_core::glam::{DQuat, DVec3};
use riggen_core::{Command, JointId, LinkId, Pose, origin_for_world};
use transform_gizmo_egui::{
    Gizmo, GizmoConfig, GizmoExt, GizmoMode, GizmoOrientation, GizmoVisuals, math::Transform,
};

use super::{RiggenApp, Selection, Tool};

/// What the gizmo is attached to this frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GizmoTarget {
    /// Moves the link (its parent joint's origin); the subtree follows.
    Link(LinkId),
    /// Moves the pivot; the geometry stays.
    Joint(JointId),
}

impl GizmoTarget {
    /// `"link l3"` / `"joint j7"`, the same spelling `Selection::describe`
    /// uses, for `debug_state`.
    pub fn describe(self) -> String {
        match self {
            Self::Link(l) => format!("link {l}"),
            Self::Joint(j) => format!("joint {j}"),
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
    /// Whether the gizmo owned the cursor at the end of the last frame. Fed
    /// to `Viewport::set_input_suppressed` *before* the viewport runs, so it
    /// is one frame behind — the same lag egui's own interaction has.
    pub(crate) captured: bool,
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
        let link = match target {
            GizmoTarget::Link(l) => l,
            GizmoTarget::Joint(j) => self.robot.joints.get(&j)?.child,
        };
        riggen_core::fk(&self.robot, &self.q).get(&link).copied()
    }

    /// Draws and drives the gizmo. Called inside the central panel *after*
    /// `Viewport::ui`, so its interaction widget is registered later and
    /// therefore wins the pointer; the toolbar is drawn after it in turn.
    pub(crate) fn gizmo_ui(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        let Some(target) = self.gizmo_target() else {
            self.end_gizmo_drag(None);
            self.gizmo_state.captured = false;
            return;
        };
        let Some(world) = self.gizmo_world(target) else {
            return;
        };

        let aspect = rect.width().max(1.0) / rect.height().max(1.0);
        let modes = match self.tool {
            Tool::Rotate => GizmoMode::all_rotate(),
            _ => GizmoMode::all_translate(),
        };
        self.gizmo_state.gizmo.update_config(GizmoConfig {
            view_matrix: self.viewport.camera.view_matrix().as_dmat4().into(),
            projection_matrix: self.viewport.camera.proj_matrix(aspect).as_dmat4().into(),
            viewport: rect,
            modes,
            // Local: the handles follow the frame being edited, which is
            // what "put this joint's axis along that bore" needs.
            orientation: GizmoOrientation::Local,
            pixels_per_point: ui.ctx().pixels_per_point(),
            visuals: GizmoVisuals {
                // The axes triad's colours, so red/green/blue means the same
                // thing in the corner and under the cursor.
                x_color: egui::Color32::from_rgb(230, 64, 64),
                y_color: egui::Color32::from_rgb(89, 217, 89),
                z_color: egui::Color32::from_rgb(77, 140, 242),
                // 75 px (the crate's default) is a small target for a
                // handle that has to be hit on the first try.
                gizmo_size: 110.0,
                ..Default::default()
            },
            ..Default::default()
        });

        let transform =
            Transform::from_scale_rotation_translation(DVec3::ONE, world.r.normalize(), world.t);
        let result = self.gizmo_state.gizmo.interact(ui, &[transform]);
        self.gizmo_state.captured = self.gizmo_state.gizmo.is_focused();

        match result {
            Some((_, transforms)) => {
                let Some(next) = transforms.first() else {
                    return;
                };
                let pose = Pose::new(
                    DVec3::from(next.translation),
                    DQuat::from(next.rotation).normalize(),
                );
                self.gizmo_state.drag = Some((target, pose));
                // Only a link drag moves anything in the world; a pivot
                // move leaves the geometry exactly where it is.
                if let GizmoTarget::Link(link) = target {
                    self.preview_world = Some((link, pose));
                }
                self.sync_scene();
                ui.ctx().request_repaint();
            }
            None => self.end_gizmo_drag(Some(target)),
        }
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
        self.commit_gizmo(target, pose);
    }

    /// One gesture, one command.
    fn commit_gizmo(&mut self, target: GizmoTarget, world: Pose) {
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
                let _ = self.apply(Command::SetJoint(joint_id, joint));
            }
            GizmoTarget::Joint(joint_id) => {
                let Some(joint) = self.robot.joints.get(&joint_id).cloned() else {
                    return;
                };
                let Some(origin) = origin_for_world(&self.robot, joint.child, world) else {
                    return;
                };
                let _ = self.apply(Command::MoveJointFrame {
                    joint: joint_id,
                    origin,
                    // In the child frame, which is the frame the gizmo just
                    // moved: the axis rides along unchanged.
                    axis: joint.axis,
                });
            }
        }
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

    /// Where `world` lands on screen, in egui logical points — what aims a
    /// scripted click at a part or at the gizmo.
    pub fn project_world(&self, world: DVec3) -> Option<egui::Pos2> {
        self.viewport.project(world)
    }
}
