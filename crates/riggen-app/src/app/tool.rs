//! The active tool and the toolbar that floats in the viewport's top-left
//! corner beside the `View | Edit` control, in Edit only
//! (docs/01-architecture.md §Panels and menus, ADR-0021).
//!
//! A tool is *modal*: it decides what a click and a drag in the viewport
//! mean. `Select` is the M1 behaviour and the resting state — `Esc` always
//! comes back to it. The four editing tools rewrite frames, and every
//! frame-rewriting command in `riggen-core` works in the **zero
//! configuration** — which is what Edit mode is for the whole of its
//! stay (ADR-0021 §2), so a tool never has to rewind anything itself.

use super::{RiggenApp, Selection};

/// What a viewport gesture means.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Tool {
    /// Click picks a link, drag orbits: the M1 viewport.
    #[default]
    Select,
    /// Gizmo: drag translates the selected link (its parent joint's origin)
    /// or the selected joint's pivot.
    Move,
    /// The same gizmo, rotating.
    Rotate,
    /// Click a feature to put the selected joint's origin and axis on it.
    PlaceJoint,
    /// Click a feature on the selected link, then one anywhere, to bring
    /// the first onto the second.
    Align,
}

impl Tool {
    /// Toolbar order, which is also the order the buttons are found by
    /// label in the snapshot suite.
    pub const ALL: [Tool; 5] = [
        Tool::Select,
        Tool::Move,
        Tool::Rotate,
        Tool::PlaceJoint,
        Tool::Align,
    ];

    /// The toolbar button's text, and the name `debug_state` reports.
    pub fn label(self) -> &'static str {
        match self {
            Tool::Select => "Select",
            Tool::Move => "Move",
            Tool::Rotate => "Rotate",
            Tool::PlaceJoint => "Place joint",
            Tool::Align => "Align",
        }
    }

    /// The key that switches to this tool (`shortcuts.rs`).
    ///
    /// Blender's `G` = grab and `R` = rotate, with `V` and `B` standing in
    /// for the initials of Select and Align: **`W A S D E Q` are reserved**
    /// for the fly camera the backlog wants, and a tool must not spend one
    /// of them. The digits are out too — the viewport owns
    /// `Num1/3/5/7/0` (standard views), `P` (projection) and `Home` (fit).
    pub fn shortcut(self) -> egui::Key {
        match self {
            Tool::Select => egui::Key::V,
            Tool::Move => egui::Key::G,
            Tool::Rotate => egui::Key::R,
            Tool::PlaceJoint => egui::Key::J,
            Tool::Align => egui::Key::B,
        }
    }
}

/// What the status bar says while a tool waits for a selection it can
/// use — set on tool entry and on every selection change while the tool
/// is active, cleared once the selection satisfies it, so a click that the
/// tool would otherwise ignore has a reason next to it. Public so a test
/// asserts on the constant, not on prose.
pub const MOVE_NEEDS_TARGET: &str = "move: select a link, a joint or a frame to move";
pub const ROTATE_NEEDS_TARGET: &str = "rotate: select a link, a joint or a frame to rotate";
pub const MOVE_ROOT: &str =
    "move: the root link is the world — select a child link, a joint or a frame";
pub const ROTATE_ROOT: &str =
    "rotate: the root link is the world — select a child link, a joint or a frame";
pub const PLACE_JOINT_NEEDS_JOINT: &str =
    "place joint: select the joint to place — in the tree, or by its glyph";
pub const ALIGN_NEEDS_LINK: &str = "align: select the link to bring onto another (not the root)";

const TOOL_NEEDS: [&str; 6] = [
    MOVE_NEEDS_TARGET,
    ROTATE_NEEDS_TARGET,
    MOVE_ROOT,
    ROTATE_ROOT,
    PLACE_JOINT_NEEDS_JOINT,
    ALIGN_NEEDS_LINK,
];

impl RiggenApp {
    pub fn tool(&self) -> Tool {
        self.tool
    }

    /// Switches tools. `q` is not touched: Edit is the zero configuration
    /// already (ADR-0021 §2). In View the tools are Edit's and the only
    /// tool is Select, whatever was asked for.
    pub fn set_tool(&mut self, tool: Tool) {
        // A half-finished align belongs to the gesture, not to the app.
        self.cancel_align();
        self.tool = if self.mode == super::Mode::View {
            Tool::Select
        } else {
            tool
        };
        self.refresh_tool_status();
    }

    /// What the active tool is missing, if anything: the selection it
    /// needs before a click means something. Mirrors `gizmo_target`,
    /// `place_joint` and `align_click`.
    pub fn tool_need(&self) -> Option<&'static str> {
        let root = self.robot.root;
        match self.tool {
            Tool::Select => None,
            Tool::Move | Tool::Rotate => {
                let (target, on_root) = if self.tool == Tool::Move {
                    (MOVE_NEEDS_TARGET, MOVE_ROOT)
                } else {
                    (ROTATE_NEEDS_TARGET, ROTATE_ROOT)
                };
                match self.selection {
                    Selection::None => Some(target),
                    Selection::Link(l) if l == root => Some(on_root),
                    _ => None,
                }
            }
            Tool::PlaceJoint => match self.selection {
                Selection::Joint(_) => None,
                _ => Some(PLACE_JOINT_NEEDS_JOINT),
            },
            Tool::Align => match self.selection {
                Selection::Link(l) if l != root => None,
                _ => Some(ALIGN_NEEDS_LINK),
            },
        }
    }

    /// Puts the tool's need in the status bar, or takes a need that is now
    /// met out of it. Called wherever the tool or the selection changes.
    pub(crate) fn refresh_tool_status(&mut self) {
        match self.tool_need() {
            Some(need) => self.status = Some(need.to_owned()),
            None => {
                if self
                    .status
                    .as_deref()
                    .is_some_and(|s| TOOL_NEEDS.contains(&s))
                {
                    self.status = None;
                }
            }
        }
    }

    /// The toolbar: five buttons in a popup frame, drawn beside the
    /// `View | Edit` control in Edit (`mode.rs::viewport_chrome`) and
    /// returning the tool a click chose, applied by the caller after both
    /// have drawn.
    pub(crate) fn tool_bar(&self, ui: &mut egui::Ui) -> Option<Tool> {
        let mut chosen = None;
        egui::Frame::popup(ui.style()).show(ui, |ui| {
            ui.horizontal(|ui| {
                for tool in Tool::ALL {
                    // The key is in the tooltip, not on the button: a
                    // binding nobody can find is folklore, and a toolbar
                    // that spells out five of them is a toolbar nobody
                    // can read.
                    if ui
                        .selectable_label(self.tool == tool, tool.label())
                        .on_hover_text(format!("{} ({})", tool.label(), tool.shortcut().name()))
                        .clicked()
                    {
                        chosen = Some(tool);
                    }
                }
            });
        });
        chosen
    }
}
