//! The link tree (left panel): one row per link with its parent joint's
//! name and kind, a row per named frame under the link it hangs on, click
//! to select, double-click or F2 to rename inline, drag a row onto another
//! to reparent it (`keep_world_pose: true`, so the part stays where it is).
//! The panel draws from the document and pushes every edit through a
//! command *after* drawing, so nothing mutates the tree while it is being
//! walked.

use std::fmt;

use riggen_core::{Command, FrameId, JointId, Link, LinkId};

use crate::app::{RiggenApp, Selection};

/// What a tree row can rename inline: a link or one of its frames. Joints
/// are renamed in the properties panel, not here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RenameTarget {
    Link(LinkId),
    Frame(FrameId),
}

impl fmt::Display for RenameTarget {
    /// `"l3"` / `"f7"` — what `debug_state().ui.renaming` reports.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Link(l) => write!(f, "{l}"),
            Self::Frame(id) => write!(f, "{id}"),
        }
    }
}

/// Inline-rename state: what is being renamed and the text so far.
#[derive(Debug, Clone, Default)]
pub(crate) struct TreeState {
    pub(crate) renaming: Option<(RenameTarget, String)>,
    /// The joint whose row the pointer was over while this frame was drawn:
    /// its glyph is highlighted in the viewport (`glyphs.rs`). Consumed once
    /// per frame by `update_glyph_hover`.
    pub(crate) hovered_joint: Option<JointId>,
    /// The same for a frame row and its triad glyph.
    pub(crate) hovered_frame: Option<FrameId>,
    /// Set when a rename starts so the text field grabs focus on its first
    /// frame, then cleared.
    focus_rename: bool,
}

/// The frame row's marker: a crosshair, since a frame *is* a pose. Kept
/// out of the name so a rename never has to strip it.
const FRAME_MARK: &str = "⌖";

/// What a row asked for. Collected while drawing, applied after.
enum TreeAction {
    Select(Selection),
    StartRename(RenameTarget),
    /// The pointer is over this joint's row.
    HoverJoint(JointId),
    /// The pointer is over this frame's row.
    HoverFrame(FrameId),
    /// A keystroke in the rename field: the buffer lives on `self`.
    TypeRename(RenameTarget, String),
    CommitRename(RenameTarget, String),
    CancelRename,
    Reparent {
        link: LinkId,
        new_parent: LinkId,
    },
}

impl RiggenApp {
    /// Starts renaming a link or a frame inline (F2, double-click).
    pub(crate) fn start_rename_target(&mut self, target: RenameTarget) {
        let name = match target {
            RenameTarget::Link(l) => self.robot.links.get(&l).map(|l| l.name.clone()),
            RenameTarget::Frame(f) => self.robot.frames.get(&f).map(|f| f.name.clone()),
        };
        if let Some(name) = name {
            self.tree.renaming = Some((target, name));
            self.tree.focus_rename = true;
        }
    }

    /// Starts renaming `link` inline (F2, double-click).
    pub fn start_rename(&mut self, link: LinkId) {
        self.start_rename_target(RenameTarget::Link(link));
    }

    pub(crate) fn tree_panel(&mut self, ui: &mut egui::Ui) {
        let mut actions = Vec::new();
        egui::Panel::left("tree_panel")
            .resizable(true)
            .default_size(240.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.heading("Links");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui
                            .button("− Remove")
                            .on_hover_text("Remove the selection: a link with its subtree, or a frame (Delete)")
                            .clicked()
                        {
                            self.remove_selected();
                        }
                        if ui
                            .button("+ Link")
                            .on_hover_text("Add an empty link under the selection")
                            .clicked()
                        {
                            let parent = self.insertion_parent();
                            if let Ok(link) = self.add_link(Link::new("link"), parent) {
                                self.select(Selection::Link(link));
                                self.start_rename(link);
                            }
                        }
                    });
                });
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| {
                    let root = self.robot.root;
                    self.tree_row(ui, root, &mut actions);
                    // Space below the rows so a drop onto the root's level is
                    // reachable and the panel does not end at the last row.
                    ui.allocate_space(egui::vec2(ui.available_width(), 16.0));
                });
            });

        // The rename field asked for focus on the frame it appeared; once.
        self.tree.focus_rename = false;
        for action in actions {
            match action {
                TreeAction::Select(selection) => self.select(selection),
                TreeAction::StartRename(target) => self.start_rename_target(target),
                TreeAction::HoverJoint(joint) => self.tree.hovered_joint = Some(joint),
                TreeAction::HoverFrame(frame) => self.tree.hovered_frame = Some(frame),
                TreeAction::TypeRename(target, text) => self.tree.renaming = Some((target, text)),
                TreeAction::CommitRename(target, name) => {
                    self.tree.renaming = None;
                    // Same name → no-op command, dropped by History.
                    let _ = self.apply(match target {
                        RenameTarget::Link(l) => Command::RenameLink(l, name),
                        RenameTarget::Frame(f) => Command::RenameFrame(f, name),
                    });
                }
                TreeAction::CancelRename => self.tree.renaming = None,
                TreeAction::Reparent { link, new_parent } => {
                    let _ = self.apply(Command::Reparent {
                        link,
                        new_parent,
                        keep_world_pose: true,
                    });
                }
            }
        }
    }

    /// One link and, indented under it, its frames and then its children
    /// (via the joints whose parent it is, in id order). Frames first: they
    /// belong to *this* link, the children are their own subtrees.
    fn tree_row(&self, ui: &mut egui::Ui, link: LinkId, actions: &mut Vec<TreeAction>) {
        let children: Vec<LinkId> = self
            .robot
            .child_joints(link)
            .map(|j| self.robot.joints[&j].child)
            .collect();
        let frames: Vec<FrameId> = self
            .robot
            .frames
            .iter()
            .filter(|(_, f)| f.parent == link)
            .map(|(&id, _)| id)
            .collect();
        let id = ui.make_persistent_id(("tree_row", link));
        if children.is_empty() && frames.is_empty() {
            ui.horizontal(|ui| {
                // Same indent the collapsing toggle would have taken.
                ui.add_space(ui.spacing().indent);
                self.row_contents(ui, link, id, actions);
            });
            return;
        }
        egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), id, true)
            .show_header(ui, |ui| self.row_contents(ui, link, id, actions))
            .body(|ui| {
                for frame in frames {
                    self.frame_row(ui, frame, actions);
                }
                for child in children {
                    self.tree_row(ui, child, actions);
                }
            });
    }

    /// A named frame under its link: the frame marker, the name (click to
    /// select, double-click to rename), and a weak `frame` label so a row
    /// is never mistaken for a link.
    fn frame_row(&self, ui: &mut egui::Ui, frame: FrameId, actions: &mut Vec<TreeAction>) {
        let Some(f) = self.robot.frames.get(&frame) else {
            return;
        };
        ui.horizontal(|ui| {
            ui.add_space(ui.spacing().indent);
            let selected = self.selection == Selection::Frame(frame);
            let hovered = self.hovered_frame() == Some(frame);
            if self.rename_field(ui, RenameTarget::Frame(frame), actions) {
                return;
            }
            let mut name = egui::RichText::new(format!("{FRAME_MARK} {}", f.name));
            // Hovered but not selected reads as "this one", not as a second
            // selection — the same rule the joint label follows.
            if hovered && !selected {
                name = name.color(egui::Color32::from_rgb(255, 236, 179));
            }
            let response = ui.add(egui::Button::selectable(selected, name));
            let label = ui.add(egui::Label::new(
                egui::RichText::new("frame").small().weak(),
            ));
            if response.hovered() || label.hovered() {
                actions.push(TreeAction::HoverFrame(frame));
            }
            if response.double_clicked() {
                actions.push(TreeAction::StartRename(RenameTarget::Frame(frame)));
            } else if response.clicked() {
                actions.push(TreeAction::Select(Selection::Frame(frame)));
            }
        });
    }

    /// The row itself: a drop zone (reparent onto this link) around the
    /// name, which is the drag source, and the parent joint's label.
    ///
    /// Not `dnd_drag_source`: that lays a drag-only widget over the row and
    /// egui then swallows clicks on it (`hit_test.rs`: a top-most widget
    /// that senses only drags hides the click-widget under it). The name
    /// senses click *and* drag instead and sets the payload itself.
    fn row_contents(
        &self,
        ui: &mut egui::Ui,
        link: LinkId,
        _id: egui::Id,
        actions: &mut Vec<TreeAction>,
    ) {
        let (_, dropped) = ui.dnd_drop_zone::<LinkId, _>(egui::Frame::NONE, |ui| {
            self.row_name(ui, link, actions);
            if let Some(joint) = self.robot.parent_joint(link) {
                self.row_joint(ui, joint, actions);
            }
        });
        if let Some(dragged) = dropped
            && *dragged != link
        {
            actions.push(TreeAction::Reparent {
                link: *dragged,
                new_parent: link,
            });
        }
    }

    /// The inline-rename text field, when `target` is the thing being
    /// renamed; `true` when it drew, so the caller skips its own row.
    fn rename_field(
        &self,
        ui: &mut egui::Ui,
        target: RenameTarget,
        actions: &mut Vec<TreeAction>,
    ) -> bool {
        let Some((renaming, text)) = &self.tree.renaming else {
            return false;
        };
        if *renaming != target {
            return false;
        }
        let mut text = text.clone();
        let edit = ui.add(
            egui::TextEdit::singleline(&mut text)
                .id_salt(("rename", target.to_string()))
                .desired_width(120.0),
        );
        if self.tree.focus_rename {
            edit.request_focus();
        }
        // Escape surrenders focus too, so it is checked first.
        let escaped = ui.input(|i| i.key_pressed(egui::Key::Escape));
        let entered = ui.input(|i| i.key_pressed(egui::Key::Enter));
        if escaped {
            actions.push(TreeAction::CancelRename);
        } else if entered || edit.lost_focus() {
            actions.push(TreeAction::CommitRename(target, text));
        } else if edit.changed() {
            // Keep typing: the buffer lives on `self`, updated after the
            // draw like every other action.
            actions.push(TreeAction::TypeRename(target, text));
        }
        true
    }

    fn row_name(&self, ui: &mut egui::Ui, link: LinkId, actions: &mut Vec<TreeAction>) {
        if self.rename_field(ui, RenameTarget::Link(link), actions) {
            return;
        }
        let name = &self.robot.links[&link].name;
        let selected = self.selection == Selection::Link(link);
        let response =
            ui.add(egui::Button::selectable(selected, name).sense(egui::Sense::click_and_drag()));
        response.dnd_set_drag_payload(link);
        // Hovering a link's row highlights the joint that holds it up: the
        // row and the glyph are two views of the same edge.
        if response.hovered()
            && let Some(joint) = self.robot.parent_joint(link)
        {
            actions.push(TreeAction::HoverJoint(joint));
        }
        if response.double_clicked() {
            actions.push(TreeAction::StartRename(RenameTarget::Link(link)));
        } else if response.clicked() {
            actions.push(TreeAction::Select(Selection::Link(link)));
        }
    }

    fn row_joint(&self, ui: &mut egui::Ui, joint: JointId, actions: &mut Vec<TreeAction>) {
        let j = &self.robot.joints[&joint];
        let selected = self.selection == Selection::Joint(joint);
        let hovered = self.hovered_joint == Some(joint);
        let mut text = egui::RichText::new(format!(
            "{} · {}",
            j.name,
            format!("{:?}", j.kind).to_lowercase()
        ))
        .small();
        // Hovered but not selected reads as "this one", not as a second
        // selection: the row brightens rather than taking the selectable
        // background.
        text = if hovered && !selected {
            text.strong().color(egui::Color32::from_rgb(255, 236, 179))
        } else {
            text.weak()
        };
        let response = ui.selectable_label(selected, text);
        if response.hovered() {
            actions.push(TreeAction::HoverJoint(joint));
        }
        if response.clicked() {
            actions.push(TreeAction::Select(Selection::Joint(joint)));
        }
    }
}
