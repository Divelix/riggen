//! The link tree (left panel): one row per link with its parent joint's
//! name and kind, click to select, double-click or F2 to rename inline,
//! drag a row onto another to reparent it (`keep_world_pose: true`, so the
//! part stays where it is). The panel draws from the document and pushes
//! every edit through a command *after* drawing, so nothing mutates the
//! tree while it is being walked.

use riggen_core::{Command, JointId, Link, LinkId};

use crate::app::{RiggenApp, Selection};

/// Inline-rename state: the link being renamed and the text so far.
#[derive(Debug, Clone, Default)]
pub(crate) struct TreeState {
    pub(crate) renaming: Option<(LinkId, String)>,
    /// The joint whose row the pointer was over while this frame was drawn:
    /// its glyph is highlighted in the viewport (`glyphs.rs`). Consumed once
    /// per frame by `update_glyph_hover`.
    pub(crate) hovered_joint: Option<JointId>,
    /// Set when a rename starts so the text field grabs focus on its first
    /// frame, then cleared.
    focus_rename: bool,
}

/// What a row asked for. Collected while drawing, applied after.
enum TreeAction {
    Select(Selection),
    StartRename(LinkId),
    /// The pointer is over this joint's row.
    HoverJoint(JointId),
    /// A keystroke in the rename field: the buffer lives on `self`.
    TypeRename(LinkId, String),
    CommitRename(LinkId, String),
    CancelRename,
    Reparent {
        link: LinkId,
        new_parent: LinkId,
    },
}

impl RiggenApp {
    /// Starts renaming `link` inline (F2, double-click).
    pub fn start_rename(&mut self, link: LinkId) {
        if let Some(l) = self.robot.links.get(&link) {
            self.tree.renaming = Some((link, l.name.clone()));
            self.tree.focus_rename = true;
        }
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
                            .on_hover_text("Remove the selected link and its subtree (Delete)")
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
                TreeAction::StartRename(link) => self.start_rename(link),
                TreeAction::HoverJoint(joint) => self.tree.hovered_joint = Some(joint),
                TreeAction::TypeRename(link, text) => self.tree.renaming = Some((link, text)),
                TreeAction::CommitRename(link, name) => {
                    self.tree.renaming = None;
                    // Same name → no-op command, dropped by History.
                    let _ = self.apply(Command::RenameLink(link, name));
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

    /// One link and, indented under it, its children (via the joints whose
    /// parent it is, in id order).
    fn tree_row(&self, ui: &mut egui::Ui, link: LinkId, actions: &mut Vec<TreeAction>) {
        let children: Vec<LinkId> = self
            .robot
            .child_joints(link)
            .map(|j| self.robot.joints[&j].child)
            .collect();
        let id = ui.make_persistent_id(("tree_row", link));
        if children.is_empty() {
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
                for child in children {
                    self.tree_row(ui, child, actions);
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

    fn row_name(&self, ui: &mut egui::Ui, link: LinkId, actions: &mut Vec<TreeAction>) {
        let name = &self.robot.links[&link].name;
        if let Some((renaming, text)) = &self.tree.renaming
            && *renaming == link
        {
            let mut text = text.clone();
            let edit = ui.add(
                egui::TextEdit::singleline(&mut text)
                    .id_salt(("rename", link))
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
                actions.push(TreeAction::CommitRename(link, text));
            } else if edit.changed() {
                // Keep typing: the buffer lives on `self`, updated after
                // the draw like every other action.
                actions.push(TreeAction::TypeRename(link, text));
            }
            return;
        }
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
            actions.push(TreeAction::StartRename(link));
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
