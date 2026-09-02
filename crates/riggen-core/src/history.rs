//! Snapshot undo/redo (docs/02-data-model.md §Commands and history). The
//! document is small, so a [`History`] entry is a whole `Robot` clone: undo
//! is a swap, nothing has an inverse, and a refused command costs nothing.
//!
//! A *gesture* is the one exception to "one command, one entry": a scrubbed
//! number field previews *through* the document, one `Set…` per frame, and
//! the user dragged once — so every apply under the same [`GestureId`]
//! lands in the entry the first one opened (plans/panels-and-numbers OPEN 1:
//! one gesture = one history entry).

use crate::command::{Command, Created, EditError};
use crate::robot::Robot;

/// Names a gesture — a drag from press to release — so the applies it
/// makes coalesce into one undo entry. The value is the caller's (the
/// widget's id hash, say); the history only compares it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GestureId(pub u64);

/// Pre-states for undo, popped states for redo, and where the saved
/// document sits in that stack.
#[derive(Debug, Clone, Default)]
pub struct History {
    undo: Vec<Robot>,
    redo: Vec<Robot>,
    /// `Some(n)`: the document with `n` entries on the undo stack is the one
    /// on disk. `None`: it was undone and edited past, so no reachable state
    /// matches the file until the next save.
    saved_depth: Option<usize>,
    /// The gesture the top undo entry belongs to, while it is still open:
    /// another apply under this id advances the document without a new
    /// entry. Anything else that touches the stack closes it.
    gesture: Option<GestureId>,
}

impl History {
    /// Empty history for a document that counts as saved (a fresh `New` or
    /// a just-loaded file).
    pub fn new() -> Self {
        Self {
            undo: Vec::new(),
            redo: Vec::new(),
            saved_depth: Some(0),
            gesture: None,
        }
    }

    /// Runs `command` on a clone, validates, then pushes the pre-state and
    /// commits. A refused command leaves `robot` and the history untouched.
    /// A command that changes nothing (the properties panel re-committing
    /// what the document already holds) is dropped without an entry.
    /// Returns what `AddLink` / `AddFrame` created. Ends any open gesture:
    /// a plain edit is its own entry.
    pub fn apply(
        &mut self,
        robot: &mut Robot,
        command: Command,
    ) -> Result<Option<Created>, EditError> {
        self.gesture = None;
        self.apply_coalescing(robot, command, None)
    }

    /// [`apply`](Self::apply) inside a gesture: the first changing apply
    /// under `gesture` pushes the pre-state as usual and opens the gesture;
    /// every later one under the same id, until [`end_gesture`]
    /// (Self::end_gesture) or anything else touches the stack, only
    /// advances the document. One drag, one undo entry, however many
    /// frames it previewed through. A refused or no-op command neither
    /// opens nor closes anything.
    pub fn apply_in_gesture(
        &mut self,
        robot: &mut Robot,
        command: Command,
        gesture: GestureId,
    ) -> Result<Option<Created>, EditError> {
        self.apply_coalescing(robot, command, Some(gesture))
    }

    /// Closes the open gesture, if any: the next apply under the same id
    /// starts a new entry. Release calls this.
    pub fn end_gesture(&mut self) {
        self.gesture = None;
    }

    fn apply_coalescing(
        &mut self,
        robot: &mut Robot,
        command: Command,
        gesture: Option<GestureId>,
    ) -> Result<Option<Created>, EditError> {
        let mut next = robot.clone();
        let created = command.apply(&mut next)?;
        if next == *robot {
            return Ok(created);
        }
        if gesture.is_some() && gesture == self.gesture {
            *robot = next;
            return Ok(created);
        }
        // The saved state was in the redo stack: it is now unreachable.
        if self.saved_depth.is_some_and(|d| d > self.undo.len()) {
            self.saved_depth = None;
        }
        self.redo.clear();
        self.undo.push(std::mem::replace(robot, next));
        self.gesture = gesture;
        Ok(created)
    }

    /// Restores the previous state; `false` when there is none.
    pub fn undo(&mut self, robot: &mut Robot) -> bool {
        self.gesture = None;
        match self.undo.pop() {
            Some(prev) => {
                self.redo.push(std::mem::replace(robot, prev));
                true
            }
            None => false,
        }
    }

    /// Re-applies the last undone state; `false` when there is none.
    pub fn redo(&mut self, robot: &mut Robot) -> bool {
        self.gesture = None;
        match self.redo.pop() {
            Some(next) => {
                self.undo.push(std::mem::replace(robot, next));
                true
            }
            None => false,
        }
    }

    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    /// Entries on the undo stack — how many edits the current state is past
    /// the initial one.
    pub fn undo_depth(&self) -> usize {
        self.undo.len()
    }

    /// The current document was just written to disk. A save is a
    /// boundary: it ends any open gesture, so what the file holds is a
    /// whole entry and the next apply dirties the document again.
    pub fn mark_saved(&mut self) {
        self.gesture = None;
        self.saved_depth = Some(self.undo.len());
    }

    /// Whether the current document differs from the saved one — by
    /// history position, not by content: an edit and its exact reversal
    /// still count as dirty.
    pub fn is_dirty(&self) -> bool {
        self.saved_depth != Some(self.undo.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::LinkId;
    use crate::pose::Pose;
    use crate::robot::{Joint, Link};

    fn add_link(history: &mut History, robot: &mut Robot, name: &str) -> LinkId {
        let root = robot.root;
        history
            .apply(
                robot,
                Command::AddLink {
                    link: Box::new(Link::new(name)),
                    parent: root,
                    joint: Joint::fixed(format!("{name}_joint"), root, root),
                },
            )
            .unwrap()
            .and_then(Created::link)
            .unwrap()
    }

    #[test]
    fn apply_pushes_and_undo_redo_restore_equal_documents() {
        let mut robot = Robot::new("r");
        let mut history = History::new();
        let empty = robot.clone();
        assert!(!history.can_undo() && !history.can_redo());
        assert!(!history.undo(&mut robot), "nothing to undo");
        assert!(!history.redo(&mut robot), "nothing to redo");

        let arm = add_link(&mut history, &mut robot, "arm");
        let one = robot.clone();
        history
            .apply(&mut robot, Command::RenameLink(arm, "upper_arm".into()))
            .unwrap();
        let two = robot.clone();
        assert_eq!(history.undo_depth(), 2);
        assert!(history.can_undo() && !history.can_redo());

        assert!(history.undo(&mut robot));
        assert_eq!(robot, one);
        assert!(history.undo(&mut robot));
        assert_eq!(robot, empty);
        assert!(!history.undo(&mut robot));
        assert!(history.redo(&mut robot));
        assert_eq!(robot, one);
        assert!(history.redo(&mut robot));
        assert_eq!(robot, two);
        assert!(!history.redo(&mut robot));
        assert_eq!(history.undo_depth(), 2);
    }

    #[test]
    fn a_new_edit_clears_redo() {
        let mut robot = Robot::new("r");
        let mut history = History::new();
        add_link(&mut history, &mut robot, "a");
        assert!(history.undo(&mut robot));
        assert!(history.can_redo());
        add_link(&mut history, &mut robot, "b");
        assert!(!history.can_redo());
        assert_eq!(history.undo_depth(), 1);
        assert!(robot.links.values().any(|l| l.name == "b"));
        assert!(!robot.links.values().any(|l| l.name == "a"));
    }

    #[test]
    fn a_refused_command_touches_nothing() {
        let mut robot = Robot::new("r");
        let mut history = History::new();
        let arm = add_link(&mut history, &mut robot, "arm");
        let before = robot.clone();
        let err = history
            .apply(&mut robot, Command::RenameLink(arm, "base_link".into()))
            .unwrap_err();
        assert!(matches!(err, EditError::Invalid(_)));
        assert_eq!(robot, before);
        assert_eq!(history.undo_depth(), 1);
        assert!(!history.can_redo());
        // Even a command that fails after the mutation (validation) — the
        // id counter did not move either, because the clone took the hit.
        assert_eq!(robot.next_id, before.next_id);
    }

    #[test]
    fn a_no_op_command_adds_no_entry() {
        let mut robot = Robot::new("r");
        let mut history = History::new();
        let arm = add_link(&mut history, &mut robot, "arm");
        let joint = robot.parent_joint(arm).unwrap();
        history.mark_saved();
        history
            .apply(&mut robot, Command::RenameLink(arm, "arm".into()))
            .unwrap();
        let same = robot.joints[&joint].clone();
        history
            .apply(&mut robot, Command::SetJoint(joint, same))
            .unwrap();
        history
            .apply(&mut robot, Command::SetLinkMaterial(arm, None))
            .unwrap();
        assert_eq!(history.undo_depth(), 1);
        assert!(!history.is_dirty());
        let _ = Pose::IDENTITY;
    }

    fn rename(history: &mut History, robot: &mut Robot, link: LinkId, name: &str, g: GestureId) {
        history
            .apply_in_gesture(robot, Command::RenameLink(link, name.into()), g)
            .unwrap();
    }

    #[test]
    fn a_gesture_is_one_entry_however_many_applies() {
        let mut robot = Robot::new("r");
        let mut history = History::new();
        let arm = add_link(&mut history, &mut robot, "arm");
        let before = robot.clone();
        let depth = history.undo_depth();

        let drag = GestureId(7);
        for name in ["a1", "a2", "a3", "a4", "a5"] {
            rename(&mut history, &mut robot, arm, name, drag);
        }
        assert_eq!(robot.links[&arm].name, "a5");
        assert_eq!(history.undo_depth(), depth + 1, "one entry for the drag");
        history.end_gesture();

        assert!(history.undo(&mut robot));
        assert_eq!(robot, before, "one undo restores the pre-drag state");
        assert!(history.redo(&mut robot));
        assert_eq!(robot.links[&arm].name, "a5");
    }

    #[test]
    fn a_different_gesture_or_a_release_starts_a_new_entry() {
        let mut robot = Robot::new("r");
        let mut history = History::new();
        let arm = add_link(&mut history, &mut robot, "arm");
        let depth = history.undo_depth();

        rename(&mut history, &mut robot, arm, "a1", GestureId(1));
        rename(&mut history, &mut robot, arm, "a2", GestureId(1));
        rename(&mut history, &mut robot, arm, "b1", GestureId(2));
        assert_eq!(history.undo_depth(), depth + 2, "another id, another entry");

        history.end_gesture();
        rename(&mut history, &mut robot, arm, "b2", GestureId(2));
        assert_eq!(
            history.undo_depth(),
            depth + 3,
            "released and pressed again"
        );

        // A plain apply closes the gesture, and the id does not reopen it.
        rename(&mut history, &mut robot, arm, "c1", GestureId(3));
        history
            .apply(&mut robot, Command::RenameLink(arm, "plain".into()))
            .unwrap();
        rename(&mut history, &mut robot, arm, "c2", GestureId(3));
        assert_eq!(history.undo_depth(), depth + 6);

        // So does undo: the entry it popped must not be advanced.
        rename(&mut history, &mut robot, arm, "d1", GestureId(4));
        assert!(history.undo(&mut robot));
        assert_eq!(robot.links[&arm].name, "c2");
        rename(&mut history, &mut robot, arm, "d2", GestureId(4));
        assert!(history.undo(&mut robot));
        assert_eq!(robot.links[&arm].name, "c2");
    }

    #[test]
    fn a_gesture_dirties_and_saves_like_a_single_apply() {
        let mut robot = Robot::new("r");
        let mut history = History::new();
        let arm = add_link(&mut history, &mut robot, "arm");
        history.mark_saved();
        assert!(!history.is_dirty());

        let drag = GestureId(1);
        // A no-op under a gesture opens nothing and dirties nothing.
        rename(&mut history, &mut robot, arm, "arm", drag);
        assert!(!history.is_dirty());
        assert_eq!(history.undo_depth(), 1);

        rename(&mut history, &mut robot, arm, "a1", drag);
        assert!(history.is_dirty());
        rename(&mut history, &mut robot, arm, "a2", drag);
        assert!(history.is_dirty());
        assert_eq!(history.undo_depth(), 2);
        history.end_gesture();
        assert!(history.undo(&mut robot));
        assert!(!history.is_dirty(), "one undo is back at the saved state");

        // Saving mid-gesture is a boundary: the next apply under the same id
        // is a new entry, so the document is dirty again.
        assert!(history.redo(&mut robot));
        rename(&mut history, &mut robot, arm, "b1", GestureId(2));
        history.mark_saved();
        assert!(!history.is_dirty());
        rename(&mut history, &mut robot, arm, "b2", GestureId(2));
        assert!(history.is_dirty());
        assert!(history.undo(&mut robot));
        assert_eq!(robot.links[&arm].name, "b1");
        assert!(!history.is_dirty());
    }

    #[test]
    fn new_history_is_clean_and_edits_dirty_it() {
        let mut robot = Robot::new("r");
        let mut history = History::new();
        assert!(!history.is_dirty());
        add_link(&mut history, &mut robot, "a");
        assert!(history.is_dirty());
        history.mark_saved();
        assert!(!history.is_dirty());
        // Undoing below the saved mark is dirty; redoing back to it is not.
        assert!(history.undo(&mut robot));
        assert!(history.is_dirty());
        assert!(history.redo(&mut robot));
        assert!(!history.is_dirty());
        // Editing above it and undoing back is clean again.
        add_link(&mut history, &mut robot, "b");
        assert!(history.is_dirty());
        assert!(history.undo(&mut robot));
        assert!(!history.is_dirty());
    }

    #[test]
    fn editing_past_the_saved_mark_is_dirty_until_saved_again() {
        let mut robot = Robot::new("r");
        let mut history = History::new();
        add_link(&mut history, &mut robot, "a");
        add_link(&mut history, &mut robot, "b");
        history.mark_saved(); // saved at depth 2
        assert!(history.undo(&mut robot)); // depth 1
        add_link(&mut history, &mut robot, "c"); // depth 2, but not the saved state
        assert!(history.is_dirty());
        // No amount of undo reaches the saved document any more.
        assert!(history.undo(&mut robot));
        assert!(history.is_dirty());
        assert!(history.undo(&mut robot));
        assert!(history.is_dirty());
        assert!(history.redo(&mut robot));
        assert!(history.redo(&mut robot));
        assert!(history.is_dirty());
        history.mark_saved();
        assert!(!history.is_dirty());
        assert!(history.undo(&mut robot));
        assert!(history.is_dirty());
    }
}
