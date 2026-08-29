//! Snapshot undo/redo (docs/02-data-model.md §Commands and history). The
//! document is small, so a [`History`] entry is a whole `Robot` clone: undo
//! is a swap, nothing has an inverse, and a refused command costs nothing.

use crate::command::{Command, EditError};
use crate::ids::LinkId;
use crate::robot::Robot;

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
}

impl History {
    /// Empty history for a document that counts as saved (a fresh `New` or
    /// a just-loaded file).
    pub fn new() -> Self {
        Self {
            undo: Vec::new(),
            redo: Vec::new(),
            saved_depth: Some(0),
        }
    }

    /// Runs `command` on a clone, validates, then pushes the pre-state and
    /// commits. A refused command leaves `robot` and the history untouched.
    /// A command that changes nothing (the properties panel re-committing
    /// what the document already holds) is dropped without an entry.
    /// Returns the link `AddLink` created.
    pub fn apply(
        &mut self,
        robot: &mut Robot,
        command: Command,
    ) -> Result<Option<LinkId>, EditError> {
        let mut next = robot.clone();
        let created = command.apply(&mut next)?;
        if next == *robot {
            return Ok(created);
        }
        // The saved state was in the redo stack: it is now unreachable.
        if self.saved_depth.is_some_and(|d| d > self.undo.len()) {
            self.saved_depth = None;
        }
        self.redo.clear();
        self.undo.push(std::mem::replace(robot, next));
        Ok(created)
    }

    /// Restores the previous state; `false` when there is none.
    pub fn undo(&mut self, robot: &mut Robot) -> bool {
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

    /// The current document was just written to disk.
    pub fn mark_saved(&mut self) {
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
