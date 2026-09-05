//! The window's two modes (ADR-0021): **View** is the posed robot, **Edit**
//! is the v0.3 editor at the zero configuration. `Tab` switches them
//! (`shortcuts.rs`).

use super::{RiggenApp, Selection};

/// Which of the two windows the user is looking at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// The joint tree, the glyphs alone under the cursor, the wheel posing
    /// a hovered joint.
    View,
    /// The link tree, the properties panel, the toolbar and the gizmos.
    Edit,
}

impl Mode {
    /// The name the control, the status bar and `debug_state` use.
    pub fn label(self) -> &'static str {
        match self {
            Mode::View => "View",
            Mode::Edit => "Edit",
        }
    }

    /// What `Tab` switches to.
    pub fn other(self) -> Mode {
        match self {
            Mode::View => Mode::Edit,
            Mode::Edit => Mode::View,
        }
    }
}

impl RiggenApp {
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Switches modes. Edit is the zero configuration (ADR-0021 §2): going
    /// there stashes `q` and rewinds, coming back restores it — a rewind,
    /// not an edit, so no history entry either way. A selected joint is a
    /// thing in both modes and survives; a selected link or frame has no
    /// row in View and clears.
    pub fn set_mode(&mut self, mode: Mode) {
        if mode == self.mode {
            return;
        }
        self.mode = mode;
        match mode {
            Mode::Edit => {
                self.stashed_q = Some(std::mem::take(&mut self.q));
            }
            Mode::View => {
                if let Some(q) = self.stashed_q.take() {
                    self.q = q;
                    // Edit may have removed a joint or moved its limits.
                    self.clamp_q_to_document();
                }
            }
        }
        self.sync_scene();
        if matches!(self.selection, Selection::Link(_) | Selection::Frame(_)) {
            self.select(Selection::None);
        }
    }
}
