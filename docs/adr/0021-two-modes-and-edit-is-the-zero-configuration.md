# ADR-0021: The window has two modes, and Edit is the zero configuration

- Status: Accepted
- Date: 2026-09-05
- Supersedes: the per-tool `q` reset of `docs/01-architecture.md` §Panels
  (plans/m2-placement-ux OPEN 1); narrows the `Reparent`-at-the-current-`q`
  paragraph of `docs/02-data-model.md` §Commands (plans/panels-and-numbers
  OPEN 4) to the SDK
- Amends: the switch table of [ADR-0010](0010-gizmo-egui-glue-is-ours.md)
  §Decision 3 as [ADR-0019](0019-the-wheel-is-claimable-and-a-drag-keeps-the-hover-pick.md)
  §1 last published it, with a mode policy on top of it

## Context

The window has one mode, and it is the editor's. `riggen robot.urdf`, a
dropped `.riggen`, the web demo's sample arm — every one opens into five
placement tools, a link tree, a properties panel, and a floating Joints
window over the viewport that is the only way to pose the robot. SEED §3
says the common case is editing an existing model rather than building
one; before either, it is *looking* at it — does the imported arm move the
way the file says, are the limits right, which way does joint 3 turn. That
question is answered today by finding a slider in a window that covers the
robot, while a click on the robot selects a mesh and the wheel zooms away
from the joint under the cursor.

The two activities want opposite pointers. Posing wants the joint to be
the only thing under the cursor and the wheel to turn it; editing wants
the mesh under the cursor, the gizmo to claim the left drag, and — because
every frame-rewriting command works in the zero configuration — the robot
*un*-posed. Today the second wins by default and the first is a window.
Everything else in `docs/03-roadmap.md` §The window (the joint tree,
joints-only picking, the wheel on a glyph, the glyph's hover target, the
visibility row, zen) needs to know which mode it is in.

Three standing decisions are in the way:

- **The zero-configuration rule.** The four editing tools reset `q` on
  entry, not as a history entry, with a status line saying so. It exists
  because `MoveJointFrame` and the gizmo's commands re-express poses at
  `q = 0`, and it means the robot snaps to zero on every tool entry with
  an explanation the user has to read — the "why did it jump" minute v0.3
  was about removing.
- **`Reparent` at the current `q`** (02 §Commands, decided 2026-09-02 as a
  paragraph): the tree drop keeps the world pose at the pose the user
  sees, because "a tree edit is made while posing". Once posing has a
  mode of its own, a tree edit is never made while posing.
- **The Joints window's open-itself rule** (plans/panels-and-numbers OPEN
  3): the window opens when a document with a movable joint arrives and
  when the first movable joint is created, and remembers a close until
  the next document. It exists because posing had no home.

The switch table (ADR-0010 §3, ADR-0018 §3, ADR-0019 §1) is five switches
the app sets every frame, one channel each. It can express everything
View needs without a sixth: "both picks off" is `set_pick_suppressed`,
"the wheel is mine" is `set_wheel_claimed`. What it cannot express is
*when* — that is a policy above the table, and today the policy is one
mode's.

## Decision

**Two modes, `Tab` between them. View is the posed robot; Edit is the
zero configuration.**

1. **View** is for looking and posing. The left panel is the **joint
   tree**: the movable joints in kinematic order, fixed joints collapsed
   through, each row a scrubber that drags and steps by the wheel at the
   rotate ring's quantum (ADR-0019 §2: 5° a notch, 1° with shift; a
   prismatic joint steps by the properties panel's metre floor scaled the
   way its scrubbers scale), showing the value and both limits. A
   follower's row is read-only at its resolved `q` with the rule under it
   (ADR-0013). "Reset all" puts every joint back to zero. The properties
   panel and the toolbar are not drawn. In the viewport **only joint
   glyphs answer the cursor**: `set_pick_suppressed` is on
   unconditionally, so a mesh is neither hover-tinted nor selectable; a
   click on a glyph selects its joint; the wheel over a hovered glyph
   claims the wheel (`set_wheel_claimed`, the way a rotate ring does) and
   steps that joint's `q` at the same quantum. Posing through the wheel is
   **not** a committing gesture: `q` is derived state, never a command
   (01 §The document is the only state), so there is no history entry —
   the same wheel over a ring in Edit commits, and the mode is what tells
   the two apart. The tool keys do nothing but put a hint in the status
   bar (`Tab to edit`); `Esc` has nothing to leave.

2. **Edit** is the v0.3 editor at `q = 0`. The link tree, properties,
   toolbar, gizmos, tools and tree drag are exactly what v0.3 left, with
   one difference: `q` is zero for the whole of the mode. Entering Edit
   **stashes** `q` and rewinds to zero — the same non-history rewind
   `set_tool` did per tool — and leaving it **restores** the stash. The
   pose is never lost; it is simply not shown while frames are being
   rewritten. The per-tool reset and its status line are deleted: a tool
   enters into a configuration that is already zero. Joints are
   highlighted in the tree but not posable there — posing is View's.

3. **`Tab`** switches modes. It is consumed in `handle_shortcuts` like
   every bare key there, and yields to a focused `TextEdit`, where `Tab`
   is the field's own. A **`View | Edit`** segmented control sits at the
   viewport's top-left in both modes, the toolbar to its right in Edit,
   with `Tab` in its tooltip — a shortcut nobody can find is folklore (01
   §Panels) — and the status bar names the mode. Selection carries over:
   a selected joint survives the switch (it is a thing in both modes), a
   selected link or frame clears (View cannot show it selected).

4. **The open rule.** A document that arrives whole — File › Open, a
   dropped `.riggen`, `.urdf` or `.xml`, Import URDF / MJCF, the CLI's
   path argument, the demo's sample — opens in **View** when it has a
   movable joint. File › New and a drop of meshes alone open in
   **Edit**: a mesh drop is a link (ADR-0006), and building is editing.
   This is the open-itself rule's judgement moved from a window to the
   mode; the window and its rule are deleted with the Joints window,
   which the joint tree replaces. The mode is never persisted: the open
   rule decides it on every open.

5. **`Reparent` at `q`.** The GUI's tree drag is in Edit, so it reparents
   at zero. `Reparent`'s `at: JointState` stays in the command for the
   SDK's `q=`, whose caller can be posed; the 02 paragraph that justified
   it from the tree drop is narrowed to that.

6. **The viewport crate does not change.** The mode is a policy on the
   five switches the app already sets (01 §Picking and snapping) and on
   which panels are drawn. The layer rule (ADR-0010) holds; the glyph
   hover is the app's own screen-space test and is untouched by
   `set_pick_suppressed`.

## Consequences

- Opening a robot shows a viewer: the joint tree at the left, the robot
  alone in the viewport, the wheel and a drag on a row posing it. The
  editor is one `Tab` away and comes back to the same pose with another.
- The robot **snaps to zero once**, on `Tab` into Edit, instead of on
  every tool entry — and it is a mode change the user asked for, not a
  side effect of picking a tool. If the by-hand run finds that snap reads
  as breakage, the fallback is the idea's option B (Edit keeps the pose
  until a tool is picked) with the stash-and-restore kept; that would be
  a new ADR.
- The zero-configuration status line, `Tool::edits_frames`'s reset role,
  the Joints window, its open-itself state and the Window › Joints menu
  item are deleted. `debug_state().ui.windows` never contains `"joints"`
  again; `debug_state().ui.mode` names the mode.
- Nearly every snapshot golden opens a document with a movable joint and
  now opens in View, so every editing scenario switches through a
  harness helper that opens and presses `Tab`, and every golden is
  refreshed once (ADR-0003). That refresh is the largest cost of this
  decision and the reason it lands before the glyph redesign refreshes
  the same goldens.
- Two wheels with two semantics under one cursor — a committing step over
  a ring in Edit, a derived-state step over a glyph in View — are told
  apart only by the mode. The mode is visible in three places (the
  control, the status bar, the panel set) so that is a distinction the
  user can see.
- A document with no movable joint that arrives as a document has nothing
  to pose; what View shows for it is open (the plan's second open
  question) and does not change the rule above.

## Alternatives considered

- **B — Two modes, Edit keeps the pose with `q` locked, and the per-tool
  reset stays.** Gentler on `Tab` (nothing moves), but Edit then has two
  states — posed, then zero after the first tool — and the pose is lost
  on that first tool until View re-poses it by hand. It keeps the jump
  and the status line that explains it, which is what the mode was meant
  to remove. Kept as the fallback above.
- **C — No mode: a Joints tab in the left panel, a hovered glyph claims
  the wheel always, a toggle locks `q`.** Cheapest, no golden migration,
  no ADR. It does not change what the cursor does — meshes stay pickable,
  the toolbar and properties stay in the way, the demo still opens an
  editor — and the glyph's wheel claim competes with the rotate ring's in
  the same mode. It is the Joints window relocated.
- **A sixth switch, "view mode", in the viewport crate.** Would give the
  viewport a policy it has no reason to know and put a mode into a crate
  whose contract is a table of single-channel switches. The five suffice;
  the policy is the app's.
- **Tool keys switch to Edit and pick the tool.** One key fewer for the
  user who knows the keys, but a bare letter pressed in a viewer would
  then rewind the robot to zero and replace the panel set — a large
  effect for a key whose owner is not the mode it was pressed in. A hint
  is the smaller surprise.
- **Persist the mode across runs.** Rejected: the open rule answers the
  question every time from what was opened, and a remembered Edit would
  make `riggen robot.urdf` open into the editor for the researcher who
  last built something.
