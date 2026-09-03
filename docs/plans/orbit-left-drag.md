# Plan: orbit-left-drag

- Started: 2026-09-03
- Milestone: v0.3 — the hand-feel debt
- Idea: `docs/ideas/orbit-left-drag.md` (absorbed)
- Idea (verbatim from the human): "/idea orbit-left-drag" — the v0.3 line
  "Orbit on left-drag with click-to-select still working (an idea first: the
  rule is the hard part)".

## Goal

The viewport answers the button the audience reaches for first: a
**left-drag orbits**, **shift+left** and a **right-drag pan**, and middle,
shift+middle and the wheel keep everything they have. Click-to-select
survives because egui already arbitrates it — a press that stays inside
`InputOptions::max_click_dist` and under `max_click_duration` is a click,
anything else is a drag — and a test pins those two constants rather than
trusting the defaults. The one new rule is a **fourth pointer switch**: while
a gizmo handle is under the cursor or its drag is in flight, the gizmo claims
the *primary* drag, so a left-drag from a handle moves the part and nothing
else; the middle and right drags are never claimed, and a joint or frame
glyph — which suppresses picks — never withholds the left drag. A trackpad
with no middle button can turn the sample arm on the web demo, and the
README stops being the only place that says how.

## Non-goals

- The other three items under the roadmap's "The viewport answers the
  mouse": tool keyboard shortcuts, the rotate gizmo on the wheel, and
  snapping during a gizmo drag. Each gets its own plan.
- "The overlay tells the truth" (depth-tested overlay, driven/actuated
  badges) — the other open v0.3 viewport bullet, planned separately.
- Box select and multi-select. Giving the bare left-drag to the camera
  forecloses them; ADR-0018 records that, and reopening it is that ADR's to
  overturn. `shift+left` is spent on pan, so a later box select would need
  another modifier.
- Touch gestures. A one-finger drag reaches egui as a primary drag, so touch
  orbit falls out of this plan for free, but pinch-zoom and two-finger pan
  stay backlog lines and nothing here is tested on touch.
- A preference to turn the mapping off (idea option D): riggen persists no
  preferences, and one boolean does not justify the surface.

## Design deltas

- **ADR-0018** (new), amending [ADR-0010](../adr/0010-gizmo-egui-glue-is-ours.md)
  §Decision 3: a fourth row in the pointer-switch table —
  `set_primary_drag_claimed`, which turns off *only* `dragged_by(Primary)`
  and leaves the middle and right drags, the wheel and both picks alone —
  and the record that the bare left-drag now belongs to the camera. It does
  not disturb §Decision 2: the gizmo's widget still senses clicks only, and
  `orbit_works_from_a_gizmo_handle` still passes as written, because that
  test drags with the middle button.
- **`riggen-viewport`** (`viewport/mod.rs`): `handle_input` gains
  `dragged_by(Primary)` → orbit, shift → pan, and `dragged_by(Secondary)` →
  pan; the middle branch is unchanged. New `primary_drag_claimed` field with
  `set_primary_drag_claimed`, read only by the primary branch.
  `pointer_policy()` returns a four-tuple. The layer rule holds: the viewport
  still decides from `Response` and a switch, and still knows nothing of
  gizmos.
- **`riggen-app`**: `app/mod.rs` sets the new switch from `gizmo_captured()`
  — the same `over_handle || drag in flight` the gizmo already computes —
  next to the three it sets today; the glyph hover deliberately does not feed
  it. `debug/mod.rs`: `InputDebug` gains `primary_drag_claimed`, and `is_off`
  counts it, so the JSON stays absent from goldens that suppress nothing.
- **`docs/01-architecture.md`** §Frame loop (the camera paragraph's binding
  list), §Picking and snapping (the switch table and the gizmo paragraph's
  "orbit and pan start from a handle" sentence, which is now true of the
  middle and right buttons only), §Testing (the generalised drag helper).
- **`README.md`** §first run: the bindings line.

## Steps

- [x] Step 1 — **ADR-0018**: the fourth switch and the bare left-drag.
  Write `docs/adr/0018-*.md` — context (the trackpad and the MuJoCo muscle
  memory), the decision (the mapping, and the switch that claims only the
  primary drag), consequences (box select foreclosed, touch orbit free,
  `orbit_works_from_a_gizmo_handle` unchanged), alternatives (the idea's B,
  C, D). No code changes; nothing behaves differently yet.
- [x] Step 2 — **The viewport takes the left and right drags.**
  `handle_input` orbits on primary, pans on shift+primary and on secondary;
  `primary_drag_claimed` and its setter exist and gate the primary branch
  only (nothing sets it yet). Generalise the harness's `middle_drag` into
  `camera_drag(harness, from, to, button, modifiers)` and keep `middle_drag`
  as its caller. Tests: over empty viewport, a left-drag orbits without
  moving the target, shift+left pans without orbiting, a right-drag pans, and
  the middle pair still does what it did. Snapshot scenario `orbit_left_drag`
  — the sample arm after a left-drag — and the docs edits to 01 §Frame loop,
  01 §Testing and the README ride in this commit.
- [x] Step 3 — **The gizmo claims the primary drag.**
  `app/mod.rs` sets `set_primary_drag_claimed(self.gizmo_captured())`;
  `InputDebug` reports it. Tests: from a gizmo handle, a left-drag moves the
  part and the camera does not turn, while the existing
  `orbit_works_from_a_gizmo_handle` middle/shift-middle assertions stay
  green; from a *joint glyph* — picks suppressed, drag not claimed — a
  left-drag orbits; a right-drag from a handle pans. The 01
  §Picking and snapping table and gizmo paragraph are updated here.
- [ ] Step 4 — **Click-to-select survives, and the thresholds are pinned.**
  A test that reads `max_click_dist` and `max_click_duration` off the context
  rather than hard-coding 6.0 / 0.8, asserts the values, then drives a press
  that moves *under* the threshold (selects, camera unmoved) and one that
  moves well past it (orbits, selection unchanged). Extend it over a
  placement tool, where `select_suppressed` is on: the click still places and
  the drag still orbits.

## Acceptance

`cargo test -p riggen-app` green, including the new
`left_drag_orbits_and_a_click_still_selects`,
`left_drag_from_a_gizmo_handle_moves_the_part` and
`orbit_works_from_a_gizmo_handle`; the `orbit_left_drag` snapshot matches;
`cargo fmt --check` and `cargo clippy --all-targets -- -D warnings` pass.
By hand once, on the built web demo: a one-button trackpad turns the sample
arm and can still select a link.

## Docs to update on completion

- `docs/01-architecture.md` §Frame loop, §Picking and snapping, §Testing —
  updated in steps 2 and 3, verified as a drift check at retirement.
- `README.md` — the bindings line, updated in step 2.
- `docs/03-roadmap.md` §v0.3 — the "viewport answers the mouse" bullet loses
  its first clause; the status paragraph records what landed.
- `AGENTS.md` current state — the orbit rule is no longer open; the
  remaining v0.3 work is the three other mouse items and the overlay.
- `docs/ideas/orbit-left-drag.md` — deleted with this plan's first commit
  (absorbed above; git keeps it).

## Open questions

- **Does shift+left pan, or is shift+left reserved?** *Decided 2026-09-03
  (human, step 1): shift+left pans.* It is the only pan a one-button
  trackpad has, and shift already means pan on the middle button. ADR-0018
  §Consequences records that box select now needs a third modifier or must
  overturn the ADR.
- `⚠ OPEN:` **Right-drag pan on the web.** eframe's web backend already
  `preventDefault`s `contextmenu` on the canvas, so the browser menu will not
  fire — but that is read from the source, not observed. *Agent, by step 4*:
  confirmed on the built demo as part of the by-hand acceptance, and if it
  does fire, right-drag pan is dropped and shift+left carries the pan alone.
