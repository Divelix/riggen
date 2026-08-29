# ADR-0006: A dropped mesh is a link; removal takes the subtree; import scale is an app setting

- Status: Accepted
- Date: 2026-08-29

## Context

Three questions came up while building M1's tree and file handling, each
with a plausible alternative, and each shaping how a user's first minute
with the tool feels:

1. What does dropping an STL onto the window produce?
2. What does removing a link in the middle of the tree do to its children?
3. Where does the mm → m scale of an STL come from when there is no
   per-file import dialog?

## Decision

1. **A dropped mesh becomes a new link** named after the file stem (made
   XML-valid, deduplicated: `arm`, `arm_2`) with a `Fixed` joint at
   identity, under the selected link (a selected joint counts as its
   child) or the root. The selection does not move, so a multi-file drop
   lands the files side by side. "Add mesh to this link…" in the
   properties panel is the route for a second geom on one link.
2. **`RemoveLink` removes the whole subtree** — the link, its parent joint,
   every descendant and any frame on them — as one command and one undo
   step. The root is refused.
3. **The import scale is one app-wide setting** (`File › Import units`:
   mm, cm, m, in; default mm), remembered through eframe storage and
   copied onto each dropped mesh's `MeshAsset::scale`, which the properties
   panel edits per asset afterwards.

## Consequences

- The document never holds a loose part: everything drawn is a link with a
  place in the tree (ADR-0005 made that an invariant; this makes it the
  default gesture).
- Removing a link never re-hangs children onto its parent; a user who
  wants that reparents the children first, which `Reparent` with
  `keep_world_pose` makes cheap.
- A batch of mm files imports right with one setting and no dialog; a
  mixed batch needs the per-asset scale afterwards. A per-drop dialog is
  in the backlog if that turns out to be common.

## Alternatives considered

- **Dropping with a link selected adds a geom to it** — makes the common
  case (a new part) need a deselect first, and the rarer case has a button.
- **Splicing children onto the removed link's parent** — matches no tree
  UI the users know, and produces poses nobody asked for.
- **A per-drop import dialog** (02-data-model's original text) — a modal on
  every drop; deferred until a mixed-units workflow shows up.
