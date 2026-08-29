# ADR-0005: Ids are per-document counters; joints are the edges of the link tree

- Status: Accepted
- Date: 2026-08-29

## Context

02-data-model asked for two things at once: ids that are `slotmap` keys and
ids that serialise as readable `"l3"` / `"j7"` strings so a `.riggen` diff
can be read. A `slotmap` key carries a generation counter next to its index;
writing only the index loses it, so a load would need a remap pass that
rebuilds every key and rewrites every reference. The document is a few
hundred entries at most, so `slotmap`'s dense O(1) storage buys nothing here.

Separately, the command list had `AddLink(Link)` and `AddJoint(Joint)` as
independent commands. Between the two a link exists with no parent joint,
so the tree invariant ("every non-root link has exactly one parent joint")
could only be checked on save and export, and every consumer of the document
— FK, the tree panel, the viewport sync — had to tolerate orphans.

## Decision

1. **Ids are `u32` newtypes** (`LinkId`, `JointId`, `GeomId`, `MeshId`,
   `FrameId`) handed out by one per-document counter (`IdGen`, serialised as
   `next_id`), stored in `BTreeMap<Id, T>`, serialised as `"l3"` strings and
   never reused within a document's life. Save/load is lossless with no
   remap; a `BTreeMap` keeps the file order deterministic.
2. **Joints are tree edges.** A link is created together with its parent
   joint (`AddLink { link, parent, joint }`), removed with its subtree
   (`RemoveLink`), and moved by `Reparent`, which rewrites the parent joint.
   There is no `AddJoint` / `RemoveJoint`: "connect two links" *is*
   `Reparent`. The tree invariant therefore holds after every command, not
   only at save time, and `validate` on a command's result is a safety net
   rather than the only guard.

## Consequences

- `Robot` fields are `BTreeMap`s; iteration is in id order, which is also
  creation order — good enough for the tree panel until an explicit order
  is wanted.
- The root link is the only link without a parent joint; `RemoveLink(root)`
  is refused, and `SetRoot` (M3) has to re-hang the old root as a child.
- A dropped mesh always lands as a link under a parent (the selection or
  the root) with a `Fixed` joint at identity; "just a loose part" does not
  exist in the document.
- 02-data-model's `SlotMap` fields and `AddJoint` / `RemoveJoint` lines are
  superseded by this ADR and rewritten when plan `m1-document-tree-joints`
  retires.

## Alternatives considered

- **`slotmap` with a remap on load** — correct but a second id space to
  keep in sync, for a performance property the document never needs.
- **Orphan links allowed between commands, validated on save/export** —
  every reader of the document has to handle a forest, and the tree panel
  would need an "unattached" section that exists only to be a bug.
- **Joint-less links for the root only** — that is what this is: the root
  is the single exception and is special-cased once.
