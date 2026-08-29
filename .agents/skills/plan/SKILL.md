---
name: plan
description: Write an executable plan under docs/plans/ from the template — commit-sized checkbox steps, acceptance test, design deltas, docs-to-update list — for a feature, refactor, or milestone. Use when the human says "plan", "let's do", "implement", names a milestone ("plan M0"), or accepts an idea's recommendation. Reads the idea file if one exists and absorbs it. Never starts executing.
argument-hint: <slug | milestone | accepted idea slug>
---

# /plan — from decision to todo

A plan is a todo the agent can execute step by step with a commit per step,
and the human can read in two minutes.

## Do

1. **Check the limit.** `ls docs/plans/` — two active plans (excluding
   `TEMPLATE.md`) means stop and ask which to retire first.
2. **Gather.** Read `docs/03-roadmap.md` for the milestone's in/out/accept
   lists, the design docs the work touches, the relevant ADRs, and
   `docs/ideas/<slug>.md` if the plan comes from an idea. If it does, the
   plan's *Goal* and *Design deltas* absorb the idea's decision and
   recommendation, and the idea file is deleted in the same change (git keeps
   it; the plan header cites it as `Idea: docs/ideas/<slug>.md (absorbed)`).
3. **Write `docs/plans/<slug>.md`** from `docs/plans/TEMPLATE.md`:
   - *Goal* is one paragraph of what is true when done; *Non-goals* fence the
     scope.
   - *Design deltas* name each design doc section and type that changes. A
     non-obvious decision becomes an ADR — list it as a step.
   - *Steps* are commit-sized: each has an observable result and its own test
     or snapshot, and could be reverted alone. Order them so the riskiest
     unknown retires first. Prefix nothing; numbering is the checkbox order.
   - *Acceptance* is an executable check, ideally the milestone's own.
   - *Docs to update* is written now, while the deltas are fresh — it is what
     `/retire-plan` executes.
   - *Open questions* carry `⚠ OPEN:` items, each with who decides and by
     which step.
4. Reply with the step list and the open questions, then **stop**. The
   human edits the plan before `/work` starts it.

## Don't

- Don't execute a step. Don't create branches. Don't write code.
- Don't re-argue the idea's decision in the plan; link the ADR instead.
- Don't plan past the milestone's "out" list; put the overflow in
  `docs/BACKLOG.md`.

`$ARGUMENTS`
