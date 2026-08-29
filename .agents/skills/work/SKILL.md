---
name: work
description: Execute the next unchecked step(s) of an active plan in docs/plans/ — implement, test, run the pre-commit checks, commit with the plan-step suffix, tick the box. Use when the human says "work on <plan>", "next step", "continue the plan", or "do steps 3-5". Stops at the step boundary or when a step's open question blocks it.
argument-hint: <plan slug> [step number or range]
---

# /work — one step, one commit

## Do

1. Open `docs/plans/<slug>.md`. The target is the first unchecked step, or
   the range given. Re-read the step's *Design deltas* and any `⚠ OPEN:` that
   names it; if the open question is the human's and unanswered, stop and
   ask — do not guess around it.
2. Implement the step and its test or snapshot. Docs that the step changes
   are edited in the same step (`.agents/rules/git.md`).
3. Run the checks the hook will run: `cargo fmt --all`, `cargo clippy
   --workspace --all-targets -- -D warnings`, `cargo test --workspace`. For
   a visible UI change, capture a snapshot and look at the image.
4. Commit: `type(scope): summary (plans/<slug> step N)`; body says why and
   cites docs/ADRs. Tick the box in the plan and include the plan file in
   the same commit.
5. If the step revealed that the plan is wrong, edit the plan (add/split
   steps, record the finding under *Open questions*) in that commit and say
   so in the reply — a plan is a living todo, not a contract.
6. Reply: what landed, what the commit is, what the next step is, anything
   surprising. Then stop unless a range was requested.

## Don't

- Don't skip ahead or fold two steps into one commit "because they're small".
- Don't tick a box whose test didn't run.
- Don't bypass the hook. If checks fail, fix them or stop and report.
- Don't retire the plan when the last box is ticked — that is `/retire-plan`,
  which the human triggers after seeing the acceptance run.

`$ARGUMENTS`
