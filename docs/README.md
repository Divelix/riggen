# Riggen docs

Numbered documents are the living design; ADRs record decisions and their
reasons at the moment they were taken and are never edited after acceptance
(supersede with a new one). `SEED.md` at the repo root is the charter: problem,
competition, differentiators, chosen stack.

| Doc | What it holds |
|---|---|
| [01-architecture](01-architecture.md) | Crate layout, layer rule, frame loop, threading, file format, testing |
| [02-data-model](02-data-model.md) | Core types, kinematics, inertials, `ResolvedRobot`, URDF/MJCF conventions |
| [03-roadmap](03-roadmap.md) | Milestones with acceptance tests |
| [adr/](adr/README.md) | Architecture decision records |

Conventions used in these docs: `⚠ OPEN:` marks a question deliberately left
for implementation time; a decision that closes it gets an ADR.

## Document lifecycle

Three tiers, three lifetimes. Which tier a sentence belongs to is decided by
how long it should stay true.

| Tier | Files | Lifetime | Rule |
|---|---|---|---|
| Charter + decisions | `SEED.md`, `adr/` | Append-only | `SEED.md` is frozen at kickoff. A change of mind is a new ADR that supersedes an old one; the old one is never edited. |
| Design | `01-…`, `02-…`, `03-…` | Living | Present tense; describes the system as it is *now*. The commit that changes behaviour updates the doc. No "as of M2" prose — git blame is the history. Milestone progress is one status line per milestone in `03-roadmap.md`, nothing more. |
| Ideas | `ideas/<slug>.md` | Until decided | A **brainstorm**, not a todo: problem, options with trade-offs, cost, conflicts, recommendation, the decision for the human. From `ideas/TEMPLATE.md`. Accepted → absorbed by its plan and deleted; rejected → one line under "Rejected" in `BACKLOG.md` with the reason, file deleted; parked → kept with `Status: Parked`. |
| Plans | `plans/<slug>.md` | Ephemeral | Created from `plans/TEMPLATE.md` when an idea is picked up; edited together; executed with checkboxes ticked and commits referencing it; on completion the durable parts move to tier 1/2 and **the plan is deleted**. Deletion is the "done" signal; git keeps it. At most two plans active. |

Raw ideas go in `BACKLOG.md`, one line each. A line that needs thinking
becomes an idea; one that is obvious goes straight to a plan; not every idea
becomes a plan. The pipeline is walked by the shared skills `/idea`, `/plan`,
`/work`, `/retire-plan` (`.agents/skills/`, symlinked into `.claude/skills/`),
under the rules in `.agents/rules/`. `visual-debug`, beside them, is how the
agent sees the GUI it is changing (ADR-0003).

`AGENTS.md`'s "Current state" is capped at ~15 lines and speaks at milestone
granularity only. RoboCAD's grew into a changelog because progress narrative
had nowhere else to go; here it goes into the roadmap's status lines and the
plan being executed.

**Drift review** at every milestone boundary: the agent reads each design doc
against the code and lists discrepancies; the milestone is not done until the
list is empty. This is the scheduled replacement for finding drift by accident.
