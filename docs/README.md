# Riggen docs

The design docs are the living design; ADRs record decisions and their
reasons at the moment they were taken and are never edited after acceptance
(supersede with a new one). `SEED.md` at the repo root is the charter: problem,
competition, differentiators, chosen stack.

| Doc | What it holds |
|---|---|
| [Architecture](ARCHITECTURE.md) | Crate layout, layer rule, frame loop, threading, file format, testing |
| [Data Model](DATA-MODEL.md) | Core types, kinematics, inertials, `ResolvedRobot`, URDF/MJCF/SDF conventions |
| [Roadmap](ROADMAP.md) | Milestones with acceptance tests |
| [adr/](adr/README.md) | Architecture decision records |

Conventions used in these docs: `⚠ OPEN:` marks a question deliberately left
for implementation time; a decision that closes it gets an ADR.

## Document lifecycle

Three tiers, three lifetimes. Which tier a sentence belongs to is decided by
how long it should stay true.

| Tier | Files | Lifetime | Rule |
|---|---|---|---|
| Charter + decisions | `SEED.md`, `adr/` | Append-only | `SEED.md` is frozen at kickoff. A change of mind is a new ADR that supersedes an old one; the old one is never edited. |
| Design | `ARCHITECTURE.md`, `DATA-MODEL.md`, `ROADMAP.md` | Living | Present tense; describes the system as it is *now*. The commit that changes behaviour updates the doc. No "as of M2" prose — git blame is the history. Milestone progress is one status line per milestone in `ROADMAP.md`, nothing more; a closed section is under ~40 lines, enforced by `/close-cycle`. |
| Ideas | `ideas/<slug>.md` | Until decided | A **brainstorm**, not a todo: problem, options with trade-offs, cost, conflicts, recommendation, the decision for the human. From `ideas/TEMPLATE.md`. Accepted → absorbed by its plan and deleted; rejected → one line under "Rejected" in `BACKLOG.md` with the reason, file deleted; parked → kept with `Status: Parked`. |
| Plans | `plans/<slug>.md` | Ephemeral | Created from `plans/TEMPLATE.md` when an idea is picked up; edited together; executed with checkboxes ticked and commits referencing it; on completion the durable parts move to tier 1/2 and **the plan is deleted**. Deletion is the "done" signal; git keeps it. At most two plans active. |

Raw ideas go in `BACKLOG.md`, one line each. A line that needs thinking
becomes an idea; one that is obvious goes straight to a plan; not every idea
becomes a plan. The pipeline is walked by the shared skills `/idea`, `/plan`,
`/work`, `/retire-plan` and, at a cycle boundary, `/close-cycle`
(`.agents/skills/`, symlinked into `.claude/skills/`),
under the rules in `.agents/rules/`. `visual-debug`, beside them, is how the
agent sees the GUI it is changing (ADR-0003).

`AGENTS.md`'s "Current state" is capped at ~15 lines and speaks at milestone
granularity only. RoboCAD's grew into a changelog because progress narrative
had nowhere else to go; here it goes into the roadmap's status lines and the
plan being executed.

**Drift review** at every milestone boundary: the agent reads each design doc
against the code and lists discrepancies; the milestone is not done until the
list is empty. This is the scheduled replacement for finding drift by accident,
and it is step 2 of `/close-cycle`.

A finished cycle **compresses in place**: `ROADMAP.md` keeps one section per
cycle — goal, one status line, in/out/accept, **under ~40 lines** — and the
next cycle is appended below it. There is never a second roadmap file; the
design docs are topics, not versions. The narrative a cycle accumulates while
it runs goes at that boundary: a measurement or a rationale that lives only in
the roadmap is relocated first, to the topic doc that owns it
(`ARCHITECTURE.md` §Python distribution, §The web build, §Testing;
`DATA-MODEL.md`) or to an ADR. `notes/` is gitignored personal notes and is
never a destination.

There is **no `CHANGELOG.md`**. What landed lives in the git log (a commit per
plan step), the status line, and the ADRs — all written for the next agent —
and the one user-facing telling is the **GitHub Release body**, three to five
plain bullets `/close-cycle` derives from the status line and hands to the
human with the tag. Derived once per cycle, so there is nothing to maintain
per commit and nothing to go stale.
