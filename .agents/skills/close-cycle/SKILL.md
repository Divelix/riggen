---
name: close-cycle
description: Close a finished release cycle or milestone — verify nothing is still open, run the mandated drift review of every design doc against the code, compress the finished section of docs/ROADMAP.md to its status line, open the next cycle's section, update the spine and AGENTS.md, and hand the tag to the human. Use when the human says "close the cycle", "v0.2 is done", "milestone is done", "what's next after this release", or when the last line of a roadmap cycle has been retired. Never tags and never pushes.
argument-hint: <cycle or milestone, e.g. v0.2 — and optionally the next cycle's theme>
---

# /close-cycle — one roadmap, one section per cycle

`docs/ROADMAP.md` is a **living design doc**, not a log. It is never
forked into a second roadmap file and never accumulates plans: a finished
cycle shrinks to its status line, and the next cycle is appended below it.
The design docs are *topics* — Architecture, Data Model, Roadmap — so
a second roadmap file would only ever raise "which one is current?".

## Do

1. **Check it is actually finished.** Every line of the cycle's section
   either done or explicitly moved to `docs/BACKLOG.md`; `docs/plans/`
   holding nothing but `TEMPLATE.md`; `cargo fmt --check`, `cargo clippy
   --workspace --all-targets -- -D warnings` and `cargo test --workspace`
   green. If not, stop and report exactly what is open — do not close
   around it.
2. **Drift review** (`docs/README.md` mandates it at every boundary, and
   this is the only place it happens): read **every** design doc —
   `01-…`, `02-…`, `03-…` — against the code and fix each sentence that is
   no longer true. The cycle is not closed until the list is empty. List
   what you fixed in the reply; if you fixed nothing, say why you believe
   nothing had drifted.
3. **Compress the finished section.** Exactly five parts, in this order,
   and nothing else:

   ```
   ## vN — <theme>
   *Goal: …*                        ≤ 3 lines
   **Status: done <date>, tag `vN.N.0`.** <the risk retired, the ADRs
   taken>                           ≤ 5 lines, one paragraph
   - <in bullets, as written when the section opened>
   **Out:** …
   **Accept:** …
   ```

   `/plan` reads in/out/accept and `/retire-plan` writes the status line,
   so those four survive; **the narrative between them goes.** A closed
   section over **~40 lines** is not compressed — go again; the closed
   ones run 22 to 43, and the 43 is v0.4, the only cycle with two
   independent halves. What the status line owes the next cycle is the
   *risk* and the *ADR numbers*, not the story of how each was retired:
   the ADR tells that, at length, and is append-only.

   The in-list is protected as *what the cycle committed to*, not as a
   description of the code that landed: a bullet that grew type names and
   outcomes while the cycle ran shrinks back to its one or two lines, since
   the detail now lives in `ARCHITECTURE.md` / `DATA-MODEL.md`.

   Before deleting any sentence, grep the fact in `docs/`, `crates/`,
   `README.md` and the workflows. One that lives *only* here is
   **relocated, never dropped** — to the topic doc that owns it:

   | Fact | Home |
   |---|---|
   | wheel / binary / extension sizes, publishing behaviour | `ARCHITECTURE.md` §Python distribution |
   | wasm bundle size, the `web` profile's worth | `ARCHITECTURE.md` §The web build |
   | a measured timing beside the test that guards it | `ARCHITECTURE.md` §Testing |
   | a convention, a type, a format rule | `DATA-MODEL.md` |
   | why a decision went the way it did | the ADR — a new one if none says it |

   `docs/notes/` is **never** a destination: it is gitignored personal
   notes (`.gitignore`), not a design doc. A fact whose only home would
   be a retrospective — a guess this section made that the work overturned
   — is dropped, because git holds it and the ADR holds the fact it was
   wrong about.
4. **Open the next section.** `## vN — <theme>` with goal, in, out and
   accept, drawn from `docs/BACKLOG.md`. The theme and the in/out split are
   the **human's call**: propose, do not decide. If the answer is not
   obvious from the backlog, stop and ask.
5. **`Spine:`** at the top of the roadmap gains the new cycle.
6. **`AGENTS.md` "Current state"**: one sentence for the closed cycle, a
   new `**Next:**`, block still under ~15 lines.
7. **Write the release notes into the reply** — the one user-facing
   artefact of a cycle, and the reason there is no `CHANGELOG.md`. Three
   to five bullets **derived from the status line you just wrote**, in the
   reply itself and in no file:

   - What a user can now *do* that they could not before. Not what was
     built — `Robot::actuators` is not a change, "an imported MJCF keeps
     its actuators, and exporting gives them back" is.
   - No ADR numbers, no plan slugs, no crate or type names, no schema
     versions. Someone who has read nothing but `README.md` is the reader.
   - Refusals a user will hit belong here too ("composite joints and
     `<attach>` are still refused on import"), because that is what an
     issue gets filed about.

   The human pastes them into the GitHub Release body when they tag;
   `release.yml`'s `generate_release_notes: true` puts the commit list
   underneath. Nothing to maintain between cycles, and nothing to go stale.
8. Commit as `docs: close <cycle>` with a body listing the docs updated and
   the drift fixed. Then **tell the human to tag** `vN.N.0`, with the
   release-note bullets beside the instruction — tags and pushes are
   theirs (`.agents/rules/git.md`), never yours.

## Don't

- Don't create `04-roadmap.md`, an `ARCHIVE.md`, or a `CHANGELOG.md`. What
  landed already lives in four places — the git log (a commit per plan
  step), the status line, the ADRs, and the GitHub Release — and a
  hand-maintained fifth is the one with no owner, so it is the one that
  goes stale. The user-facing changelog is the Release body, **derived**
  at step 7 and written once per cycle, never maintained per commit.
- Don't leave the finished section at full length "because it is useful" —
  a cycle that keeps 40 lines is what makes the file look unmaintainable
  after three of them.
- Don't invent the next cycle's scope. A roadmap section is a commitment
  the human makes, not one the agent proposes into existence.
- Don't tag, push, or open plans for the new cycle here. `/idea` and
  `/plan` come after, one line at a time.

`$ARGUMENTS`
