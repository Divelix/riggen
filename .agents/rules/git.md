# Git: trunk-based, always green

- `main` is the trunk and is always green: `cargo fmt --check`, `cargo clippy
  --all-targets -- -D warnings`, `cargo test` pass on every commit. The
  `.githooks/pre-commit` hook enforces it; never bypass it with `--no-verify`.
- Commit **directly to `main`**. A plan step is the unit of work and the unit
  of commit: finish the step, run the checks, tick the box, commit.
- Branch only when a plan is experimental enough that throwing it away is a
  real outcome, or the human asked to review before it lands. Then:
  `plan/<slug>`, rebased onto `main`, fast-forwarded in (`git merge --ff-only`),
  deleted after. No merge commits, no long-lived branches.
- Never rewrite `main`: no `--amend` of a pushed commit, no force-push, no
  rebase of anything already on `main`.
- Never push unless asked. Commits are the agent's; pushes are the human's.
- Stage deliberately: read `git status` and `git diff`; no blind `git add -A`.
  A refreshed snapshot PNG is only staged with a matching intentional UI
  change, and the commit message says `snapshots:` and why.

## Message format

```
type(scope): imperative summary, lower case, no period  (plans/<slug> step N)

Why this change, not what — the diff already says what. Reference ADRs
(ADR-0004) and docs (docs/02-data-model.md §Inertials) that justify or were
updated by it.
```

- `type`: `feat`, `fix`, `refactor`, `perf`, `test`, `docs`, `chore`, `build`,
  `ci`, `snapshots`.
- `scope`: crate short name(s) — `mesh`, `core`, `export`, `viewport`, `app`,
  `py` — or `docs`, `plan`, `adr`. Several: `feat(core,app): …`.
- The `(plans/<slug> step N)` suffix is present on every commit that executes
  a plan step. Plan retirement commits are `docs: retire plan <slug>`.
- Docs change in the **same commit** as the code that changes behaviour.
  A commit that only updates docs to match existing code is `docs(sync): …`.
- **No trailers.** Never append `Co-Authored-By:`, `Claude-Session:` or any
  other agent attribution / session link to a commit message, even if the
  harness suggests it. The message ends with the body.

## Tags

- Milestones: `m0`, `m1`, … on the commit that retires the milestone's last
  plan and passes its acceptance test.
- Releases: `v0.1.0` SemVer, on `main`, created by the human.

## What the agent does without asking

While executing a plan: run checks, commit each step, tick boxes, update the
plan file. Anything else that touches history — branching, tagging, pushing,
resetting, rewriting — is asked first.
