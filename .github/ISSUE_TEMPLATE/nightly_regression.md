---
name: Nightly regression
about: Track a failing nightly smoke or integration chain
title: "[NIGHTLY] "
labels: nightly-regression, ci
assignees: ''

---

**Failing job**
- [ ] smoke
- [ ] distributed-smoke
- [ ] godot-headless-smoke

**Run link**
Link to the failing GitHub Actions run:

**Failure first seen**
- Date:
- Commit / branch:
- Trigger:
  - [ ] nightly schedule
  - [ ] workflow_dispatch

**Impact**
- [ ] Blocks release readiness
- [ ] Blocks nightly signal only
- [ ] Unknown

**Observed failure**
Paste the key failing lines or summarize the symptom:

**Artifacts checked**
- [ ] smoke-artifacts
- [ ] distributed-smoke-artifacts
- [ ] godot-headless-smoke-artifacts

**Local reproduction**
Command used:

Result:

**Suspected owner**
- [ ] Core / Workflow
- [ ] Web / Godot
- [ ] Distributed Runtime
- [ ] Unknown

**Next action**
- [ ] Fix in current branch
- [ ] Re-run to confirm flake
- [ ] Escalate to release owner
- [ ] Needs environment investigation

**Additional context**
Anything else that would help the next engineer continue quickly.
