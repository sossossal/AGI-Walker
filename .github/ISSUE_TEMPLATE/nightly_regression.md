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
- [ ] ros2-bridge-smoke

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

**Failure classification**
- [ ] Test assertion failure
- [ ] Dependency or vulnerability finding
- [ ] Environment / missing executable
- [ ] Timeout or flaky startup
- [ ] Artifact contract mismatch
- [ ] Unknown

**Artifacts checked**
- [ ] smoke-artifacts
- [ ] distributed-smoke-artifacts
- [ ] godot-headless-smoke-artifacts
- [ ] ros2-bridge-smoke-artifacts
- [ ] security-release-preflight report
- [ ] dynamic Godot evidence bundle

**Local reproduction**
Command used:

Result:

**Contract or release impact**
- [ ] Public schema/API/CLI contract affected
- [ ] Release evidence or readiness gate affected
- [ ] Security preflight affected
- [ ] Hardware/no-hardware acceptance affected
- [ ] No known contract impact

**Suspected owner**
- [ ] Core / Workflow
- [ ] Web / Godot
- [ ] Distributed Runtime
- [ ] Release / Security
- [ ] Hardware / ROS2
- [ ] Unknown

**Next action**
- [ ] Fix in current branch
- [ ] Re-run to confirm flake
- [ ] Escalate to release owner
- [ ] Needs environment investigation

**Residual risk**
What remains blocked if this is not fixed before the next release?

**Additional context**
Anything else that would help the next engineer continue quickly.
