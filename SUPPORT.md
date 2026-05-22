# Support Policy

This project separates community support, release/deployment support, security reports and hardware or live-environment evidence. Use the right channel so maintainers can triage without asking for the same context again.

## Where To Start

- General setup and usage: start with [README.md](README.md) and [docs/README.md](docs/README.md).
- Current support boundaries: read [Support Matrix](docs/guides/SUPPORT_MATRIX.md) and [Known Limitations](docs/guides/KNOWN_LIMITATIONS.md).
- Customer deployment: read [Customer Installation Guide](docs/guides/CUSTOMER_INSTALLATION_GUIDE.md), [Deployment Matrix](docs/guides/DEPLOYMENT_MATRIX.md) and [Production Deployment Runbook](PRODUCTION_DEPLOYMENT_RUNBOOK.md).
- Dynamic Godot robot generation: read [Dynamic Godot Robot Generation](docs/guides/DYNAMIC_GODOT_ROBOT_GENERATION.md).
- Security vulnerabilities: follow [Security Policy](SECURITY.md), not a public issue.

## Issue Triage

Use the repository issue templates:

- Bug report: reproducible defect, broken contract, failed local workflow or incorrect evidence.
- Feature request: planned capability, contract change or delivery improvement.
- Nightly regression: failing scheduled or manual CI smoke, release gate, security preflight or live-environment profile.

Include exact commands, artifact paths, branch/commit, environment details and residual risk. For live Godot, ROS2 or hardware issues, include whether the required external executable, runtime or device was available.

## Supported Baseline

The current supported delivery baseline is Docker Compose plus the Web Panel. Godot, ROS2 and distributed profiles are conditional extension surfaces and require explicit environment preparation and evidence.

Python `3.12` is recommended for local validation in this workspace, while CI may use its configured Python matrix. See [Support Matrix](docs/guides/SUPPORT_MATRIX.md) for the formal support boundary.

## Not Support Requests

Do not use public issues for:

- exploitable vulnerabilities, credentials or private customer data;
- requests that require deleting tracked sample/evidence directories without compatibility review;
- hardware safety claims without real-device evidence;
- unsupported deployment targets such as Kubernetes production delivery unless a separate acceptance plan is provided.
