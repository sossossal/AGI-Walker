# Security Policy

AGI-Walker includes Web APIs, workflow execution, Godot integration, deployment scripts and release evidence tooling. Please report suspected security issues privately.

## Supported Versions

Security review focuses on the current default branch and active release branches. Historical snapshots under `archive/` are kept for audit context and are not maintained as supported release lines.

## Reporting a Vulnerability

Do not open a public GitHub issue for exploitable vulnerabilities, secrets, credential exposure, privilege escalation, data loss, remote code execution or deployment misconfiguration.

Use GitHub private vulnerability reporting when it is available for this repository. If private reporting is not available, contact the maintainers through a private channel and include:

- affected commit, branch or release;
- affected component, such as Web Panel, deployment, MCP, Godot smoke tooling or release evidence tooling;
- reproduction steps or proof of concept;
- expected impact and any known workarounds;
- whether credentials, personal data, customer data or robot hardware safety could be affected.

## Handling Expectations

- Maintainers should acknowledge valid reports before public disclosure.
- Sensitive details should stay private until a fix or mitigation is available.
- Fixes should include validation evidence and, when relevant, release evidence or deployment guidance.
- Security exceptions must remain time-bound and tracked through the repository security preflight workflow.

## Security Validation

Relevant local checks:

```powershell
py -3.12 tools\run_security_release_preflight.py --output-root test_env\release_evidence --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed
```

Additional security guidance:

- [Security Baseline](docs/guides/SECURITY_BASELINE.md)
- [Incident Response Matrix](docs/guides/INCIDENT_RESPONSE_MATRIX.md)
- [Audit Trail Policy](docs/guides/AUDIT_TRAIL_POLICY.md)
