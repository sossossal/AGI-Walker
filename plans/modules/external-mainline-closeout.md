# External Mainline Closeout Module Plan

## Scope

Harden the external-mainline orchestration service so local artifacts, managed inputs, and report paths stay inside the declared project root.

## Goals

- Require managed `--inputs-file`, `--output`, checklist output, customer config/report, vulnerability review report, industrial rehearsal report, refresh-time industrial rehearsal output root, and customer overrides file to be project-root-relative paths.
- Reject absolute paths and parent-directory traversal before reading, writing, or passing local artifact paths to delegated tools.
- Preserve `--project-root` as the explicit trusted root and keep customer confirmation evidence values outside this local path contract because they may be external evidence locators.
- Keep managed-input bootstrap/refresh behavior and existing stdout fields compatible.

## Non-Goals

- Do not change external customer or industrial live evidence readiness semantics.
- Do not promote remaining external-input blockers to local code blockers.
- Do not rewrite the external-mainline artifact schema.

## Contracts

- External-mainline local file arguments are project-root-relative.
- `--project-root` may be absolute and defines the allowed filesystem boundary.
- Invalid local path inputs fail before file IO or delegated command execution with a `ValueError` surfaced by the CLI as an argparse error.

## Validation

- `py_compile` for the external-mainline service and runner CLI.
- Targeted pytest for `tests/test_external_mainline_ops.py` and `tests/test_external_mainline_execution_plan.py`.
- Next-stage readiness snapshot remains blocked only by external-input work, not code/config actions.

## Status

- 2026-07-12: Planned path-boundary hardening for external-mainline managed inputs and local artifacts.
