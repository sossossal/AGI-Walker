# Project Archive

This file summarizes what the historical archive in this folder is for.

## Archive Scope

The repository went through several overlapping cleanup and migration phases:

- Documentation normalization
- Entry point consolidation
- MCP server compatibility fixes
- CLI and Web workflow clarification
- Historical plan and release material preservation

Most archive files in this directory were originally written during those transitions. Their primary value now is historical context rather than active instruction.

## Current Supported Runtime Paths

The current top-level entry points are:

- CLI: `python -m agi_walker.cli`
- Web panel: `python -m web_panel.server`
- MCP server: `agi-walker-mcp`

## Canonical Current Documentation

Use these documents before reading anything else in the archive:

- `README.md`
- `docs/CURRENT_STATUS.md`
- `docs/API_REFERENCE.md`
- `docs/guides/CLI_GUIDE.md`
- `docs/guides/WEB_PANEL_GUIDE.md`
- `docs/mcp.md`

## What Remains Useful In The Archive

- Completion reports can still be used to understand why certain cleanup tasks happened.
- Release materials can still be used as templates for future announcements.
- Planning notes are useful when prioritizing debt that was identified earlier but not fully removed.
- OpenNeuro and ROS 2 related archive notes are helpful as migration context, not as guaranteed current workflows.

## Archive Rule

If an archive file references an old path, old package name, or a no-longer-supported launch flow, prefer the current codebase and the current guides.
