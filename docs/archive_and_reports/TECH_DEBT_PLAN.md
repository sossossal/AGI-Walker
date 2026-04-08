# Tech Debt Plan

This archived note records debt categories that were visible during repository cleanup.

## Debt Categories

- stale launch files
- outdated import paths
- mismatched package exports
- historical docs that drifted away from the code
- partial subsystem coverage outside the main CLI, Web, and MCP paths

## Still Relevant Priorities

1. Remove or repair legacy launch flows.
2. Align packaging metadata with the modules that actually exist.
3. Keep user-facing docs tied to tested workflows.
4. Expand regression coverage where drift is likely.

## Current High-Risk Areas

- ROS 2 workspace packaging and launch metadata
- historical experimental integrations
- any path that still points to older project layouts

## Archive Note

This file is a debt snapshot, not a live issue tracker.
