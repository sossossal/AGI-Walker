# Blockers Resolved

This archived note records categories of blockers that were cleared during earlier cleanup work.

## Historically Resolved Blockers

- Corrupted UTF-8 documentation that made key entry pages unreadable
- MCP server startup drift caused by package API changes
- Import-time warnings caused by eager module loading
- Confusion around the actual supported runtime entry points

## What This Means Today

These blocker categories are useful because they explain why several core documents, tests, and MCP modules were rewritten.

## What May Still Need Work

Some historical subsystems can still carry drift even after these blockers were cleared. The main remaining examples are legacy launch flows, archive notes, and parts of the ROS 2 workspace.

## Current Reference

Use current tests and current guides to determine present status. This page only explains why certain cleanup work happened.
