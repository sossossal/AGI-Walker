# C++ Plugin Build

This archived note preserves the idea of a native plugin or GDExtension build path around the Godot side of the project.

## Historical Goal

The aim was to support lower-level performance-sensitive functionality with a C++ build layer while still keeping higher-level workflows accessible from Python and Godot scripts.

## What To Assume Today

- Native build flows should be treated as specialized and secondary.
- The active repository surface is still documented through the main README, CLI guide, Web guide, and MCP guide.
- Any native build path should be revalidated against the current Godot integration layout before use.

## Current Related Documents

- `docs/guides/COMPILE_OPTIMIZED.md`
- `docs/guides/GODOT_INTEGRATION_GUIDE.md`
- `docs/guides/GODOT_TESTING_GUIDE.md`

## Archive Note

Do not assume this build path is production-ready without fresh validation.
