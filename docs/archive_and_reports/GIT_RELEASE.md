# Git Release

This archived note describes a simple Git-oriented release flow for repository maintenance.

## Recommended Sequence

1. Review working tree changes carefully.
2. Run the relevant tests for the modified area.
3. Commit with a message that describes the functional outcome.
4. Push the branch.
5. Create a tag only after validation is complete.
6. Publish release notes that match the actual shipped state.

## Scope Reminder

For this repository, release messaging should focus on:

- active entry points
- behavior changes
- migration notes
- known limitations

Avoid presenting archive-only material as live product surface.

## Archive Note

This file intentionally stays generic because exact Git hosting and release automation can change over time.
