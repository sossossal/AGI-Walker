# Release Checklist

This archived checklist captures a reasonable manual release flow for the repository.

Active guide: `docs/guides/RELEASE_GUIDE.md`

## Pre-Release Checks

- Verify the main entry docs are readable.
- Run targeted tests for the area you changed.
- Confirm the documented runtime entry points still match the code.
- Review open known-drift items such as legacy launch scripts.

## Suggested Validation

```bash
python -m pytest tests/test_docs_utf8.py -q
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
```

## Release Material

- Update the top-level `README.md` if user-facing workflows changed.
- Refresh `docs/CURRENT_STATUS.md` if support level changed.
- Prepare short release notes that describe behavior changes, not just file churn.

## Archive Note

This checklist is a reusable template, not an automated release pipeline.
