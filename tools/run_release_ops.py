#!/usr/bin/env python
"""Unified control-plane entry for deterministic release-ops actions."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


def _configure_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


_configure_stdio()
PROJECT_ROOT = _find_repo_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.release_ops_contracts import (  # noqa: E402
    ReleaseOpRequest,
    ReleaseOpSessionContext,
)
from agi_walker.ops.release_ops import (  # noqa: E402
    execute_release_op,
    list_release_ops_actions,
    list_release_ops_policy_profiles,
)


def _load_request_payload(path: str | None) -> dict[str, object]:
    if not path:
        return {}
    request_path = Path(path)
    try:
        payload = json.loads(request_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ValueError(f"--request-file is unreadable: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("--request-file must contain a JSON object")
    return payload


def _write_json(path: str, payload: object) -> Path:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a unified deterministic release-ops action."
    )
    parser.add_argument("action", nargs="?", default=None)
    parser.add_argument(
        "--request-file",
        default=None,
        help="Optional JSON object passed to the action-specific request contract.",
    )
    parser.add_argument(
        "--result-file",
        default=None,
        help="Optional JSON output for the normalized release-op result or action list.",
    )
    parser.add_argument(
        "--list-actions",
        action="store_true",
        help="List supported release-ops actions and policy levels.",
    )
    parser.add_argument(
        "--list-policy-profiles",
        action="store_true",
        help="List supported release-ops policy profiles.",
    )
    parser.add_argument(
        "--policy-profile",
        default=None,
        help=(
            "Optional control-plane policy profile. Defaults to local_safe_refresh "
            "when omitted."
        ),
    )
    parser.add_argument("--engagement-id", default=None)
    parser.add_argument("--window-id", default=None)
    parser.add_argument("--change-ticket", default=None)
    parser.add_argument("--channel", default=None)
    parser.add_argument(
        "--event-stream-file",
        default=None,
        help="Optional JSONL event stream written by the control-plane dispatcher.",
    )
    parser.add_argument(
        "--evidence-report-file",
        default=None,
        help="Optional canonical release_evidence_report wrapper for the action result.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.list_actions:
        actions = list_release_ops_actions()
        if args.result_file:
            written = _write_json(args.result_file, actions)
            print(f"release_ops_actions_written={written}")
        print(f"release_ops_actions={len(actions)}")
        return 0
    if args.list_policy_profiles:
        profiles = list_release_ops_policy_profiles()
        if args.result_file:
            written = _write_json(args.result_file, profiles)
            print(f"release_ops_policy_profiles_written={written}")
        print(f"release_ops_policy_profiles={len(profiles)}")
        return 0
    if not args.action:
        parser.error("the following arguments are required: action")
    try:
        request_payload = _load_request_payload(args.request_file)
        request = ReleaseOpRequest(
            action=args.action,
            request=request_payload,
            session=ReleaseOpSessionContext(
                engagement_id=args.engagement_id,
                window_id=args.window_id,
                change_ticket=args.change_ticket,
                channel=args.channel,
            ),
            event_stream_file=args.event_stream_file,
            evidence_report_file=args.evidence_report_file,
        )
        if args.policy_profile:
            request.policy_profile = args.policy_profile
        result = execute_release_op(request)
    except ValueError as exc:
        parser.error(str(exc))

    normalized_result = {
        "action": result.action,
        "policy_level": result.policy_level,
        "policy_profile": result.policy_profile,
        "request_type": result.request_type,
        "status": result.status,
        "summary": result.summary,
        "output_path": str(result.output_path) if result.output_path else None,
        "evidence_report_path": (
            str(result.evidence_report_path) if result.evidence_report_path else None
        ),
        "evidence_report_payload": result.evidence_report_payload,
        "session": result.session,
        "event_stream_path": (
            str(result.event_stream_path) if result.event_stream_path else None
        ),
        "event_count": result.event_count,
        "payload": result.payload,
    }
    if args.result_file:
        written = _write_json(args.result_file, normalized_result)
        print(f"release_op_result_written={written}")
    print(f"release_op_action={result.action}")
    print(f"release_op_policy_level={result.policy_level}")
    print(f"release_op_policy_profile={result.policy_profile}")
    print(f"release_op_request_type={result.request_type}")
    print(f"release_op_status={result.status}")
    if result.session:
        print(
            "release_op_session_context="
            f"{json.dumps(result.session, ensure_ascii=False, separators=(',', ':'))}"
        )
    if result.output_path is not None:
        print(f"release_op_output_path={result.output_path}")
    if result.evidence_report_path is not None:
        print(f"release_op_evidence_report_path={result.evidence_report_path}")
    if result.event_stream_path is not None:
        print(f"release_op_event_stream_path={result.event_stream_path}")
        print(f"release_op_event_count={result.event_count}")
    print(f"release_op_summary={result.summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
