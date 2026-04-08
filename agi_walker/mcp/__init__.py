from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from mcp.server import Server

    from agi_walker.core.api.mcp_tools import MCPToolProvider
    from agi_walker.mcp.server import ToolDefinition


def create_server(provider: "MCPToolProvider | None" = None) -> "Server":
    from agi_walker.mcp.server import create_server as _create_server

    return _create_server(provider)


def get_tool_definitions(
    provider: "MCPToolProvider | None" = None,
) -> dict[str, "ToolDefinition"]:
    from agi_walker.mcp.server import get_tool_definitions as _get_tool_definitions

    return _get_tool_definitions(provider)


def main() -> Any:
    from agi_walker.mcp.server import main as _main

    return _main()


def run() -> None:
    from agi_walker.mcp.server import run as _run

    _run()


__all__ = ["create_server", "get_tool_definitions", "main", "run"]
