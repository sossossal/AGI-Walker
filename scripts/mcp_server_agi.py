import asyncio
import sys
import logging
from mcp.server.models import InitializationOptions
from mcp.server import Notification, Server
from mcp.server.stdio import stdio_server
import mcp.types as types

# 导入 AGI-Walker 工具提供者
from agi_walker.core.api.mcp_tools import MCPToolProvider

# 配置日志到标准错误 (Stdio Server 必须保留 stdout 用于通讯)
logging.basicConfig(level=logging.INFO, stream=sys.stderr)
logger = logging.getLogger("agi-walker-mcp")

server = Server("agi-walker-control")
provider = MCPToolProvider()

@server.list_tools()
async def handle_list_tools() -> list[types.Tool]:
    """列出 AGI-Walker 可用的控制工具"""
    return [
        types.Tool(
            name="mission_execute",
            description="根据自然语言指令规划并执行机器人任务图 (TaskGraph)。例如：'optimize robot' 或 'patrol and inspect'。",
            inputSchema={
                "type": "object",
                "properties": {
                    "instruction": {"type": "string", "description": "语义任务指令"}
                },
                "required": ["instruction"],
            },
        ),
        types.Tool(
            name="robot_telemetry",
            description="获取机器人实时硬件负载、温度以及算法状态（RAG 统计等）。",
            inputSchema={"type": "object", "properties": {}},
        ),
        types.Tool(
            name="rag_query",
            description="根据当前机器人姿态检索最匹配的历史成功经验。",
            inputSchema={
                "type": "object",
                "properties": {
                    "orient": {
                        "type": "array", 
                        "items": {"type": "number"},
                        "description": "当前姿态 [roll, pitch, yaw]"
                    }
                },
                "required": ["orient"],
            },
        )
    ]

@server.call_tool()
async def handle_call_tool(
    name: str, arguments: dict | None
) -> list[types.TextContent | types.ImageContent | types.EmbeddedResource]:
    """处理来自 Gemini CLI 的工具调用"""
    if name == "mission_execute":
        instruction = arguments.get("instruction", "")
        result = await provider.execute_mission(instruction)
        return [types.TextContent(type="text", text=result)]
    
    elif name == "robot_telemetry":
        result = provider.get_telemetry()
        return [types.TextContent(type="text", text=result)]
    
    elif name == "rag_query":
        orient = arguments.get("orient", [0, 0, 0])
        result = provider.query_rag(orient)
        return [types.TextContent(type="text", text=result)]
    
    else:
        raise ValueError(f"Unknown tool: {name}")

async def main():
    async with stdio_server() as (read_stream, write_thread):
        await server.run(
            read_stream,
            write_thread,
            InitializationOptions(
                server_name="agi-walker-control",
                server_version="3.0.0",
                capabilities=server.get_capabilities(),
            ),
        )

if __name__ == "__main__":
    asyncio.run(main())
