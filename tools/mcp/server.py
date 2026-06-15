#!/usr/bin/env python3
"""openpilot MCP server.

Exposes openpilot's route, log, schema and CAN tooling over the Model Context Protocol
so an LLM client can inspect drives without loading whole logs into context.
"""
from mcp.server.fastmcp import FastMCP

from openpilot.tools.mcp import routes, schema, logs, can

mcp = FastMCP("openpilot")

# routes
mcp.tool()(routes.route_info)

# schema introspection
mcp.tool()(schema.list_log_messages)
mcp.tool()(schema.describe_message)

# log viewing
mcp.tool()(logs.read_log)

# CAN
mcp.tool()(can.list_dbcs)
mcp.tool()(can.dbc_info)
mcp.tool()(can.parse_can)


def main():
  mcp.run()


if __name__ == "__main__":
  main()
