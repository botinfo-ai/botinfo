#!/bin/bash
# Start the MCP server the way its README does (python server.py in its folder), recording its PID so the harness
# can kill exactly that process when the agent's host would (its children, ros2 topic pub, are left alone).
cd /opt/go2-mcp
echo $$ > /tmp/mcp-server.pid
exec /opt/go2-mcp/.venv/bin/python server.py
