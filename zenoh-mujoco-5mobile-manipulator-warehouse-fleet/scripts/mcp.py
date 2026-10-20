#!/usr/bin/env python3
import json
import sys
from pathlib import Path

automation = Path(__file__).resolve().parents[3] / "RobotLens-Automation" / "scripts"
sys.path.insert(0, str(automation))
from mcp_client import McpClient, live_sessions


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: mcp.py <command> [json-args]", file=sys.stderr)
        return 2
    arguments = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    sessions = live_sessions()
    if not sessions:
        print("no live RobotLens session", file=sys.stderr)
        return 1
    with McpClient(sessions[0].socket) as client:
        result = client.call(sys.argv[1], arguments) if sys.argv[1].startswith("robotlens.") else client.run(sys.argv[1], arguments)
        print(json.dumps(result.result, indent=2))
        for index, image in enumerate(result.images):
            path = Path(__file__).resolve().parents[1] / "evidence" / f"screenshot-{index}.png"
            path.parent.mkdir(exist_ok=True)
            path.write_bytes(image)
            print(f"image: {path.relative_to(Path(__file__).resolve().parents[1])}", file=sys.stderr)
        for item in result.limitations:
            print(f"limitation: {json.dumps(item)}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
