from __future__ import annotations

import json
import os
import threading
import time
import urllib.error
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
from typing import Any

import paramiko


ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"

JETSON_HOST = os.environ.get("JETSON_HOST", "192.168.1.80")
JETSON_USER = os.environ.get("JETSON_USER", "nvidia")
JETSON_PASSWORD = os.environ.get("JETSON_PASSWORD", "nvidia")
JETSON_HTTP_PORT = int(os.environ.get("JETSON_HTTP_PORT", "8090"))
JETSON_REPO = os.environ.get("JETSON_REPO", "/home/nvidia/Desktop/78arm")

WRENCH_START = (
    "cd /home/nvidia/Desktop/78arm && "
    "bash Dual_Camera_HandEye/tools/start_wrench_rgb_orbbec_depth_jetson.sh"
)

TOOLS: list[dict[str, Any]] = [
    {"id": "wrench", "english": "wrench", "zh": "扳手", "category": "hand tool", "active": True},
    {"id": "screwdriver", "english": "screwdriver", "zh": "螺丝刀", "category": "hand tool", "active": False},
    {"id": "pliers", "english": "pliers", "zh": "钳子", "category": "hand tool", "active": False},
    {"id": "hammer", "english": "hammer", "zh": "锤子", "category": "hand tool", "active": False},
    {"id": "hex_key", "english": "hex key", "zh": "内六角扳手", "category": "hand tool", "active": False},
    {"id": "socket", "english": "socket", "zh": "套筒", "category": "hand tool", "active": False},
    {"id": "ratchet", "english": "ratchet", "zh": "棘轮扳手", "category": "hand tool", "active": False},
    {"id": "drill", "english": "drill", "zh": "电钻", "category": "power tool", "active": False},
    {"id": "tape_measure", "english": "tape measure", "zh": "卷尺", "category": "measure", "active": False},
    {"id": "utility_knife", "english": "utility knife", "zh": "美工刀", "category": "cutting", "active": False},
    {"id": "scissors", "english": "scissors", "zh": "剪刀", "category": "cutting", "active": False},
    {"id": "soldering_iron", "english": "soldering iron", "zh": "电烙铁", "category": "electronics", "active": False},
    {"id": "multimeter", "english": "multimeter", "zh": "万用表", "category": "electronics", "active": False},
    {"id": "caliper", "english": "caliper", "zh": "卡尺", "category": "measure", "active": False},
]

STATE_LOCK = threading.Lock()
STATE: dict[str, Any] = {
    "selected_tool": None,
    "selected_at": None,
    "last_action": "idle",
    "last_message": "",
    "last_ssh_stdout": "",
    "last_ssh_stderr": "",
}


def tool_by_id(tool_id: str) -> dict[str, Any] | None:
    for tool in TOOLS:
        if tool["id"] == tool_id:
            return tool
    return None


def ssh_exec(command: str, timeout: float = 35.0) -> dict[str, Any]:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    started = time.time()
    try:
        client.connect(
            hostname=JETSON_HOST,
            username=JETSON_USER,
            password=JETSON_PASSWORD,
            timeout=8,
            banner_timeout=8,
            auth_timeout=8,
        )
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        out = stdout.read().decode("utf-8", errors="replace")
        err = stderr.read().decode("utf-8", errors="replace")
        code = stdout.channel.recv_exit_status()
        return {
            "ok": code == 0,
            "exit_code": code,
            "stdout": out,
            "stderr": err,
            "elapsed_sec": round(time.time() - started, 3),
        }
    except Exception as exc:
        return {
            "ok": False,
            "exit_code": None,
            "stdout": "",
            "stderr": repr(exc),
            "elapsed_sec": round(time.time() - started, 3),
        }
    finally:
        try:
            client.close()
        except Exception:
            pass


def fetch_jetson_json(path: str, timeout: float = 2.0) -> tuple[int, dict[str, Any]]:
    url = f"http://{JETSON_HOST}:{JETSON_HTTP_PORT}{path}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            body = response.read().decode("utf-8", errors="replace")
        payload = json.loads(body)
        if isinstance(payload, dict):
            payload["_source_url"] = url
            return HTTPStatus.OK, payload
        return HTTPStatus.BAD_GATEWAY, {"ok": False, "error": "Jetson returned non-object JSON", "source": url}
    except urllib.error.HTTPError as exc:
        try:
            body = exc.read().decode("utf-8", errors="replace")
        except Exception:
            body = ""
        return exc.code, {"ok": False, "error": str(exc), "body": body, "source": url}
    except Exception as exc:
        return HTTPStatus.BAD_GATEWAY, {"ok": False, "error": repr(exc), "source": url}


def fetch_jetson_bytes(path: str, timeout: float = 2.0) -> tuple[int, bytes, str]:
    url = f"http://{JETSON_HOST}:{JETSON_HTTP_PORT}{path}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            body = response.read()
            content_type = response.headers.get("Content-Type", "application/octet-stream")
        return HTTPStatus.OK, body, content_type
    except urllib.error.HTTPError as exc:
        try:
            body = exc.read()
        except Exception:
            body = str(exc).encode("utf-8", errors="replace")
        return exc.code, body, "text/plain; charset=utf-8"
    except Exception as exc:
        return HTTPStatus.BAD_GATEWAY, repr(exc).encode("utf-8", errors="replace"), "text/plain; charset=utf-8"


def state_payload() -> dict[str, Any]:
    with STATE_LOCK:
        current = dict(STATE)
    selected = current.get("selected_tool")
    tool = tool_by_id(selected) if selected else None
    return {
        "jetson": {
            "host": JETSON_HOST,
            "user": JETSON_USER,
            "http_port": JETSON_HTTP_PORT,
            "stream_url": f"http://{JETSON_HOST}:{JETSON_HTTP_PORT}/stream.mjpg",
            "preview_url": f"http://{JETSON_HOST}:{JETSON_HTTP_PORT}/",
            "latest_url": f"http://{JETSON_HOST}:{JETSON_HTTP_PORT}/latest.json",
        },
        "tool": tool,
        "state": current,
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "ToolVisionWeb/0.1"

    def do_GET(self) -> None:
        route = urlparse(self.path).path
        if route == "/" or route == "/index.html":
            self.send_file(STATIC / "index.html", "text/html; charset=utf-8")
            return
        if route == "/api/tools":
            self.send_json(HTTPStatus.OK, {"tools": TOOLS})
            return
        if route == "/api/state":
            self.send_json(HTTPStatus.OK, state_payload())
            return
        if route == "/api/health":
            status, payload = fetch_jetson_json("/healthz")
            self.send_json(status, payload)
            return
        if route == "/api/latest":
            status, payload = fetch_jetson_json("/latest.json")
            self.send_json(status, payload)
            return
        if route == "/api/raw.jpg":
            status, body, content_type = fetch_jetson_bytes("/raw.jpg")
            self.send_bytes(status, body, content_type)
            return
        if route.startswith("/static/"):
            rel = route.removeprefix("/static/")
            target = (STATIC / rel).resolve()
            if STATIC.resolve() not in target.parents:
                self.send_error(HTTPStatus.FORBIDDEN)
                return
            content_type = "text/css" if target.suffix == ".css" else "application/octet-stream"
            self.send_file(target, content_type)
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        if self.path.startswith("/api/select"):
            payload = self.read_json_body()
            tool_id = str(payload.get("tool_id", "")).strip()
            tool = tool_by_id(tool_id)
            if tool is None:
                self.send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "unknown tool"})
                return

            with STATE_LOCK:
                STATE["selected_tool"] = tool_id
                STATE["selected_at"] = time.time()

            if not tool["active"]:
                message = f"{tool['english']} ({tool['zh']}): not detected"
                with STATE_LOCK:
                    STATE["last_action"] = "selected_unavailable"
                    STATE["last_message"] = message
                    STATE["last_ssh_stdout"] = ""
                    STATE["last_ssh_stderr"] = ""
                self.send_json(HTTPStatus.OK, {"ok": True, "started": False, "message": message, **state_payload()})
                return

            result = ssh_exec(WRENCH_START)
            message = "wrench detector started" if result["ok"] else "wrench detector start failed"
            with STATE_LOCK:
                STATE["last_action"] = "started_wrench" if result["ok"] else "start_failed"
                STATE["last_message"] = message
                STATE["last_ssh_stdout"] = result["stdout"]
                STATE["last_ssh_stderr"] = result["stderr"]
            status = HTTPStatus.OK if result["ok"] else HTTPStatus.BAD_GATEWAY
            self.send_json(status, {"ok": result["ok"], "started": result["ok"], "message": message, "ssh": result, **state_payload()})
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8"))
            return payload if isinstance(payload, dict) else {}
        except Exception:
            return {}

    def send_file(self, path: Path, content_type: str) -> None:
        if not path.exists() or not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        body = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_bytes(status, body, "application/json; charset=utf-8")

    def send_bytes(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args: Any) -> None:
        return


def main() -> int:
    host = os.environ.get("TOOL_VISION_HOST", "127.0.0.1")
    port = int(os.environ.get("TOOL_VISION_PORT", "8765"))
    server = ThreadingHTTPServer((host, port), Handler)
    print(f"Tool vision web: http://{host}:{port}")
    print(f"Jetson: {JETSON_USER}@{JETSON_HOST}, stream=http://{JETSON_HOST}:{JETSON_HTTP_PORT}/stream.mjpg")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
