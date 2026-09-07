"""Read-only capture readiness, without the ROS CLI discovery daemon.

Use one short-lived DDS participant and a real Foxglove WebSocket handshake.
Nothing is published, recorded, enabled, stopped or restarted by this command.
"""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import time


def check_foxglove(address, port, timeout):
    host = "127.0.0.1" if address in {"", "0.0.0.0"} else address
    host = "::1" if host == "::" else host
    key = base64.b64encode(os.urandom(16)).decode()
    # Current SDK-backed Bridge negotiates sdk.v1; older Bridge uses websocket.v1.
    protocols = ("foxglove.sdk.v1", "foxglove.websocket.v1")
    request = (
        f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\n"
        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
        f"Sec-WebSocket-Protocol: {', '.join(protocols)}\r\n\r\n"
    ).encode()
    deadline = time.monotonic() + timeout
    with socket.create_connection((host, port), timeout=timeout) as connection:
        connection.sendall(request)
        response = b""
        while b"\r\n\r\n" not in response:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Foxglove handshake timed out")
            connection.settimeout(remaining)
            chunk = connection.recv(4096)
            if not chunk or len(response) + len(chunk) > 65536:
                raise ValueError("Invalid Foxglove upgrade response")
            response += chunk
        lines = response.split(b"\r\n\r\n", 1)[0].decode("latin1").split("\r\n")
        if lines[0].split()[1] != "101":
            raise ValueError(f"Foxglove upgrade rejected: {lines[0]}")
        headers = dict((k.lower(), v.strip()) for k, v in
                       (line.split(":", 1) for line in lines[1:] if ":" in line))
        expected = base64.b64encode(hashlib.sha1(
            (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
        if (headers.get("sec-websocket-accept") != expected
                or headers.get("sec-websocket-protocol") not in protocols):
            raise ValueError("Endpoint did not negotiate the Foxglove protocol")
        # Close this observation-only probe using a masked, empty close frame.
        connection.sendall(b"\x88\x80" + os.urandom(4))


def main():
    import rclpy
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--system-config", type=Path, required=True)
    parser.add_argument("--topic", action="append", required=True)
    parser.add_argument("--timeout-s", type=float, default=8.0)
    args = parser.parse_args()
    if not 0 < args.timeout_s <= 30:
        parser.error("timeout must be in (0, 30] seconds")
    bridge = json.loads(args.system_config.read_text())["stack"]["foxglove"]
    rclpy.init(args=[])
    node = rclpy.create_node(f"memnav_capture_readiness_{os.getpid()}",
                            enable_rosout=False, start_parameter_services=False)
    started = time.monotonic()
    deadline = started + args.timeout_s
    missing = list(args.topic)
    bridge_error = "not checked"
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            missing = [topic for topic in args.topic if node.count_publishers(topic) == 0]
            try:
                check_foxglove(bridge["address"], int(bridge["port"]),
                               min(0.5, max(0.01, deadline - time.monotonic())))
                bridge_error = None
            except (OSError, ValueError, IndexError) as error:
                bridge_error = f"{type(error).__name__}: {error}"
            if not missing and bridge_error is None:
                print(json.dumps({"capture_ready": True,
                                  "elapsed_s": round(time.monotonic() - started, 3),
                                  "topics_with_publishers": args.topic,
                                  "foxglove": "websocket_protocol_verified"}))
                return 0
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0., deadline-time.monotonic())))
        print(json.dumps({"capture_ready": False, "missing_publishers": missing,
                          "foxglove_error": bridge_error,
                          "waited_s": round(time.monotonic() - started, 3)}), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
