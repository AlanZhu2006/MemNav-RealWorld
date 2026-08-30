#!/usr/bin/env python3
"""Create or update one Foxglove organization layout from a tracked JSON file."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from typing import Any, Optional
from urllib import error, parse, request


DEFAULT_API_BASE = "https://api.foxglove.dev/v1"
PERMISSIONS = ("CREATOR_WRITE", "ORG_READ", "ORG_WRITE")


class ApiError(RuntimeError):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(f"Foxglove API returned HTTP {status}: {message}")
        self.status = int(status)


class FoxgloveClient:
    def __init__(
        self,
        api_key: str,
        *,
        api_base: str = DEFAULT_API_BASE,
        timeout_s: float = 20.0,
        max_attempts: int = 4,
    ) -> None:
        if not api_key:
            raise ValueError("FOXGLOVE_API_KEY is required")
        self.api_key = api_key
        self.api_base = api_base.rstrip("/")
        self.timeout_s = float(timeout_s)
        self.max_attempts = max(1, int(max_attempts))

    def request(
        self,
        method: str,
        path: str,
        payload: Optional[dict[str, Any]] = None,
    ) -> Any:
        body = None
        headers = {
            "Accept": "application/json",
            "Authorization": f"Bearer {self.api_key}",
            "User-Agent": "MemNav-RealWorld-layout-sync/1",
        }
        if payload is not None:
            body = json.dumps(
                payload, ensure_ascii=False, separators=(",", ":")
            ).encode("utf-8")
            headers["Content-Type"] = "application/json"
        url = f"{self.api_base}/{path.lstrip('/')}"
        for attempt in range(1, self.max_attempts + 1):
            call = request.Request(
                url, data=body, headers=headers, method=method.upper()
            )
            try:
                with request.urlopen(call, timeout=self.timeout_s) as response:
                    response_body = response.read()
                    return (
                        json.loads(response_body.decode("utf-8"))
                        if response_body
                        else None
                    )
            except error.HTTPError as exc:
                response_body = exc.read().decode("utf-8", errors="replace")
                response_body = response_body.replace(self.api_key, "[REDACTED]")
                retryable = exc.code == 429 or 500 <= exc.code < 600
                if retryable and attempt < self.max_attempts:
                    retry_after = exc.headers.get("Retry-After")
                    delay = (
                        float(retry_after)
                        if retry_after and retry_after.replace(".", "", 1).isdigit()
                        else min(2 ** (attempt - 1), 8)
                    )
                    time.sleep(delay)
                    continue
                raise ApiError(exc.code, response_body[:500]) from None
            except error.URLError as exc:
                if attempt < self.max_attempts:
                    time.sleep(min(2 ** (attempt - 1), 8))
                    continue
                raise RuntimeError(f"Foxglove API connection failed: {exc.reason}") from None
        raise AssertionError("unreachable")


def load_layout(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise ValueError(f"layout file does not exist: {path}") from None
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid layout JSON at {path}: {exc}") from None
    if not isinstance(payload, dict) or not payload:
        raise ValueError("Foxglove layout data must be a non-empty JSON object")
    return payload


def layout_payload(
    *,
    name: str,
    folder_name: str,
    permission: str,
    data: dict[str, Any],
) -> dict[str, Any]:
    return {
        "name": name,
        "folderName": folder_name,
        "permission": permission,
        "data": data,
    }


def sync_layout(
    client: FoxgloveClient,
    *,
    layout_id: Optional[str],
    name: str,
    folder_name: str,
    permission: str,
    data: dict[str, Any],
) -> dict[str, Any]:
    existing = None
    if layout_id:
        encoded_id = parse.quote(layout_id, safe="")
        try:
            existing = client.request(
                "GET", f"layouts/{encoded_id}?includeData=true"
            )
        except ApiError as exc:
            if exc.status != 404:
                raise
    else:
        layouts = client.request("GET", "layouts?includeData=true")
        matches = [item for item in layouts if item.get("name") == name]
        if len(matches) > 1:
            raise ValueError(
                f"multiple organization layouts are named {name!r}; "
                "remove duplicates or pass --layout-id"
            )
        if matches:
            existing = matches[0]

    desired = layout_payload(
        name=name,
        folder_name=folder_name,
        permission=permission,
        data=data,
    )
    if existing is None:
        created = client.request("POST", "layouts", desired)
        return {"action": "created", "layout": created}

    current = {
        "name": existing.get("name"),
        "folderName": existing.get("folderName") or "",
        "permission": existing.get("permission"),
        "data": existing.get("data"),
    }
    if current == desired:
        return {"action": "unchanged", "layout": existing}

    encoded_id = parse.quote(str(existing["id"]), safe="")
    updated = client.request("PATCH", f"layouts/{encoded_id}", desired)
    return {"action": "updated", "layout": updated}


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--layout-id")
    parser.add_argument("--name", required=True)
    parser.add_argument("--folder-name", default="")
    parser.add_argument("--permission", choices=PERMISSIONS, default="ORG_WRITE")
    parser.add_argument("--api-base", default=DEFAULT_API_BASE)
    parser.add_argument("--result", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if args.layout_id is not None and not 1 <= len(args.layout_id) <= 36:
        parser.error("--layout-id must contain 1 to 36 characters")
    if "/" in args.folder_name:
        parser.error("--folder-name cannot contain forward slashes")
    return args


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    try:
        data = load_layout(args.layout)
        digest = hashlib.sha256(
            json.dumps(
                data, ensure_ascii=False, sort_keys=True, separators=(",", ":")
            ).encode("utf-8")
        ).hexdigest()
        if args.dry_run:
            result = {
                "action": "validated",
                "layout": {"id": args.layout_id or "server-generated", "name": args.name},
                "sha256": digest,
            }
        else:
            api_key = os.environ.get("FOXGLOVE_API_KEY", "")
            if not api_key:
                raise ValueError(
                    "FOXGLOVE_API_KEY is missing; configure it as a GitHub Actions secret"
                )
            client = FoxgloveClient(api_key, api_base=args.api_base)
            result = sync_layout(
                client,
                layout_id=args.layout_id,
                name=args.name,
                folder_name=args.folder_name,
                permission=args.permission,
                data=data,
            )
            result["sha256"] = digest
        output = json.dumps(result, ensure_ascii=False, sort_keys=True)
        print(output)
        if args.result:
            args.result.parent.mkdir(parents=True, exist_ok=True)
            args.result.write_text(output + "\n", encoding="utf-8")
        return 0
    except (ApiError, RuntimeError, ValueError) as exc:
        print(f"foxglove layout sync failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
