#!/usr/bin/env python3
"""
Minimal CLI to drive a valve channel open/close on Tasmota (gateway.be)
and stop automatically on current rise (CURRENT_CUTOFF) or timeout (TIME_LIMIT).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict


def send_cmnd(host: str, port: int, cmnd: str, timeout_s: float = 15.0) -> Dict[str, Any]:
    query = urllib.parse.urlencode({"cmnd": cmnd})
    url = f"http://{host}:{port}/cm?{query}"
    req = urllib.request.Request(url, headers={"Referer": f"http://{host}/"}, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            return json.loads(resp.read().decode("utf-8", errors="replace"))
    except urllib.error.URLError as exc:
        raise RuntimeError(f"HTTP connection to {host}:{port} failed: {exc}") from exc


def drive(
    host: str,
    port: int,
    ch: int,
    direction: str,
    cutoff_ma: float,
    max_ms: int,
    inrush_ms: int,
    sample_ms: int,
) -> int:
    payload = json.dumps(
        {
            "ch": ch,
            "dir": direction,
            "max_ms": max_ms,
            "cutoff_ma": round(cutoff_ma, 1),
            "inrush_ms": inrush_ms,
            "sample_ms": sample_ms,
        },
        separators=(",", ":"),
    )
    res = send_cmnd(host, port, f"RawDrive {payload}")
    if "error" in res:
        print(f"ERROR: {res['error']}", file=sys.stderr)
        return 1

    deadline = time.monotonic() + (max_ms / 1000.0) + 15.0
    chunk_limit = 200
    offset = 0
    while True:
        if time.monotonic() > deadline:
            send_cmnd(host, port, "RawStop")
            print("ERROR: Timeout waiting for RawResult", file=sys.stderr)
            return 1
        rr = send_cmnd(host, port, f'RawResult {{"offset":{offset},"limit":{chunk_limit}}}').get("RawResult", {})
        status = rr.get("status")
        if status == "IDLE":
            print("ERROR: Gateway returned IDLE", file=sys.stderr)
            return 1
        chunk = rr.get("trace_ma", [])
        for val in chunk:
            print(f"{offset:4d} | {round(float(val), 1):6.1f} mA", flush=True)
            offset += 1
        if status == "DONE" and offset >= int(rr.get("trace_len", offset)):
            break
        if len(chunk) < chunk_limit:
            time.sleep(0.25)

    print(
        f"ch={rr.get('ch')} dir={rr.get('dir')} stop_reason={rr.get('stop_reason')} "
        f"elapsed_ms={rr.get('elapsed_ms')} peak_ma={rr.get('peak_ma'):.1f} samples={offset}",
        flush=True,
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Minimal open/close motor driver with current cutoff")
    parser.add_argument("--host", default="172.31.2.100", help="Tasmota host/IP (default: 172.31.2.100)")
    parser.add_argument("--port", type=int, default=80, help="Tasmota HTTP port (default: 80)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    for direction in ("open", "close"):
        p = sub.add_parser(direction, help=f"Drive valve {direction} until current cutoff or max-ms")
        p.add_argument("--ch", type=int, default=1, choices=[1, 2, 3, 4], help="Channel 1..4 (default: 1)")
        p.add_argument("--cutoff-ma", type=float, default=30.0, help="Stop current in mA (default: 30.0)")
        p.add_argument("--max-ms", type=int, default=120000, help="Max duration in ms (default: 120000)")
        p.add_argument("--inrush-ms", type=int, default=250, help="Inrush blanking in ms (default: 250)")
        p.add_argument("--sample-ms", type=int, default=50, help="Sample interval in ms (default: 50)")

    sub.add_parser("stop", help="Stop all channels (RawStop)")
    sub.add_parser("status", help="Show RawStatus")

    args = parser.parse_args()
    if args.cmd == "stop":
        print(json.dumps(send_cmnd(args.host, args.port, "RawStop"), indent=2))
        return 0
    if args.cmd == "status":
        print(json.dumps(send_cmnd(args.host, args.port, "RawStatus"), indent=2))
        return 0
    return drive(
        host=args.host,
        port=args.port,
        ch=args.ch,
        direction=args.cmd,
        cutoff_ma=args.cutoff_ma,
        max_ms=args.max_ms,
        inrush_ms=args.inrush_ms,
        sample_ms=args.sample_ms,
    )


if __name__ == "__main__":
    sys.exit(main())
