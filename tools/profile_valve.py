#!/usr/bin/env python3
"""
Phase 0 / Phase 1 Remote Auto-Profiling & Valve Calibration CLI for ATB-Flow.

Communicates with a Tasmota ESP32-S3 node running `gateway.be` via HTTP:
  - `check`     : Smoke-test INA219 current sensor (`RawStatus`) and DS18B20 (`Status 10`)
  - `pulse`     : Initial test pulse (Auto-Profiling) to determine I_free, sigma_free,
                  I_stall_open, I_stall_close, and delta_I_touch
  - `calibrate` : Full 3-point calibration (Top stop -> Touch point -> Closed stop -> Open stroke)
                  and persistence to `valve_profile.json` + trace files in `traces/`
  - `set-pos`   : Move a calibrated valve channel to a target position u in [0..100]%
  - `stop`      : Emergency stop (`RawStop`)
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import statistics
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Pure Mathematical & Signal Processing Functions (Section 5.1 of System Design)
# ---------------------------------------------------------------------------

def compute_free_run_stats(
    trace_ma: List[float],
    sample_ms: int = 25,
    inrush_ms: int = 100,
    tail_trim_samples: int = 2,
) -> Dict[str, float]:
    """
    Calculate mean free-running current (I_free), standard deviation (sigma_free),
    and derived thresholds from a free-running motor current trace (t > inrush_ms).

    Formulas (Section 5.1.1):
      I_stall_open  = 2.2 * I_free
      I_stall_close = 2.5 * I_free
      delta_I_touch = max(0.35 * I_free, 4.0 * sigma_free)
    """
    if sample_ms <= 0:
        raise ValueError("sample_ms must be > 0")

    skip_samples = int(math.ceil(inrush_ms / float(sample_ms))) + 1
    end_idx = len(trace_ma) - tail_trim_samples if len(trace_ma) > skip_samples + tail_trim_samples else len(trace_ma)
    window = [float(x) for x in trace_ma[skip_samples:end_idx] if float(x) >= 0.0]

    if len(window) < 3:
        raise ValueError(
            f"Not enough free-running samples after inrush ({len(window)} samples). "
            "Check motor wiring or increase max_ms."
        )

    i_free = statistics.fmean(window)
    sigma_free = statistics.stdev(window) if len(window) >= 2 else 0.0

    i_stall_open = 2.2 * i_free
    i_stall_close = 2.5 * i_free
    delta_i_touch = max(0.35 * i_free, 4.0 * sigma_free)

    return {
        "i_free_ma": round(i_free, 3),
        "sigma_free_ma": round(sigma_free, 3),
        "i_stall_open_ma": round(i_stall_open, 3),
        "i_stall_close_ma": round(i_stall_close, 3),
        "delta_i_touch_ma": round(delta_i_touch, 3),
        "touch_threshold_ma": round(i_free + delta_i_touch, 3),
        "sample_count": len(window),
    }


def detect_touch_point(
    trace_ma: List[float],
    i_free_ma: float,
    delta_i_touch_ma: float,
    sample_ms: int = 25,
    inrush_ms: int = 150,
    window_size: int = 5,
) -> Optional[Tuple[int, int, float]]:
    """
    Detect the valve plunger contact point (Touch-Point, u = 100%) during a closing run.

    Criterion (Section 5.1.2):
      First point after inrush_ms where the 5-sample moving average of current
      is >= I_free + delta_I_touch.

    Returns:
      (sample_index, t_touch_ms, moving_avg_ma) or None if not detected.
    """
    if sample_ms <= 0 or window_size < 1:
        raise ValueError("sample_ms and window_size must be >= 1")

    threshold = i_free_ma + delta_i_touch_ma
    start_idx = max(int(math.ceil(inrush_ms / float(sample_ms))) + 1, window_size - 1)

    if len(trace_ma) <= start_idx:
        return None

    for idx in range(start_idx, len(trace_ma)):
        win = trace_ma[idx - window_size + 1 : idx + 1]
        avg = sum(win) / float(window_size)
        if avg >= threshold:
            # Touch occurred at the start/center of the rising window;
            # use idx * sample_ms as documented in 5.1.2
            t_touch_ms = idx * sample_ms
            return idx, t_touch_ms, round(avg, 3)

    return None


def compute_calibration_parameters(
    t_touch_ms: int,
    t_close_ms: int,
    t_open_total_ms: int,
) -> Dict[str, float]:
    """
    Compute working stroke durations and ratio from 3-point calibration timings (Section 5.1.2):
      T_hub_zu  = t_close - t_touch
      eta_work  = T_hub_zu / t_close
      T_hub_auf = eta_work * t_open_total
    """
    if t_close_ms <= 0 or t_open_total_ms <= 0:
        raise ValueError("t_close_ms and t_open_total_ms must be > 0")
    if t_touch_ms < 0 or t_touch_ms >= t_close_ms:
        raise ValueError(f"Invalid t_touch_ms ({t_touch_ms}) for t_close_ms ({t_close_ms})")

    t_hub_zu_ms = float(t_close_ms - t_touch_ms)
    eta_work = t_hub_zu_ms / float(t_close_ms)
    t_hub_auf_ms = eta_work * float(t_open_total_ms)

    return {
        "t_touch_ms": int(t_touch_ms),
        "t_close_ms": int(t_close_ms),
        "t_open_total_ms": int(t_open_total_ms),
        "t_hub_zu_ms": round(t_hub_zu_ms, 1),
        "t_hub_auf_ms": round(t_hub_auf_ms, 1),
        "eta_work": round(eta_work, 4),
    }


def compute_move_plan(
    u_old_pct: float,
    u_new_pct: float,
    cal: Dict[str, Any],
) -> Dict[str, Any]:
    """
    Calculate the drive direction, duration delta_t_ms, and cutoff current
    for moving a valve from u_old_pct to u_new_pct (Section 5.1.3).

    Formula:
      delta_t = (|u_new - u_old| / 100) * (T_hub_auf if u_new > u_old else T_hub_zu)
    At 0% or 100%, drive until stall current (I_stall) to eliminate cumulative drift.
    """
    u_old = max(0.0, min(100.0, float(u_old_pct)))
    u_new = max(0.0, min(100.0, float(u_new_pct)))
    delta_u = u_new - u_old

    if abs(delta_u) < 1e-6:
        return {
            "action": "NONE",
            "dir": None,
            "max_ms": 0,
            "cutoff_ma": 0.0,
            "target_pct": u_new,
            "full_stall_sync": False,
        }

    direction = "open" if delta_u > 0 else "close"
    stroke_ms = float(cal["t_hub_auf_ms"]) if direction == "open" else float(cal["t_hub_zu_ms"])
    cutoff_ma = float(cal["i_stall_open_ma"]) if direction == "open" else float(cal["i_stall_close_ma"])

    # Drift elimination at endpoints 0% and 100%
    if u_new <= 0.0:
        max_ms = min(35000, int(float(cal["t_close_ms"]) * 1.35))
        return {
            "action": "DRIVE_TO_STALL",
            "dir": "close",
            "max_ms": max_ms,
            "cutoff_ma": cutoff_ma,
            "target_pct": 0.0,
            "full_stall_sync": True,
        }

    if u_new >= 100.0:
        # Note: 100% valve opening corresponds to the Touch-Point (plunger just lifts off pin).
        # However, per Section 5.1.3 ("Bei Ziel 0% oder 100% Fahrt bis I_stall zur Drift-Eliminierung"),
        # driving to upper stall and optionally stepping to touch point re-synchronizes position.
        max_ms = min(35000, int(float(cal["t_open_total_ms"]) * 1.35))
        return {
            "action": "DRIVE_TO_STALL",
            "dir": "open",
            "max_ms": max_ms,
            "cutoff_ma": cutoff_ma,
            "target_pct": 100.0,
            "full_stall_sync": True,
        }

    delta_t_ms = int(round((abs(delta_u) / 100.0) * stroke_ms))
    return {
        "action": "DRIVE_TIMED",
        "dir": direction,
        "max_ms": max(25, delta_t_ms),
        "cutoff_ma": cutoff_ma,
        "target_pct": round(u_new, 1),
        "full_stall_sync": False,
    }


def render_ascii_trace(
    trace_ma: List[float],
    sample_ms: int = 25,
    touch_idx: Optional[int] = None,
    threshold_ma: Optional[float] = None,
    width: int = 60,
) -> str:
    """Render a compact ASCII bar chart of a current trace for terminal inspection."""
    if not trace_ma:
        return "(empty trace)"

    peak = max(max(trace_ma), threshold_ma or 0.0, 1.0)
    lines = [
        f"Time (ms) | Current (mA) | Plot (max={peak:.1f} mA)",
        "-" * (28 + width),
    ]

    # Downsample if trace is very long (> 80 rows)
    step = max(1, len(trace_ma) // 80)
    for idx in range(0, len(trace_ma), step):
        val = trace_ma[idx]
        t_ms = idx * sample_ms
        bar_len = int(round((max(0.0, val) / peak) * width))
        bar = "#" * bar_len
        marker = ""
        if touch_idx is not None and abs(idx - touch_idx) < step:
            marker = " <-- TOUCH POINT (u=100%)"
        lines.append(f"{t_ms:8d} | {val:10.1f} mA | {bar}{marker}")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# HTTP Client for Tasmota DumbValveGateway
# ---------------------------------------------------------------------------

class TasmotaGatewayClient:
    def __init__(self, host: str, port: int = 80, timeout_s: float = 45.0) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.base_url = f"http://{host}:{port}/cm"

    def send_command(self, cmnd: str) -> Dict[str, Any]:
        query = urllib.parse.urlencode({"cmnd": cmnd})
        url = f"{self.base_url}?{query}"
        req = urllib.request.Request(
            url,
            headers={"Referer": f"http://{self.host}/"},
            method="GET",
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
                return json.loads(raw)
        except urllib.error.URLError as exc:
            raise RuntimeError(f"HTTP connection to {self.host}:{self.port} failed: {exc}") from exc
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid JSON response from {self.host}: {exc}") from exc

    def raw_status(self) -> Dict[str, Any]:
        res = self.send_command("RawStatus")
        if "RawStatus" not in res:
            raise RuntimeError(f"Unexpected RawStatus response: {res}")
        return res["RawStatus"]

    def raw_stop(self) -> Dict[str, Any]:
        return self.send_command("RawStop")

    def raw_drive(
        self,
        ch: int,
        direction: str,
        max_ms: int = 5000,
        cutoff_ma: float = 150.0,
        inrush_ms: int = 100,
        sample_ms: int = 25,
    ) -> Dict[str, Any]:
        payload = json.dumps(
            {
                "ch": ch,
                "dir": direction,
                "max_ms": int(max_ms),
                "cutoff_ma": round(float(cutoff_ma), 1),
                "inrush_ms": int(inrush_ms),
                "sample_ms": int(sample_ms),
            },
            separators=(",", ":"),
        )
        res = self.send_command(f"RawDrive {payload}")
        if "error" in res:
            raise RuntimeError(f"Gateway returned error: {res['error']}")
        if "RawDrive" not in res:
            raise RuntimeError(f"Unexpected RawDrive response: {res}")
        return res["RawDrive"]

    def status_sensors(self) -> Dict[str, Any]:
        return self.send_command("Status 10")


# ---------------------------------------------------------------------------
# High-Level Workflows (Check, Pulse, Calibrate, Set-Pos)
# ---------------------------------------------------------------------------

def save_trace(trace_dir: Path, label: str, drive_result: Dict[str, Any]) -> Path:
    trace_dir.mkdir(parents=True, exist_ok=True)
    ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    ch = drive_result.get("ch", 0)
    direction = drive_result.get("dir", "unknown")
    fname = trace_dir / f"{ts}_ch{ch}_{label}_{direction}.json"
    fname.write_text(json.dumps(drive_result, indent=2) + "\n", encoding="utf-8")
    return fname


def load_valve_profile(path: Path) -> Dict[str, Any]:
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {"host": "", "updated_at": "", "channels": {}}


def save_valve_profile(path: Path, profile: Dict[str, Any]) -> None:
    profile["updated_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
    path.write_text(json.dumps(profile, indent=2) + "\n", encoding="utf-8")


def run_check(client: TasmotaGatewayClient) -> int:
    print(f"=== Hardware Check on {client.host}:{client.port} ===")
    status = client.raw_status()
    print(f"  INA219 connected : {status.get('ina219_connected')} (addr {status.get('ina219_addr')})")
    print(f"  Idle current     : {status.get('idle_ma'):.2f} mA")
    print(f"  Motor busy       : {status.get('is_moving')}")
    print(f"  Standby GPIO     : GPIO {status.get('pin_stby')}")
    print(f"  Channel pins     : {json.dumps(status.get('ch_pins'))}")

    if not status.get("ina219_connected"):
        print("\n[FAIL] INA219 not detected on I2C bus (0x40)! Check SDA (GPIO8), SCL (GPIO9), VCC (3V3), GND.")
        return 1

    if float(status.get("idle_ma", 0.0)) > 5.0:
        print(f"\n[WARN] Unexpectedly high idle current ({status.get('idle_ma')} mA) while motors are stopped!")

    sensors = client.status_sensors().get("StatusSNS", {})
    print("\n--- Sensor Telemetry (Status 10) ---")
    print(json.dumps(sensors, indent=2))
    print("\n[OK] Hardware smoke test passed.")
    return 0


def run_auto_profile_pulse(
    client: TasmotaGatewayClient,
    ch: int,
    initial_dir: str = "open",
    pulse_ms: int = 1500,
    inrush_ms: int = 100,
    sample_ms: int = 25,
    trace_dir: Optional[Path] = None,
) -> Dict[str, float]:
    """
    Execute Section 5.1.1 Initial Test Pulse (Auto-Profiling):
    - Drive for 1500 ms with cutoff_ma = 400.
    - If immediate stall (< 150 ms after start) occurs, reverse direction automatically.
    - Compute I_free, sigma_free, I_stall_open, I_stall_close, delta_I_touch.
    """
    print(f"\n[Step 1] Auto-Profiling test pulse on channel {ch} (dir={initial_dir}, max_ms={pulse_ms})...")
    res = client.raw_drive(
        ch=ch,
        direction=initial_dir,
        max_ms=pulse_ms,
        cutoff_ma=400.0,
        inrush_ms=inrush_ms,
        sample_ms=sample_ms,
    )
    if trace_dir:
        save_trace(trace_dir, "pulse1", res)

    if res.get("stop_reason") == "I2C_ERROR":
        raise RuntimeError("Gateway aborted drive with I2C_ERROR (INA219 read failed).")

    # Check if blocked immediately (< 250 ms elapsed or < 6 samples)
    trace = res.get("trace_ma", [])
    early_stall = (
        res.get("elapsed_ms", 0) < 250
        or len(trace) < 8
        or (len(trace) >= 5 and statistics.fmean(trace[3:]) > 75.0)
    )

    if early_stall:
        rev_dir = "close" if initial_dir == "open" else "open"
        print(f"  -> Immediate end-stop detected in '{initial_dir}' direction. Reversing to '{rev_dir}'...")
        time.sleep(0.3)
        res = client.raw_drive(
            ch=ch,
            direction=rev_dir,
            max_ms=pulse_ms,
            cutoff_ma=400.0,
            inrush_ms=inrush_ms,
            sample_ms=sample_ms,
        )
        if trace_dir:
            save_trace(trace_dir, "pulse_rev", res)

    stats = compute_free_run_stats(
        trace_ma=res.get("trace_ma", []),
        sample_ms=sample_ms,
        inrush_ms=inrush_ms,
    )
    print(
        f"  -> Free-run current : I_free = {stats['i_free_ma']:.2f} mA "
        f"(sigma = {stats['sigma_free_ma']:.2f} mA, n = {stats['sample_count']})"
    )
    print(
        f"  -> Derived limits   : I_stall_open = {stats['i_stall_open_ma']:.1f} mA | "
        f"I_stall_close = {stats['i_stall_close_ma']:.1f} mA | "
        f"delta_I_touch = {stats['delta_i_touch_ma']:.2f} mA "
        f"(Touch >= {stats['touch_threshold_ma']:.1f} mA)"
    )
    return stats


def run_calibrate(
    client: TasmotaGatewayClient,
    ch: int,
    profile_path: Path,
    trace_dir: Path,
    inrush_ms: int = 125,
    sample_ms: int = 25,
) -> int:
    print(f"=== 3-Point Valve Calibration: Channel {ch} on {client.host} ===")

    # 1. Auto-profile pulse
    stats = run_auto_profile_pulse(
        client=client,
        ch=ch,
        initial_dir="open",
        pulse_ms=1500,
        inrush_ms=inrush_ms,
        sample_ms=sample_ms,
        trace_dir=trace_dir,
    )
    time.sleep(0.4)

    # 2. Drive to upper mechanical stop (t = 0 reference)
    print(f"\n[Step 2] Driving to upper mechanical stop (dir='open', cutoff={stats['i_stall_open_ma']:.1f} mA)...")
    res_top = client.raw_drive(
        ch=ch,
        direction="open",
        max_ms=35000,
        cutoff_ma=stats["i_stall_open_ma"],
        inrush_ms=inrush_ms,
        sample_ms=sample_ms,
    )
    save_trace(trace_dir, "homing_open", res_top)
    print(
        f"  -> Reached upper stop: reason={res_top['stop_reason']}, "
        f"elapsed={res_top['elapsed_ms']} ms, peak={res_top['peak_ma']:.1f} mA"
    )
    if res_top["stop_reason"] != "CURRENT_CUTOFF":
        print(f"[WARN] Upper stop did not trigger CURRENT_CUTOFF (reason={res_top['stop_reason']}).")
    time.sleep(0.5)

    # 3. Full closing stroke: measure t_touch (u = 100%) and t_close (u = 0%)
    print(
        f"\n[Step 3] Full closing stroke (dir='close', cutoff={stats['i_stall_close_ma']:.1f} mA) "
        "to detect Touch-Point and closed end-stop..."
    )
    res_close = client.raw_drive(
        ch=ch,
        direction="close",
        max_ms=35000,
        cutoff_ma=stats["i_stall_close_ma"],
        inrush_ms=inrush_ms,
        sample_ms=sample_ms,
    )
    close_trace_file = save_trace(trace_dir, "cal_close", res_close)
    close_trace = res_close.get("trace_ma", [])
    t_close_ms = int(res_close.get("elapsed_ms", len(close_trace) * sample_ms))

    # Re-evaluate free-run stats from the first subtle part of the close stroke if enough samples exist,
    # otherwise use the pulse stats.
    touch = detect_touch_point(
        trace_ma=close_trace,
        i_free_ma=stats["i_free_ma"],
        delta_i_touch_ma=stats["delta_i_touch_ma"],
        sample_ms=sample_ms,
        inrush_ms=max(150, inrush_ms),
    )

    if touch is None:
        print("\n" + render_ascii_trace(close_trace, sample_ms=sample_ms))
        raise RuntimeError(
            "Could not detect a Touch-Point during closing stroke! "
            "Is the actuator mounted on a valve? Trace saved to " + str(close_trace_file)
        )

    touch_idx, t_touch_ms, touch_avg_ma = touch
    print(
        f"  -> Touch-Point (u=100%) detected at t_touch = {t_touch_ms} ms "
        f"(sample #{touch_idx}, 5-sample avg = {touch_avg_ma:.1f} mA)"
    )
    print(
        f"  -> Closed stop (u=0%) reached at t_close = {t_close_ms} ms "
        f"(reason={res_close['stop_reason']}, peak={res_close['peak_ma']:.1f} mA)"
    )
    print("\n--- Closing Stroke Current Profile ---")
    print(
        render_ascii_trace(
            close_trace,
            sample_ms=sample_ms,
            touch_idx=touch_idx,
            threshold_ma=stats["touch_threshold_ma"],
        )
    )
    time.sleep(0.5)

    # 4. Full opening stroke: measure t_open_total from 0% to upper stop
    print(f"\n[Step 4] Full opening stroke (dir='open', cutoff={stats['i_stall_open_ma']:.1f} mA)...")
    res_open = client.raw_drive(
        ch=ch,
        direction="open",
        max_ms=35000,
        cutoff_ma=stats["i_stall_open_ma"],
        inrush_ms=inrush_ms,
        sample_ms=sample_ms,
    )
    save_trace(trace_dir, "cal_open", res_open)
    t_open_total_ms = int(res_open.get("elapsed_ms", len(res_open.get("trace_ma", [])) * sample_ms))
    print(
        f"  -> Upper stop reached at t_open_total = {t_open_total_ms} ms "
        f"(reason={res_open['stop_reason']}, peak={res_open['peak_ma']:.1f} mA)"
    )

    # 5. Move from upper mechanical stop down to Touch-Point (u = 100% ready state)
    print(f"\n[Step 5] Positioning plunger from upper stop to Touch-Point (u=100%, {t_touch_ms} ms)...")
    time.sleep(0.4)
    client.raw_drive(
        ch=ch,
        direction="close",
        max_ms=t_touch_ms,
        cutoff_ma=stats["i_stall_close_ma"],
        inrush_ms=inrush_ms,
        sample_ms=sample_ms,
    )

    cal_params = compute_calibration_parameters(
        t_touch_ms=t_touch_ms,
        t_close_ms=t_close_ms,
        t_open_total_ms=t_open_total_ms,
    )

    channel_record = {
        **stats,
        **cal_params,
        "current_pos_pct": 100.0,
        "calibrated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    }

    profile = load_valve_profile(profile_path)
    profile["host"] = client.host
    profile.setdefault("channels", {})[str(ch)] = channel_record
    save_valve_profile(profile_path, profile)

    print("\n=== Calibration Summary ===")
    print(json.dumps(channel_record, indent=2))
    print(f"\n[OK] Saved calibration for channel {ch} to {profile_path}")
    return 0


def run_set_position(
    client: TasmotaGatewayClient,
    ch: int,
    target_pct: float,
    profile_path: Path,
    inrush_ms: int = 125,
    sample_ms: int = 25,
) -> int:
    profile = load_valve_profile(profile_path)
    ch_key = str(ch)
    if ch_key not in profile.get("channels", {}):
        raise RuntimeError(f"Channel {ch} is not calibrated in {profile_path}. Run 'calibrate --ch {ch}' first.")

    cal = profile["channels"][ch_key]
    u_old = float(cal.get("current_pos_pct", 100.0))
    plan = compute_move_plan(u_old_pct=u_old, u_new_pct=target_pct, cal=cal)

    if plan["action"] == "NONE":
        print(f"Channel {ch} is already at {u_old:.1f}%.")
        return 0

    print(
        f"Moving channel {ch}: {u_old:.1f}% -> {plan['target_pct']:.1f}% "
        f"(action={plan['action']}, dir={plan['dir']}, max_ms={plan['max_ms']}, cutoff={plan['cutoff_ma']:.1f} mA)"
    )
    res = client.raw_drive(
        ch=ch,
        direction=plan["dir"],
        max_ms=plan["max_ms"],
        cutoff_ma=plan["cutoff_ma"],
        inrush_ms=inrush_ms,
        sample_ms=sample_ms,
    )
    print(
        f"  -> Drive completed: stop_reason={res['stop_reason']}, "
        f"elapsed={res['elapsed_ms']} ms, peak={res['peak_ma']:.1f} mA"
    )

    # If we synchronized to upper mechanical stop for 100%, step down to Touch-Point (u = 100%)
    if plan["full_stall_sync"] and plan["target_pct"] >= 100.0 and int(cal.get("t_touch_ms", 0)) > 0:
        t_touch_ms = int(cal["t_touch_ms"])
        print(f"  -> Stepping {t_touch_ms} ms from upper stop to Touch-Point (u = 100%)...")
        time.sleep(0.3)
        client.raw_drive(
            ch=ch,
            direction="close",
            max_ms=t_touch_ms,
            cutoff_ma=float(cal["i_stall_close_ma"]),
            inrush_ms=inrush_ms,
            sample_ms=sample_ms,
        )

    cal["current_pos_pct"] = plan["target_pct"]
    cal["updated_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
    save_valve_profile(profile_path, profile)
    print(f"[OK] Channel {ch} position updated to {plan['target_pct']:.1f}% in {profile_path}")
    return 0


# ---------------------------------------------------------------------------
# CLI Entrypoint
# ---------------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ATB-Flow Phase 0 Auto-Profiling and 3-Point Valve Calibration CLI"
    )
    parser.add_argument(
        "--host",
        default="tasmota-hkv-test",
        help="Tasmota hostname or IP address (default: tasmota-hkv-test)",
    )
    parser.add_argument("--port", type=int, default=80, help="Tasmota HTTP port (default: 80)")
    parser.add_argument(
        "--profile",
        type=Path,
        default=Path("valve_profile.json"),
        help="Path to valve_profile.json (default: ./valve_profile.json)",
    )
    parser.add_argument(
        "--trace-dir",
        type=Path,
        default=Path("traces"),
        help="Directory to store raw current traces (default: ./traces)",
    )

    sub = parser.add_subparsers(dest="subcommand", required=True)

    sub.add_parser("check", help="Check INA219 current sensor (RawStatus) and DS18B20 sensors (Status 10)")
    sub.add_parser("stop", help="Immediately stop all motor channels (RawStop)")

    p_pulse = sub.add_parser("pulse", help="Run initial 1500ms Auto-Profiling test pulse on a channel")
    p_pulse.add_argument("--ch", type=int, required=True, choices=[1, 2, 3, 4], help="Valve channel (1..4)")
    p_pulse.add_argument("--dir", default="open", choices=["open", "close"], help="Initial pulse direction")
    p_pulse.add_argument("--max-ms", type=int, default=1500, help="Pulse duration in ms (default: 1500)")

    p_cal = sub.add_parser("calibrate", help="Run full 3-point calibration and save to valve_profile.json")
    p_cal.add_argument("--ch", type=int, required=True, choices=[1, 2, 3, 4], help="Valve channel (1..4)")

    p_pos = sub.add_parser("set-pos", help="Move calibrated channel to target opening percentage (0..100)")
    p_pos.add_argument("--ch", type=int, required=True, choices=[1, 2, 3, 4], help="Valve channel (1..4)")
    p_pos.add_argument("--pos", type=float, required=True, help="Target valve opening in percent (0..100)")

    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    client = TasmotaGatewayClient(host=args.host, port=args.port)

    if args.subcommand == "check":
        return run_check(client)
    if args.subcommand == "stop":
        res = client.raw_stop()
        print(json.dumps(res, indent=2))
        return 0
    if args.subcommand == "pulse":
        run_auto_profile_pulse(
            client=client,
            ch=args.ch,
            initial_dir=args.dir,
            pulse_ms=args.max_ms,
            trace_dir=args.trace_dir,
        )
        return 0
    if args.subcommand == "calibrate":
        return run_calibrate(
            client=client,
            ch=args.ch,
            profile_path=args.profile,
            trace_dir=args.trace_dir,
        )
    if args.subcommand == "set-pos":
        return run_set_position(
            client=client,
            ch=args.ch,
            target_pct=args.pos,
            profile_path=args.profile,
        )

    return 1


if __name__ == "__main__":
    sys.exit(main())
