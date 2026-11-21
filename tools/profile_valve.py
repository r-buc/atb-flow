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
    skip_samples = int(math.ceil(inrush_ms / float(sample_ms))) + 1
    start_idx = skip_samples + window_size - 1

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


def robust_median_sigma(values: List[float]) -> Tuple[float, float]:
    """Return (median, robust_sigma) using MAD (1.4826 * MAD, floored at 1.0 mA)."""
    if not values:
        raise ValueError("values must not be empty")
    med = statistics.median(values)
    mad = statistics.median([abs(x - med) for x in values])
    return float(med), max(1.0, 1.4826 * float(mad))


def linear_regression_with_sigma(values: List[float]) -> Tuple[float, float, float]:
    """
    Ordinary least squares fit y = b + m * idx for idx = 0..len(values)-1.
    Returns (intercept b, slope m, robust residual sigma).
    """
    n = len(values)
    if n < 2:
        val = float(values[0]) if n == 1 else 0.0
        return val, 0.0, 1.0
    sx = (n - 1) * n / 2.0
    sxx = (n - 1) * n * (2 * n - 1) / 6.0
    sy = sum(values)
    sxy = sum(idx * y for idx, y in enumerate(values))
    denom = n * sxx - sx * sx
    m = (n * sxy - sx * sy) / denom if denom > 1e-9 else 0.0
    b = (sy - m * sx) / n
    residuals = [y - (b + m * idx) for idx, y in enumerate(values)]
    _, rsig = robust_median_sigma(residuals)
    return float(b), float(m), float(rsig)


def detect_touch_point_knee(
    trace_ma: List[float],
    elapsed_ms: int,
    start_idx: Optional[int] = None,
    min_free_samples: int = 5,
    min_ramp_samples: int = 10,
    tail_trim_samples: int = 3,
) -> Optional[Dict[str, Any]]:
    """
    Threshold-free Touch-Point detection via 2-segment piecewise linear regression:
      Segment 1 (free-run, i <= k):  I(i) = c
      Segment 2 (spring,   i >  k):  I(i) = c + m * (i - k),  with m > 0

    Trims the initial inrush/adapter-hump window (~5s, capped at n/8) and final
    hard-seat spike samples so the knee point is not pulled off-center.
    Uses effective sample duration dt_ms = elapsed_ms / len(trace_ma).
    """
    n_full = len(trace_ma)
    if elapsed_ms <= 0 or n_full < 25:
        return None

    dt_ms = float(elapsed_ms) / float(n_full)
    if start_idx is None:
        start_idx = min(int(round(5000.0 / dt_ms)), max(10, n_full // 8))
    end_idx = max(start_idx + min_free_samples + min_ramp_samples + 1, n_full - tail_trim_samples)
    end_idx = min(end_idx, n_full)

    if end_idx < start_idx + min_free_samples + min_ramp_samples:
        return None

    y = [float(v) for v in trace_ma[:end_idx]]
    n = len(y)
    cnt = n - start_idx
    sy = sum(y[start_idx:])
    best_sse = float("inf")
    best: Optional[Tuple[int, float, float, float]] = None

    for k in range(start_idx + min_free_samples, n - min_ramp_samples):
        r_len = n - 1 - k  # number of samples with i > k (x = 1..r_len)
        sx = r_len * (r_len + 1) / 2.0
        sxx = r_len * (r_len + 1) * (2 * r_len + 1) / 6.0
        sxy = sum((i - k) * y[i] for i in range(k + 1, n))
        denom = cnt * sxx - sx * sx
        if denom <= 1e-9:
            continue
        m = (cnt * sxy - sx * sy) / denom
        if m <= 0.05:
            continue
        c = (sy - m * sx) / cnt
        sse = 0.0
        for i in range(start_idx, n):
            pred = c + (m * (i - k) if i > k else 0.0)
            diff = y[i] - pred
            sse += diff * diff
        if sse < best_sse:
            best_sse = sse
            best = (k, c, m, math.sqrt(sse / cnt))

    if best is None:
        return None

    k, c, m, rmse = best
    t_touch_ms = int(round(k * dt_ms))
    seat_pred_ma = c + m * (n_full - 1 - k)
    return {
        "touch_idx": k,
        "t_touch_ms": t_touch_ms,
        "dt_ms": round(dt_ms, 2),
        "i_free_close_ma": round(c, 3),
        "ramp_slope_ma_per_sample": round(m, 4),
        "ramp_slope_ma_per_s": round(m * 1000.0 / dt_ms, 3),
        "rmse_ma": round(rmse, 3),
        "seat_pred_ma": round(seat_pred_ma, 2),
    }


def detect_open_liftoff_knee(
    trace_ma: List[float],
    elapsed_ms: int,
    start_idx: int = 3,
    min_ramp_samples: int = 10,
    min_free_samples: int = 10,
) -> Optional[Dict[str, Any]]:
    """
    Threshold-free lift-off detection on the opening stroke via descending
    2-segment piecewise linear regression:
      Segment 1 (spring unloading, i <  k):  I(i) = c + m * (k - i),  with m > 0
      Segment 2 (free-run plateau, i >= k):  I(i) = c

    Directly measures T_hub_auf (the spring-assisted working stroke duration)
    and t_leer_open = elapsed_ms - T_hub_auf for cross-checking against t_touch_close.
    """
    n_full = len(trace_ma)
    if elapsed_ms <= 0 or n_full < 30:
        return None

    dt_ms = float(elapsed_ms) / float(n_full)
    # Trim the final ~6s (upper adapter hump + mechanical stop)
    tail_trim = min(int(round(6000.0 / dt_ms)), max(5, n_full // 5))
    end_idx = n_full - tail_trim
    if end_idx < start_idx + min_ramp_samples + min_free_samples:
        return None

    y = [float(v) for v in trace_ma[:end_idx]]
    n = len(y)
    cnt = n - start_idx
    sy = sum(y[start_idx:])
    best_sse = float("inf")
    best: Optional[Tuple[int, float, float, float]] = None

    for k in range(start_idx + min_ramp_samples, n - min_free_samples):
        l_len = k - start_idx
        sx = l_len * (l_len + 1) / 2.0
        sxx = l_len * (l_len + 1) * (2 * l_len + 1) / 6.0
        sxy = sum((k - i) * y[i] for i in range(start_idx, k))
        denom = cnt * sxx - sx * sx
        if denom <= 1e-9:
            continue
        m = (cnt * sxy - sx * sy) / denom
        if m <= 0.05:
            continue
        c = (sy - m * sx) / cnt
        sse = 0.0
        for i in range(start_idx, n):
            pred = c + (m * (k - i) if i < k else 0.0)
            diff = y[i] - pred
            sse += diff * diff
        if sse < best_sse:
            best_sse = sse
            best = (k, c, m, math.sqrt(sse / cnt))

    if best is None:
        return None

    k, c, m, rmse = best
    t_hub_auf_ms = int(round(k * dt_ms))
    t_leer_open_ms = max(0, int(elapsed_ms) - t_hub_auf_ms)
    return {
        "liftoff_idx": k,
        "t_hub_auf_ms": t_hub_auf_ms,
        "t_leer_open_ms": t_leer_open_ms,
        "dt_ms": round(dt_ms, 2),
        "i_free_open_ma": round(c, 3),
        "unload_slope_ma_per_s": round(m * 1000.0 / dt_ms, 3),
        "rmse_ma": round(rmse, 3),
    }


class LiveOpenStallDetector:
    """
    Threshold-free live stall detector for opening strokes:
    - Ignores breakaway/inrush phase automatically via a rolling baseline window
      excluding the most recent 4 samples.
    - Detects immediate stall if already at the upper mechanical stop.
    - Triggers on 2 consecutive samples >= max(2.0 * median, median + max(15, 4.5 * sigma_rob)).
    """

    def __init__(self, initial_stall_ma: float = 45.0) -> None:
        self.trace: List[float] = []
        self.initial_stall_ma = float(initial_stall_ma)
        self._consec = 0

    def feed(self, sample_ma: float) -> Optional[Dict[str, Any]]:
        val = float(sample_ma)
        i = len(self.trace)
        self.trace.append(val)

        # Already jammed at upper stop during initial homing (not decaying)
        if 15 <= i < 20:
            m1 = statistics.median(self.trace[5:10])
            m2 = statistics.median(self.trace[i - 5 : i + 1])
            if m2 >= self.initial_stall_ma and m2 >= m1 - 2.0:
                return {
                    "reason": "AUTO_STALL_OPEN_INITIAL",
                    "sample_idx": i,
                    "current_ma": round(val, 2),
                    "baseline_ma": round(m2, 2),
                    "threshold_ma": round(self.initial_stall_ma, 2),
                }

        if i >= 20:
            w_start = max(10, i - 65)
            w_end = i - 4
            if w_end - w_start >= 6:
                med, sig = robust_median_sigma(self.trace[w_start:w_end])
                thresh = max(2.0 * med, med + max(15.0, 4.5 * sig))
                if val >= thresh:
                    self._consec += 1
                    if self._consec >= 2:
                        return {
                            "reason": "AUTO_STALL_OPEN",
                            "sample_idx": i,
                            "current_ma": round(val, 2),
                            "baseline_ma": round(med, 2),
                            "sigma_ma": round(sig, 2),
                            "threshold_ma": round(thresh, 2),
                        }
                else:
                    self._consec = 0
        return None

    def baseline_stats(self) -> Tuple[float, float]:
        """Compute robust (median, sigma) from the settled free-run portion of the open trace."""
        if len(self.trace) > 30:
            return robust_median_sigma(self.trace[20:-5])
        if len(self.trace) > 10:
            return robust_median_sigma(self.trace[5:-2])
        return robust_median_sigma(self.trace)


class LiveCloseSeatDetector:
    """
    Threshold-free live hard-seat detector for closing strokes:
    - Ignores inrush, free-run plateau, and the initial Touch-Point jump (valve spring preload).
    - Arms only once the entire preceding 30-sample regression window is already
      on the rising spring ramp (`win_mean >= base_med + 12 mA` and `slope >= 0.05 mA/sample`).
    - Triggers when 2 consecutive samples jump >= max(12 mA, 4.5 * residual_sigma)
      above the linear ramp extrapolation (indicating hard valve seat contact).
    """

    def __init__(self, arm_delta_ma: float = 12.0, min_ramp_slope: float = 0.05, min_excess_ma: float = 12.0) -> None:
        self.trace: List[float] = []
        self.arm_delta_ma = float(arm_delta_ma)
        self.min_ramp_slope = float(min_ramp_slope)
        self.min_excess_ma = float(min_excess_ma)
        self._consec = 0

    def feed(self, sample_ma: float) -> Optional[Dict[str, Any]]:
        val = float(sample_ma)
        i = len(self.trace)
        self.trace.append(val)

        if i < 45:
            return None

        base_end = min(i - 35, 45)
        if base_end <= 10:
            base_end = min(i - 5, 35)
        base_med, _ = robust_median_sigma(self.trace[10:base_end])

        win = self.trace[i - 34 : i - 4]
        win_mean = sum(win) / float(len(win))
        b, m, rsig = linear_regression_with_sigma(win)
        if win_mean < base_med + self.arm_delta_ma or m < self.min_ramp_slope:
            self._consec = 0
            return None

        pred = b + m * 34.0
        excess = val - pred
        thresh_excess = max(self.min_excess_ma, 4.5 * rsig)
        if excess >= thresh_excess:
            self._consec += 1
            if self._consec >= 2:
                return {
                    "reason": "AUTO_SEAT_CLOSE",
                    "sample_idx": i,
                    "current_ma": round(val, 2),
                    "pred_ma": round(pred, 2),
                    "excess_ma": round(excess, 2),
                    "threshold_excess_ma": round(thresh_excess, 2),
                }
        else:
            self._consec = 0
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
        max_ms = min(60000, int(float(cal["t_close_ms"]) * 1.35))
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
        max_ms = min(60000, int(float(cal["t_open_total_ms"]) * 1.35))
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
    sample_ms: float = 25.0,
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
        t_ms = int(round(idx * float(sample_ms)))
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

    def raw_result(self, offset: int = 0, limit: int = 200) -> Dict[str, Any]:
        payload = json.dumps({"offset": int(offset), "limit": int(limit)}, separators=(",", ":"))
        res = self.send_command(f"RawResult {payload}")
        if "RawResult" not in res:
            raise RuntimeError(f"Unexpected RawResult response: {res}")
        return res["RawResult"]

    def raw_drive(
        self,
        ch: int,
        direction: str,
        max_ms: int = 5000,
        cutoff_ma: float = 150.0,
        inrush_ms: int = 100,
        sample_ms: int = 50,
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

        drive_info = res["RawDrive"]
        if drive_info.get("status") != "STARTED":
            return drive_info

        deadline = time.monotonic() + (max_ms / 1000.0) + 15.0
        chunk_limit = 200
        while True:
            if time.monotonic() > deadline:
                self.raw_stop()
                raise RuntimeError(f"Timed out waiting for RawResult after {max_ms} ms")
            rr = self.raw_result(offset=0, limit=chunk_limit)
            status = rr.get("status")
            if status == "DONE":
                break
            if status == "IDLE":
                raise RuntimeError("Gateway reported IDLE while waiting for RawDrive result")
            time.sleep(0.25)

        trace_ma = list(rr.get("trace_ma", []))
        trace_len = int(rr.get("trace_len", len(trace_ma)))
        while len(trace_ma) < trace_len:
            next_rr = self.raw_result(offset=len(trace_ma), limit=chunk_limit)
            chunk = next_rr.get("trace_ma", [])
            if not chunk:
                break
            trace_ma.extend(chunk)

        rr["trace_ma"] = trace_ma
        return rr

    def raw_drive_auto(
        self,
        ch: int,
        direction: str,
        detector: Any,
        safe_max_ma: float = 120.0,
        max_ms: int = 60000,
        inrush_ms: int = 500,
        sample_ms: int = 50,
        poll_s: float = 0.15,
        live_output: bool = False,
    ) -> Dict[str, Any]:
        """
        Start RawDrive with a hardware safety limit (`safe_max_ma`) and stream
        `trace_ma` chunks live into `detector.feed(val)`. Immediately issues
        `RawStop` when the detector signals an automatic end-stop.
        """
        payload = json.dumps(
            {
                "ch": ch,
                "dir": direction,
                "max_ms": int(max_ms),
                "cutoff_ma": round(float(safe_max_ma), 1),
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

        deadline = time.monotonic() + (max_ms / 1000.0) + 15.0
        chunk_limit = 200
        trace_ma: List[float] = []
        auto_stop: Optional[Dict[str, Any]] = None
        rr: Dict[str, Any] = {}

        while True:
            if time.monotonic() > deadline:
                self.raw_stop()
                raise RuntimeError(f"Timed out waiting for RawResult after {max_ms} ms")

            rr = self.raw_result(offset=len(trace_ma), limit=chunk_limit)
            status = rr.get("status")
            if status == "IDLE":
                raise RuntimeError("Gateway reported IDLE while waiting for RawDrive result")

            chunk = rr.get("trace_ma", [])
            for val in chunk:
                fval = float(val)
                idx = len(trace_ma)
                trace_ma.append(fval)
                if live_output:
                    print(f"  {idx:4d} | {fval:6.1f} mA", flush=True)
                if auto_stop is None:
                    hit = detector.feed(fval)
                    if hit is not None:
                        auto_stop = hit
                        self.raw_stop()
                        break

            if auto_stop is not None:
                # Fetch final metadata & any remaining samples after RawStop
                time.sleep(0.1)
                rr = self.raw_result(offset=len(trace_ma), limit=chunk_limit)
                tail = rr.get("trace_ma", [])
                trace_ma.extend(float(v) for v in tail)
                break

            if status == "DONE" and len(trace_ma) >= int(rr.get("trace_len", len(trace_ma))):
                break

            if len(chunk) < chunk_limit:
                time.sleep(poll_s)

        rr["trace_ma"] = trace_ma
        if auto_stop is not None:
            rr["stop_reason"] = auto_stop["reason"]
            rr["auto_stop"] = auto_stop
        return rr

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
    sample_ms: int = 50,
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
    safe_max_ma: float = 120.0,
    sample_ms: int = 50,
    max_ms: int = 60000,
    live_output: bool = False,
) -> int:
    print(
        f"=== Automatic Threshold-Free Valve Calibration: Channel {ch} on {client.host} "
        f"(safe_max={safe_max_ma:.1f} mA) ==="
    )

    # 0. Homing to upper mechanical stop (auto-detects stall or already-at-top)
    print("\n[Step 0] Homing to TOP (dir='open', auto stall detection)...")
    home_det = LiveOpenStallDetector()
    res_home = client.raw_drive_auto(
        ch=ch,
        direction="open",
        detector=home_det,
        safe_max_ma=safe_max_ma,
        max_ms=max_ms,
        inrush_ms=500,
        sample_ms=sample_ms,
        live_output=live_output,
    )
    save_trace(trace_dir, "homing_open", res_home)
    print(
        f"  -> Homing stopped: reason={res_home['stop_reason']}, "
        f"elapsed={res_home['elapsed_ms']} ms, peak={res_home['peak_ma']:.1f} mA, "
        f"samples={len(res_home.get('trace_ma', []))}"
    )
    time.sleep(0.5)

    # 1. Full closing stroke: auto-detects hard valve seat via dI/dt spike over linear ramp
    print("\n[Step 1] Full closing stroke (dir='close', auto seat detection)...")
    close_det = LiveCloseSeatDetector()
    res_close = client.raw_drive_auto(
        ch=ch,
        direction="close",
        detector=close_det,
        safe_max_ma=safe_max_ma,
        max_ms=max_ms,
        inrush_ms=500,
        sample_ms=sample_ms,
        live_output=live_output,
    )
    close_file = save_trace(trace_dir, "cal_close", res_close)
    close_trace = res_close.get("trace_ma", [])
    t_close_ms = int(res_close.get("elapsed_ms", len(close_trace) * sample_ms))
    dt_close_ms = float(t_close_ms) / max(1, len(close_trace))
    print(
        f"  -> Close stopped: reason={res_close['stop_reason']}, "
        f"elapsed={t_close_ms} ms, peak={res_close['peak_ma']:.1f} mA, "
        f"samples={len(close_trace)} (dt={dt_close_ms:.1f} ms)"
    )

    knee = detect_touch_point_knee(trace_ma=close_trace, elapsed_ms=t_close_ms)
    if knee is None:
        print("\n" + render_ascii_trace(close_trace, sample_ms=dt_close_ms))
        raise RuntimeError(
            "Could not fit a Touch-Point knee on closing stroke! "
            f"Is the actuator mounted on a valve? Trace saved to {close_file}"
        )

    touch_idx = int(knee["touch_idx"])
    t_touch_ms = int(knee["t_touch_ms"])
    i_free_close_ma = float(knee["i_free_close_ma"])
    # Learn permanent close cutoff from predicted ramp end (or live auto-stop pred)
    seat_pred_ma = float(res_close.get("auto_stop", {}).get("pred_ma", knee["seat_pred_ma"]))
    i_stall_close_ma = round(min(safe_max_ma, max(seat_pred_ma + 8.0, seat_pred_ma * 1.08)), 1)

    print(
        f"  -> Knee Touch-Point (u=100%): t_touch={t_touch_ms} ms (sample #{touch_idx}), "
        f"I_free_close={i_free_close_ma:.2f} mA, slope={knee['ramp_slope_ma_per_s']:.2f} mA/s, "
        f"RMSE={knee['rmse_ma']:.2f} mA"
    )
    print(f"  -> Learned I_stall_close = {i_stall_close_ma:.1f} mA (seat_pred={seat_pred_ma:.1f} mA)")
    print("\n--- Closing Stroke Current Profile ---")
    print(
        render_ascii_trace(
            close_trace,
            sample_ms=dt_close_ms,
            touch_idx=touch_idx,
            threshold_ma=i_stall_close_ma,
        )
    )
    time.sleep(0.5)

    # 2. Full opening stroke: measure t_open_total and learn I_free_open / I_stall_open
    print("\n[Step 2] Full opening stroke (dir='open', auto stall detection)...")
    open_det = LiveOpenStallDetector()
    res_open = client.raw_drive_auto(
        ch=ch,
        direction="open",
        detector=open_det,
        safe_max_ma=safe_max_ma,
        max_ms=max_ms,
        inrush_ms=500,
        sample_ms=sample_ms,
        live_output=live_output,
    )
    save_trace(trace_dir, "cal_open", res_open)
    open_trace = res_open.get("trace_ma", [])
    t_open_total_ms = int(res_open.get("elapsed_ms", len(open_trace) * sample_ms))
    dt_open_ms = float(t_open_total_ms) / max(1, len(open_trace))
    i_free_open_ma, sigma_open_ma = open_det.baseline_stats()
    i_stall_open_ma = round(max(2.1 * i_free_open_ma, i_free_open_ma + max(18.0, 4.5 * sigma_open_ma)), 1)

    open_knee = detect_open_liftoff_knee(trace_ma=open_trace, elapsed_ms=t_open_total_ms)
    liftoff_idx = int(open_knee["liftoff_idx"]) if open_knee else None

    print(
        f"  -> Open stopped: reason={res_open['stop_reason']}, "
        f"elapsed={t_open_total_ms} ms, peak={res_open['peak_ma']:.1f} mA, "
        f"samples={len(open_trace)} (dt={dt_open_ms:.1f} ms)"
    )
    print(
        f"  -> Learned I_free_open = {i_free_open_ma:.2f} mA (sigma={sigma_open_ma:.2f} mA), "
        f"I_stall_open = {i_stall_open_ma:.1f} mA"
    )
    if open_knee:
        leer_diff_ms = abs(t_touch_ms - int(open_knee["t_leer_open_ms"]))
        print(
            f"  -> Open Lift-Off Knee (u=100%): T_hub_auf={open_knee['t_hub_auf_ms']} ms "
            f"(sample #{liftoff_idx}), t_leer_open={open_knee['t_leer_open_ms']} ms, "
            f"slope=-{open_knee['unload_slope_ma_per_s']:.2f} mA/s, RMSE={open_knee['rmse_ma']:.2f} mA"
        )
        print(
            f"  -> Cross-check free-run gap: close={t_touch_ms} ms vs open={open_knee['t_leer_open_ms']} ms "
            f"(diff={leer_diff_ms} ms)"
        )
    print("\n--- Opening Stroke Current Profile ---")
    print(
        render_ascii_trace(
            open_trace,
            sample_ms=dt_open_ms,
            touch_idx=liftoff_idx,
            threshold_ma=i_stall_open_ma,
        )
    )

    # 3. Position plunger from upper stop down to Touch-Point (u = 100%)
    if t_touch_ms > 0:
        print(f"\n[Step 3] Positioning plunger from upper stop to Touch-Point (u=100%, {t_touch_ms} ms)...")
        time.sleep(0.4)
        client.raw_drive(
            ch=ch,
            direction="close",
            max_ms=t_touch_ms,
            cutoff_ma=i_stall_close_ma,
            inrush_ms=1000,
            sample_ms=sample_ms,
        )

    cal_params = compute_calibration_parameters(
        t_touch_ms=t_touch_ms,
        t_close_ms=t_close_ms,
        t_open_total_ms=t_open_total_ms,
    )
    if open_knee:
        # Use directly measured spring-assisted opening stroke duration instead of eta_work * t_open_total
        cal_params["t_hub_auf_ms"] = float(open_knee["t_hub_auf_ms"])
        cal_params["t_leer_open_ms"] = int(open_knee["t_leer_open_ms"])

    channel_record = {
        "i_free_open_ma": round(i_free_open_ma, 3),
        "sigma_open_ma": round(sigma_open_ma, 3),
        "i_free_close_ma": round(i_free_close_ma, 3),
        "ramp_slope_ma_per_s": knee["ramp_slope_ma_per_s"],
        "knee_rmse_ma": knee["rmse_ma"],
        "open_knee_rmse_ma": open_knee["rmse_ma"] if open_knee else None,
        "i_stall_open_ma": i_stall_open_ma,
        "i_stall_close_ma": i_stall_close_ma,
        "safe_max_ma": safe_max_ma,
        "dt_close_ms": round(dt_close_ms, 2),
        "dt_open_ms": round(dt_open_ms, 2),
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
    sample_ms: int = 50,
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
    # Opening out of a closed valve seat needs longer breakaway blanking (~2s)
    eff_inrush_ms = 2000 if (plan["dir"] == "open" and u_old <= 5.0) else max(inrush_ms, 1000)
    res = client.raw_drive(
        ch=ch,
        direction=plan["dir"],
        max_ms=plan["max_ms"],
        cutoff_ma=plan["cutoff_ma"],
        inrush_ms=eff_inrush_ms,
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

    p_cal = sub.add_parser(
        "calibrate",
        help="Run automatic threshold-free valve calibration and save to valve_profile.json",
    )
    p_cal.add_argument("--ch", type=int, required=True, choices=[1, 2, 3, 4], help="Valve channel (1..4)")
    p_cal.add_argument(
        "--safe-max-ma",
        type=float,
        default=120.0,
        help="Hardware safety cutoff current in mA (default: 120.0)",
    )
    p_cal.add_argument("--max-ms", type=int, default=60000, help="Max drive duration per stroke in ms (default: 60000)")
    p_cal.add_argument("--live", action="store_true", help="Print current samples live line-by-line during calibration")

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
            safe_max_ma=args.safe_max_ma,
            max_ms=args.max_ms,
            live_output=args.live,
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
