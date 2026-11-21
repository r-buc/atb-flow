# On-device signal processing & threshold-free auto-calibration algorithms for ATB-Flow.
# Uses O(N) prefix-sum 2-segment piecewise linear regression so knee detection runs in
# a few milliseconds on ESP32-S3 without blocking the Tasmota main loop.

import math

var valve_cal = module('valve_cal')

def _min(a, b)
  return a < b ? a : b
end

def _max(a, b)
  return a > b ? a : b
end

def _round1(x)
  return math.floor(real(x) * 10.0 + 0.5) / 10.0
end

def _sort_copy(arr, start_idx, end_idx)
  var out = []
  var i = start_idx
  while i < end_idx
    out.push(real(arr[i]))
    i += 1
  end
  # Insertion sort (fast for small rolling windows of 10..60 samples)
  var n = size(out)
  i = 1
  while i < n
    var key = out[i]
    var j = i - 1
    while j >= 0 && out[j] > key
      out[j + 1] = out[j]
      j -= 1
    end
    out[j + 1] = key
    i += 1
  end
  return out
end

def _median_sorted(sorted)
  var n = size(sorted)
  if n == 0 return 0.0 end
  var mid = n / 2
  if (n % 2) == 1
    return sorted[mid]
  end
  return (sorted[mid - 1] + sorted[mid]) * 0.5
end

def robust_median_sigma(arr, start_idx, end_idx)
  var s = _sort_copy(arr, start_idx, end_idx)
  var n = size(s)
  if n == 0 return [0.0, 1.0] end
  var med = _median_sorted(s)
  var devs = []
  var i = 0
  while i < n
    devs.push(math.abs(s[i] - med))
    i += 1
  end
  var mad = _median_sorted(_sort_copy(devs, 0, n))
  var sig = _max(1.0, 1.4826 * mad)
  return [med, sig]
end

def linreg_sigma(arr, start_idx, end_idx)
  var n = end_idx - start_idx
  if n < 2
    var v = n == 1 ? real(arr[start_idx]) : 0.0
    return [v, 0.0, 1.0]
  end
  var fn = real(n)
  var sx = (fn - 1.0) * fn * 0.5
  var sxx = (fn - 1.0) * fn * (2.0 * fn - 1.0) / 6.0
  var sy = 0.0
  var sxy = 0.0
  var i = 0
  while i < n
    var y = real(arr[start_idx + i])
    sy += y
    sxy += real(i) * y
    i += 1
  end
  var denom = fn * sxx - sx * sx
  var m = denom > 1e-9 ? (fn * sxy - sx * sy) / denom : 0.0
  var b = (sy - m * sx) / fn
  var res = []
  i = 0
  while i < n
    res.push(real(arr[start_idx + i]) - (b + m * real(i)))
    i += 1
  end
  var stats = robust_median_sigma(res, 0, n)
  return [b, m, stats[1]]
end

# ---------------------------------------------------------------------------
# Live Stall Detectors (fed sample-by-sample inside _drive_tick)
# ---------------------------------------------------------------------------

class OpenStallDetector
  var trace, initial_stall_ma, _consec

  def init(initial_stall_ma)
    self.trace = []
    self.initial_stall_ma = initial_stall_ma != nil ? real(initial_stall_ma) : 45.0
    self._consec = 0
  end

  def feed(sample_ma)
    var val = real(sample_ma)
    var i = size(self.trace)
    self.trace.push(val)

    # Immediate stall check if plunger is already at the upper mechanical stop
    if i >= 15 && i < 20
      var m1 = _median_sorted(_sort_copy(self.trace, 5, 10))
      var m2 = _median_sorted(_sort_copy(self.trace, i - 5, i + 1))
      if m2 >= self.initial_stall_ma && m2 >= m1 - 2.0
        return {
          "reason": "AUTO_STALL_OPEN_INITIAL",
          "sample_idx": i,
          "current_ma": _round1(val),
          "baseline_ma": _round1(m2),
          "threshold_ma": _round1(self.initial_stall_ma)
        }
      end
    end

    if i >= 20
      var w_start = _max(10, i - 65)
      var w_end = i - 4
      if (w_end - w_start) >= 6
        var st = robust_median_sigma(self.trace, w_start, w_end)
        var med = st[0]
        var sig = st[1]
        var thresh = _max(2.0 * med, med + _max(15.0, 4.5 * sig))
        if val >= thresh
          self._consec += 1
          if self._consec >= 2
            return {
              "reason": "AUTO_STALL_OPEN",
              "sample_idx": i,
              "current_ma": _round1(val),
              "baseline_ma": _round1(med),
              "sigma_ma": _round1(sig),
              "threshold_ma": _round1(thresh)
            }
          end
        else
          self._consec = 0
        end
      end
    end
    return nil
  end

  def baseline_stats()
    var n = size(self.trace)
    if n > 30
      return robust_median_sigma(self.trace, 20, n - 5)
    elif n > 10
      return robust_median_sigma(self.trace, 5, n - 2)
    end
    return robust_median_sigma(self.trace, 0, n)
  end
end

class CloseSeatDetector
  var trace, arm_delta_ma, min_ramp_slope, min_excess_ma, _consec

  def init(arm_delta_ma, min_ramp_slope, min_excess_ma)
    self.trace = []
    self.arm_delta_ma = arm_delta_ma != nil ? real(arm_delta_ma) : 12.0
    self.min_ramp_slope = min_ramp_slope != nil ? real(min_ramp_slope) : 0.05
    self.min_excess_ma = min_excess_ma != nil ? real(min_excess_ma) : 12.0
    self._consec = 0
  end

  def feed(sample_ma)
    var val = real(sample_ma)
    var i = size(self.trace)
    self.trace.push(val)

    if i < 45 return nil end

    var base_end = _min(i - 35, 45)
    if base_end <= 10
      base_end = _min(i - 5, 35)
    end
    var base_st = robust_median_sigma(self.trace, 10, base_end)
    var base_med = base_st[0]

    var w_start = i - 34
    var w_end = i - 4
    var w_sum = 0.0
    var j = w_start
    while j < w_end
      w_sum += self.trace[j]
      j += 1
    end
    var win_mean = w_sum / real(w_end - w_start)

    var lr = linreg_sigma(self.trace, w_start, w_end)
    var b = lr[0]
    var m = lr[1]
    var rsig = lr[2]

    if win_mean < base_med + self.arm_delta_ma || m < self.min_ramp_slope
      self._consec = 0
      return nil
    end

    var pred = b + m * 34.0
    var excess = val - pred
    var thresh_excess = _max(self.min_excess_ma, 4.5 * rsig)
    if excess >= thresh_excess
      self._consec += 1
      if self._consec >= 2
        return {
          "reason": "AUTO_SEAT_CLOSE",
          "sample_idx": i,
          "current_ma": _round1(val),
          "pred_ma": _round1(pred),
          "excess_ma": _round1(excess),
          "threshold_excess_ma": _round1(thresh_excess)
        }
      end
    else
      self._consec = 0
    end
    return nil
  end
end

# ---------------------------------------------------------------------------
# O(N) Prefix-Sum 2-Segment Piecewise Linear Knee Detectors
# ---------------------------------------------------------------------------

def detect_touch_knee(trace_ma, elapsed_ms)
  var n_full = size(trace_ma)
  if elapsed_ms <= 0 || n_full < 25 return nil end

  var dt_ms = real(elapsed_ms) / real(n_full)
  var start_idx = _min(int(math.floor(5000.0 / dt_ms + 0.5)), _max(10, n_full / 8))
  var end_idx = _min(n_full, _max(start_idx + 16, n_full - 3))
  if end_idx < start_idx + 15 return nil end

  var cnt = real(end_idx - start_idx)
  # Prefix sums over [start_idx .. end_idx - 1]
  var py = [0.0]
  var piy = [0.0]
  var syy = 0.0
  var i = start_idx
  while i < end_idx
    var y = real(trace_ma[i])
    py.push(py[-1] + y)
    piy.push(piy[-1] + real(i) * y)
    syy += y * y
    i += 1
  end
  var sy = py[-1]

  var best_sse = 1e30
  var best_k = -1
  var best_c = 0.0
  var best_m = 0.0

  var k = start_idx + 5
  var k_max = end_idx - 10
  while k < k_max
    var r_len = real(end_idx - 1 - k)
    var sx = r_len * (r_len + 1.0) * 0.5
    var sxx = r_len * (r_len + 1.0) * (2.0 * r_len + 1.0) / 6.0
    var p_off = (k + 1) - start_idx
    var sum_y_right = py[-1] - py[p_off]
    var sum_iy_right = piy[-1] - piy[p_off]
    var sxy = sum_iy_right - real(k) * sum_y_right
    var denom = cnt * sxx - sx * sx
    if denom > 1e-9
      var m = (cnt * sxy - sx * sy) / denom
      if m > 0.05
        var c = (sy - m * sx) / cnt
        var sse = syy - c * sy - m * sxy
        if sse < best_sse
          best_sse = sse
          best_k = k
          best_c = c
          best_m = m
        end
      end
    end
    k += 1
  end

  if best_k < 0 return nil end
  if best_sse < 0.0 best_sse = 0.0 end
  var rmse = math.sqrt(best_sse / cnt)
  var t_touch_ms = int(math.floor(real(best_k) * dt_ms + 0.5))
  var seat_pred = best_c + best_m * real(n_full - 1 - best_k)
  return {
    "touch_idx": best_k,
    "t_touch_ms": t_touch_ms,
    "dt_ms": _round1(dt_ms),
    "i_free_close_ma": _round1(best_c),
    "ramp_slope_ma_per_s": _round1(best_m * 1000.0 / dt_ms),
    "rmse_ma": _round1(rmse),
    "seat_pred_ma": _round1(seat_pred)
  }
end

def detect_open_liftoff_knee(trace_ma, elapsed_ms)
  var n_full = size(trace_ma)
  if elapsed_ms <= 0 || n_full < 30 return nil end

  var dt_ms = real(elapsed_ms) / real(n_full)
  var start_idx = 3
  var tail_trim = _min(int(math.floor(6000.0 / dt_ms + 0.5)), _max(5, n_full / 5))
  var end_idx = n_full - tail_trim
  if end_idx < start_idx + 20 return nil end

  var cnt = real(end_idx - start_idx)
  var py = [0.0]
  var piy = [0.0]
  var syy = 0.0
  var i = start_idx
  while i < end_idx
    var y = real(trace_ma[i])
    py.push(py[-1] + y)
    piy.push(piy[-1] + real(i) * y)
    syy += y * y
    i += 1
  end
  var sy = py[-1]

  var best_sse = 1e30
  var best_k = -1
  var best_c = 0.0
  var best_m = 0.0

  var k = start_idx + 10
  var k_max = end_idx - 10
  while k < k_max
    var l_len = real(k - start_idx)
    var sx = l_len * (l_len + 1.0) * 0.5
    var sxx = l_len * (l_len + 1.0) * (2.0 * l_len + 1.0) / 6.0
    var p_off = k - start_idx
    var sum_y_left = py[p_off]
    var sum_iy_left = piy[p_off]
    var sxy = real(k) * sum_y_left - sum_iy_left
    var denom = cnt * sxx - sx * sx
    if denom > 1e-9
      var m = (cnt * sxy - sx * sy) / denom
      if m > 0.05
        var c = (sy - m * sx) / cnt
        var sse = syy - c * sy - m * sxy
        if sse < best_sse
          best_sse = sse
          best_k = k
          best_c = c
          best_m = m
        end
      end
    end
    k += 1
  end

  if best_k < 0 return nil end
  if best_sse < 0.0 best_sse = 0.0 end
  var rmse = math.sqrt(best_sse / cnt)
  var t_hub_auf_ms = int(math.floor(real(best_k) * dt_ms + 0.5))
  var t_leer_open_ms = _max(0, int(elapsed_ms) - t_hub_auf_ms)
  return {
    "liftoff_idx": best_k,
    "t_hub_auf_ms": t_hub_auf_ms,
    "t_leer_open_ms": t_leer_open_ms,
    "dt_ms": _round1(dt_ms),
    "i_free_open_ma": _round1(best_c),
    "unload_slope_ma_per_s": _round1(best_m * 1000.0 / dt_ms),
    "rmse_ma": _round1(rmse)
  }
end

valve_cal.robust_median_sigma = robust_median_sigma
valve_cal.linreg_sigma = linreg_sigma
valve_cal.OpenStallDetector = OpenStallDetector
valve_cal.CloseSeatDetector = CloseSeatDetector
valve_cal.detect_touch_knee = detect_touch_knee
valve_cal.detect_open_liftoff_knee = detect_open_liftoff_knee

return valve_cal
