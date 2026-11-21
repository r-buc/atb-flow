import json
import string
import math
import gpio
import valve_cal

class DumbValveGateway
  var wire, ina_addr, pin_stby, ch_pins, is_moving
  var _drive, _run_id
  var profile_path, profiles, cal_state

  def init()
    self.ina_addr = 0x40
    self.pin_stby = 14
    self.ch_pins = {1: [5, 6], 2: [7, 15], 3: [16, 17], 4: [18, 21]}
    self.is_moving = false
    self._drive = nil
    self._run_id = 0
    self.profile_path = "valve_profile.json"
    self.profiles = {}
    self.cal_state = {"state": "IDLE", "ch": 0, "error": nil}

    gpio.pin_mode(self.pin_stby, gpio.OUTPUT)
    gpio.digital_write(self.pin_stby, gpio.LOW)

    for ch : self.ch_pins.keys()
      for p : self.ch_pins[ch]
        gpio.pin_mode(p, gpio.OUTPUT)
        gpio.digital_write(p, gpio.LOW)
      end
    end

    self._load_profiles()
    self.wire = tasmota.wire_scan(self.ina_addr)
    tasmota.add_cmd('RawDrive',    /cmd, idx, payload -> self.cmd_raw_drive(payload))
    tasmota.add_cmd('RawStop',     /cmd, idx, payload -> self.cmd_raw_stop())
    tasmota.add_cmd('RawStatus',   /cmd, idx, payload -> self.cmd_raw_status())
    tasmota.add_cmd('RawResult',   /cmd, idx, payload -> self.cmd_raw_result(payload))
    tasmota.add_cmd('ValveCal',    /cmd, idx, payload -> self.cmd_valve_cal(payload))
    tasmota.add_cmd('ValvePos',    /cmd, idx, payload -> self.cmd_valve_pos(payload))
    tasmota.add_cmd('ValveStatus', /cmd, idx, payload -> self.cmd_valve_status(payload))
  end

  def _load_profiles()
    try
      var f = open(self.profile_path, 'r')
      var raw = f.read()
      f.close()
      var data = json.load(raw)
      if data != nil && type(data) == 'instance' && classname(data) == 'map'
        self.profiles = data
      end
    except ..
      self.profiles = {}
    end
  end

  def _save_profiles()
    try
      var f = open(self.profile_path, 'w')
      f.write(json.dump(self.profiles))
      f.close()
    except .. as e, msg
      log(string.format("GW: Failed to save %s [%s]: %s", self.profile_path, str(e), str(msg)), 1)
    end
  end

  def stop_all()
    gpio.digital_write(self.pin_stby, gpio.LOW)
    for ch : self.ch_pins.keys()
      gpio.digital_write(self.ch_pins[ch][0], gpio.LOW)
      gpio.digital_write(self.ch_pins[ch][1], gpio.LOW)
    end
    self.is_moving = false
  end

  def read_current_ma()
    if !self.wire
      self.wire = tasmota.wire_scan(self.ina_addr)
      if !self.wire return -1.0 end
    end
    var raw = self.wire.read(self.ina_addr, 0x01, 2)
    if raw == nil return -1.0 end
    if raw > 32767 raw -= 65536 end
    return math.abs(real(raw) * 0.1)
  end

  def _finish_drive(reason, elapsed)
    self.stop_all()
    var d = self._drive
    if d != nil
      d["stop_reason"] = reason
      d["elapsed_ms"] = elapsed
      d["done"] = true
      var cb = d["on_done"]
      d["on_done"] = nil
      if cb != nil && reason != "USER_STOP"
        cb(d)
      end
    end
  end

  def _drive_tick(run_id)
    var d = self._drive
    if d == nil || d["done"] || d["run_id"] != run_id
      return
    end

    try
      var elapsed = tasmota.millis() - d["start_t"]
      var cur = self.read_current_ma()
      if cur < 0.0
        self._finish_drive("I2C_ERROR", elapsed)
        return
      end

      d["trace"].push(cur)
      if cur > d["peak_ma"] d["peak_ma"] = cur end

      if elapsed > d["inrush_ms"] && cur >= d["cutoff_ma"]
        self._finish_drive("CURRENT_CUTOFF", elapsed)
        return
      end

      var det = d["detector"]
      if det != nil
        var hit = det.feed(cur)
        if hit != nil
          d["auto_stop"] = hit
          self._finish_drive(hit["reason"], elapsed)
          return
        end
      end

      if elapsed >= d["max_ms"]
        self._finish_drive("TIME_LIMIT", elapsed)
        return
      end

      tasmota.set_timer(d["sample_ms"], / -> self._drive_tick(run_id))
    except .. as e, msg
      var elapsed = tasmota.millis() - d["start_t"]
      self._finish_drive("INTERNAL_ERROR", elapsed)
      log(string.format("GW: RawDrive error [%s]: %s", str(e), str(msg)), 1)
    end
  end

  def _start_drive(ch, dir, max_ms, cutoff_ma, inrush_ms, sample_ms, detector, on_done)
    self.stop_all()
    self._run_id += 1
    var run_id = self._run_id
    self.is_moving = true
    self._drive = {
      "ch": ch,
      "dir": dir,
      "start_t": tasmota.millis(),
      "max_ms": max_ms,
      "cutoff_ma": cutoff_ma,
      "inrush_ms": inrush_ms,
      "sample_ms": sample_ms,
      "trace": [],
      "peak_ma": 0.0,
      "stop_reason": nil,
      "elapsed_ms": 0,
      "done": false,
      "run_id": run_id,
      "detector": detector,
      "auto_stop": nil,
      "on_done": on_done
    }

    try
      var pins = self.ch_pins[ch]
      gpio.digital_write(pins[0], dir == "open" ? gpio.HIGH : gpio.LOW)
      gpio.digital_write(pins[1], dir == "open" ? gpio.LOW : gpio.HIGH)
      gpio.digital_write(self.pin_stby, gpio.HIGH)
      self._drive_tick(run_id)
    except .. as e, msg
      self._finish_drive("INTERNAL_ERROR", 0)
      log(string.format("GW: RawDrive start error [%s]: %s", str(e), str(msg)), 1)
    end
  end

  def cmd_raw_stop()
    if self.cal_state["state"] != "IDLE" && self.cal_state["state"] != "DONE" && self.cal_state["state"] != "ERROR"
      self.cal_state["state"] = "STOPPED"
      self.cal_state["error"] = "USER_STOP"
    end
    if self._drive != nil && !self._drive["done"]
      self._finish_drive("USER_STOP", tasmota.millis() - self._drive["start_t"])
    else
      self.stop_all()
    end
    tasmota.resp_cmnd(json.dump({"RawStop": "OK"}))
  end

  def cmd_raw_result(payload)
    var offset = 0
    var limit = 200
    if payload != nil && payload != ""
      var p = json.load(payload)
      if p != nil && type(p) == 'instance' && classname(p) == 'map'
        if p.contains("offset") offset = int(p["offset"]) end
        if p.contains("limit")  limit = int(p["limit"]) end
      end
    end
    if offset < 0 offset = 0 end
    if limit < 1 limit = 1 end
    if limit > 250 limit = 250 end

    var d = self._drive
    if d == nil
      tasmota.resp_cmnd(json.dump({"RawResult": {"status": "IDLE"}}))
      return
    end

    var total = size(d["trace"])
    var end_idx = offset + limit
    if end_idx > total end_idx = total end
    var chunk = []
    var i = offset
    while i < end_idx
      chunk.push(d["trace"][i])
      i += 1
    end

    if !d["done"]
      tasmota.resp_cmnd(json.dump({
        "RawResult": {
          "status": "MOVING",
          "ch": d["ch"],
          "dir": d["dir"],
          "elapsed_ms": tasmota.millis() - d["start_t"],
          "peak_ma": d["peak_ma"],
          "sample_ms": d["sample_ms"],
          "trace_len": total,
          "offset": offset,
          "trace_ma": chunk
        }
      }))
      return
    end

    tasmota.resp_cmnd(json.dump({
      "RawResult": {
        "status": "DONE",
        "ch": d["ch"],
        "dir": d["dir"],
        "stop_reason": d["stop_reason"],
        "elapsed_ms": d["elapsed_ms"],
        "peak_ma": d["peak_ma"],
        "sample_ms": d["sample_ms"],
        "trace_len": total,
        "offset": offset,
        "trace_ma": chunk
      }
    }))
  end

  def cmd_raw_status()
    var cur = self.read_current_ma()
    var channels = {}
    for ch : self.ch_pins.keys()
      channels[str(ch)] = self.ch_pins[ch]
    end
    tasmota.resp_cmnd(json.dump({
      "RawStatus": {
        "ina219_connected": self.wire != nil && cur >= 0.0,
        "ina219_addr": string.format("0x%02X", self.ina_addr),
        "idle_ma": cur,
        "is_moving": self.is_moving,
        "pin_stby": self.pin_stby,
        "ch_pins": channels
      }
    }))
  end

  def cmd_raw_drive(payload)
    if self.is_moving
      tasmota.resp_cmnd(json.dump({"error": "BUSY"}))
      return
    end
    if payload == nil || payload == ""
      tasmota.resp_cmnd(json.dump({"error": "INVALID_ARGS"}))
      return
    end
    var p = json.load(payload)
    if p == nil || type(p) != 'instance' || classname(p) != 'map' || !p.contains("ch") || !p.contains("dir")
      tasmota.resp_cmnd(json.dump({"error": "INVALID_ARGS"}))
      return
    end

    var ch = int(p["ch"])
    if !self.ch_pins.contains(ch)
      tasmota.resp_cmnd(json.dump({"error": "INVALID_CHANNEL"}))
      return
    end

    var dir = str(p["dir"])
    if dir != "open" && dir != "close"
      tasmota.resp_cmnd(json.dump({"error": "INVALID_DIR"}))
      return
    end

    var max_ms = p.contains("max_ms") ? int(p["max_ms"]) : 5000
    if max_ms < 1 max_ms = 1 end
    if max_ms > 120000 max_ms = 120000 end

    var cutoff_ma = p.contains("cutoff_ma") ? real(p["cutoff_ma"]) : 150.0
    if cutoff_ma <= 0.0 cutoff_ma = 150.0 end

    var inrush_ms = p.contains("inrush_ms") ? int(p["inrush_ms"]) : 100
    if inrush_ms < 0 inrush_ms = 0 end
    if inrush_ms > 5000 inrush_ms = 5000 end

    var sample_ms = p.contains("sample_ms") ? int(p["sample_ms"]) : 50
    if sample_ms < 10 sample_ms = 10 end
    if sample_ms > 1000 sample_ms = 1000 end

    self._start_drive(ch, dir, max_ms, cutoff_ma, inrush_ms, sample_ms, nil, nil)

    tasmota.resp_cmnd(json.dump({
      "RawDrive": {
        "status": "STARTED",
        "ch": ch,
        "dir": dir,
        "max_ms": max_ms,
        "sample_ms": sample_ms
      }
    }))
  end

  # -------------------------------------------------------------------------
  # On-Device Automatic Threshold-Free Calibration (ValveCal)
  # -------------------------------------------------------------------------

  def _cal_fail(msg)
    self.cal_state["state"] = "ERROR"
    self.cal_state["error"] = msg
    log("GW: ValveCal error: " + str(msg), 1)
  end

  def cmd_valve_cal(payload)
    if self.is_moving
      tasmota.resp_cmnd(json.dump({"error": "BUSY"}))
      return
    end

    var ch = 1
    var safe_max_ma = 120.0
    var max_ms = 60000
    var sample_ms = 50

    if payload != nil && payload != ""
      var p = json.load(payload)
      if p != nil && type(p) == 'instance' && classname(p) == 'map'
        if p.contains("ch") ch = int(p["ch"]) end
        if p.contains("safe_max_ma") safe_max_ma = real(p["safe_max_ma"]) end
        if p.contains("max_ms") max_ms = int(p["max_ms"]) end
        if p.contains("sample_ms") sample_ms = int(p["sample_ms"]) end
      else
        ch = int(payload)
      end
    end

    if !self.ch_pins.contains(ch)
      tasmota.resp_cmnd(json.dump({"error": "INVALID_CHANNEL"}))
      return
    end
    if safe_max_ma < 30.0 safe_max_ma = 30.0 end
    if max_ms < 5000 max_ms = 5000 end
    if max_ms > 120000 max_ms = 120000 end

    var ctx = {
      "ch": ch,
      "safe_max_ma": safe_max_ma,
      "max_ms": max_ms,
      "sample_ms": sample_ms
    }
    self.cal_state = {"state": "HOMING_OPEN", "ch": ch, "error": nil}
    log(string.format("GW: ValveCal ch=%d started (safe_max=%.1f mA)", ch, safe_max_ma), 2)

    var home_det = valve_cal.OpenStallDetector()
    self._start_drive(ch, "open", max_ms, safe_max_ma, 500, sample_ms, home_det,
      /d -> self._cal_after_home(ctx, d))

    tasmota.resp_cmnd(json.dump({
      "ValveCal": {
        "status": "STARTED",
        "ch": ch,
        "safe_max_ma": safe_max_ma
      }
    }))
  end

  def _cal_after_home(ctx, d)
    if d["stop_reason"] == "I2C_ERROR" || d["stop_reason"] == "INTERNAL_ERROR"
      self._cal_fail("Homing failed: " + str(d["stop_reason"]))
      return
    end
    self.cal_state["state"] = "CAL_CLOSE"
    log(string.format("GW: ValveCal ch=%d homing done (%dms, %s) -> closing",
      ctx["ch"], d["elapsed_ms"], str(d["stop_reason"])), 2)
    tasmota.set_timer(500, / -> self._cal_start_close(ctx))
  end

  def _cal_start_close(ctx)
    if self.cal_state["state"] != "CAL_CLOSE" return end
    var close_det = valve_cal.CloseSeatDetector()
    self._start_drive(ctx["ch"], "close", ctx["max_ms"], ctx["safe_max_ma"], 500, ctx["sample_ms"], close_det,
      /d -> self._cal_after_close(ctx, d))
  end

  def _cal_after_close(ctx, d)
    if d["stop_reason"] == "I2C_ERROR" || d["stop_reason"] == "INTERNAL_ERROR"
      self._cal_fail("Close stroke failed: " + str(d["stop_reason"]))
      return
    end
    var knee = valve_cal.detect_touch_knee(d["trace"], d["elapsed_ms"])
    if knee == nil
      self._cal_fail("Could not detect Touch-Point knee on closing stroke")
      return
    end

    var seat_pred = knee["seat_pred_ma"]
    if d["auto_stop"] != nil && d["auto_stop"].contains("pred_ma")
      seat_pred = d["auto_stop"]["pred_ma"]
    end
    var cand_close = seat_pred + 8.0
    if seat_pred * 1.08 > cand_close cand_close = seat_pred * 1.08 end
    if cand_close > ctx["safe_max_ma"] cand_close = ctx["safe_max_ma"] end

    ctx["t_close_ms"] = d["elapsed_ms"]
    ctx["t_touch_ms"] = knee["t_touch_ms"]
    ctx["i_free_close_ma"] = knee["i_free_close_ma"]
    ctx["ramp_slope_ma_per_s"] = knee["ramp_slope_ma_per_s"]
    ctx["knee_rmse_ma"] = knee["rmse_ma"]
    ctx["i_stall_close_ma"] = math.floor(cand_close * 10.0 + 0.5) / 10.0

    self.cal_state["state"] = "CAL_OPEN"
    log(string.format("GW: ValveCal ch=%d close done (%dms, touch=%dms, I_stall_close=%.1fmA) -> opening",
      ctx["ch"], ctx["t_close_ms"], ctx["t_touch_ms"], ctx["i_stall_close_ma"]), 2)
    tasmota.set_timer(500, / -> self._cal_start_open(ctx))
  end

  def _cal_start_open(ctx)
    if self.cal_state["state"] != "CAL_OPEN" return end
    var open_det = valve_cal.OpenStallDetector()
    ctx["open_det"] = open_det
    self._start_drive(ctx["ch"], "open", ctx["max_ms"], ctx["safe_max_ma"], 500, ctx["sample_ms"], open_det,
      /d -> self._cal_after_open(ctx, d))
  end

  def _cal_after_open(ctx, d)
    if d["stop_reason"] == "I2C_ERROR" || d["stop_reason"] == "INTERNAL_ERROR"
      self._cal_fail("Open stroke failed: " + str(d["stop_reason"]))
      return
    end

    var t_open_total = d["elapsed_ms"]
    var t_close = ctx["t_close_ms"]
    var t_touch = ctx["t_touch_ms"]
    var t_hub_zu = t_close - t_touch
    var eta_work = t_close > 0 ? real(t_hub_zu) / real(t_close) : 1.0
    var t_hub_auf = int(math.floor(eta_work * real(t_open_total) + 0.5))
    var t_leer_open = t_open_total - t_hub_auf
    var open_knee_rmse = nil

    var open_knee = valve_cal.detect_open_liftoff_knee(d["trace"], t_open_total)
    if open_knee != nil
      t_hub_auf = open_knee["t_hub_auf_ms"]
      t_leer_open = open_knee["t_leer_open_ms"]
      open_knee_rmse = open_knee["rmse_ma"]
    end

    var st_open = ctx["open_det"].baseline_stats()
    var i_free_open = math.floor(st_open[0] * 10.0 + 0.5) / 10.0
    var sig_open = math.floor(st_open[1] * 10.0 + 0.5) / 10.0
    var pad = 4.5 * sig_open
    if pad < 18.0 pad = 18.0 end
    var i_stall_open = 2.1 * i_free_open
    if i_free_open + pad > i_stall_open i_stall_open = i_free_open + pad end
    i_stall_open = math.floor(i_stall_open * 10.0 + 0.5) / 10.0

    var rec = {
      "i_free_open_ma": i_free_open,
      "sigma_open_ma": sig_open,
      "i_free_close_ma": ctx["i_free_close_ma"],
      "ramp_slope_ma_per_s": ctx["ramp_slope_ma_per_s"],
      "knee_rmse_ma": ctx["knee_rmse_ma"],
      "open_knee_rmse_ma": open_knee_rmse,
      "i_stall_open_ma": i_stall_open,
      "i_stall_close_ma": ctx["i_stall_close_ma"],
      "safe_max_ma": ctx["safe_max_ma"],
      "t_touch_ms": t_touch,
      "t_close_ms": t_close,
      "t_open_total_ms": t_open_total,
      "t_hub_zu_ms": t_hub_zu,
      "t_hub_auf_ms": t_hub_auf,
      "t_leer_open_ms": t_leer_open,
      "current_pos_pct": 100.0
    }
    self.profiles[str(ctx["ch"])] = rec
    self._save_profiles()

    if t_touch > 0
      self.cal_state["state"] = "POS_TOUCH"
      log(string.format("GW: ValveCal ch=%d open done (%dms, T_hub_auf=%dms) -> moving to touch (%dms)",
        ctx["ch"], t_open_total, t_hub_auf, t_touch), 2)
      tasmota.set_timer(400, / -> self._start_drive(
        ctx["ch"], "close", t_touch, ctx["i_stall_close_ma"], 1000, ctx["sample_ms"], nil,
        /d2 -> self._cal_done(ctx)
      ))
    else
      self._cal_done(ctx)
    end
  end

  def _cal_done(ctx)
    self.cal_state["state"] = "DONE"
    self.cal_state["error"] = nil
    log(string.format("GW: ValveCal ch=%d DONE", ctx["ch"]), 2)
  end

  # -------------------------------------------------------------------------
  # On-Device Positioning (ValvePos) & Status (ValveStatus)
  # -------------------------------------------------------------------------

  def cmd_valve_pos(payload)
    if self.is_moving
      tasmota.resp_cmnd(json.dump({"error": "BUSY"}))
      return
    end
    if payload == nil || payload == ""
      tasmota.resp_cmnd(json.dump({"error": "INVALID_ARGS"}))
      return
    end
    var p = json.load(payload)
    if p == nil || type(p) != 'instance' || classname(p) != 'map' || !p.contains("ch") || !p.contains("pos")
      tasmota.resp_cmnd(json.dump({"error": "INVALID_ARGS"}))
      return
    end

    var ch = int(p["ch"])
    var ch_key = str(ch)
    if !self.ch_pins.contains(ch)
      tasmota.resp_cmnd(json.dump({"error": "INVALID_CHANNEL"}))
      return
    end
    if !self.profiles.contains(ch_key)
      tasmota.resp_cmnd(json.dump({"error": "NOT_CALIBRATED"}))
      return
    end

    var cal = self.profiles[ch_key]
    var u_old = real(cal["current_pos_pct"])
    var u_new = real(p["pos"])
    if u_new < 0.0 u_new = 0.0 end
    if u_new > 100.0 u_new = 100.0 end
    var delta_u = u_new - u_old

    if math.abs(delta_u) < 0.1
      tasmota.resp_cmnd(json.dump({"ValvePos": {"status": "NOOP", "ch": ch, "pos": u_old}}))
      return
    end

    var dir = delta_u > 0.0 ? "open" : "close"
    var cutoff_ma = dir == "open" ? real(cal["i_stall_open_ma"]) : real(cal["i_stall_close_ma"])
    var inrush_ms = (dir == "open" && u_old <= 5.0) ? 2000 : 1000
    var max_ms = 0
    var full_sync = false

    if u_new <= 0.0
      max_ms = int(real(cal["t_close_ms"]) * 1.35)
      if max_ms > 120000 max_ms = 120000 end
      full_sync = true
    elif u_new >= 100.0
      max_ms = int(real(cal["t_open_total_ms"]) * 1.35)
      if max_ms > 120000 max_ms = 120000 end
      full_sync = true
    else
      var stroke = dir == "open" ? real(cal["t_hub_auf_ms"]) : real(cal["t_hub_zu_ms"])
      max_ms = int(math.floor((math.abs(delta_u) / 100.0) * stroke + 0.5))
      if max_ms < 50 max_ms = 50 end
    end

    self.cal_state = {"state": "MOVING", "ch": ch, "error": nil}
    self._start_drive(ch, dir, max_ms, cutoff_ma, inrush_ms, 50, nil,
      /d -> self._pos_done(ch_key, u_new, full_sync, cal))

    tasmota.resp_cmnd(json.dump({
      "ValvePos": {
        "status": "STARTED",
        "ch": ch,
        "from_pct": u_old,
        "target_pct": u_new,
        "dir": dir,
        "max_ms": max_ms
      }
    }))
  end

  def _pos_done(ch_key, target_pct, full_sync, cal)
    if full_sync && target_pct >= 100.0 && int(cal["t_touch_ms"]) > 0
      var t_touch = int(cal["t_touch_ms"])
      tasmota.set_timer(300, / -> self._start_drive(
        int(ch_key), "close", t_touch, real(cal["i_stall_close_ma"]), 1000, 50, nil,
        /d2 -> self._pos_finalize(ch_key, target_pct)
      ))
      return
    end
    self._pos_finalize(ch_key, target_pct)
  end

  def _pos_finalize(ch_key, target_pct)
    if self.profiles.contains(ch_key)
      self.profiles[ch_key]["current_pos_pct"] = target_pct
      self._save_profiles()
    end
    self.cal_state["state"] = "IDLE"
  end

  def cmd_valve_status(payload)
    var res = {
      "state": self.cal_state["state"],
      "active_ch": self.cal_state["ch"],
      "error": self.cal_state["error"],
      "is_moving": self.is_moving,
      "channels": self.profiles
    }
    if payload != nil && payload != ""
      var ch_key = str(int(payload))
      res["channel"] = self.profiles.contains(ch_key) ? self.profiles[ch_key] : nil
    end
    tasmota.resp_cmnd(json.dump({"ValveStatus": res}))
  end
end

return DumbValveGateway()
