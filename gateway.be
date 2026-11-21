import json
import string
import math
import gpio

class DumbValveGateway
  var wire, ina_addr, pin_stby, ch_pins, is_moving

  def init()
    self.ina_addr = 0x40
    self.pin_stby = 14
    self.ch_pins = {1: [5, 6], 2: [7, 15], 3: [16, 17], 4: [18, 21]}
    self.is_moving = false

    gpio.pin_mode(self.pin_stby, gpio.OUTPUT)
    gpio.digital_write(self.pin_stby, gpio.LOW)

    for ch : self.ch_pins.keys()
      for p : self.ch_pins[ch]
        gpio.pin_mode(p, gpio.OUTPUT)
        gpio.digital_write(p, gpio.LOW)
      end
    end

    self.wire = tasmota.wire_scan(self.ina_addr)
    tasmota.add_cmd('RawDrive',  /cmd, idx, payload -> self.cmd_raw_drive(payload))
    tasmota.add_cmd('RawStop',   /cmd, idx, payload -> self.cmd_raw_stop())
    tasmota.add_cmd('RawStatus', /cmd, idx, payload -> self.cmd_raw_status())
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

  def cmd_raw_stop()
    self.stop_all()
    tasmota.resp_cmnd(json.dump({"RawStop": "OK"}))
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
    if max_ms > 35000 max_ms = 35000 end

    var cutoff_ma = p.contains("cutoff_ma") ? real(p["cutoff_ma"]) : 150.0
    if cutoff_ma <= 0.0 cutoff_ma = 150.0 end

    var inrush_ms = p.contains("inrush_ms") ? int(p["inrush_ms"]) : 100
    if inrush_ms < 0 inrush_ms = 0 end
    if inrush_ms > 5000 inrush_ms = 5000 end

    var sample_ms = p.contains("sample_ms") ? int(p["sample_ms"]) : 25
    if sample_ms < 10 sample_ms = 10 end
    if sample_ms > 1000 sample_ms = 1000 end

    self.stop_all()
    self.is_moving = true

    var start_t = tasmota.millis()
    var trace = []
    var peak_ma = 0.0
    var stop_reason = "TIME_LIMIT"
    var elapsed = 0

    try
      var pins = self.ch_pins[ch]
      gpio.digital_write(pins[0], dir == "open" ? gpio.HIGH : gpio.LOW)
      gpio.digital_write(pins[1], dir == "open" ? gpio.LOW : gpio.HIGH)
      gpio.digital_write(self.pin_stby, gpio.HIGH)

      while (tasmota.millis() - start_t) < max_ms
        var cur = self.read_current_ma()
        if cur < 0.0
          stop_reason = "I2C_ERROR"
          break
        end
        trace.push(cur)
        if cur > peak_ma peak_ma = cur end
        if (tasmota.millis() - start_t) > inrush_ms && cur >= cutoff_ma
          stop_reason = "CURRENT_CUTOFF"
          break
        end
        tasmota.delay(sample_ms)
      end
      elapsed = tasmota.millis() - start_t
    except .. as e, msg
      stop_reason = "INTERNAL_ERROR"
      elapsed = tasmota.millis() - start_t
      log(string.format("GW: RawDrive error [%s]: %s", str(e), str(msg)), 1)
    end

    self.stop_all()
    tasmota.resp_cmnd(json.dump({
      "RawDrive": {
        "ch": ch,
        "dir": dir,
        "stop_reason": stop_reason,
        "elapsed_ms": elapsed,
        "peak_ma": peak_ma,
        "sample_ms": sample_ms,
        "trace_ma": trace
      }
    }))
  end
end

return DumbValveGateway()
