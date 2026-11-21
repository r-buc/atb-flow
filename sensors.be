import json
import string
import math

class SensorManager
  var config_path, config, max_age_s
  var _ble_last_seen, _ble_cache

  def init(config_path)
    self.config_path = (config_path != nil && config_path != "") ? str(config_path) : "sensors.json"
    self.max_age_s = 1200  # 20 min max-age check (cooling dew-point safety)
    self._ble_last_seen = {}
    self._ble_cache = {}
    self.config = self._default_config()
    self._load_config()

    tasmota.add_cmd('SensorScan',      /cmd, idx, payload -> self.cmd_sensor_scan())
    tasmota.add_cmd('SensorMap',       /cmd, idx, payload -> self.cmd_sensor_map(payload))
    tasmota.add_cmd('SensorWaterbath', /cmd, idx, payload -> self.cmd_sensor_waterbath(payload))
    tasmota.add_cmd('SensorStatus',    /cmd, idx, payload -> self.cmd_sensor_status())

    # tasmota32s3-bluetooth.bin (USE_BLE_ESP32 / xsns_62_esp32_mi_ble.ino) publishes BLE
    # sensors via TELE/SENSOR rules rather than tasmota.read_sensors() (FUNC_JSON_APPEND).
    try
      tasmota.add_rule("?#mac", /val, trig, payload -> self._on_ble_rule(payload), "atb_sensors_ble")
    except ..
    end

    # MI32.period is reset to TelePeriod on every boot in MI32Init(); apply 60s after init.
    try
      tasmota.set_timer(3000, / -> self._apply_mi32_period())
    except ..
    end
  end

  def _apply_mi32_period()
    try
      tasmota.cmd("MI32Period 60")
    except ..
    end
  end

  def _default_config()
    return {
      "ds18b20": {"vl": nil, "rl": {}},
      "rooms": {}
    }
  end

  def _round2(val)
    if val == nil return nil end
    var v = real(val)
    if v >= 0.0
      return math.floor(v * 100.0 + 0.5) / 100.0
    end
    return -math.floor((-v) * 100.0 + 0.5) / 100.0
  end

  def normalize_id(raw_id)
    if raw_id == nil return "" end
    var s = string.toupper(str(raw_id))
    s = string.replace(s, ":", "")
    s = string.replace(s, "-", "")
    s = string.replace(s, " ", "")
    return s
  end

  def calc_dew_point(temp_c, hum_pct)
    if temp_c == nil || hum_pct == nil return nil end
    var t = real(temp_c)
    var rh = real(hum_pct)
    if rh <= 0.0 || rh > 100.0 return nil end
    # Magnus formula: a = 17.62, b = 243.12 degC
    var a = 17.62
    var b = 243.12
    var gamma = math.log(rh / 100.0) + (a * t) / (b + t)
    if (a - gamma) == 0.0 return nil end
    return self._round2((b * gamma) / (a - gamma))
  end

  def _load_config()
    try
      var f = open(self.config_path, 'r')
      var raw = f.read()
      f.close()
      var data = json.load(raw)
      if data != nil && type(data) == 'instance' && classname(data) == 'map'
        var cfg = self._default_config()
        if data.contains("ds18b20") && type(data["ds18b20"]) == 'instance' && classname(data["ds18b20"]) == 'map'
          var ds = data["ds18b20"]
          if ds.contains("vl") && ds["vl"] != nil
            cfg["ds18b20"]["vl"] = ds["vl"]
          end
          if ds.contains("rl") && type(ds["rl"]) == 'instance' && classname(ds["rl"]) == 'map'
            cfg["ds18b20"]["rl"] = ds["rl"]
          end
        end
        if data.contains("rooms") && type(data["rooms"]) == 'instance' && classname(data["rooms"]) == 'map'
          cfg["rooms"] = data["rooms"]
        end
        self.config = cfg
      end
    except ..
      self.config = self._default_config()
    end
  end

  def _save_config()
    try
      var f = open(self.config_path, 'w')
      f.write(json.dump(self.config))
      f.close()
    except .. as e, msg
      log(string.format("SENS: Failed to save %s [%s]: %s", self.config_path, str(e), str(msg)), 1)
    end
  end

  def _now_local_epoch()
    try
      var rtc = tasmota.rtc()
      if rtc != nil && type(rtc) == 'instance' && classname(rtc) == 'map'
        # MI32 MIBLEsensors[].lastTime uses Rtc.local_time
        if rtc.contains("local") && int(rtc["local"]) > 1000000000
          return int(rtc["local"])
        end
        if rtc.contains("utc") && int(rtc["utc"]) > 1000000000
          return int(rtc["utc"])
        end
      end
    except ..
    end
    return nil
  end

  def _ble_age_s(mac, pkt_time)
    var now_local = self._now_local_epoch()
    if now_local != nil && pkt_time != nil && int(pkt_time) > 1000000000
      var age = now_local - int(pkt_time)
      if age < 0 age = 0 end
      return age
    end
    # Fallback for USE_BLE_ESP32 (no per-sensor epoch Time in JSON) or missing RTC
    var now_ms = tasmota.millis()
    if self._ble_last_seen.contains(mac)
      var diff_s = int((now_ms - self._ble_last_seen[mac]) / 1000)
      if diff_s < 0 diff_s = 0 end
      return diff_s
    end
    return nil
  end

  def _extract_ble_entry(k, v, now_ms)
    var mac_raw = nil
    if v.contains("MAC")
      mac_raw = v["MAC"]
    elif v.contains("mac")
      mac_raw = v["mac"]
    end
    if mac_raw == nil return nil end
    var mac = self.normalize_id(mac_raw)
    if mac == "" return nil end

    var temp = v.contains("Temperature") && v["Temperature"] != nil ? real(v["Temperature"]) : nil
    var hum  = v.contains("Humidity")    && v["Humidity"] != nil    ? real(v["Humidity"])    : nil
    var dp   = v.contains("DewPoint")    && v["DewPoint"] != nil    ? real(v["DewPoint"])    : self.calc_dew_point(temp, hum)
    var bat  = v.contains("Battery")     && v["Battery"] != nil     ? int(v["Battery"])      : nil
    var rssi = v.contains("RSSI")        && v["RSSI"] != nil        ? int(v["RSSI"])         : nil
    var pkt_t = v.contains("Time")       && v["Time"] != nil        ? int(v["Time"])         : nil

    if temp != nil || hum != nil
      self._ble_last_seen[mac] = now_ms
    end

    return {
      "key": k,
      "mac": mac,
      "temp": temp,
      "hum": hum,
      "dew_point": dp,
      "battery": bat,
      "rssi": rssi,
      "time": pkt_t
    }
  end

  def _on_ble_rule(payload)
    if payload == nil || type(payload) != 'instance' || classname(payload) != 'map'
      return
    end
    var now_ms = tasmota.millis()
    for k : payload.keys()
      var v = payload[k]
      if type(v) != 'instance' || classname(v) != 'map'
        continue
      end
      var entry = self._extract_ble_entry(k, v, now_ms)
      if entry != nil
        self._ble_cache[entry["mac"]] = entry
      end
    end
  end

  def read_raw()
    var ds = {}
    var ble = {}
    for mac : self._ble_cache.keys()
      ble[mac] = self._ble_cache[mac]
    end

    var raw_str = nil
    try
      raw_str = tasmota.read_sensors()
    except .. as e, msg
      log(string.format("SENS: read_sensors failed [%s]: %s", str(e), str(msg)), 1)
      return {"ds18b20": ds, "ble": ble}
    end
    if raw_str == nil || raw_str == ""
      return {"ds18b20": ds, "ble": ble}
    end
    var data = json.load(raw_str)
    if data == nil || type(data) != 'instance' || classname(data) != 'map'
      return {"ds18b20": ds, "ble": ble}
    end

    var now_ms = tasmota.millis()
    for k : data.keys()
      var v = data[k]
      if type(v) != 'instance' || classname(v) != 'map'
        continue
      end

      # 1-Wire DS18B20 entries: "DS18B20" or "DS18B20-1", "DS18B20-2", ...
      if string.startswith(k, "DS18B20") && v.contains("Id") && v.contains("Temperature")
        var sid = self.normalize_id(v["Id"])
        if sid != "" && v["Temperature"] != nil
          ds[sid] = {
            "key": k,
            "id": sid,
            "temp": real(v["Temperature"])
          }
        end
        continue
      end

      # MI32 BLE entries when !USE_BLE_ESP32 (xsns_62_esp32_mi.ino includes them in read_sensors)
      var entry = self._extract_ble_entry(k, v, now_ms)
      if entry != nil
        self._ble_cache[entry["mac"]] = entry
        ble[entry["mac"]] = entry
      end
    end

    return {"ds18b20": ds, "ble": ble}
  end

  def _find_ds_role(sid)
    var ds_cfg = self.config["ds18b20"]
    if ds_cfg["vl"] != nil && ds_cfg["vl"].contains("id") && ds_cfg["vl"]["id"] == sid
      return "vl"
    end
    for ch : ds_cfg["rl"].keys()
      var entry = ds_cfg["rl"][ch]
      if entry != nil && entry.contains("id") && entry["id"] == sid
        return "rl_" + str(ch)
      end
    end
    return nil
  end

  def _find_ble_room(mac)
    for rname : self.config["rooms"].keys()
      var r = self.config["rooms"][rname]
      if r != nil && r.contains("mac") && r["mac"] == mac
        return rname
      end
    end
    return nil
  end

  def cmd_sensor_scan()
    var raw = self.read_raw()
    var ds_list = []
    for sid : raw["ds18b20"].keys()
      var item = raw["ds18b20"][sid]
      ds_list.push({
        "id": sid,
        "key": item["key"],
        "temp": self._round2(item["temp"]),
        "role": self._find_ds_role(sid)
      })
    end

    var ble_list = []
    for mac : raw["ble"].keys()
      var item = raw["ble"][mac]
      ble_list.push({
        "mac": mac,
        "key": item["key"],
        "temp": item["temp"] != nil ? self._round2(item["temp"]) : nil,
        "hum": item["hum"] != nil ? self._round2(item["hum"]) : nil,
        "dew_point": item["dew_point"],
        "battery": item["battery"],
        "rssi": item["rssi"],
        "room": self._find_ble_room(mac)
      })
    end

    tasmota.resp_cmnd(json.dump({
      "SensorScan": {
        "ds18b20": ds_list,
        "ble": ble_list
      }
    }))
  end

  def cmd_sensor_map(payload)
    if payload == nil || payload == ""
      tasmota.resp_cmnd(json.dump({"SensorMap": self.config}))
      return
    end
    var p = json.load(payload)
    if p == nil || type(p) != 'instance' || classname(p) != 'map'
      tasmota.resp_cmnd(json.dump({"error": "INVALID_ARGS"}))
      return
    end

    var changed = false

    # Map Vorlauf DS18B20: {"vl": "3C01F0961234", "offset": 0.0}
    if p.contains("vl")
      if p["vl"] == nil || str(p["vl"]) == ""
        self.config["ds18b20"]["vl"] = nil
      else
        var vl_id = self.normalize_id(p["vl"])
        if vl_id == ""
          tasmota.resp_cmnd(json.dump({"error": "INVALID_ID"}))
          return
        end
        var offset = 0.0
        if p.contains("offset") && p["offset"] != nil
          offset = real(p["offset"])
        elif self.config["ds18b20"]["vl"] != nil && self.config["ds18b20"]["vl"]["id"] == vl_id
          offset = real(self.config["ds18b20"]["vl"]["offset"])
        end
        self.config["ds18b20"]["vl"] = {"id": vl_id, "offset": self._round2(offset)}
      end
      changed = true
    end

    # Map Ruecklauf DS18B20: {"ch": 1, "id": "3C01F0965678", "offset": -0.12}
    if p.contains("ch") && p.contains("id")
      var ch = int(p["ch"])
      if ch < 1 || ch > 4
        tasmota.resp_cmnd(json.dump({"error": "INVALID_CHANNEL"}))
        return
      end
      var ch_key = str(ch)
      var rl_id = self.normalize_id(p["id"])
      if rl_id == ""
        tasmota.resp_cmnd(json.dump({"error": "INVALID_ID"}))
        return
      end
      var offset = 0.0
      if p.contains("offset") && p["offset"] != nil
        offset = real(p["offset"])
      elif self.config["ds18b20"]["rl"].contains(ch_key) && self.config["ds18b20"]["rl"][ch_key]["id"] == rl_id
        offset = real(self.config["ds18b20"]["rl"][ch_key]["offset"])
      end
      self.config["ds18b20"]["rl"][ch_key] = {"id": rl_id, "offset": self._round2(offset)}
      changed = true
    end

    # Delete Ruecklauf channel mapping: {"delete_ch": 1}
    if p.contains("delete_ch")
      var dch_key = str(int(p["delete_ch"]))
      if self.config["ds18b20"]["rl"].contains(dch_key)
        self.config["ds18b20"]["rl"].remove(dch_key)
        changed = true
      end
    end

    # Map BLE room sensor: {"room": "wohnzimmer", "mac": "A4:C1:38:12:34:56", "channels": [1,2], "allow_cooling": true, "host_hkv": false}
    if p.contains("room")
      var rname = str(p["room"])
      if rname == ""
        tasmota.resp_cmnd(json.dump({"error": "INVALID_ROOM"}))
        return
      end
      var existing = self.config["rooms"].contains(rname) ? self.config["rooms"][rname] : nil
      var mac = nil
      if p.contains("mac") && p["mac"] != nil
        mac = self.normalize_id(p["mac"])
      elif existing != nil && existing.contains("mac")
        mac = existing["mac"]
      end
      if mac == nil || size(mac) != 12
        tasmota.resp_cmnd(json.dump({"error": "INVALID_MAC"}))
        return
      end
      var channels = (existing != nil && existing.contains("channels")) ? existing["channels"] : []
      if p.contains("channels") && type(p["channels"]) == 'instance' && classname(p["channels"]) == 'list'
        channels = p["channels"]
      end
      var allow_cooling = (existing != nil && existing.contains("allow_cooling")) ? existing["allow_cooling"] : true
      if p.contains("allow_cooling")
        allow_cooling = bool(p["allow_cooling"])
      end
      var host_hkv = (existing != nil && existing.contains("host_hkv")) ? existing["host_hkv"] : false
      if p.contains("host_hkv")
        host_hkv = bool(p["host_hkv"])
      end
      self.config["rooms"][rname] = {
        "mac": mac,
        "channels": channels,
        "allow_cooling": allow_cooling,
        "host_hkv": host_hkv
      }
      changed = true
    end

    # Delete room mapping: {"delete_room": "wohnzimmer"}
    if p.contains("delete_room")
      var dr = str(p["delete_room"])
      if self.config["rooms"].contains(dr)
        self.config["rooms"].remove(dr)
        changed = true
      end
    end

    if !changed
      tasmota.resp_cmnd(json.dump({"error": "INVALID_ARGS"}))
      return
    end

    self._save_config()
    tasmota.resp_cmnd(json.dump({"SensorMap": {"status": "OK", "config": self.config}}))
  end

  def cmd_sensor_waterbath(payload)
    var ref_temp = nil
    if payload != nil && payload != ""
      var p = json.load(payload)
      if p != nil && type(p) == 'instance' && classname(p) == 'map' && p.contains("ref_temp")
        ref_temp = real(p["ref_temp"])
      else
        ref_temp = real(payload)
      end
    end

    var raw = self.read_raw()["ds18b20"]
    var entries = []
    var ds_cfg = self.config["ds18b20"]

    if ds_cfg["vl"] != nil && ds_cfg["vl"].contains("id")
      var vid = ds_cfg["vl"]["id"]
      if raw.contains(vid)
        entries.push({"role": "vl", "cfg": ds_cfg["vl"], "raw": raw[vid]["temp"]})
      end
    end
    for ch : ds_cfg["rl"].keys()
      var rcfg = ds_cfg["rl"][ch]
      if rcfg != nil && rcfg.contains("id") && raw.contains(rcfg["id"])
        entries.push({"role": "rl_" + str(ch), "cfg": rcfg, "raw": raw[rcfg["id"]]["temp"]})
      end
    end

    if size(entries) == 0
      tasmota.resp_cmnd(json.dump({"error": "NO_MAPPED_SENSORS_ONLINE"}))
      return
    end

    if ref_temp == nil
      var sum_t = 0.0
      for e : entries
        sum_t += e["raw"]
      end
      ref_temp = sum_t / real(size(entries))
    end
    ref_temp = self._round2(ref_temp)

    var results = {}
    for e : entries
      var off = self._round2(ref_temp - e["raw"])
      e["cfg"]["offset"] = off
      results[e["role"]] = {
        "id": e["cfg"]["id"],
        "raw_temp": self._round2(e["raw"]),
        "offset": off
      }
    end

    self._save_config()
    tasmota.resp_cmnd(json.dump({
      "SensorWaterbath": {
        "status": "OK",
        "ref_temp": ref_temp,
        "sensors": results
      }
    }))
  end

  def read_mapped()
    var raw = self.read_raw()
    var ds_raw = raw["ds18b20"]
    var ble_raw = raw["ble"]
    var ds_cfg = self.config["ds18b20"]

    var vl_out = nil
    var vl_temp = nil
    var mapped_ds_ids = {}

    if ds_cfg["vl"] != nil && ds_cfg["vl"].contains("id")
      var vid = ds_cfg["vl"]["id"]
      var voff = real(ds_cfg["vl"]["offset"])
      mapped_ds_ids[vid] = true
      if ds_raw.contains(vid)
        var r = ds_raw[vid]["temp"]
        vl_temp = self._round2(r + voff)
        vl_out = {
          "id": vid,
          "raw_temp": self._round2(r),
          "offset": voff,
          "temp": vl_temp,
          "online": true
        }
      else
        vl_out = {"id": vid, "offset": voff, "temp": nil, "online": false}
      end
    end

    var rl_out = {}
    for ch : ds_cfg["rl"].keys()
      var rcfg = ds_cfg["rl"][ch]
      var rid = rcfg["id"]
      var roff = real(rcfg["offset"])
      mapped_ds_ids[rid] = true
      if ds_raw.contains(rid)
        var r = ds_raw[rid]["temp"]
        var t = self._round2(r + roff)
        var dt = (vl_temp != nil) ? self._round2(vl_temp - t) : nil
        rl_out[ch] = {
          "id": rid,
          "raw_temp": self._round2(r),
          "offset": roff,
          "temp": t,
          "delta_t": dt,
          "online": true
        }
      else
        rl_out[ch] = {"id": rid, "offset": roff, "temp": nil, "delta_t": nil, "online": false}
      end
    end

    var unmapped_ds = []
    for sid : ds_raw.keys()
      if !mapped_ds_ids.contains(sid)
        unmapped_ds.push({"id": sid, "temp": self._round2(ds_raw[sid]["temp"])})
      end
    end

    var rooms_out = {}
    var mapped_macs = {}
    for rname : self.config["rooms"].keys()
      var rcfg = self.config["rooms"][rname]
      var mac = rcfg["mac"]
      mapped_macs[mac] = true
      if ble_raw.contains(mac)
        var b = ble_raw[mac]
        var age = self._ble_age_s(mac, b["time"])
        var stale = (age != nil) ? (age > self.max_age_s) : false
        rooms_out[rname] = {
          "mac": mac,
          "temp": b["temp"] != nil ? self._round2(b["temp"]) : nil,
          "hum": b["hum"] != nil ? self._round2(b["hum"]) : nil,
          "dew_point": b["dew_point"],
          "battery": b["battery"],
          "rssi": b["rssi"],
          "age_s": age,
          "stale": stale,
          "online": !stale && b["temp"] != nil,
          "channels": rcfg["channels"],
          "allow_cooling": rcfg["allow_cooling"],
          "host_hkv": rcfg["host_hkv"]
        }
      else
        rooms_out[rname] = {
          "mac": mac,
          "temp": nil,
          "hum": nil,
          "dew_point": nil,
          "battery": nil,
          "rssi": nil,
          "age_s": nil,
          "stale": true,
          "online": false,
          "channels": rcfg["channels"],
          "allow_cooling": rcfg["allow_cooling"],
          "host_hkv": rcfg["host_hkv"]
        }
      end
    end

    var unmapped_ble = []
    for mac : ble_raw.keys()
      if !mapped_macs.contains(mac)
        unmapped_ble.push({"mac": mac, "key": ble_raw[mac]["key"], "rssi": ble_raw[mac]["rssi"]})
      end
    end

    return {
      "vl": vl_out,
      "rl": rl_out,
      "rooms": rooms_out,
      "unmapped_ds18b20": unmapped_ds,
      "unmapped_ble": unmapped_ble
    }
  end

  def cmd_sensor_status()
    tasmota.resp_cmnd(json.dump({"SensorStatus": self.read_mapped()}))
  end
end

return SensorManager()
