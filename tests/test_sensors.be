# Unit test suite for sensors.be (SensorManager)
# Run with standalone Berry interpreter from repository root:
#   berry -m tests:. tests/test_sensors.be

import json
import string
import math
import os
import mock_tasmota
import gpio
import global

print("--- atb-flow Berry SensorManager Unit Tests ---")
var _pass = 0
var _fail = 0

def check(name, cond)
    if cond
        print("  PASS: " + name)
        _pass += 1
    else
        print("  FAIL: " + name)
        _fail += 1
    end
end

def near(a, b, tol)
    if a == nil || b == nil return false end
    return math.abs(real(a) - real(b)) <= tol
end

var test_cfg_path = "/tmp/test_atb_flow_sensors.json"
try os.remove(test_cfg_path) except .. end

# ---------------------------------------------------------------------------
# 1. Initialization & Command Registration
# ---------------------------------------------------------------------------
print("\n[1] Initialization & Command Registration")
import sensors
var sm = sensors
sm.config_path = test_cfg_path
sm.config = sm._default_config()

check("sensors instance created",              type(sm) == 'instance')
check("SensorScan command registered",         global.tasmota._cmds.contains('SensorScan'))
check("SensorMap command registered",          global.tasmota._cmds.contains('SensorMap'))
check("SensorWaterbath command registered",    global.tasmota._cmds.contains('SensorWaterbath'))
check("SensorStatus command registered",       global.tasmota._cmds.contains('SensorStatus'))
check("BLE rule ?#mac registered",             global.tasmota._rules.contains('?#mac'))

# ---------------------------------------------------------------------------
# 2. SensorScan (DS18B20 + MI32 BLE via both TELE rule and read_sensors)
# ---------------------------------------------------------------------------
print("\n[2] SensorScan Parsing")
var now_local = global.tasmota.rtc()['local']
global.tasmota._sensors_json = json.dump({
    "Time": "2026-11-21T20:55:00",
    "DS18B20-1": {"Id": "3c01f0961234", "Temperature": 32.1},
    "DS18B20-2": {"Id": "3C01F0965678", "Temperature": 28.4},
    "DS18B20-3": {"Id": "3C01F0969ABC", "Temperature": 27.9},
    "ATC-123456": {
        "Temperature": 22.5,
        "Humidity": 50.0,
        "Battery": 92,
        "RSSI": -64,
        "Time": now_local - 45,
        "MAC": "A4C138123456"
    },
    "BTHome-654321": {
        "Temperature": 24.0,
        "Humidity": 65.0,
        "DewPoint": 17.0,
        "Battery": 88,
        "RSSI": -71,
        "Time": now_local - 1500,
        "MAC": "A4C138654321"
    }
})

# Also simulate USE_BLE_ESP32 rule event with lowercase "mac" and no per-sensor "Time"
global.tasmota.emit_ble_tele({
    "Time": "2026-11-21T20:55:00",
    "ATC998877": {
        "mac": "a4c138998877",
        "Temperature": 21.0,
        "Humidity": 45.0,
        "DewPoint": 8.6,
        "Battery": 95,
        "RSSI": -58
    }
})

global.tasmota._cmds['SensorScan']('SensorScan', 1, '')
var scan = json.load(global.tasmota.last_response)['SensorScan']
check("SensorScan found 3 DS18B20 sensors",    size(scan['ds18b20']) == 3)
check("SensorScan found 3 BLE sensors (2 read_sensors + 1 rule)", size(scan['ble']) == 3)

# Verify Magnus dew-point calculation when DewPoint is absent in BLE JSON
# T = 22.5 C, RH = 50% -> Td ~ 11.57 C
var dp_calc = sm.calc_dew_point(22.5, 50.0)
check("Magnus dew point (22.5C, 50% RH) ~ 11.57C", near(dp_calc, 11.57, 0.05))
check("Magnus dew point invalid RH returns nil",   sm.calc_dew_point(22.5, 0.0) == nil)

# ---------------------------------------------------------------------------
# 3. SensorMap (VL, RL, Rooms, Normalization, Validation)
# ---------------------------------------------------------------------------
print("\n[3] SensorMap & Validation")
global.tasmota._cmds['SensorMap']('SensorMap', 1, '{"vl":"3c:01-f096 1234"}')
var m_vl = json.load(global.tasmota.last_response)['SensorMap']
check("VL mapped & normalized",                m_vl['status'] == 'OK' && m_vl['config']['ds18b20']['vl']['id'] == '3C01F0961234')

global.tasmota._cmds['SensorMap']('SensorMap', 1, '{"ch":1,"id":"3C01F0965678","offset":0.1}')
var m_rl1 = json.load(global.tasmota.last_response)['SensorMap']
check("RL ch 1 mapped with offset",            m_rl1['config']['ds18b20']['rl']['1']['id'] == '3C01F0965678' && near(m_rl1['config']['ds18b20']['rl']['1']['offset'], 0.1, 0.001))

global.tasmota._cmds['SensorMap']('SensorMap', 1, '{"ch":2,"id":"3C01F0969ABC"}')
global.tasmota._cmds['SensorMap']('SensorMap', 1, '{"ch":9,"id":"3C01F0969ABC"}')
check("Invalid channel 9 rejected",            json.load(global.tasmota.last_response)['error'] == 'INVALID_CHANNEL')

global.tasmota._cmds['SensorMap']('SensorMap', 1, '{"room":"wohnzimmer","mac":"a4:c1:38:12:34:56","channels":[1,2],"allow_cooling":true,"host_hkv":true}')
var m_r1 = json.load(global.tasmota.last_response)['SensorMap']
check("Room wohnzimmer mapped & MAC normalized", m_r1['config']['rooms']['wohnzimmer']['mac'] == 'A4C138123456')
check("Room wohnzimmer host_hkv == true",        m_r1['config']['rooms']['wohnzimmer']['host_hkv'] == true)

global.tasmota._cmds['SensorMap']('SensorMap', 1, '{"room":"bad","mac":"A4C138654321","channels":[2],"allow_cooling":false}')
global.tasmota._cmds['SensorMap']('SensorMap', 1, '{"room":"bad2","mac":"INVALID"}')
check("Invalid MAC rejected",                    json.load(global.tasmota.last_response)['error'] == 'INVALID_MAC')

# ---------------------------------------------------------------------------
# 4. SensorStatus (Offsets, Delta-T, DewPoint, Stale/Max-Age Check)
# ---------------------------------------------------------------------------
print("\n[4] SensorStatus (Delta-T, DewPoint & Max-Age)")
global.tasmota._cmds['SensorStatus']('SensorStatus', 1, '')
var st = json.load(global.tasmota.last_response)['SensorStatus']
check("VL online & temp == 32.1",              st['vl']['online'] == true && near(st['vl']['temp'], 32.1, 0.01))
# RL1 raw 28.4 + offset 0.1 = 28.5 -> delta_t = 32.1 - 28.5 = 3.6
check("RL1 offset applied (28.5)",             near(st['rl']['1']['temp'], 28.5, 0.01))
check("RL1 delta_t == 3.6",                    near(st['rl']['1']['delta_t'], 3.6, 0.01))
# RL2 raw 27.9 + offset 0.0 = 27.9 -> delta_t = 32.1 - 27.9 = 4.2
check("RL2 delta_t == 4.2",                    near(st['rl']['2']['delta_t'], 4.2, 0.01))
check("No unmapped DS18B20 left",              size(st['unmapped_ds18b20']) == 0)
check("1 unmapped BLE sensor (ATC998877)",     size(st['unmapped_ble']) == 1)

# Simulate BLE rule sensor aging past 20 min via millis()
sm.config['rooms']['flur'] = {"mac": "A4C138998877", "channels": [3], "allow_cooling": true, "host_hkv": false}
global.tasmota._cmds['SensorStatus']('SensorStatus', 1, '')
var flur_fresh = json.load(global.tasmota.last_response)['SensorStatus']['rooms']['flur']
check("Rule-cached BLE flur is online",        flur_fresh['online'] == true && flur_fresh['stale'] == false && flur_fresh['age_s'] == 0)
global.tasmota.delay(1300000)
global.tasmota._cmds['SensorStatus']('SensorStatus', 1, '')
var flur_stale = json.load(global.tasmota.last_response)['SensorStatus']['rooms']['flur']
check("Rule-cached BLE flur turns stale after 20m", flur_stale['stale'] == true && flur_stale['online'] == false && flur_stale['age_s'] >= 1300)
sm.config['rooms'].remove('flur')

# Wohnzimmer: age 45s <= 1200s -> online, not stale, calculated dew point
var wz = st['rooms']['wohnzimmer']
check("Wohnzimmer online & not stale",         wz['online'] == true && wz['stale'] == false && wz['age_s'] == 45)
check("Wohnzimmer dew_point calculated",       near(wz['dew_point'], 11.57, 0.05))

# Bad: age 1500s > 1200s (20 min) -> stale == true, online == false
var bad = st['rooms']['bad']
check("Bad detected as stale (> 20 min)",      bad['stale'] == true && bad['online'] == false && bad['age_s'] == 1500)
check("Bad allow_cooling == false",            bad['allow_cooling'] == false)
check("Bad preserves provided DewPoint 17.0",  near(bad['dew_point'], 17.0, 0.01))

# ---------------------------------------------------------------------------
# 5. SensorWaterbath (Mean & Explicit Reference Calibration + Persistence)
# ---------------------------------------------------------------------------
print("\n[5] SensorWaterbath Calibration & Persistence")
# Simulate all 3 DS18B20 in a 30.0 C waterbath with slight individual errors
global.tasmota._sensors_json = json.dump({
    "DS18B20-1": {"Id": "3C01F0961234", "Temperature": 30.30},
    "DS18B20-2": {"Id": "3C01F0965678", "Temperature": 29.70},
    "DS18B20-3": {"Id": "3C01F0969ABC", "Temperature": 30.00}
})
global.tasmota._cmds['SensorWaterbath']('SensorWaterbath', 1, '')
var wb = json.load(global.tasmota.last_response)['SensorWaterbath']
check("Waterbath status == OK",                wb['status'] == 'OK')
check("Waterbath mean ref_temp == 30.0",       near(wb['ref_temp'], 30.0, 0.01))
check("Waterbath VL offset == -0.30",          near(wb['sensors']['vl']['offset'], -0.30, 0.01))
check("Waterbath RL1 offset == +0.30",         near(wb['sensors']['rl_1']['offset'], 0.30, 0.01))
check("Waterbath RL2 offset == 0.00",          near(wb['sensors']['rl_2']['offset'], 0.00, 0.01))

# After waterbath calibration, all 3 sensors must read 30.0 C and delta_t == 0.0
global.tasmota._cmds['SensorStatus']('SensorStatus', 1, '')
var st_wb = json.load(global.tasmota.last_response)['SensorStatus']
check("Post-waterbath VL == 30.0",             near(st_wb['vl']['temp'], 30.0, 0.01))
check("Post-waterbath RL1 == 30.0, dT == 0",   near(st_wb['rl']['1']['temp'], 30.0, 0.01) && near(st_wb['rl']['1']['delta_t'], 0.0, 0.01))
check("Post-waterbath RL2 == 30.0, dT == 0",   near(st_wb['rl']['2']['temp'], 30.0, 0.01) && near(st_wb['rl']['2']['delta_t'], 0.0, 0.01))

# Explicit reference temperature (30.5 C)
global.tasmota._cmds['SensorWaterbath']('SensorWaterbath', 1, '30.5')
var wb2 = json.load(global.tasmota.last_response)['SensorWaterbath']
check("Waterbath explicit ref_temp == 30.5",   near(wb2['ref_temp'], 30.5, 0.01))
check("Waterbath VL offset vs 30.5 == +0.20",  near(wb2['sensors']['vl']['offset'], 0.20, 0.01))

# Reload config from disk to verify persistence
sm.config = sm._default_config()
sm._load_config()
check("Persisted VL offset reloaded (+0.20)",  near(sm.config['ds18b20']['vl']['offset'], 0.20, 0.01))
check("Persisted room wohnzimmer reloaded",    sm.config['rooms'].contains('wohnzimmer'))

# ---------------------------------------------------------------------------
# 6. Delete mappings & Bootloader check
# ---------------------------------------------------------------------------
print("\n[6] Delete Mappings & Bootloader")
global.tasmota._cmds['SensorMap']('SensorMap', 1, '{"delete_room":"bad","delete_ch":2}')
check("Deleted room bad removed",              !sm.config['rooms'].contains('bad'))
check("Deleted RL ch 2 removed",               !sm.config['ds18b20']['rl'].contains('2'))

import autoexec
global.tasmota.run_timers()
check("global.sensors populated by autoexec.be", global.sensors != nil)
check("MI32Period 60 applied via boot timer",   size(global.tasmota._cmd_log) > 0 && global.tasmota._cmd_log[size(global.tasmota._cmd_log) - 1] == "MI32Period 60")

# Cleanup temp config file
try os.remove(test_cfg_path) except .. end

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print(string.format("\n=== Results: %d passed, %d failed ===", _pass, _fail))
if _fail > 0
    raise 'test_failure', string.format("%d test(s) failed", _fail)
end
