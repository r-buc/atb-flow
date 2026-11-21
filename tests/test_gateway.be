# Unit test suite for gateway.be (DumbValveGateway)
# Run with standalone Berry interpreter from repository root:
#   berry -m tests:. tests/test_gateway.be

import json
import string
import mock_tasmota
import gpio
import global

print("--- atb-flow Berry Gateway Unit Tests ---")
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

# ---------------------------------------------------------------------------
# 1. Load gateway.be and verify initial safe state (all pins OUTPUT + LOW)
# ---------------------------------------------------------------------------
print("\n[1] Initialization & Safe Pin State")
import gateway
var gw = gateway
check("gateway instance created",           type(gw) == 'instance')
check("RawDrive command registered",        global.tasmota._cmds.contains('RawDrive'))
check("RawStop command registered",         global.tasmota._cmds.contains('RawStop'))
check("RawStatus command registered",       global.tasmota._cmds.contains('RawStatus'))
check("RawResult command registered",       global.tasmota._cmds.contains('RawResult'))
check("ValveCal command registered",        global.tasmota._cmds.contains('ValveCal'))
check("ValvePos command registered",        global.tasmota._cmds.contains('ValvePos'))
check("ValveStatus command registered",     global.tasmota._cmds.contains('ValveStatus'))

var expected_pins = [14, 5, 6, 7, 15, 16, 17, 18, 21]
for p : expected_pins
    check(string.format("GPIO %d mode is OUTPUT", p), gpio._modes[p] == gpio.OUTPUT)
    check(string.format("GPIO %d initialized LOW", p), gpio._states[p] == gpio.LOW)
end

# ---------------------------------------------------------------------------
# 2. RawStatus command & signed 16-bit INA219 current conversion
# ---------------------------------------------------------------------------
print("\n[2] RawStatus & Signed INA219 Conversion")
global.tasmota._wire.set_trace_ma([0.3, -28.5])
global.tasmota._cmds['RawStatus']('RawStatus', 1, '')
var st = json.load(global.tasmota.last_response)['RawStatus']
check("RawStatus ina219_connected == true", st['ina219_connected'] == true)
check("RawStatus idle_ma == 0.3",           st['idle_ma'] > 0.25 && st['idle_ma'] < 0.35)
check("RawStatus is_moving == false",       st['is_moving'] == false)

# Negative shunt voltage (reverse current polarity) must be converted via math.abs()
var neg_cur = gw.read_current_ma()
check("Signed 16-bit negative shunt voltage -> +28.5 mA", neg_cur > 28.4 && neg_cur < 28.6)

# ---------------------------------------------------------------------------
# 3. Argument validation in RawDrive
# ---------------------------------------------------------------------------
print("\n[3] RawDrive Argument Validation")
global.tasmota._cmds['RawDrive']('RawDrive', 1, '')
check("Empty payload -> INVALID_ARGS",      json.load(global.tasmota.last_response)['error'] == 'INVALID_ARGS')

global.tasmota._cmds['RawDrive']('RawDrive', 1, '{"ch":99,"dir":"open"}')
check("Channel 99 -> INVALID_CHANNEL",      json.load(global.tasmota.last_response)['error'] == 'INVALID_CHANNEL')

global.tasmota._cmds['RawDrive']('RawDrive', 1, '{"ch":1,"dir":"sideways"}')
check("Invalid dir -> INVALID_DIR",         json.load(global.tasmota.last_response)['error'] == 'INVALID_DIR')

# ---------------------------------------------------------------------------
# 4. Inrush blanking & TIME_LIMIT completion
# ---------------------------------------------------------------------------
print("\n[4] RawDrive Inrush Blanking & TIME_LIMIT")
# First 2 samples (t=0, t=25ms) have 160mA inrush (> cutoff 100mA),
# but since inrush_ms=100, they must be ignored; afterwards 26mA free-run until max_ms=150
global.tasmota._wire.set_trace_ma([160.0, 145.0, 26.0, 25.5, 26.2, 25.8, 26.0, 26.0])
global.tasmota._cmds['RawDrive'](
    'RawDrive', 1,
    '{"ch":1,"dir":"open","max_ms":150,"cutoff_ma":100,"inrush_ms":100,"sample_ms":25}'
)
var rd1_start = json.load(global.tasmota.last_response)['RawDrive']
check("RawDrive returns STARTED immediately",        rd1_start['status'] == 'STARTED')
global.tasmota.run_timers()
global.tasmota._cmds['RawResult']('RawResult', 1, '')
var rd1 = json.load(global.tasmota.last_response)['RawResult']
check("Inrush ignored -> stop_reason == TIME_LIMIT", rd1['stop_reason'] == 'TIME_LIMIT')
check("elapsed_ms == 150",                           rd1['elapsed_ms'] == 150)
check("peak_ma == 160.0",                            rd1['peak_ma'] == 160.0)
check("trace_ma has 7 samples",                      size(rd1['trace_ma']) == 7)
check("All pins returned to LOW after drive",        gpio._states[14] == gpio.LOW && gpio._states[5] == gpio.LOW && gpio._states[6] == gpio.LOW)

# ---------------------------------------------------------------------------
# 5. CURRENT_CUTOFF detection after inrush_ms
# ---------------------------------------------------------------------------
print("\n[5] RawDrive CURRENT_CUTOFF Detection")
# Sample 0 (0ms): 140mA (inrush)
# Sample 1 (25ms): 25mA
# Sample 2 (50ms): 26mA
# Sample 3 (75ms): 27mA
# Sample 4 (100ms): 28mA
# Sample 5 (125ms > inrush 100ms): 95mA >= cutoff 80mA -> immediate cutoff!
global.tasmota._wire.set_trace_ma([140.0, 25.0, 26.0, 27.0, 28.0, 95.0, 98.0])
global.tasmota._cmds['RawDrive'](
    'RawDrive', 1,
    '{"ch":2,"dir":"close","max_ms":5000,"cutoff_ma":80,"inrush_ms":100,"sample_ms":25}'
)
global.tasmota.run_timers()
global.tasmota._cmds['RawResult']('RawResult', 1, '')
var rd2 = json.load(global.tasmota.last_response)['RawResult']
check("Stall after inrush -> CURRENT_CUTOFF",        rd2['stop_reason'] == 'CURRENT_CUTOFF')
check("Stopped at elapsed_ms == 125",                rd2['elapsed_ms'] == 125)
check("Channel 2 pins returned to LOW",              gpio._states[7] == gpio.LOW && gpio._states[15] == gpio.LOW && gpio._states[14] == gpio.LOW)

# ---------------------------------------------------------------------------
# 6. Fail-safe on I2C read failure during drive
# ---------------------------------------------------------------------------
print("\n[6] Fail-Safe on I2C Read Failure (I2C_ERROR)")
global.tasmota._wire._fail_read = true
global.tasmota._cmds['RawDrive'](
    'RawDrive', 1,
    '{"ch":3,"dir":"open","max_ms":5000,"cutoff_ma":100,"inrush_ms":100,"sample_ms":25}'
)
global.tasmota.run_timers()
global.tasmota._cmds['RawResult']('RawResult', 1, '')
var rd3 = json.load(global.tasmota.last_response)['RawResult']
check("I2C read failure aborts immediately -> I2C_ERROR", rd3['stop_reason'] == 'I2C_ERROR')
check("elapsed_ms == 0 on immediate I2C error",           rd3['elapsed_ms'] == 0)
check("Standby pin LOW after I2C error",                  gpio._states[14] == gpio.LOW)
global.tasmota._wire._fail_read = false

# ---------------------------------------------------------------------------
# 7. Mutex protection (BUSY when already moving) & RawStop
# ---------------------------------------------------------------------------
print("\n[7] Mutex Protection & RawStop")
gw.is_moving = true
global.tasmota._cmds['RawDrive']('RawDrive', 1, '{"ch":1,"dir":"open"}')
check("Concurrent RawDrive rejected with BUSY",      json.load(global.tasmota.last_response)['error'] == 'BUSY')

global.tasmota._cmds['RawStop']('RawStop', 1, '')
check("RawStop clears is_moving",                    gw.is_moving == false)
check("RawStop returns OK",                          json.load(global.tasmota.last_response)['RawStop'] == 'OK')

# ---------------------------------------------------------------------------
# 8. On-Device Auto-Calibration Math (valve_cal.be)
# ---------------------------------------------------------------------------
print("\n[8] On-Device Auto-Calibration Math (valve_cal.be)")
import valve_cal

var close_trace = []
var k_i = 0
while k_i < 10
    close_trace.push(25.0)
    k_i += 1
end
k_i = 0
while k_i < 100
    close_trace.push(25.0)
    k_i += 1
end
k_i = 0
while k_i < 150
    close_trace.push(25.0 + 0.35 * real(k_i + 1))
    k_i += 1
end
var ck = valve_cal.detect_touch_knee(close_trace, size(close_trace) * 95)
check("detect_touch_knee returns map",               ck != nil)
check("detect_touch_knee idx near 109",              ck != nil && ck['touch_idx'] >= 105 && ck['touch_idx'] <= 114)
check("detect_touch_knee i_free_close near 25 mA",   ck != nil && ck['i_free_close_ma'] >= 24.0 && ck['i_free_close_ma'] <= 26.0)

var open_trace = [48.0, 50.0, 42.0]
k_i = 0
while k_i < 120
    open_trace.push(40.0 - (22.0 / 120.0) * real(k_i))
    k_i += 1
end
k_i = 0
while k_i < 180
    open_trace.push(18.0)
    k_i += 1
end
k_i = 0
while k_i < 30
    open_trace.push(45.0)
    k_i += 1
end
var ok = valve_cal.detect_open_liftoff_knee(open_trace, size(open_trace) * 95)
check("detect_open_liftoff_knee returns map",        ok != nil)
check("detect_open_liftoff_knee idx near 123",       ok != nil && ok['liftoff_idx'] >= 118 && ok['liftoff_idx'] <= 128)

# ---------------------------------------------------------------------------
# 9. Bootloader (autoexec.be)
# ---------------------------------------------------------------------------
print("\n[9] Bootloader (autoexec.be)")
import autoexec
check("global.gateway populated by autoexec.be",     global.gateway != nil)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print(string.format("\n=== Results: %d passed, %d failed ===", _pass, _fail))
if _fail > 0
    raise 'test_failure', string.format("%d test(s) failed", _fail)
end
