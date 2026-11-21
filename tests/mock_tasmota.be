# Mock Tasmota & I2C Wire environment for standalone Berry unit testing of atb-flow.
# Import this file FIRST before loading gateway.be or autoexec.be.

import string
import global

class MockWire
    var addr
    var _samples_raw    # list of raw 16-bit unsigned values to return sequentially
    var _default_raw    # fallback raw value when _samples_raw is empty
    var _fail_read      # when true, read() returns nil to simulate I2C bus failure

    def init(addr)
        self.addr         = addr
        self._samples_raw = []
        self._default_raw = 250   # 25.0 mA (250 * 0.1 mA)
        self._fail_read   = false
    end

    # Helper: queue a list of currents in mA (can be positive or negative)
    def set_trace_ma(ma_list)
        self._samples_raw = []
        for ma : ma_list
            var raw = int(ma * 10.0)
            if raw < 0
                raw += 65536
            end
            self._samples_raw.push(raw)
        end
    end

    def read(addr, reg, length)
        if self._fail_read || addr != self.addr
            return nil
        end
        if size(self._samples_raw) > 0
            var val = self._samples_raw[0]
            self._samples_raw.remove(0)
            return val
        end
        return self._default_raw
    end
end

class MockTasmota
    var _cmds
    var _timers
    var _epoch_ms
    var last_response
    var _wire
    var _wire_scan_enabled

    def init()
        self._cmds              = {}
        self._timers            = []
        self._epoch_ms          = 1000
        self.last_response      = nil
        self._wire              = MockWire(0x40)
        self._wire_scan_enabled = true
    end

    def add_cmd(name, callback)
        self._cmds[name] = callback
    end

    def resp_cmnd(json_str)
        self.last_response = str(json_str)
    end

    def wire_scan(addr)
        if !self._wire_scan_enabled
            return nil
        end
        if addr == 0x40
            return self._wire
        end
        return nil
    end

    def millis()
        return self._epoch_ms
    end

    # Advance simulated clock during delay(ms) so RawDrive loops progress naturally
    def delay(ms)
        self._epoch_ms += int(ms)
    end

    def set_timer(ms, cb)
        self._timers.push({'t': self._epoch_ms + ms, 'cb': cb})
    end

    def run_timers()
        while size(self._timers) > 0
            var next_t = self._timers[0]
            self._timers.remove(0)
            if next_t['t'] > self._epoch_ms
                self._epoch_ms = next_t['t']
            end
            next_t['cb']()
        end
    end

    def wifi(key)
        var info = {'ip': '192.168.1.50', 'mac': 'AA:BB:CC:DD:EE:FF'}
        if key != nil return info[key] end
        return info
    end

    def eth(key)
        var info = {'ip': '0.0.0.0', 'mac': '00:11:22:33:44:55'}
        if key != nil return info[key] end
        return info
    end

    def memory()
        return {'heap_free': 180000, 'frag': 5}
    end
end

def log(msg, level)
    print(string.format("LOG [%s]: %s", str(level), str(msg)))
end

global.tasmota = MockTasmota()
global.log     = log
