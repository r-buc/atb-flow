import global
import string
import gateway

# ---------------------------------------------------------------------------
# Phase 0 / Phase 1: Dumb Valve Gateway bootloader
# Immediately initializes DumbValveGateway on boot so all motor driver pins
# (DRV8833 AIN/BIN and EEP/STBY) are actively pulled LOW before network init.
# ---------------------------------------------------------------------------
global.gateway = gateway

def _log_boot_status()
  var ip = tasmota.wifi('ip')
  if !ip || ip == '' || ip == '0.0.0.0'
    ip = tasmota.eth('ip')
  end
  if !ip || ip == '' || ip == '0.0.0.0'
    tasmota.set_timer(1000, _log_boot_status)
    return
  end
  var mem = tasmota.memory()
  log(string.format("ATB-FLOW: gateway ready on %s (heap_free=%d)", ip, mem['heap_free']), 2)
end

_log_boot_status()
