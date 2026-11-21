# Mock gpio module for standalone Berry unit testing of gateway.be
import global

var gpio_mod = module('gpio')
gpio_mod.INPUT  = 0
gpio_mod.OUTPUT = 1
gpio_mod.LOW    = 0
gpio_mod.HIGH   = 1

gpio_mod._modes  = {}
gpio_mod._states = {}
gpio_mod._writes = []

gpio_mod.reset = def()
  gpio_mod._modes  = {}
  gpio_mod._states = {}
  gpio_mod._writes = []
end

gpio_mod.pin_mode = def(pin, mode)
  gpio_mod._modes[pin] = mode
end

gpio_mod.digital_write = def(pin, val)
  gpio_mod._states[pin] = val
  gpio_mod._writes.push([pin, val])
end

gpio_mod.digital_read = def(pin)
  return gpio_mod._states.contains(pin) ? gpio_mod._states[pin] : gpio_mod.LOW
end

return gpio_mod
