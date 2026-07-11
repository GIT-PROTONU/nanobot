"""Minimal PCA9685 16-channel PWM driver over Linux I2C (smbus2).

Deliberately dependency-light (no Adafruit Blinka) because Blinka's board
auto-detection does not know the Allwinner H5 — we just talk to /dev/i2c-N
directly, which works on any mainline kernel.
"""
import time

from smbus2 import SMBus

# Register map
_MODE1 = 0x00
_MODE2 = 0x01
_PRESCALE = 0xFE
_LED0_ON_L = 0x06  # each channel = 4 regs: ON_L, ON_H, OFF_L, OFF_H

# MODE1 bits
_RESTART = 0x80
_SLEEP = 0x10
_AI = 0x20  # auto-increment
# MODE2 bits
_OUTDRV = 0x04  # totem-pole outputs


class PCA9685:
    def __init__(self, bus: int, address: int = 0x40, freq_hz: float = 1000.0):
        self.address = address
        self._bus = SMBus(bus)
        self._bus.write_byte_data(self.address, _MODE2, _OUTDRV)
        self._bus.write_byte_data(self.address, _MODE1, _AI)
        time.sleep(0.005)
        self.set_pwm_freq(freq_hz)

    def set_pwm_freq(self, freq_hz: float):
        # 25 MHz internal osc, 12-bit (4096) resolution.
        prescale = int(round(25_000_000.0 / (4096.0 * freq_hz)) - 1)
        prescale = max(3, min(255, prescale))
        old = self._bus.read_byte_data(self.address, _MODE1)
        self._bus.write_byte_data(self.address, _MODE1, (old & 0x7F) | _SLEEP)
        self._bus.write_byte_data(self.address, _PRESCALE, prescale)
        self._bus.write_byte_data(self.address, _MODE1, old)
        time.sleep(0.005)
        self._bus.write_byte_data(self.address, _MODE1, old | _RESTART | _AI)

    def set_pwm(self, channel: int, on: int, off: int):
        # Mask 0x1F (not 0x0F) on the high bytes so the full-ON/OFF flag (bit 4 =
        # 0x10) survives; the 12-bit count itself never reaches bit 4 of the high byte.
        base = _LED0_ON_L + 4 * channel
        self._bus.write_i2c_block_data(self.address, base,
                                       [on & 0xFF, (on >> 8) & 0x1F,
                                        off & 0xFF, (off >> 8) & 0x1F])

    def set_duty(self, channel: int, duty: float):
        """duty in [0.0, 1.0]."""
        duty = max(0.0, min(1.0, duty))
        if duty <= 0.0:
            self.set_pwm(channel, 0, 0x1000)   # full off (OFF_H bit 4)
        elif duty >= 1.0:
            self.set_pwm(channel, 0x1000, 0)   # full on (ON_H bit 4)
        else:
            self.set_pwm(channel, 0, int(round(duty * 4095)))

    def set_servo_us(self, channel: int, microseconds: float, period_us: float = 20000.0):
        """Servo helper — set the PWM frequency to ~50 Hz first."""
        self.set_duty(channel, microseconds / period_us)

    def all_off(self):
        for ch in range(16):
            self.set_pwm(ch, 0, 0)

    def close(self):
        try:
            self.all_off()
        finally:
            self._bus.close()
