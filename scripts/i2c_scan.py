#!/usr/bin/env python3
"""Scan the I2C buses for devices (a dependency-light i2cdetect).

    pixi run python scripts/i2c_scan.py          # all /dev/i2c-* buses
    pixi run python scripts/i2c_scan.py 1        # just bus 1

Expected on this robot: 0x40 (PCA9685) and 0x3c (SSD1306) on bus 1.
Uses a zero-length write probe (fast NAK on absent addresses) rather than
read_byte, which can stall some controllers.
"""
import glob
import sys

from smbus2 import SMBus, i2c_msg

KNOWN = {0x40: "PCA9685", 0x3C: "SSD1306"}


def scan(busnum: int) -> None:
    path = f"/dev/i2c-{busnum}"
    try:
        bus = SMBus(busnum)
    except Exception as exc:
        print(f"{path}: open failed: {exc}")
        return
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.i2c_rdwr(i2c_msg.write(addr, []))
            found.append(addr)
        except Exception:
            pass
    bus.close()
    pretty = " ".join(
        f"0x{a:02x}" + (f"({KNOWN[a]})" if a in KNOWN else "") for a in found
    ) or "(none)"
    print(f"{path}: {pretty}")


def main() -> None:
    if len(sys.argv) > 1:
        buses = [int(a) for a in sys.argv[1:]]
    else:
        buses = sorted(int(p.rsplit("-", 1)[-1]) for p in glob.glob("/dev/i2c-*"))
    if not buses:
        print("no I2C buses found (enable i2c overlays + reboot)")
        return
    for b in buses:
        scan(b)


if __name__ == "__main__":
    main()
