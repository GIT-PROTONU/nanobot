Import("env")
import os

# Pre-build patches applied to the fetched zenoh-pico lib. Each entry is idempotent
# (re-running is a no-op) and survives a lib re-fetch since it runs before every build.
PATCHES = [
    # 1) zenoh-pico's arduino-esp32 serial open calls HardwareSerial(uart).begin(baudrate)
    #    WITHOUT explicit pins, relying on the core's default UART pins. That proved
    #    unreliable here (UART2 didn't drive GPIO17). Force explicit pins from the locator.
    (
        ".pio/libdeps/esp32dev/zenoh-pico/src/system/arduino/esp32/network.cpp",
        "sock->_serial->begin(baudrate);",
        "sock->_serial->begin(baudrate, SERIAL_8N1, rxpin, txpin);",
        "forced explicit UART pins in network.cpp",
    ),
]

for lib, old, new, desc in PATCHES:
    if not os.path.exists(lib):
        print(f"[patch_zenohpico] {lib} not present yet (lib not fetched?)")
        continue
    s = open(lib).read()
    if old in s:
        open(lib, "w").write(s.replace(old, new))
        print(f"[patch_zenohpico] {desc}")
    elif new in s:
        print(f"[patch_zenohpico] already patched: {desc}")
    else:
        print(f"[patch_zenohpico] PATTERN NOT FOUND for: {desc}  (zenoh-pico upstream changed?)")
