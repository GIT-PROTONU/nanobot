# nanobot_coprocessor — ESP32 firmware (native zenoh-pico over serial)

The ESP32-WROOM motor/encoder/LDS coprocessor. It runs as a **native zenoh peer over
serial** (no micro-ROS agent, no Fast-DDS), talking straight to the Humble `rmw_zenoh`
graph in rmw_zenoh's exact wire format. Topic contract (see `src/main.cpp` header):

- **sub** `cmd_vel`, `led`, `lds_target_rpm`, `fan_pwm`, `motor_trim`, `motor_accel`,
  `reset_ticks`, `laser_pwm` (`Int32MultiArray [v1,v2]`, 0..255 → line laser PWM on
  GPIO 23/32 via LEDC ch 6-7; laser 3 was removed 2026-08-18 — see `LASER1_PIN` in main.cpp)
- **pub** `wheel_ticks`, `left/right_wheel_suspended`, `esp32_temp`, `esp32_hall`,
  `lds_rpm`, `lds_hz`, `lds_duty`, `esp32_heartbeat`, `wheel_trim`

## How the link works (and why)
- **Wire format**: the firmware emits Humble rmw_zenoh's format (keyexpr
  `0/<topic>/<dds_type>/TypeHashNotSupported`, CDR-LE payload, attachment
  `seq(i64)+ts(i64)+0x10+gid[16]`) plus liveliness tokens, so a publisher shows up in
  the ROS graph and `ros2 topic echo` decodes it under `rmw_zenoh`.
- **Serial-capable zenohd**: the conda `libzenohc` is built WITHOUT `transport_serial`,
  so the stock `rmw_zenohd` can't do serial. A `zenohd` built with the feature
  (`tools/build_zenohd_serial.sh`) opens a serial listener and routes to `rmw_zenoh`
  over TCP. `scripts/stack.sh` runs this serial zenohd on the board.
- **Link is on UART2**, not the USB UART0. UART0 is shared with the boot ROM log,
  DTR/RTS auto-reset, and console (zenoh-pico issue #357 = unreliable data over it), so
  the zenoh link uses **UART2** + a separate USB-serial adapter, leaving UART0/USB free
  for flashing + the debug console. This is the community-standard setup.

## zenoh-pico gotcha
The shipped `zenoh-pico/include/zenoh-pico/config.h` **hard-`#define`s** the feature
flags, so `-DZ_FEATURE_*` build flags are silently overridden. We enable serial via
`-DZENOH_GENERIC` + `include/zenoh_generic_config.h` (which sets
`Z_FEATURE_LINK_SERIAL 1`). zenoh-pico's arduino-esp32 serial only supports fixed pin
pairs: UART0=1/3 (USB), UART1=10/9 (flash, unusable), **UART2=17/16** — hence the link
is UART2.

## Build the serial zenohd (once)
```
tools/build_zenohd_serial.sh x86_64    # -> ./zenohd-x86_64  (dev host)
tools/build_zenohd_serial.sh aarch64   # -> ./zenohd-aarch64 (board)
```

## Build / flash (dev PC only — not on the board)
```
pio run -t upload          # flash over USB/UART0
pio device monitor         # debug console (115200 on the CP2102 port)
```
Tunables (pins, PID gains, diff-drive limits) are inline `#define`s at the top of
`src/main.cpp` (there is no `include/config.h`).

### Console capture without a TTY (`pio device monitor` fails in non-interactive shells)
`pio device monitor` needs a real terminal (tcgetattr on the CP2102), so it fails in a
scripted / non-interactive session. Read the console directly instead:

```python
import serial, time
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.5, rtscts=False, dsrdtr=False)
# reset once via RTS (EN low) then release, then LEAVE the lines alone —
# every serial.Serial() open pulses DTR/RTS on the CP2102 auto-reset circuit,
# which drops the chip into ROM download mode ("waiting for download") and
# looks like a reset storm / boot loop. It is NOT a firmware problem.
ser.setRTS(True); time.sleep(0.05); ser.setRTS(False)
time.sleep(1.0)
t0 = time.time()
while time.time() - t0 < 5:
    d = ser.read(8192)
    if d:
        sys.stdout.write(d.decode(errors="replace")); sys.stdout.flush()
```

Verified 2026-08-11: a clean reset boots the current firmware stably
(`[nano] ticks L=… | lds rpm≈300 hz≈450`, no resets). Repeated port opens that pulse
DTR/RTS produce the ROM `waiting for download` / `try 0x400805e4` noise instead — that
is an artifact of line toggling, not a flash or firmware fault.

## Wiring (UART2 link via host USB-serial / FTDI adapter, 3.3V logic)
| FTDI | ESP32 |
|------|-------|
| TX   | GPIO16 (UART2 RX) |
| RX   | GPIO17 (UART2 TX) |
| GND  | GND |

Keep the ESP32's own USB (CP2102 / UART0) plugged in for flashing + the debug console.
