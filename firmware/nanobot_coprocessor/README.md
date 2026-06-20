# nanobot_coprocessor — ESP32 firmware (native zenoh-pico over serial)

The ESP32-WROOM motor/encoder/LDS coprocessor. It runs as a **native zenoh peer over
serial** (no micro-ROS agent, no Fast-DDS), talking straight to the Humble `rmw_zenoh`
graph in rmw_zenoh's exact wire format. Topic contract (see `src/main.cpp` header):

- **sub** `cmd_vel`, `led`, `lds_target_rpm`
- **pub** `wheel_ticks`, `left/right_wheel_suspended`, `esp32_temp`, `esp32_hall`,
  `lds_rpm`, `lds_hz`, `lds_duty`, `esp32_heartbeat`

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

## Wiring (UART2 link via host USB-serial / FTDI adapter, 3.3V logic)
| FTDI | ESP32 |
|------|-------|
| TX   | GPIO16 (UART2 RX) |
| RX   | GPIO17 (UART2 TX) |
| GND  | GND |

Keep the ESP32's own USB (CP2102 / UART0) plugged in for flashing + the debug console.
