# zenoh-pico ↔ rmw_zenoh serial spike

De-risk for running the ESP32 as a **native zenoh peer over serial** (no micro-ROS
agent, no Fast-DDS), talking to the Humble `rmw_zenoh` graph. One topic:
`std_msgs/Int32` on `/esp32_heartbeat`.

## Status (what's proven)
- ✅ **Wire format**: a zenoh publisher emitting Humble rmw_zenoh's format
  (keyexpr `0/<topic>/<dds_type>/TypeHashNotSupported`, CDR-LE payload, attachment
  `seq(i64)+ts(i64)+0x10+gid[16]`) is received + decoded by `ros2 topic echo` under
  `rmw_zenoh`.
- ✅ **Serial-capable zenohd**: the conda `libzenohc` is built WITHOUT `transport_serial`,
  so the stock `rmw_zenohd` can't do serial. A `zenohd` built with the feature
  (`tools/build_zenohd_serial.sh`) opens a serial listener fine and routes to
  `rmw_zenoh` over TCP.
- ✅ **Firmware builds** and `z_open` establishes the serial handshake. A peer connected
  over serial at least once.
- ⚠️ **Reliable data flow** was NOT achieved over the **USB UART0** (shared with the
  ESP32 boot ROM log, DTR/RTS auto-reset, and console) — see zenoh-pico issue #357.
  → This spike now uses **UART2** for the zenoh link + a **separate USB-serial adapter**,
  leaving UART0/USB for flashing + debug console. This is the community-standard setup.

## zenoh-pico gotcha
The shipped `zenoh-pico/include/zenoh-pico/config.h` **hard-`#define`s** the feature
flags, so `-DZ_FEATURE_*` build flags are silently overridden. We enable serial via
`-DZENOH_GENERIC` + `include/zenoh_generic_config.h` (which sets
`Z_FEATURE_LINK_SERIAL 1`). zenoh-pico's arduino-esp32 serial only supports fixed pin
pairs: UART0=1/3 (USB), UART1=10/9 (flash, unusable), **UART2=17/16** — hence the link
is UART2, locator `serial/17.16#baudrate=115200`.

## Build the serial zenohd (once)
```
tools/build_zenohd_serial.sh x86_64    # -> ./zenohd-x86_64  (dev host)
tools/build_zenohd_serial.sh aarch64   # -> ./zenohd-aarch64 (board, when ready)
```

## Wiring (host USB-serial / FTDI adapter, 3.3V logic)
| FTDI | ESP32 |
|------|-------|
| TX   | GPIO16 (UART2 RX) |
| RX   | GPIO17 (UART2 TX) |
| GND  | GND |

Keep the ESP32's own USB (CP2102 / UART0) plugged in for flashing + the debug console.

## Run the test
```
# 1) flash the firmware (over USB/UART0)
pio run -t upload

# 2) start the serial-capable router on the FTDI device (e.g. /dev/ttyUSB1) + TCP
./zenohd-x86_64 -l 'tcp/[::]:7447' -l 'serial//dev/ttyUSB1#baudrate=115200'

# 3) subscribe under rmw_zenoh (separate shell, in the pixi env)
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 topic echo /esp32_heartbeat std_msgs/msg/Int32

# Watch the ESP32 console (UART0/USB) for [zspike] lines:
#   pio device monitor   (or any 115200 terminal on the CP2102 port)
```
Expect `data: <incrementing>` in the echo, and `[zspike] CONNECTED` + `published #N`
on the console.

## Next steps if it works
- Cross-build `zenohd-aarch64`, drop it on the board, swap it for `rmw_zenohd` in
  `scripts/stack.sh` (rmw_zenoh nodes connect over TCP; ESP32 over serial).
- Port the full firmware (all topics from `firmware/esp32_coprocessor`) to zenoh-pico
  using the proven format, retiring the micro-ROS agent.
