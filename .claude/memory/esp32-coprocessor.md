---
name: esp32-coprocessor
description: "ESP32-WROOM coprocessor — native zenoh-pico (NOT micro-ROS) motor/encoder/LDS, wire contract"
metadata: 
  node_type: memory
  type: project
  originSessionId: 1e16332e-edf3-4369-89c0-03cd717126fc
---

ESP32-WROOM coprocessor offloading real-time motor/encoder/LDS-spin work from the H5 SBC.
Now in `firmware/nanobot_coprocessor/` (the old `firmware/esp32_coprocessor/` micro-ROS build
was deleted upstream — its leftover local `.pio/` cache is untracked, safe to `rm -rf`).

**Architecture (rewritten 2026-06-21): native zenoh-pico over UART, NO micro-ROS / agent /
Fast-DDS.** Joins the SBC's rmw_zenoh graph directly in rmw_zenoh's exact wire format +
liveliness tokens. Needs a **serial-capable zenohd** (conda libzenohc lacks transport_serial)
built via `tools/build_zenohd_serial.sh aarch64`; stack.sh runs it. Link = ESP32 UART2
(TX17/RX16) ↔ SBC /dev/ttyS1. Tunables are inline `#define`s at the top of `src/main.cpp`
(no config.h). Build/flash from dev PC: `cd firmware/nanobot_coprocessor && pio run -t upload`
(pio in ~/pio-venv; do NOT build on the board).

Wire contract: sub `cmd_vel`(Twist), `led`(Bool), `lds_target_rpm`(Float32); pub `wheel_ticks`
(Int64MultiArray [L,R]), `left/right_wheel_suspended`(Bool), `esp32_temp`/`esp32_hall`,
`lds_rpm`/`lds_hz`/`lds_duty`, `esp32_heartbeat`. Also closed-loop PID-controls the LDS02RR
spin motor (reads its RPM off UART1 RX=GPIO14).

**Encoders are SINGLE-CHANNEL** (no quadrature/PCNT) → no hardware direction. Fix applied
2026-06-21 (branch `slam`): ISR signs each tick by the last commanded wheel direction via an
**int8 dir flag** (`g_left_dir/g_right_dir`, set in cmd_cb) — never read a float in the ISR
(ESP32 FPU unsafe). Counts are now signed; `wheel_odometry` integrates them unchanged. This
matters for [[slam-nav]] (unsigned counts made /odom read reverse as forward). Related:
[[deployment-state]], [[project-overview]].

**Low-power + safety additions (2026-07-14):**
- **LDS spin motor parks and CPU downclocks to 80 MHz whenever the SBC isn't genuinely
  present** (gated on `linkAlive()` — see [[esp32-zenoh-pico-integration]] — NOT bare `ready`,
  which false-positives). Before this fix the LDS PID ran unconditionally off `g_lds_target`
  (defaults to 300 rpm at boot) regardless of SBC state, so the lidar kept spinning even with
  the SBC fully powered off. Restores to 240 MHz / resumes spin instantly on reconnect. 80 MHz
  is still PLL-locked so APB/UART baud timing is unaffected.
- **Motor H-bridge pins (25/26/27/33) are driven low as the very first lines of `setup()`**,
  before `Serial.begin()`'s ~300ms startup delay — fixes an observed brief uncommanded spin on
  ESP power-up (the pins sat in their floating ROM-bootloader default for that whole window
  previously).
- Drive-motor safety was already adequate and needed no change: `CMD_TIMEOUT_MS` (500ms)
  zeroes both wheel duties independent of the zenoh link state, so a disconnected/absent SBC
  already meant zero motor duty within 500ms of boot.
- PWM is 20kHz/10-bit for all 4 LEDC channels (drive motors, LDS spin, SBC fan) — see
  `PWM_FREQ_HZ`/`PWM_RES_BITS` near the top of main.cpp. Considered dropping to 5kHz; PWM
  frequency is unrelated to the hardware ground-fault incident (see
  [[esp32-hardware-fried-ground-fix]]) since the ESP32 GPIOs only drive DRV8871's logic-level
  IN pins, not the motor current directly — change deferred, not yet made.

**2026-07-15: SBC cooling fan now gets the same `linkAlive()` park treatment as the LDS
spin motor above.** The fan previously had NO watchdog at all — deliberately, so a brief
SBC hiccup wouldn't stop cooling — but that meant it also just held its last commanded duty
forever after a genuine, intentional SBC shutdown (user: "when the sbc is off but power
still on fan still runs, should be off"). Now `ledcWrite(CH_FAN, …)` only applies
`g_fan_duty` while `alive`; otherwise it's forced to 0 (and `g_fan_duty` itself zeroed) and
resumes the instant `sys_monitor` reconnects. `FAN_BOOT_DUTY` changed 0.4→0.0 (now moot —
the gate zeroes it within one 100 Hz tick of boot regardless). Built + flashed via
`pio run -t upload` over `/dev/ttyUSB0`. See [[cooling-fan-control]] for the SBC-side curve
tuning done alongside this in the same session.

**2026-07-15: right motor harness pins were swapped** — `INVERT_RIGHT` flipped
`false`→`true` (fwd/rev pins for the right DRV8871 were crossed vs the left). This is what
let the robot drive correctly for the first time this session; firmware rebuilt ~20:12
local, right before the test session's nav_node boot, so very likely already flashed.

**2026-07-15: bad-encoder-signal diagnostic + tick reset (built, NOT yet flashed).**
New `/wheel_stray_ticks` (Int64MultiArray [L,R]) counts ISR ticks landing while a wheel is
commanded+settled stopped (`STRAY_SETTLE_MS`=300ms coast-down grace) — should read 0;
nonzero with the robot motionless means encoder-line noise/ground-bounce, not real
rotation (relevant given the ground-bounce origin of the earlier
[[esp32-hardware-fried-ground-fix]] failure). New `/reset_ticks` (Bool) zeros both
`wheel_ticks` and `wheel_stray_ticks`; `wheel_odometry` also watches it and re-seeds its
prev-tick baseline so `/odom` doesn't jump. Web Coprocessor card: "stray ticks L/R"
readout (red if nonzero) + "🔁 Reset ticks" button. `pio run` build verified clean; not
yet flashed to hardware.
