---
name: motors-dead-after-gpio-reassign
description: 2026-07-04 motors stopped driving right after the e60e99e GPIO reassign; LDS/link/encoders fine — suspect removed MOTOR_STBY (GPIO23) or motor IN wiring; also --once pubs to ESP32 can be lost
metadata: 
  node_type: memory
  type: project
  originSessionId: 59a6f545-92fd-428c-bf44-72d88fcd903b
---

**Open issue (2026-07-04):** joystick/`/cmd_vel` no longer drives the wheels, starting right
after commit `e60e99e` (reassign ESP32 GPIOs: motors→25/27/26/33, LDS PWM→18, right
suspend→21, right encoder→5, **MOTOR_STBY (GPIO23) drive removed as "unused"**).

Verified live on the board:
- ESP32 alive on the graph (heartbeat, temp) and its **RX path works**: streaming
  `/lds_target_rpm 300` → duty 0.74, **lidar spun to 299.9 RPM** → the NEW firmware is
  flashed and the new LDS PWM wiring (GPIO18) is correct.
- Streaming `/cmd_vel 0.15 m/s @10 Hz for 1.5 s` → `/wheel_ticks` frozen → motors truly dead,
  not a comms problem.
- Both `/[left|right]_wheel_suspended` read **true** (robot state at the time unknown —
  either lifted, or the switches aren't landed on the new pins 4/21; `INPUT_PULLUP` +
  `ACTIVE_HIGH` means an unwired pin reads "suspended"). Suspend does NOT gate motors
  (publish-only in fw; slam_nav's pickup halt is a one-shot stop), so it's a separate flag,
  not the drive blocker.

**Prime suspect:** the H-bridge STBY line. Firmware used to hold GPIO23 HIGH; if the driver
(TB6612-style) has STBY wired to 23 (now floating) or unconnected, all motor outputs are
disabled while everything else works. Fix candidates: tie STBY to 3.3 V in hardware, or
restore the `pinMode(23,OUTPUT); digitalWrite(23,HIGH)` lines + reflash (harmless if truly
unused). Second suspect: motor IN wires not actually moved to the new 25/27/26/33 layout.

**Gotcha found on the way:** a single `ros2 topic pub --once` to the ESP32 (zenoh-pico over
serial) can be silently lost — stream with `-r`/`-t` when testing. Also `/lds_target_rpm`
had been zeroed since boot (boot default is 300), probably by the web-UI slider restore; I
left it back at 0 after the test. See [[esp32-zenoh-pico-integration]] and [[pin-bus-map]].
