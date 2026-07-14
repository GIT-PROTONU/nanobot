---
name: esp32-hardware-fried-ground-fix
description: ESP32 coprocessor fried (ran hot when powered) from a ground-bounce path through the motor-driver GPIO signal wires — root cause + in-progress hardware fix
metadata: 
  node_type: memory
  type: project
  originSessionId: c294fe08-680a-4e70-b539-29952bb50a47
---

On 2026-07-14 the ESP32 coprocessor got very hot just from being powered (before any motor
command) and was replaced. Root cause, worked out with the user: the DRV8871 motor drivers'
`GND` was only bonded to the ESP32's `GND` **indirectly**, through a buck converter's output
ground, not a direct low-impedance bond. Motor PWM switching + back-EMF flyback current
returning through `GND` needs a solid low-impedance path; without one, some of that current
finds its way back through the next-lowest-impedance path instead — the GPIO signal wires
(ESP32 GPIO → DRV8871 `IN` pin) via the pin's internal ESD/protection diodes. Continuous fault
current through those protection structures reads exactly as "board runs hot even at idle."

**Why:** no dedicated direct ground bond between the DRV8871s and the ESP32 — the buck
converter's output ground was standing in as the only bridge between motor-power ground and
logic ground.

**How to apply:** if the ESP32 (or a GPIO connected to a motor driver) runs hot or dies again,
check this first — measure resistance between DRV8871 `GND` and ESP32 `GND` directly (not
through the buck converter); near-zero is required.

Fix in progress (as of 2026-07-14, hardware rework, not yet firmware):
- **Star ground**: direct low-gauge bond between DRV8871 `GND` (both drivers) and ESP32 `GND`,
  not routed through the buck converter. Primary fix — user is doing this now.
- **Series resistors (100-330Ω) on every GPIO→IN signal line** — cheap insurance against
  residual ground bounce regardless of how clean the star ground turns out; recommended
  regardless of the ground-fix outcome.
- Deferred until after ground+resistor is verified: **additional VM bulk capacitance**
  (existing cap is 47µF/50V per driver — marginal for the ~3A stall current of the Roborock S5
  wheel motor in use; TI-recommended local decoupling for this current range is more like
  100-220µF low-ESR electrolytic + a 0.1-1µF ceramic, both as close to `VM`/`GND` as possible).
  Plan: verify ground+resistor alone resolves it (incl. a deliberate stall test — hold a wheel
  under load) before adding this; it addresses a secondary/compounding factor (VM ripple), not
  the primary fried-GPIO mechanism.
- Also recommended, not yet done: wire the DRV8871 `nFAULT` pins (currently unused, open-drain,
  can be wire-OR'd onto one spare ESP32 GPIO — see [[pin-bus-map]] for free pins) so the
  firmware can detect overcurrent/thermal-shutdown/undervoltage instead of silently continuing
  to command a shut-down driver. Most relevant for a stalled/jammed wheel (this motor's
  realistic worst case, since it's geared for torque and could hold near-stall current
  continuously if physically blocked).

**Replacement board flashed same day** with the latest firmware (see [[esp32-coprocessor]])
and confirmed linked to the SBC (`/esp32_heartbeat` publishing, incrementing) — the hardware
rework above is still pending/in-progress at time of writing, done in parallel with the
firmware being otherwise ready.
