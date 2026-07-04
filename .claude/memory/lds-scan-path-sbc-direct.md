---
name: lds-scan-path-sbc-direct
description: Web UI lidar points come ONLY from the SBC reading ttyS2 directly; ESP reads RPM only and never relays scan
metadata: 
  node_type: memory
  type: project
  originSessionId: 4086bf11-99d4-4415-8d86-92d2933de963
---

The LDS02RR data line **fans out to two independent readers**, and they are NOT a
relay chain:
- **Scan** → SBC **UART2 `/dev/ttyS2`** (RX = **PA1**) → `lds_driver_py` → `/scan`
  + `/dev/shm/nano_scan.bin` → web UI polls `/scan.bin`. This is the ONLY source of
  the map/scan points in the web UI.
- **RPM** → ESP32 **UART1 GPIO14** → `/lds_rpm`/`/lds_hz`. The ESP reads **only the
  RPM and ignores the scan payload** — it does **not** forward scan data to the SBC.

**Therefore: "ESP sees the lidar / RPM is fine but the web UI shows no points" is a
SBC-receive problem, not a lidar or software problem.** The two paths share only the
lidar's TX line + a common ground; the SBC branch (PA1) can be dead while the ESP
branch works.

**Decisive, code-independent test** (used 2026-06-23): stop the sensor node and read
the raw port — `timeout 3 cat /dev/ttyS2`. **0 bytes = no electrical signal on PA1**
(wiring), not config/driver. A wrong baud/driver would give garbage or a parse error,
never silence.

**Even better — kernel UART counters, no node-stop needed (used 2026-07-04):**
`sudo cat /proc/tty/driver/serial` twice a few seconds apart. Port 2 = ttyS2. A healthy
LDS02RR stream is ~9.9 kB/s (450 pkt/s × 22 B) with `fe`/`brk` flat. That day: rx grew
only ~1 kB/s while **`fe` (framing errors) grew ~550/s and `brk` ~5/s**, raw bytes had
no `FA` sync at all, yet ESP32 reported rpm 304 / 430 valid Hz — i.e. **signal present
but degraded on the SBC branch only** (loose/oxidised wire, cold joint, or bad ground
to PA1 — header pin 22; PA0/UART2-TX is pin 11). Framing errors at correct stty baud =
electrical, not software. After a wiring fix, verify: `fe` stops climbing, rx ≈ 10 kB/s,
`/dev/shm/nano_scan.bin` appears. Caveat: `pkill -f sensor_hub` self-kills your own SSH shell (the pattern
is in its argv) — write a script FILE and run it **by path** so its argv is just the
path (same gotcha as [[deployment-state]]'s plink note). Restore with `stack.sh up`.

Confirmed clean that day: `lds_node.py` + the `lds_driver:` block of `robot.yaml`
(`port: /dev/ttyS2`, `baud: 115200`) were unchanged; overlay `uart2` present; driver
logs `LDS open on /dev/ttyS2 @115200`; `stty` 115200 — yet raw read = 0 bytes. Root
cause is physical on PA1: TX/RX swap (landed on PA0/UART2-TX), missing common ground to
the SBC, or the SBC branch of the TX split is open.

**2026-07-04 (later, still open): now FULLY dead — kernel counters `rx:0` since boot**
(`2: uart:U6_16550A mmio:0x01C28800 ... tx:0 rx:0`, flat across samples) while ESP32 still
gets good frames. Worse than the morning's degraded-but-present state → the PA1 branch went
from marginal to open, almost certainly disturbed in the motor/GPIO rewiring session
([[motors-dead-after-gpio-reassign]] — same day, same loom). Check the LDS-TX→PA1 (header
pin 22) splice first. **Driver-side blind spot fixed the same day:** the `lost`/`err`
RX-health counters only rode the scan blob, which `_publish` writes **only after a complete
valid revolution** — so a fully-dead RX showed *nothing* in the web UI. `lds_node.py` now has
a 2 s health heartbeat (`_health_tick`) writing a points-free blob
`{stale:1, rx:<bytes>, err:<crc>, open:0|1}` whenever scans stop, and `index.html` renders it
as "port open failed" / "no RX data (wiring?)" / "RX stopped" / "RX garbled · err N" in red —
so dead/garbled/stopped are now distinguishable at a glance, no ssh needed.

**RESOLVED 2026-07-04 (evening): explicit floating RX inputs on BOTH branches fixed it.**
The LDS's weak TX drives two receivers, and both had internal bias on their RX pin:
the ESP32's UART driver (`uart_set_pin` inside `Serial1.begin`) silently enables the
~45k pull-up on GPIO14, and the SBC's PA1 bias was unspecified. Fix: firmware now calls
`gpio_set_pull_mode(LDS_RX_PIN, GPIO_FLOATING)` right after `Serial1.begin` (main.cpp),
and the SBC gets a `uart2-rx-float` user overlay (`bias-disable` merged into the
kernel's `&uart2_pins` label — PA0/PA1; built+installed by `deploy/sbc-setup.sh`, same
pattern as i2c0-400k). After reflash + reboot: ttyS2 stream clean (`lost:0 err:0`,
scan blob updating every rev, ~9 kB/s rx), ESP32 link fine. Verify the overlay took
with `ls /proc/device-tree/soc/pinctrl@1c20800/uart2-pins/` → `bias-disable` present.
Lesson: on a fanned-out weak TX line, every receiver pin must be a true high-impedance
input — a hidden driver-default pull-up on ONE branch can corrupt or kill the stream
on the OTHER. (Marginal wiring from the same-day rewiring session may have contributed;
the explicit-float config removes the bias variable permanently.)

**Port-health proof — RX↔TX loopback (2026-06-23):** to test the UART itself, bridge
the LDS's RX and TX pins, stop `sensor_hub` (it holds `/dev/ttyS2`; killing it was
needed for *exclusive* access — a concurrent test gives **false negatives** because
the live driver eats the looped-back bytes), then write+read frames on the port. 5/5
echoed byte-for-byte @115200 = port fully healthy. That day the LDS outage turned out
to be a **faulty USB power supply**, NOT the UART or wiring — see
[[slam-map-empty-lidar-spin]]. So order of checks: power → loopback the port → then
PA1 wiring. Run the test from a script FILE by path (same self-kill gotcha as above).

Doc gotcha: `nanopi-neo-plus2-pinmap.md` lists UART2's base as `1c2dc00` — a **typo**;
the live `/dev/ttyS2` is `1c28800.serial` = UART2 (PA0/PA1). See [[pin-bus-map]],
[[slam-map-empty-lidar-spin]] (the other branch: RPM/Hz=0 = lidar unpowered), and
[[esp32-zenoh-pico-integration]] (the dual-read design).
