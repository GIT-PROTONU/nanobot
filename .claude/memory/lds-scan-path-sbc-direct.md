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
