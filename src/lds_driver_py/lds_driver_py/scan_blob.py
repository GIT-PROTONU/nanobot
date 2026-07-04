"""Shared /dev/shm scan-blob writer for the web UI's lidar panel.

Format: one JSON header line ({"seq","amin","ainc","n"} + optional extras), '\n', then the raw float32
ranges. Atomic via .tmp + os.replace so a polling reader never sees a torn file. inf
(no-hit) packs as float32 inf, which the browser treats as "no point". Shared by the
real driver (lds_driver_py.lds_node) and the Gazebo dev-sim bridge (sim_hardware), so
both write byte-for-byte the same thing the web UI already parses.
"""
import array
import json
import os

SCAN_FILE = "/dev/shm/nano_scan.bin"


def write_scan_blob(seq, amin, ainc, ranges, path=SCAN_FILE, extra=None):
    # extra: optional dict of additional header fields (e.g. the real driver's
    # {"lost","err"} RX-health counters). Readers ignore keys they don't know.
    h = {"seq": seq, "amin": amin, "ainc": ainc, "n": len(ranges)}
    if extra:
        h.update(extra)
    header = json.dumps(h).encode() + b"\n"
    tmp = path + ".tmp"
    try:
        with open(tmp, "wb") as f:
            f.write(header)
            f.write(array.array("f", ranges).tobytes())
        os.replace(tmp, path)
    except OSError:
        pass
