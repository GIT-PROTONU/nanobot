#!/usr/bin/env python3
"""End-to-end smoke test for the stack's browser gateway + hubs — `pixi run smoke`.

Boots the REAL installed executables on this machine (a zenoh router when the env
ships one, sys_monitor, and app_hub = web_control+oled_display+behavior) and drives
the same surface a browser uses. This is the contract check for the /telemetry frame
(telemetry.py <-> app.js have no type system between them), the publish/param
whitelists, the vitals blob, and the SIGTERM shutdown path. Missing hardware is fine:
every node degrades (no OLED/camera/TTS/LLM) without failing the gateway.

Asserts:
  * GET /            -> 200, serves the control page
  * GET /telemetry   -> SSE frames with the expected keys; OLED face echo after a
                        POST /publish; /diagnostics present when a router is up
  * POST /publish    -> whitelisted ok / non-whitelisted refused / bad value refused
  * POST /param      -> non-whitelisted refused
  * POST /drive      -> clamped echo
  * vitals blob      -> sys_monitor writes /dev/shm/nano_vitals.json with cpu/mem/temp
  * SIGTERM app_hub  -> clean exit (OLED end-screen path), rc 0

Run under the pixi env with install/setup.bash sourced (the pixi task does both).
"""
import http.client
import json
import os
import signal
import subprocess
import sys
import time

PORT = 8096
ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
PARAMS = os.path.join(ROOT, "install", "robot_bringup", "share", "robot_bringup",
                      "config", "robot.yaml")
VITALS = "/dev/shm/nano_vitals.json"

_failures = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))
    if not ok:
        _failures.append(name)


def req(method, path, body=None, timeout=10.0):
    conn = http.client.HTTPConnection("127.0.0.1", PORT, timeout=timeout)
    try:
        conn.request(method, path, body=json.dumps(body) if body is not None else None)
        r = conn.getresponse()
        return r.status, r.read()
    finally:
        conn.close()


def sse_frames(deadline):
    """Yield parsed /telemetry frames until `deadline` (its own connection)."""
    conn = http.client.HTTPConnection("127.0.0.1", PORT, timeout=8.0)
    try:
        conn.request("GET", "/telemetry")
        r = conn.getresponse()
        while time.monotonic() < deadline:
            line = r.fp.readline()
            if not line:
                return
            if line.startswith(b"data: "):
                try:
                    yield json.loads(line[6:])
                except ValueError:
                    pass
    finally:
        conn.close()


def wait_http_up(timeout=25.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            status, body = req("GET", "/", timeout=2.0)
            return status, body
        except OSError:
            time.sleep(0.4)
    return 0, b""


def spawn(name, argv):
    print(f"  starting {name}: {argv[0]}")
    return subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    procs = []
    app = None
    try:
        try:
            os.remove(VITALS)
        except OSError:
            pass

        # Router (optional): needed only for the cross-process /diagnostics assert.
        router_bin = os.path.join(os.environ.get("CONDA_PREFIX", ""),
                                  "lib", "rmw_zenoh_cpp", "rmw_zenohd")
        have_router = os.path.isfile(router_bin)
        if have_router:
            procs.append(spawn("router", [router_bin]))
            time.sleep(3.0)
        else:
            print("  (no rmw_zenohd in env — skipping the cross-process /diagnostics assert)")

        procs.append(spawn("sys_monitor", [
            os.path.join(ROOT, "install", "sys_monitor", "lib", "sys_monitor",
                         "monitor_node"),
            "--ros-args", "--params-file", PARAMS]))
        app = spawn("app_hub", [
            os.path.join(ROOT, "install", "app_hub", "lib", "app_hub", "app_hub"),
            "--ros-args", "--params-file", PARAMS,
            "-p", f"web_control:web_port:={PORT}"])

        status, body = wait_http_up()
        check("control page served", status == 200 and b"NANO" in body,
              f"status={status}")

        # --- telemetry stream: base keys, then the OLED face echo -------------
        first = None
        for f in sse_frames(time.monotonic() + 15.0):
            first = f
            break
        check("telemetry frame arrives", first is not None)
        if first:
            missing = [k for k in ("susp", "oled", "esp", "lds") if k not in first]
            check("frame has base keys", not missing, f"missing={missing}")

        st, body = req("POST", "/publish", {"topic": "/oled_face", "value": "happy"})
        check("publish whitelisted topic", st == 200 and b'"ok"' in body, body[:80])
        echo = False
        for f in sse_frames(time.monotonic() + 6.0):
            if (f.get("oled") or {}).get("face") == "happy":
                echo = True
                break
        check("face echoed back in frame", echo)

        if have_router:
            diag = False
            for f in sse_frames(time.monotonic() + 12.0):
                if "diag" in f and "cpu_percent" in f["diag"]:
                    diag = True
                    break
            check("cross-process /diagnostics in frame", diag)

        # --- whitelists + teleop ----------------------------------------------
        st, body = req("POST", "/publish", {"topic": "/cmd_vel", "value": 1})
        check("non-whitelisted topic refused", b"not whitelisted" in body, body[:80])
        st, body = req("POST", "/publish", {"topic": "/goal_pose", "value": {"a": 1}})
        check("bad goal body refused", b"bad value" in body, body[:80])
        st, body = req("POST", "/param",
                       {"node": "slam_nav", "name": "match_lin", "value": 0})
        check("non-whitelisted param refused", b"not whitelisted" in body, body[:80])
        st, body = req("POST", "/drive", {"v": 0.1, "w": 0.0})
        check("drive accepted", st == 200 and b'"v": 0.1' in body, body[:80])

        # --- vitals blob (written by sys_monitor regardless of the router) -----
        v = {}
        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            try:
                with open(VITALS) as fh:
                    v = json.load(fh)
                break
            except (OSError, ValueError):
                time.sleep(0.4)
        check("vitals blob written", "cpu" in v and "t" in v,
              f"keys={sorted(v)[:8]}")

        # --- clean SIGTERM shutdown (OLED end-screen path) ----------------------
        app.send_signal(signal.SIGTERM)
        try:
            rc = app.wait(timeout=12.0)
        except subprocess.TimeoutExpired:
            rc = None
        check("app_hub exits cleanly on SIGTERM", rc == 0, f"rc={rc}")
        app = None
    finally:
        for p in ([app] if app else []) + procs:
            try:
                p.terminate()
                p.wait(timeout=5.0)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass

    if _failures:
        print(f"\nSMOKE FAILED: {len(_failures)} check(s): {', '.join(_failures)}")
        return 1
    print("\nSMOKE OK — gateway, whitelists, vitals and shutdown all behave.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
