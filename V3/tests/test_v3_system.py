#!/usr/bin/env python3
"""
Phase 16 system tests against the RUNNING V3 system (start it with V3/scripts/start_v3.sh first).
Everything goes through the web API, exactly as the UI does. Each test records what was observed;
a test is PASS only if its explicit check held. Output: V3/docs/evidence/new_hospital/system_tests.json

  T01 state API + all subsystems up         T07 cancel during navigation -> CANCELLED, robot stops
  T02 UI page, map, camera snapshot/stream  T08 reset -> READY
  T03 command whitelist (no shell / unknown T09 e-stop during navigation -> EMERGENCY_STOP, cmd held at 0,
      command / bad location rejected)          start refused while latched, reset clears it
  T04 start mission -> navigation begins    T10 LAN URL serves the UI (private address)
  T05 pause -> robot stops (twist ~0)        T11 history API lists the test missions with their outcome
  T06 resume -> robot moves again           T12 full mission collection_point -> laboratory COMPLETE
usage: test_v3_system.py [--skip-full]
"""
import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v3_api_client as api  # noqa: E402

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(V3_ROOT, "docs", "evidence", "new_hospital", "system_tests.json")
results = []


def rec(tid, name, ok, **obs):
    results.append({"id": tid, "name": name, "result": "PASS" if ok else "FAIL", "observed": obs})
    print(f"{tid} {'PASS' if ok else 'FAIL'} {name} {json.dumps(obs)[:300]}", flush=True)


def speed(s):
    tw = (s or {}).get("robot", {}).get("twist") or [0, 0]
    return abs(tw[0]), abs(tw[1])


def max_speed_over(sec):
    m = 0.0
    end = time.time() + sec
    while time.time() < end:
        m = max(m, speed(api.state())[0])
        time.sleep(0.2)
    return m


def http_status(url, data=None):
    try:
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"} if data else {})
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, r.read(200000)
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def wait_moving(timeout=90):
    return api.wait_state(lambda s: speed(s)[0] > 0.05 and api.mission_state(s) in ("GO_TO_COLLECTION", "GO_TO_LAB"), timeout)


def reset_ready():
    api.cmd("reset")
    return api.wait_state(lambda s: api.mission_state(s) == "READY", 30)[1]


def main():
    skip_full = "--skip-full" in sys.argv
    mids = []
    # T01
    s = api.state()
    sysd = s["system"]
    up = [k for k, v in sysd.items() if v is True]
    down = [k for k, v in sysd.items() if v is False]
    rec("T01", "state API, all subsystems up, mission READY", not down and api.mission_state(s) == "READY",
        up=up, down=down, mission_state=api.mission_state(s), ready_conditions=s["mission"]["ready_conditions"],
        rates_hz={k: v.get("hz") for k, v in sysd.get("topics", {}).items()})
    # T02
    st_ui, body = http_status(api.BASE + "/")
    st_map, mp = http_status(api.BASE + "/generated/map.png")
    st_jpg, jpg = http_status(api.BASE + "/api/camera.jpg")
    sock = socket.create_connection(("127.0.0.1", 8080), timeout=5)
    sock.sendall(b"GET /api/camera.mjpg HTTP/1.0\r\nHost: x\r\n\r\n")
    buf, end = b"", time.time() + 3
    while time.time() < end and buf.count(b"\xff\xd8") < 5:
        chunk = sock.recv(65536)
        if not chunk:
            break
        buf += chunk
    sock.close()
    frames = buf.count(b"\xff\xd8")
    rec("T02", "UI page, map image, camera snapshot and MJPEG stream", st_ui == 200 and (b"<html" in body.lower() or b"<title" in body.lower())
        and st_map == 200 and mp[:4] == b"\x89PNG" and st_jpg == 200 and jpg[:2] == b"\xff\xd8" and frames >= 5,
        ui=st_ui, map=st_map, map_png=mp[:4] == b"\x89PNG", jpg=st_jpg, jpg_magic=jpg[:2] == b"\xff\xd8", mjpeg_frames_in_3s=frames)
    # T03
    probes = {
        "unknown_cmd": {"cmd": "shutdown"},
        "shell_injection": {"cmd": "start; rm -rf /"},
        "exec_field": {"cmd": "exec", "args": "id"},
        "bad_location": {"cmd": "start", "source": "../../etc", "destination": "laboratory"},
        "same_location": {"cmd": "start", "source": "laboratory", "destination": "laboratory"},
    }
    codes = {k: http_status(api.BASE + "/api/mission", json.dumps(v).encode())[0] for k, v in probes.items()}
    time.sleep(1.5)
    still_ready = api.mission_state(api.state()) == "READY"
    rec("T03", "command whitelist rejects unknown/injected commands and invalid locations",
        all(c == 400 for c in codes.values()) and still_ready, http_codes=codes, mission_still_READY=still_ready)
    # T04
    r = api.cmd("start", source="collection_point", destination="laboratory")
    s, moving = wait_moving()
    mid = ((s or {}).get("mission", {}).get("mission") or {}).get("mission_id")
    mids.append(mid)
    rec("T04", "start mission -> GO_TO_COLLECTION and robot moves", moving, response=r, mission_id=mid,
        state=api.mission_state(s), speed=speed(s))
    # T05
    api.cmd("pause")
    s, paused = api.wait_state(lambda s: api.mission_state(s) == "PAUSED", 10)
    time.sleep(2.0)
    vmax = max_speed_over(4.0)
    pose_a = api.state()["robot"]["pose"]
    time.sleep(3.0)
    pose_b = api.state()["robot"]["pose"]
    drift = ((pose_a[0] - pose_b[0]) ** 2 + (pose_a[1] - pose_b[1]) ** 2) ** 0.5
    rec("T05", "pause -> PAUSED and robot stops", paused and vmax < 0.03 and drift < 0.05,
        state=api.mission_state(s), max_linear_speed_while_paused=round(vmax, 3), pose_drift_3s_m=round(drift, 3))
    # T06
    api.cmd("resume")
    s, moving = wait_moving(60)
    rec("T06", "resume -> navigation state restored and robot moves", moving, state=api.mission_state(s), speed=speed(s))
    # T07
    time.sleep(3)
    api.cmd("cancel")
    s, cancelled = api.wait_state(lambda s: api.mission_state(s) == "CANCELLED", 10)
    time.sleep(2.0)
    vmax = max_speed_over(3.0)
    rec("T07", "cancel during navigation -> CANCELLED and robot stops", cancelled and vmax < 0.03,
        state=api.mission_state(s), max_linear_speed_after_cancel=round(vmax, 3))
    # T08
    ok = reset_ready()
    rec("T08", "reset after cancel -> READY", ok, state=api.mission_state(api.state()))
    # T09
    api.cmd("start", source="collection_point", destination="laboratory")
    s, moving = wait_moving()
    mids.append(((s or {}).get("mission", {}).get("mission") or {}).get("mission_id"))
    api.cmd("estop")
    s, stopped = api.wait_state(lambda s: api.mission_state(s) == "EMERGENCY_STOP", 5)
    t_cmd = time.time()
    # 0.5 s braking allowance: the API twist is sampled from /odom and still shows the pre-stop speed while
    # the robot brakes (direct /odom + /cmd_vel measurement, new hospital: 0.31 -> 0.0 m/s within 0.5 s,
    # no non-zero /cmd_vel after the e-stop). The hold is then checked for 4 s.
    time.sleep(0.5)
    vmax = max_speed_over(4.0)
    latched = api.state()["mission"]["estop_latched"]
    api.cmd("start", source="collection_point", destination="laboratory")
    time.sleep(2)
    refused = api.mission_state(api.state()) == "EMERGENCY_STOP"
    cleared = reset_ready() and not api.state()["mission"]["estop_latched"]
    rec("T09", "e-stop -> EMERGENCY_STOP, robot held still, start refused while latched, reset clears",
        moving and stopped and vmax < 0.03 and latched and refused and cleared,
        was_moving=moving, state=api.mission_state(s), max_linear_speed_4s_after_estop=round(vmax, 3),
        latched=latched, start_refused_while_latched=refused, reset_cleared=cleared, check_window_s=round(time.time() - t_cmd, 1))
    # T10
    lan = subprocess.run(["bash", "-c", "ip -4 route get 1.1.1.1 | awk '{for(i=1;i<=NF;i++) if($i==\"src\"){print $(i+1);exit}}'"],
                         capture_output=True, text=True).stdout.strip()
    st_lan, body = http_status(f"http://{lan}:8080/") if lan else (None, b"")
    st_lan_api, _ = http_status(f"http://{lan}:8080/api/state") if lan else (None, b"")
    import ipaddress
    private = bool(lan) and ipaddress.ip_address(lan).is_private
    rec("T10", "LAN URL serves UI and API (private address only)", st_lan == 200 and st_lan_api == 200 and private,
        lan_ip=lan, private=private, ui=st_lan, api=st_lan_api)
    # T12 (full mission) before T11 so it appears in history
    full = None
    if not skip_full:
        tl = []
        final, fmid, tl = api.run_mission("collection_point", "laboratory", timeout=900, timeline=tl)
        mids.append(fmid)
        full = {"final": final, "mission_id": fmid, "timeline": tl}
    # T11
    hist = api.get("/api/history")
    rows = hist if isinstance(hist, list) else hist.get("missions", [])
    by_id = {h.get("mission_id"): h for h in rows}
    found = {m: (by_id.get(m) or {}).get("result") for m in mids if m}
    rec("T11", "history API lists the test missions with outcome", all(found.values()) and len(found) == len([m for m in mids if m]),
        missions=found)
    if full is not None:
        detail = api.history_record(full["mission_id"]) if full["mission_id"] else {}
        rec("T12", "full mission collection_point -> laboratory completes", full["final"] == "MISSION_COMPLETE",
            final=full["final"], mission_id=full["mission_id"], states=[t["state"] for t in full["timeline"]],
            duration_s=detail.get("duration_s"), distance_m=detail.get("distance_m"), replans=detail.get("replans"),
            failure_reason=detail.get("failure_reason"))
    else:
        rec("T12", "full mission (skipped by --skip-full)", False, skipped=True)
    reset_ready()
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({"run_at": time.strftime("%Y-%m-%d %H:%M:%S"), "results": results}, open(OUT, "w"), indent=1)
    print("PASSED", sum(r["result"] == "PASS" for r in results), "/", len(results))
    return 0 if all(r["result"] == "PASS" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
