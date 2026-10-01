#!/usr/bin/env python3
"""
V3 system-test client: drives the running V3 system ONLY through its web API (exactly what the UI
does) and records the observed mission-state timeline.

Library functions are used by test_v3_system.py / run_v3_missions.py; CLI for ad-hoc use:
  python3 v3_api_client.py start [source destination] | pause | resume | cancel | reset | estop | state
"""
import json
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8080"
TERMINAL = {"MISSION_COMPLETE", "FAILED", "CANCELLED", "EMERGENCY_STOP"}


def get(path, timeout=5):
    with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
        return json.loads(r.read())


def post(path, body, timeout=5):
    req = urllib.request.Request(BASE + path, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def cmd(c, **kw):
    return post("/api/mission", {"cmd": c, **kw})


def state():
    return get("/api/state")


def wait_state(pred, timeout, poll=0.5, timeline=None):
    """Poll /api/state until pred(state) or timeout; returns (state, ok). Appends state changes to timeline."""
    end = time.time() + timeout
    last = None
    s = None
    while time.time() < end:
        try:
            s = state()
        except Exception:
            time.sleep(poll)
            continue
        ms = s.get("mission", {}).get("state")
        if timeline is not None and ms != last:
            timeline.append({"t": round(time.time(), 2), "state": ms, "de_state": s.get("mission", {}).get("decision_engine_state")})
            last = ms
        if pred(s):
            return s, True
        time.sleep(poll)
    return s, False


def mission_state(s):
    return (s or {}).get("mission", {}).get("state")


def run_mission(source="collection_point", destination="laboratory", timeout=900, timeline=None):
    """Start a mission and wait for a terminal state. Returns (final_state, mission_id, timeline)."""
    timeline = [] if timeline is None else timeline
    s, ok = wait_state(lambda s: mission_state(s) in ("READY", "MISSION_COMPLETE"), 60, timeline=timeline)
    if not ok:
        return mission_state(s), None, timeline
    r = cmd("start", source=source, destination=destination)
    s, ok = wait_state(lambda s: (s.get("mission", {}).get("mission") or {}).get("source") == source
                       and mission_state(s) not in ("READY",), 30, timeline=timeline)
    mid = ((s or {}).get("mission", {}).get("mission") or {}).get("mission_id")
    s, ok = wait_state(lambda s: mission_state(s) in TERMINAL, timeout, timeline=timeline)
    return mission_state(s), mid, timeline


def history_record(mission_id):
    return get(f"/api/history/{mission_id}")


if __name__ == "__main__":
    a = sys.argv[1:] or ["state"]
    if a[0] == "state":
        print(json.dumps(state(), indent=1)[:4000])
    elif a[0] == "start":
        print(cmd("start", **({"source": a[1], "destination": a[2]} if len(a) == 3 else {})))
    else:
        print(cmd(a[0]))
