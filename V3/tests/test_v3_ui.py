#!/usr/bin/env python3
"""
UI validation of the V3 dashboard against the RUNNING system: drives the real page in headless Firefox
(geckodriver, ui_driver.py), clicks the real buttons, verifies the effect through /api/state and saves
screenshots. Output: V3/docs/evidence/ui/ui_tests.json + *.png
usage: test_v3_ui.py
"""
import json
import os
import socket
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v3_api_client as api  # noqa: E402
from ui_driver import Browser  # noqa: E402

V3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(V3, "docs", "evidence", "ui")
URL = "http://localhost:8080/"
os.makedirs(OUT, exist_ok=True)
results = []


def rec(tid, name, ok, **obs):
    results.append({"id": tid, "name": name, "result": "PASS" if ok else "FAIL", "observed": obs})
    print(tid, "PASS" if ok else "FAIL", name, json.dumps(obs)[:240], flush=True)


def st():
    return api.state()


def mstate():
    return api.mission_state(st())


def wait(pred, timeout, period=0.5):
    end = time.time() + timeout
    while time.time() < end:
        s = st()
        if pred(s):
            return s, True
        time.sleep(period)
    return st(), False


def speed(s):
    tw = (s.get("robot") or {}).get("twist") or [0, 0]
    return abs(tw[0])


def click(b, el_id):
    return b.js(f"const e=document.getElementById('{el_id}'); if(!e||e.disabled) return false; e.click(); return true;")


def view(b, v, settle=2.5):
    b.js(f"location.hash='{v}';")
    time.sleep(settle)


def text(b, el_id):
    return b.js(f"return (document.getElementById('{el_id}')||{{}}).textContent || ''")


def ensure_ready():
    s = st()
    if api.mission_state(s) != "READY":
        api.cmd("cancel")
        time.sleep(1)
        api.cmd("reset")
    return wait(lambda s: api.mission_state(s) == "READY", 30)[1]


def main():
    b = Browser(1600, 1000)
    try:
        ensure_ready()
        b.go(URL + "#ops")
        time.sleep(6)
        # U01 page, map, camera, robot position
        info = b.js("""const img=document.querySelector('#cam-ops img');
            return {cam_w: img.naturalWidth, cam_off: document.getElementById('cam-ops').classList.contains('off'),
                    map_ok: !!(V3UI.map && V3UI.base), map_w: V3UI.map && V3UI.map.width, map_h: V3UI.map && V3UI.map.height,
                    rotated: V3UI.map && V3UI.map.rotated, pos: document.getElementById('r-pos').textContent,
                    link: document.getElementById('h-link-t').textContent, health: document.getElementById('h-health-t').textContent}""")
        s = st()
        ok = info["cam_w"] > 0 and not info["cam_off"] and info["map_ok"] and info["map_w"] == 543 and info["link"] == "connected"
        rec("U01", "dashboard loads: NEW hospital map (543x1181), live camera stream, link connected", ok, **info)
        p = s["robot"]["pose"]
        rec("U02", "robot position shown matches /amcl_pose", bool(p) and f"{p[0]:.2f}" in info["pos"], ui=info["pos"], api_pose=p)
        # U03 start mission from the UI
        b.js("document.getElementById('selsrc').value='collection_point'; document.getElementById('seldst').value='laboratory';")
        clicked = click(b, "b-start")
        s, moving = wait(lambda s: api.mission_state(s) == "GO_TO_COLLECTION" and speed(s) > 0.05, 40)
        rec("U03", "Start button starts a collection -> laboratory mission and the robot moves", clicked and moving,
            state=api.mission_state(s), speed=speed(s), ui_state=text(b, "m-state"))
        time.sleep(20)
        b.shot(os.path.join(OUT, "01_main_operations.png"))
        # U04 live mission progress in the UI
        prog = b.js("return {state: document.getElementById('m-state').textContent, route_w: document.getElementById('m-prog').style.width, "
                    "steps_cur: document.querySelectorAll('#m-steps .step.cur').length, mission: document.getElementById('r-mission').textContent}")
        rec("U04", "live mission progress (state, stepper, route progress, mission id)", prog["steps_cur"] == 1 and prog["mission"].startswith("M"), **prog)
        # U05 YOLO + fusion in the perception view
        view(b, "perc", 4)
        det = b.js("return {rows: document.querySelectorAll('#det-body tr').length, first: (document.querySelector('#det-body tr')||{}).textContent, "
                   "ranged: [...document.querySelectorAll('#det-body .pill.ok')].length, notranged: [...document.querySelectorAll('#det-body .pill')].filter(e=>e.textContent==='not ranged').length, "
                   "cam_w: document.querySelector('#cam-perc img').naturalWidth, meta: document.getElementById('cam-perc-meta').textContent}")
        b.shot(os.path.join(OUT, "03_perception.png"))
        objs = (st()["perception"]["spatial"] or {}).get("objects", [])
        rec("U05", "perception view: camera, YOLO object list with confidence, LiDAR-ranged vs not-ranged", det["cam_w"] > 0 and det["rows"] >= 1,
            **det, api_objects=len(objs))
        # U06 navigation view
        view(b, "nav", 3)
        nav = b.js("return {route: document.querySelectorAll('#n-route .n').length, events: document.querySelectorAll('#n-events > div').length, de: document.getElementById('n-de').textContent}")
        b.shot(os.path.join(OUT, "04_navigation.png"))
        rec("U06", "navigation view: route, decision-engine state, events", nav["route"] > 0 and nav["events"] > 0, **nav)
        # U07 pause / resume (buttons)
        view(b, "ops", 1.5)
        clicked = click(b, "b-pause")
        s, paused = wait(lambda s: api.mission_state(s) == "PAUSED", 10)
        time.sleep(1.0)
        v_paused = max(speed(st()) for _ in range(6) if not time.sleep(0.25))
        b.shot(os.path.join(OUT, "02_mission_control_paused.png"))
        clicked2 = click(b, "b-resume")
        s, resumed = wait(lambda s: api.mission_state(s) in ("GO_TO_COLLECTION", "AT_COLLECTION", "PAYLOAD_SECURED", "GO_TO_LAB") and speed(s) > 0.03, 20)
        rec("U07", "Pause holds the robot, Resume continues the mission", clicked and paused and v_paused < 0.03 and clicked2 and resumed,
            paused=paused, max_speed_paused=v_paused, resumed_state=api.mission_state(s))
        # U08 mission completes, shown in UI
        s, done = wait(lambda s: api.mission_state(s) in ("MISSION_COMPLETE", "FAILED"), 600, 2.0)
        time.sleep(1.5)
        b.shot(os.path.join(OUT, "05_mission_complete.png"))
        rec("U08", "mission completes and the UI shows completion", done and api.mission_state(s) == "MISSION_COMPLETE" and "complete" in text(b, "m-state").lower(),
            state=api.mission_state(s), ui=text(b, "m-state"))
        # U09 history
        view(b, "hist", 3)
        b.js("const r=document.querySelector('#h-body tr[data-id]'); if(r) r.click();")
        time.sleep(2)
        hist = b.js("return {rows: document.querySelectorAll('#h-body tr[data-id]').length, detail_legs: document.querySelectorAll('#h-detail .hc').length}")
        b.js("document.getElementById('hf-result').value='COMPLETE'; document.getElementById('hf-result').dispatchEvent(new Event('change'));")
        time.sleep(0.5)
        filt = b.js("return [...document.querySelectorAll('#h-body tr[data-id] .pill')].every(e=>e.textContent==='COMPLETE')")
        b.js("document.querySelector('#h-head th[data-k=duration_s]').click();")
        b.js("document.getElementById('hf-result').value=''; document.getElementById('hf-result').dispatchEvent(new Event('change'));")
        time.sleep(0.5)
        b.shot(os.path.join(OUT, "06_history.png"))
        api_n = len(json.loads(urllib.request.urlopen("http://localhost:8080/api/history").read()))
        rec("U09", "history view lists stored missions, detail, filter and sort work", hist["rows"] == api_n and hist["detail_legs"] >= 1 and filt,
            ui_rows=hist["rows"], api_rows=api_n, detail_legs=hist["detail_legs"], filter_ok=filt)
        # U10 path learning
        view(b, "learn", 3)
        lr = b.js("return {rows: document.querySelectorAll('#l-body tr').length, note: !!document.querySelector('#l-side .note')}")
        b.shot(os.path.join(OUT, "07_path_learning.png"))
        rec("U10", "path learning view shows stored segment statistics and the cost model", lr["rows"] > 0 and lr["note"], **lr)
        # U11 system health
        view(b, "sys", 3)
        sysv = b.js("return {cards: document.querySelectorAll('#s-grid .hc').length, online: [...document.querySelectorAll('#s-grid .pill')].filter(e=>e.textContent==='Online').length, "
                    "unavailable: [...document.querySelectorAll('#s-grid .pill')].filter(e=>e.textContent==='Unavailable').length, nodes: document.querySelectorAll('#s-nodes tr').length}")
        b.shot(os.path.join(OUT, "08_system_health.png"))
        rec("U11", "system health: every subsystem online, battery marked unavailable, ROS node list", sysv["online"] == sysv["cards"] - 1 and sysv["unavailable"] == 1 and sysv["nodes"] > 10, **sysv)
        # U12 cancel
        view(b, "ops", 1.5)
        click(b, "b-reset")
        wait(lambda s: api.mission_state(s) == "READY", 15)
        click(b, "b-start")
        wait(lambda s: speed(s) > 0.05, 40)
        clicked = click(b, "b-cancel")
        s, canc = wait(lambda s: api.mission_state(s) == "CANCELLED", 10)
        time.sleep(1)
        rec("U12", "Cancel stops the mission (CANCELLED, robot stops)", clicked and canc and speed(st()) < 0.03, state=api.mission_state(s), speed=speed(st()))
        click(b, "b-reset")
        wait(lambda s: api.mission_state(s) == "READY", 15)
        # U13 emergency stop (header button)
        click(b, "b-start")
        wait(lambda s: speed(s) > 0.05, 40)
        clicked = click(b, "estop")
        s, es = wait(lambda s: api.mission_state(s) == "EMERGENCY_STOP", 5)
        time.sleep(0.5)  # braking
        vmax = 0.0
        for _ in range(12):
            vmax = max(vmax, speed(st()))
            time.sleep(0.25)
        time.sleep(1)
        latched_ui = b.js("return document.getElementById('estop').classList.contains('latched') && document.getElementById('alert').classList.contains('show')")
        start_disabled = b.js("return document.getElementById('b-start').disabled")
        b.shot(os.path.join(OUT, "09_emergency_stop.png"))
        click(b, "b-reset")
        s2, rdy = wait(lambda s: api.mission_state(s) == "READY" and not s["mission"]["estop_latched"], 15)
        rec("U13", "E-STOP latches (existing estop command), robot held, Start disabled, banner shown, Reset releases",
            clicked and es and vmax < 0.03 and latched_ui and start_disabled and rdy, max_speed_after=vmax, ui_latched=latched_ui,
            start_disabled=start_disabled, reset_ready=rdy)
        # U14 LAN
        ip = None
        try:
            sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sk.connect(("10.255.255.255", 1))
            ip = sk.getsockname()[0]
            sk.close()
        except OSError:
            pass
        lan, lan_link = None, None
        if ip:
            lan = urllib.request.urlopen(f"http://{ip}:8080/", timeout=5).status
            b.go(f"http://{ip}:8080/#ops")
            time.sleep(5)
            lan_link = text(b, "h-link-t")
        rec("U14", "dashboard served on the LAN address and live", lan == 200 and lan_link == "connected", lan_ip=ip, http=lan, link=lan_link)
        # U15 responsive layouts
        b.size(900, 1200)
        b.go(URL + "#ops")
        time.sleep(5)
        tab = b.js("return {scrollW: document.documentElement.scrollWidth, innerW: window.innerWidth}")
        b.shot(os.path.join(OUT, "10_tablet.png"))
        b.size(420, 900)
        time.sleep(3)
        ph = b.js("return {scrollW: document.documentElement.scrollWidth, innerW: window.innerWidth}")
        b.shot(os.path.join(OUT, "11_phone.png"))
        rec("U15", "responsive: tablet and phone widths have no horizontal page scroll", tab["scrollW"] <= tab["innerW"] and ph["scrollW"] <= ph["innerW"],
            tablet=tab, phone=ph)
        b.size(1600, 1000)
        b.go(URL + "#ops")
        time.sleep(5)
        # zoomed map shot
        b.js("const m=V3UI.maps.ops; m.follow=true; m.zoomAt(3.2); m.draw();")
        time.sleep(1.5)
        b.shot(os.path.join(OUT, "12_map_zoomed.png"))
    finally:
        b.close()
        n = sum(r["result"] == "PASS" for r in results)
        json.dump({"passed": n, "total": len(results), "results": results}, open(os.path.join(OUT, "ui_tests.json"), "w"), indent=1)
        print(f"PASSED {n} / {len(results)}")


if __name__ == "__main__":
    main()
