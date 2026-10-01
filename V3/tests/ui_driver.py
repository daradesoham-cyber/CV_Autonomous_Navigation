#!/usr/bin/env python3
"""Minimal W3C WebDriver client (geckodriver + headless Firefox, no extra packages) for UI validation/screenshots.
usage: ui_driver.py OUT_DIR  (V3 must be running on localhost:8080)"""
import base64, json, os, subprocess, sys, time, urllib.error, urllib.request

PORT = 4455
def req(method, path, body=None):
    r = urllib.request.Request(f"http://127.0.0.1:{PORT}{path}", data=None if body is None else json.dumps(body).encode(),
                               method=method, headers={"Content-Type": "application/json"})
    try:
        return json.loads(urllib.request.urlopen(r, timeout=60).read() or b"{}").get("value")
    except urllib.error.HTTPError as e:
        raise RuntimeError(f"WebDriver {method} {path}: {e.read()[:400]!r}") from None

class Browser:
    def __init__(self, w=1600, h=1000):
        self.p = subprocess.Popen(["geckodriver", "--port", str(PORT)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(50):
            try: req("GET", "/status"); break
            except Exception: time.sleep(0.2)
        caps = {"capabilities": {"alwaysMatch": {"browserName": "firefox", "moz:firefoxOptions": {"args": ["-headless", f"--width={w}", f"--height={h}"]}}}}
        for attempt in range(5):  # a previous headless Firefox may still be shutting down
            try:
                self.sid = req("POST", "/session", caps)["sessionId"]
                break
            except Exception:
                if attempt == 4:
                    raise
                time.sleep(3)
        self.size(w, h)
    def size(self, w, h): req("POST", f"/session/{self.sid}/window/rect", {"width": w, "height": h})
    def go(self, url): req("POST", f"/session/{self.sid}/url", {"url": url})
    def js(self, script, *args): return req("POST", f"/session/{self.sid}/execute/sync", {"script": script, "args": list(args)})
    def shot(self, path):
        open(path, "wb").write(base64.b64decode(req("GET", f"/session/{self.sid}/screenshot")))
    def close(self):
        try: req("DELETE", f"/session/{self.sid}")
        finally: self.p.terminate()
