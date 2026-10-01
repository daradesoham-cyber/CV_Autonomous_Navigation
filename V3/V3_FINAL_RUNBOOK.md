# V3 Hospital Logistics — Operator Runbook

Simulated hospital logistics robot: collects a sample at the **Collection Point (ward)** and delivers it
to the **Laboratory / Test Point**. It runs on Gazebo Sim, Nav2/AMCL, the V2.6 topological decision
engine, and V3 perception (YOLOv8n plus camera–LiDAR fusion). You control it from a web UI.

All paths below are relative to the project root (`~/CV_Autonomous_Navigation`). The scripts work out
their own location, so you can run them from any directory.

## 1. Start

| Method | What happens |
|---|---|
| Double-click **V3 Hospital Logistics** on the desktop | A terminal opens and runs `V3/scripts/v3_desktop_session.sh`. That starts Gazebo (with GUI), Nav2, perception, the mission manager and the web UI, waits until the system is **READY**, opens the browser and prints the URLs and a health check. Press **Enter** in that terminal, or close it, to stop everything. |
| `V3/scripts/start_v3.sh` | Same, from a shell. Options: `--headless` (no Gazebo GUI), `--traffic` (moving people/trolleys), `--no-browser`, `--port N`. |

READY normally arrives about 17–20 s after starting. The time is written to `V3/logs/<stamp>/startup_seconds`.
READY means all of these are true:
- AMCL has published a pose.
- The decision engine is subscribed.
- The Nav2 `navigate_to_pose` action server is available.
- YOLO detections and spatial fusion have both produced output in the last 2 s.

If the system is not READY within 240 s, the start script stops everything again and prints the end of `launch.log`.

Desktop launchers are installed, or re-installed after moving the project, with
`V3/scripts/install_desktop_launchers.sh`. This writes `~/Desktop/V3_Hospital_Logistics.desktop`,
`~/Desktop/V3_Hospital_Logistics_Stop.desktop`, and the same entries in the application menu.

## 2. Use the web UI

- Local: `http://localhost:8080`
- LAN: `http://<this-machine's-LAN-IP>:8080`. The start script prints the address.

The server accepts only private-network (RFC 1918) and loopback clients. It is **not** meant to be
exposed to the internet: do not port-forward it.

| Control | Effect |
|---|---|
| Source / Destination + **Start** | Runs the mission: drive to source → collect (3 s) → drive to destination → deliver (3 s) → MISSION_COMPLETE |
| **Pause** / **Resume** | Robot stops within about 0.5 s (the Nav2 goal is cancelled); Resume re-sends the current waypoint |
| **Cancel** | Ends the mission (CANCELLED); robot stops |
| **E-STOP** | Latched: cancels navigation and holds zero velocity at 20 Hz; Start is refused until **Reset** |
| **Reset** | From a terminal state (COMPLETE / CANCELLED / FAILED / E-STOP) back to READY |
| Traffic toggle | Starts or stops the moving people and trolleys |

The panels show:
- mission state and timeline
- robot pose and speed
- the map with the robot and the active route
- the live camera feed (MJPEG) with YOLO boxes
- fused detections (class, range, bearing, motion)
- alternative routes with their cost breakdown
- the event log
- mission history with per-mission detail

The UI can only send these fixed commands: `start pause resume cancel reset estop` plus the traffic
toggle. Any other command, or an unknown location, is rejected with HTTP 400. The UI cannot run shell
commands.

## 3. Stop / restart

```
V3/scripts/stop_v3.sh       # SIGINT to the launch process group, then SIGTERM, then SIGKILL; then removes stray V3 processes
V3/scripts/restart_v3.sh    # stop + start (takes the same options as start)
```

Nav2's `controller_server` and `planner_server` (upstream binaries) often ignore SIGINT during
shutdown, so a stop usually needs SIGTERM or SIGKILL and takes about 20 s. The stop script names the
processes it had to escalate. Afterwards nothing from V3 is left running; the stop script and the
restart test check for this.

## 4. Health check

`V3/scripts/health_check.sh` prints PASS or FAIL for each item and exits 0 only if everything passes.
It checks:
- the ROS environment and workspace
- Gazebo
- all expected nodes
- that the Nav2 and localization lifecycle nodes are active
- the TF chain
- topic rates (camera, YOLO, fusion, LiDAR, odometry)
- the web API on localhost and on the LAN IP
- both databases

## 5. Data and logs

| Path | Content |
|---|---|
| `V3/data/v3_missions.db` | Every mission: result, failure reason, legs, path, events, durations, distance, minimum LiDAR range, replans, TTC events, sign decisions |
| `V3/data/v3_navigation_memory.db` | V3's path memory: V2.6 topology plus V3's own learned segment history, which affects route choice |
| `V3/logs/<stamp>/launch.log`, `V3/logs/latest` | Full log of each run |
| `V3/docs/evidence/` | Raw test evidence for every phase |

To reset the learned path history (the topology is kept):
`python3 V3/scripts/init_v3_memory.py --force`, with V3 stopped.

## 6. Troubleshooting

| Symptom | Cause / action |
|---|---|
| "V3 is already running" | Use `stop_v3.sh` or `restart_v3.sh` |
| "a Gazebo facility world is already running" | A V2.6 instance or a test is running. Stop it first; only one simulation can run at a time. |
| "port 8080 is already in use" | Use `--port 8081`, or stop whatever is using the port |
| Not READY within 240 s | See `V3/logs/latest/launch.log`. Most often the GPU or CPU is busy, or another ROS graph is on the same `ROS_DOMAIN_ID`. |
| Mission FAILED with "leg timeout" | A leg did not reach its goal within 300 s of active (unpaused) time. The record in mission history has the route, replans and events. |
| Camera panel black | Check `camera` in `/api/state` → `system.topics`. Gazebo rendering may still be starting. |

## 7. Tests (with V3 running, unless marked standalone)

| Command | What it checks |
|---|---|
| `python3 V3/tests/test_v3_system.py` | System tests T01–T12 (API, UI, whitelist, start/pause/resume/cancel/reset/e-stop, LAN, history, full mission) |
| `python3 V3/tests/run_v3_missions.py 10 [--traffic]` | Repeated missions with raw telemetry |
| `python3 V3/tests/measure_v3_performance.py 60` | CPU / RAM / GPU / rates / latencies |
| `python3 V3/tests/test_v3_sign_gating.py` | Sign decisions (standalone; isolated ROS domain) |
| `python3 V3/tests/test_v3_path_history_influence.py` | Path history changes route choice (standalone; uses a copy of the DB) |
| `V3/tests/phase7_runner.sh v3\|v26` | Tracker/costmap scenarios A–H (standalone; V3 must be **stopped**) |
