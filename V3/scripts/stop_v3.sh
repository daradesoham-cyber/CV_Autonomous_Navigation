#!/usr/bin/env bash
# V3 Hospital Logistics - stop the complete system started by start_v3.sh.
# Graceful SIGINT to the launch process group, then SIGTERM, then SIGKILL; finally removes any
# stray processes belonging to the V3 world/launch (matched by V3-specific names only).
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
V3_ROOT="$(dirname "$SCRIPT_DIR")"
RUN_DIR="$V3_ROOT/run"
QUIET=0; WAIT=0
for a in "$@"; do case "$a" in --quiet) QUIET=1 ;; --wait) WAIT=1 ;; esac; done
say() { [ "$QUIET" = 1 ] || echo "[V3] $*"; }

group_alive() { [ -n "$1" ] && kill -0 -- "-$1" 2>/dev/null; }

if [ -f "$RUN_DIR/v3.pgid" ]; then
  PGID=$(cat "$RUN_DIR/v3.pgid")
  if group_alive "$PGID"; then
    say "stopping V3 process group $PGID (SIGINT)"
    kill -INT -- "-$PGID" 2>/dev/null
    for _ in $(seq 1 15); do group_alive "$PGID" || break; sleep 1; done
    left() { ps -o comm= --no-headers -g "$PGID" 2>/dev/null | sort | uniq -c | tr -s ' ' | paste -sd, -; }
    # Known: Nav2 controller_server / planner_server (upstream binaries) often ignore SIGINT during shutdown
    if group_alive "$PGID"; then say "still running after SIGINT: $(left) -> SIGTERM"; kill -TERM -- "-$PGID" 2>/dev/null; sleep 5; fi
    if group_alive "$PGID"; then say "still running after SIGTERM: $(left) -> SIGKILL"; kill -KILL -- "-$PGID" 2>/dev/null; sleep 1; fi
  else
    say "no running V3 process group (stale pid file)"
  fi
  rm -f "$RUN_DIR/v3.pgid" "$RUN_DIR/v3.port"
else
  say "V3 is not running (no pid file)"
fi

# stray processes that belong to V3 (e.g. from a crashed launch); never touches V2.6-only processes
STRAY=$(ps -eo pid,args | awk '/v3_hospital_world[.]sdf|hospital_logistics_world[.]sdf|v3_full_system[.]launch|lib\/hospital_logistics\/v3_/ && !/awk/ {print $1}')
if [ -n "$STRAY" ]; then
  say "removing stray V3 processes: $STRAY"
  kill -TERM $STRAY 2>/dev/null; sleep 3; kill -KILL $STRAY 2>/dev/null
fi
say "V3 stopped"
[ "$WAIT" = 1 ] && sleep 4  # desktop launcher: keep the terminal visible briefly
exit 0
