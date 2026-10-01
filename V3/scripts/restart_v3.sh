#!/usr/bin/env bash
# V3 Hospital Logistics - stop, then start again (arguments are passed to start_v3.sh).
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
"$SCRIPT_DIR/stop_v3.sh"
exec "$SCRIPT_DIR/start_v3.sh" "$@"
