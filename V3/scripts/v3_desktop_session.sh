#!/usr/bin/env bash
# Desktop double-click entry point: starts V3 (GUI Gazebo + browser), keeps this terminal as the
# control window and stops the whole system when the user presses Enter or closes the window.
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
trap '"$SCRIPT_DIR/stop_v3.sh"; exit 0' HUP INT TERM
echo "=============================================="
echo "  V3 Hospital Logistics - starting"
echo "=============================================="
"$SCRIPT_DIR/start_v3.sh" "$@"
rc=$?
if [ $rc -ne 0 ]; then
  echo ""
  echo "V3 did not start (exit code $rc). See the messages above and V3/logs/latest/launch.log."
  echo "Press Enter to close."
  read -r _
  exit $rc
fi
echo "Health check:"
"$SCRIPT_DIR/health_check.sh" 2>/dev/null | tail -n 3
echo ""
echo "Press Enter in this window to STOP V3 (closing the window also stops it)."
read -r _
"$SCRIPT_DIR/stop_v3.sh"
echo "Press Enter to close."
read -r _
