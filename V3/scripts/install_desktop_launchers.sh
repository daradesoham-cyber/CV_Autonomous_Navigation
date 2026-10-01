#!/usr/bin/env bash
# Installs the V3 desktop launchers for the current user (~/Desktop and ~/.local/share/applications).
# The .desktop spec needs absolute paths in Exec, so the files are generated here from this script's
# location instead of being stored with hard-coded paths. Re-run after moving the project.
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
APPS="$HOME/.local/share/applications"
mkdir -p "$DESK" "$APPS"
chmod +x "$SCRIPT_DIR"/*.sh
write() {  # name file exec terminal comment icon
  cat > "$2" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=$1
Comment=$5
Exec=$3
Icon=$6
Terminal=$4
Categories=Development;
StartupNotify=true
EOF
  chmod +x "$2"
  gio set "$2" metadata::trusted true 2>/dev/null || true
}
for dir in "$DESK" "$APPS"; do
  write "V3 Hospital Logistics" "$dir/V3_Hospital_Logistics.desktop" "\"$SCRIPT_DIR/v3_desktop_session.sh\"" true \
        "Start the V3 hospital logistics robot (Gazebo, Nav2, YOLO fusion, web UI)" applications-science
  write "V3 Hospital Logistics (Stop)" "$dir/V3_Hospital_Logistics_Stop.desktop" \
        "\"$SCRIPT_DIR/stop_v3.sh\" --wait" true "Stop all V3 processes" process-stop
done
echo "installed: $DESK/V3_Hospital_Logistics.desktop, $DESK/V3_Hospital_Logistics_Stop.desktop (+ application menu)"
