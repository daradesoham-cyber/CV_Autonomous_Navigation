#!/usr/bin/env bash
# ==============================================================================
# Desktop Launcher Installer for CV Autonomous Navigation V2.1
# ==============================================================================

set -e

DESKTOP_DIR="/home/soham-darade/Desktop"
APPS_DIR="/home/soham-darade/.local/share/applications"
TARGET_DESKTOP="$DESKTOP_DIR/CV_Navigation_V2.desktop"
TARGET_APPS="$APPS_DIR/CV_Navigation_V2.desktop"
LAUNCH_SCRIPT="/home/soham-darade/CV_Autonomous_Navigation/start_cv_navigation_v2.sh"

chmod +x /home/soham-darade/CV_Autonomous_Navigation/*.sh 2>/dev/null || true

cat << 'EOF' > "$TARGET_DESKTOP"
#!/usr/bin/env -S gio launch
[Desktop Entry]
Version=2.1
Type=Application
Name=CV Navigation V2.1
GenericName=Autonomous Navigation System
Comment=Launch ROS 2 Lyrical & Gazebo Autonomous Navigation V2.1 with Web Dashboard
Exec=bash -c "/home/soham-darade/CV_Autonomous_Navigation/start_cv_navigation_v2.sh; echo ''; echo 'System stopped. Press Enter to close.'; read"
Icon=applications-science
Terminal=true
Categories=Development;Robotics;Science;Education;
StartupNotify=true
Keywords=ROS2;Gazebo;Nav2;YOLO;Perception;Robotics;
EOF

chmod +x "$TARGET_DESKTOP"
if command -v gio > /dev/null 2>&1; then
    gio set "$TARGET_DESKTOP" metadata::trusted true 2>/dev/null || true
fi

mkdir -p "$APPS_DIR"
cp "$TARGET_DESKTOP" "$TARGET_APPS"
chmod +x "$TARGET_APPS"

echo "================================================================="
echo "[SUCCESS] Desktop Launcher Installed:"
echo "  - Desktop Shortcut: $TARGET_DESKTOP"
echo "  - App Menu Entry:   $TARGET_APPS"
echo "================================================================="
