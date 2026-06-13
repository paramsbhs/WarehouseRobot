#!/usr/bin/env bash
# Start the Gazebo 3D client inside the running container and serve it over
# VNC. Hardware/XQuartz GLX does not work for Gazebo on Apple Silicon, so we
# render with Mesa software GL into a virtual display (Xvfb) and stream the
# pixels with x11vnc. View it with the macOS built-in VNC client:
#   open vnc://localhost:5900     (or Finder > Go > Connect to Server)
set -e

if ! docker ps --filter name=warehouse-gazebo --format '{{.Names}}' | grep -q warehouse-gazebo; then
    echo "Container not running. Start it first: ./run_gazebo.sh"
    exit 1
fi

docker exec warehouse-gazebo pkill -f "Xvfb|gzclient|x11vnc" 2>/dev/null || true
sleep 1

docker exec -d warehouse-gazebo bash -c "Xvfb :1 -screen 0 1366x768x24 > /tmp/xvfb.log 2>&1"
sleep 2
docker exec -d warehouse-gazebo bash -c \
    "DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe gzclient > /tmp/gzclient.log 2>&1"
sleep 3
# macOS Screen Sharing refuses no-auth VNC servers, so set a password.
docker exec warehouse-gazebo bash -c "x11vnc -storepasswd warehouse /tmp/vncpass >/dev/null 2>&1"
docker exec -d warehouse-gazebo bash -c \
    "x11vnc -display :1 -forever -shared -rfbauth /tmp/vncpass -rfbport 5900 -bg > /tmp/x11vnc.log 2>&1"
sleep 2

echo "Gazebo 3D view is being served over VNC."
echo "Connect with the macOS built-in viewer:"
echo "    open vnc://localhost:5900"
echo "Password: warehouse"
