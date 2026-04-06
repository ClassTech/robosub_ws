#!/bin/bash
set -e

# Silence pygame runtime/audio warnings
export XDG_RUNTIME_DIR=/tmp
export SDL_AUDIODRIVER=dummy

# Remove stale Xvfb lock files left over from unclean shutdowns
rm -f /tmp/.X99-lock /tmp/.X11-unix/X99

# Start a virtual framebuffer large enough for the pygame window (1200x800).
# -ac disables access control so x11vnc can connect without an auth cookie.
Xvfb :99 -screen 0 1280x900x24 -ac &
export DISPLAY=:99

# Give Xvfb a moment to initialise before anything tries to open a window
sleep 1

# Start the VNC server sharing that virtual display.
# -nopw   : no password (fine for local dev)
# -forever: keep running after the first client disconnects
# -noxdamage: avoids rendering glitches in some virtual display setups
x11vnc -display :99 -forever -nopw -noxdamage -listen 0.0.0.0 -rfbport 5900 &

source /opt/ros/jazzy/setup.bash
source /robosub_ws/install/setup.bash

exec "$@"
