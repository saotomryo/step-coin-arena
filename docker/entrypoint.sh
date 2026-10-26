#!/bin/bash
set -euo pipefail
# ROS setup scripts may reference unset variables.
set +u
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p "$XROBOCON_RESOURCE_ROOT" "$CODEX_HOME"
if [[ "${1:-}" != desktop ]]; then exec "$@"; fi
shift
children=()
cleanup() {
    trap - EXIT INT TERM
    if ((${#children[@]})); then kill "${children[@]}" 2>/dev/null || true; fi
    wait || true
}
trap cleanup EXIT INT TERM
Xvfb "$DISPLAY" -screen 0 1600x1000x24 -ac +extension GLX +render -noreset & children+=("$!")
ready=false
for _ in {1..100}; do
    if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then ready=true; break; fi
    sleep .1
done
if [[ "$ready" != true ]]; then echo 'Virtual display failed to start' >&2; exit 1; fi
openbox >/tmp/openbox.log 2>&1 & children+=("$!")
x11vnc -display "$DISPLAY" -rfbport 5900 -forever -shared -nopw >/tmp/vnc.log 2>&1 & children+=("$!")
websockify --web=/usr/share/novnc 6080 localhost:5900 >/tmp/websockify.log 2>&1 & children+=("$!")
python /app/scripts/launch_competition.py & children+=("$!")
# End the desktop session if any essential child fails or the setup window closes.
set +e
wait -n "${children[@]}"
status=$?
set -e
exit "$status"
