#!/usr/bin/env bash
# PRESENTATION RUN of the mine rescue, started by hand: one command does the readiness checks, the top-down Gazebo
# view, plain-language progress, the flight, and afterwards the mission map animation and summary card.
#
# Usage: bash tools/demo/present_mine.sh [--rehearse] [world] [view height m]
#   --rehearse   everything up to the pre-flight check, NO takeoff (practise the recording)
#   world        default gen_mine_s20 (any gen_mine_* world)
#   view height  top-down camera height, default 38
#
# Arms and flies (unless --rehearse): run it yourself, watching. Ctrl+C stops only this display; the flight goes on
# with all its safety gates and lands by itself.
set -u
REHEARSE=0
[ "${1:-}" = "--rehearse" ] && { REHEARSE=1; shift; }
WORLD=${1:-gen_mine_s20}
VIEW_Z=${2:-38}
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$REPO"
[[ "$WORLD" =~ ^gen_mine_s[0-9]+$ ]] && [ -d sim/worlds/variants/$WORLD ] || { echo "Unknown mine world: $WORLD"; exit 2; }
[[ "$VIEW_Z" =~ ^[0-9]+$ ]] || { echo "View height must be a whole number of metres"; exit 2; }
MODE=$([ $REHEARSE -eq 1 ] && echo passive || echo explore)
NAME=present$([ $REHEARSE -eq 1 ] && echo r)_$(date -u +%m%d_%H%M%S)
SERIES_DIR=runs/sar_demo_20261018/restart_series_$NAME
OUT=runs/sar_demo_20261018/restart_present_logs/$NAME.out   # git-ignored (restart_*)
DOWNLOADS=/mnt/c/Users/HP/Downloads/mine_presentation

echo "=================================================================="
echo " Mine rescue demo: $WORLD  ($([ $REHEARSE -eq 1 ] && echo "REHEARSAL, no takeoff" || echo "LIVE FLIGHT"))"
echo "=================================================================="
# --- pre-checks (read-only) ---
[ -z "$(ps -eo args | grep -E '^[^ ]*/bin/px4 |^gz sim' | grep -v grep)" ] || { echo "A simulator is still running: wait for it to stop."; exit 4; }
[ -z "$(git status --porcelain)" ] || { echo "The code has uncommitted changes: the run refuses to start (git status)."; exit 4; }
if pgrep -x QGroundControl > /dev/null; then
  echo "QGroundControl is running: close it (it slows the simulation; the heartbeat replaces it)."; exit 4
fi
if ! pgrep -f '^\.venv/bin/python -u tools/gcs_heartbeat\.py' > /dev/null; then
  echo "Starting the ground-station heartbeat..."
  bash tools/demo/start_gcs_heartbeat.sh || exit 4
fi
AVAIL=$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)
echo "WSL free memory: $AVAIL MiB. Close Edge and other apps; keep the Gazebo window small."
echo

# --- the run, in its own session (Ctrl+C here never reaches it) ---
mkdir -p $(dirname $OUT)
setsid bash tools/demo/run_demo_series.sh $NAME "$MODE $WORLD 0 tf" > $OUT 2>&1 < /dev/null &
SERIES_PID=$!

# --- top-down view: once per Gazebo window, when its camera service is up (read-only otherwise) ---
(
  export GZ_PARTITION=uav_sar_validation_20261018_instance1
  seen=""
  while kill -0 $SERIES_PID 2>/dev/null; do
    gui=$(pgrep -f '^gz sim -g' | head -1)
    if [ -n "$gui" ] && [[ " $seen " != *" $gui "* ]]; then
      for i in $(seq 60); do gz service -l 2>/dev/null | grep -q '^/gui/move_to/pose$' && break; sleep 2; done
      sleep 6
      timeout 10 gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 \
        --req "pose: {position: {x: 4.0, y: 4.0, z: $VIEW_Z.0}, orientation: {w: 0.5, x: -0.5, y: 0.5, z: 0.5}}" \
        > /dev/null 2>&1 && echo "[$(date +%T)] Gazebo view set to top-down ($VIEW_Z m). Please do not move the view."
      seen="$seen $gui"
    fi
    sleep 5
  done
) &
VIEW_PID=$!
trap 'echo; echo "Display stopped. The run continues by itself and lands safely; its log: $OUT"; kill $VIEW_PID 2>/dev/null; exit 130' INT

until [ -d $SERIES_DIR ] || ! kill -0 $SERIES_PID 2>/dev/null; do sleep 1; done
.venv/bin/python -u tools/demo/present_progress.py $SERIES_DIR $OUT
wait $SERIES_PID
kill $VIEW_PID 2>/dev/null

# --- afterwards: mission map + summary card ---
RUN=$(ls -d runs/sar_demo_20261018/restart_${NAME}_1_* 2>/dev/null | grep -v "_driver$" | tail -1)
if [ $REHEARSE -eq 0 ] && [ -n "$RUN" ] && [ -s "$RUN/logs/episode.jsonl" ] && grep -q '"event": "step"' "$RUN/logs/episode.jsonl"; then
  echo
  echo "Making the mission map animation and summary card..."
  .venv/bin/python tools/mission_map.py "$RUN" | sed 's/^/  /'
  mkdir -p "$DOWNLOADS" && cp "$RUN/presentation/mission_map.gif" "$DOWNLOADS/${NAME}_mission_map.gif" \
    && cp "$RUN/presentation/mission_summary.png" "$DOWNLOADS/${NAME}_mission_summary.png" \
    && echo "Copied to Downloads\\mine_presentation\\${NAME}_mission_map.gif and ${NAME}_mission_summary.png"
fi
echo "Full log: $OUT"
