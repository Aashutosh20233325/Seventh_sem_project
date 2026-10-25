#!/usr/bin/env bash
# Start QGroundControl (its GCS heartbeat is required by PX4, NAV_DLL_ACT=2) under an owned process-group guard, unless
# one is already running. Read-only check for an existing instance; never stops anything.
# AppImage and settings (git-ignored): runs/sar_demo_20261018/restart_20261018/ (QGroundControl v5.1.5 x86_64).
set -u
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$REPO"
if pgrep -x QGroundControl > /dev/null; then echo "QGroundControl already running (pid $(pgrep -x QGroundControl | tr '\n' ' ')); nothing started"; exit 0; fi
Q=runs/sar_demo_20261018/restart_20261018
[ -x $Q/QGroundControl-x86_64.AppImage ] || { echo "QGroundControl AppImage missing: $Q"; exit 2; }
R=runs/sar_demo_20261018/restart_qgc_$(date -u +%Y%m%d_%H%M%S)
mkdir -p $R/logs
(cd $Q && XDG_CONFIG_HOME=$PWD/qgc_config XDG_CACHE_HOME=$PWD/qgc_cache setsid /usr/bin/python3 "$REPO/tools/owned_process_group.py" \
   --child-record "$REPO/$R/qgroundcontrol.child.json" -- ./QGroundControl-x86_64.AppImage --appimage-extract-and-run \
   > "$REPO/$R/logs/qgroundcontrol.log" 2>&1 < /dev/null & echo $! > "$REPO/$R/qgroundcontrol.guard.pid")
sleep 20
pgrep -x QGroundControl > /dev/null && echo "QGroundControl started (guard pid $(cat $R/qgroundcontrol.guard.pid), record $R)" || { echo "QGroundControl did not start; see $R/logs"; exit 2; }
