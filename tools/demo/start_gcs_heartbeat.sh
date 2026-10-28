#!/usr/bin/env bash
# Start the headless GCS heartbeat (tools/gcs_heartbeat.py; replaces QGroundControl, whose heartbeat PX4 requires,
# NAV_DLL_ACT=2) under an owned process-group guard, unless one is already running. Refuses while QGroundControl runs
# (both use UDP 14550). Read-only checks; never stops anything.
set -u
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$REPO"
if pgrep -f '^\.venv/bin/python -u tools/gcs_heartbeat\.py' > /dev/null; then echo "gcs_heartbeat already running (pid $(pgrep -f '^\.venv/bin/python -u tools/gcs_heartbeat\.py' | tr '\n' ' ')); nothing started"; exit 0; fi
pgrep -x QGroundControl > /dev/null && { echo "QGroundControl is running (it holds UDP 14550): close it first"; exit 2; }
R=runs/sar_demo_20261018/restart_gcs_heartbeat_$(date -u +%Y%m%d_%H%M%S)
mkdir -p $R/logs
setsid /usr/bin/python3 tools/owned_process_group.py --child-record $R/gcs_heartbeat.child.json -- \
   .venv/bin/python -u tools/gcs_heartbeat.py > $R/logs/gcs_heartbeat.log 2>&1 < /dev/null & echo $! > $R/gcs_heartbeat.guard.pid
sleep 3
pgrep -f '^\.venv/bin/python -u tools/gcs_heartbeat\.py' > /dev/null && echo "gcs_heartbeat started (guard pid $(cat $R/gcs_heartbeat.guard.pid), record $R)" || { echo "gcs_heartbeat did not start; see $R/logs"; cat $R/logs/gcs_heartbeat.log; exit 2; }
