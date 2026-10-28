#!/usr/bin/env bash
# SUPERVISED DEMO SERIES: several runs of tools/demo/run_demo.sh, one fresh stack each, back to back (no waits).
# Usage: tools/demo/run_demo_series.sh <series_name> "<mode> <world> <offset> [flat|tf]" ["..." ...]
#   e.g. tools/demo/run_demo_series.sh profdemo "ppo default 0" "ppo default 180" "ppo c2_sw 0" "baseline default 0"
# Before each run: no PX4/gz running, a ground station running (gcs_heartbeat or QGroundControl), clean git tree. (Simulation speed is gated inside each run
# by the six stability windows; close memory-heavy Windows applications before a demo.) After each run:
# tools/demo/check_episode.py applies the
# stop rules (collision, guard refusal/abort, non-success, unexpected filter reason, torch loaded, no confirmed landing,
# stack not stopped, wrong world or start heading). A readiness failure (no flight) is re-attempted once; a flown
# episode is never retried; anything else stops the series. Sends no signals. Arms and flies: explicit approval only.
set -u
SERIES=${1:-}; shift || true
[[ "$SERIES" =~ ^[a-z0-9_]{1,30}$ ]] && [ $# -ge 1 ] || { echo "usage: $0 <series_name> \"<mode> <world> <offset>\" ..."; exit 2; }
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
D=$REPO/tools/demo
cd "$REPO"
OUT=runs/sar_demo_20261018/restart_series_$SERIES
test ! -e $OUT || { echo "series $OUT exists"; exit 2; }
mkdir -p $OUT
i=0; results=()
for spec in "$@"; do
  i=$((i + 1)); set -- $spec; mode=${1:-}; world=${2:-}; offset=${3:-}; height=${4:-flat}
  for attempt in 1 2; do
    name=${SERIES}_${i}_${mode}_${world}_${offset/-/m}_${height}$([ $attempt -eq 2 ] && echo r)
    echo "== $name ($mode, world $world, offset $offset, height $height) $(date -u +%T)"
    [ -z "$(ps -eo args | grep -E '^[^ ]*/bin/px4 |^gz sim' | grep -v grep)" ] || { echo "SERIES STOP: stack processes present"; exit 4; }
    pgrep -x QGroundControl > /dev/null || pgrep -f '^\.venv/bin/python -u tools/gcs_heartbeat\.py' > /dev/null \
      || { echo "SERIES STOP: no ground station (tools/demo/start_gcs_heartbeat.sh or start_qgc.sh)"; exit 4; }
    [ -z "$(git status --porcelain)" ] || { echo "SERIES STOP: git tree not clean"; exit 4; }
    bash $D/run_demo.sh $name $mode $world $offset $height > $OUT/$name.log 2>&1
    echo "run exit $?"
    verdict=$(.venv/bin/python $D/check_episode.py $OUT/$name.log runs/sar_demo_20261018/restart_$name/logs/episode.jsonl $world $mode $offset)
    echo "RESULT $name: $verdict"
    case "$verdict" in
      ok*) results+=("$name: $verdict"); break ;;
      readiness_fail*) [ $attempt -eq 2 ] && { echo "SERIES STOP: two readiness failures in a row"; exit 6; } ;;
      *) echo "SERIES STOP: stop rule ($verdict)"; exit 7 ;;
    esac
  done
done
echo "SERIES END $(date -u +%T): ${#results[@]}/$i ok"; printf '  %s\n' "${results[@]}"
