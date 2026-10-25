#!/usr/bin/env bash
# ONE SUPERVISED DEMO RUN (PX4 SITL + Gazebo, split rendering) with every safety gate of the flown procedure.
# Usage: tools/demo/run_demo.sh <name> <ppo|baseline|passive> [default|c1_ne|c2_sw|c5_w|gen_<name>] [0|90|180|-90] [flat|tf]
#   name     lowercase letters/digits/_; run dir runs/sar_demo_20261018/restart_<name>, driver files in ..._driver/
#   mode     ppo = E1R 300k seed 2 numpy policy behind the LiDAR safety filter; baseline = baseline controller;
#            passive = everything up to the pre-flight state check, never arms or flies
#   world    default (sim/worlds/sar_walls.sdf), a Stage C variant (target moved) or a generated world
#            (world_generator.py: rough terrain and obstacles); the launcher refuses unknown names
#   offset   start-heading offset in degrees (NED, + = clockwise) from the nominal east heading
#   height   flat (default: hold 1.8 m above the take-off point) or tf (terrain following: 1.8 m above the current
#            ground from the downward range /uav/agl_range)
# Arms and flies in modes ppo/baseline: run only with explicit approval and a human watching (docs/PPO_DEMO_RUNBOOK.md).
# Procedure (identical to the flown 2026-10-25 scripts, run_stagec_lean.sh): policy sha256 check; WSL memory watchdog;
# isolated stack start; gates (topics, world target, world consistency, strict /fmu/in, render-check); six 25 s
# stability windows; passive observer stopped by exact PID; fail-closed pre-flight state check; at most one runner call
# (no retry); gated stop (refuses unless fresh landed/disarmed telemetry; never SIGKILL).
NAME=${1:-}; MODE=${2:-}; VARIANT=${3:-default}; OFFSET=${4:-0}; HEIGHT=${5:-flat}
[[ "$NAME" =~ ^[a-z0-9_]{1,60}$ ]] || { echo "NAME REFUSED: '$NAME'"; exit 3; }
case "$MODE" in passive|ppo|baseline) ;; *) echo "MODE REFUSED: '$MODE' (passive, ppo or baseline)"; exit 3;; esac
case "$VARIANT" in default|c1_ne|c2_sw|c5_w) ;; *) [[ "$VARIANT" =~ ^gen_[a-z0-9_]{1,36}$ ]] || { echo "WORLD REFUSED: '$VARIANT' (default, c1_ne, c2_sw, c5_w, gen_<name>)"; exit 3; };; esac
case "$OFFSET" in 0|90|180|-90) ;; *) echo "OFFSET REFUSED: '$OFFSET' (0, 90, 180, -90)"; exit 3;; esac
case "$HEIGHT" in flat|tf) ;; *) echo "HEIGHT REFUSED: '$HEIGHT' (flat or tf)"; exit 3;; esac
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
D=$REPO/tools/demo
if [ "$VARIANT" = default ]; then unset SAR_WORLD_VARIANT; WORLD_ARGS=""; else export SAR_WORLD_VARIANT=$VARIANT; WORLD_ARGS="--world-variant $VARIANT"; fi
cd "$REPO"
source /opt/ros/jazzy/setup.bash; source ${PX4_ROS2_WS:-/home/hp/px4_ros2_ws}/install/setup.bash
set -u
export ROS_DOMAIN_ID=43 GZ_PARTITION=uav_sar_validation_20261018_instance1 \
  SAR_LIDAR_TOPIC=/world/sar_walls/model/x500_lidar_2d_oakd_1/link/link/sensor/lidar_2d_v2/scan
R=runs/sar_demo_20261018/restart_$NAME
L=$R/logs
X=runs/sar_demo_20261018/restart_${NAME}_driver   # driver-side files (markers, memory log); exists before the run dir
FMU_PARSE=$D/fmu_in_gate_parse.py
test ! -e $R && test ! -e $X || { echo "NAME REFUSED: $R or $X exists"; exit 3; }
mkdir -p $X
OBS_S=460                                      # observer duration: ~170 s windows + 240 s + margin
ABORT=$X/${NAME}_MEMORY_ABORT
POLICY=models/e1r_300k_s2/final_policy_numpy.npz
POLICY_SHA256=fbcacf9351837628f42a009d7570e414eeba6102fd6d8b822209b29c9a860511
[ "$(sha256sum $POLICY 2>/dev/null | cut -d' ' -f1)" = "$POLICY_SHA256" ] || { echo "POLICY CHECK FAILED: $POLICY"; exit 3; }
echo "policy $POLICY sha256 ok"
RUNNER_ARGS=""
[ "$MODE" = ppo ] && RUNNER_ARGS="--policy $POLICY"
[ "$OFFSET" != 0 ] && RUNNER_ARGS="$RUNNER_ARGS --start-yaw-offset-deg $OFFSET"
[ "$HEIGHT" = tf ] && RUNNER_ARGS="$RUNNER_ARGS --terrain-following"
echo "demo: name $NAME, mode $MODE, world $VARIANT, start offset $OFFSET, height $HEIGHT, runner args: ${RUNNER_ARGS:-none}"
memlog() {  # every 10 s; flags memory exhaustion risk (MemAvailable < 600 MiB or swap used > 2.5 GiB)
  while true; do
    line=$(awk '/MemAvailable/{a=$2}/SwapTotal/{t=$2}/SwapFree/{f=$2}END{printf "%d %d", a/1024, (t-f)/1024}' /proc/meminfo)
    srv=$(ps -eo rss,args | awk '$2=="gz" && $3=="sim" && $4=="--verbose=1" {print int($1/1024)}')
    gui=$(ps -eo rss,args | awk '$2=="gz" && $3=="sim" && $4=="-g" {print int($1/1024)}')
    echo "$(date -u +%T) $(cut -d' ' -f1 /proc/uptime) avail_mib/swap_mib $line gz_server_rss ${srv:-na} gz_gui_rss ${gui:-na}" >> $X/${NAME}_mem.txt
    set -- $line; { [ "$1" -lt 600 ] || [ "$2" -gt 2560 ]; } && touch $ABORT
    sleep 10
  done
}
mono() { python3 -c 'import time; print(f"{time.monotonic():.6f}")'; }
marker() { echo "{\"k\": \"$1\", \"t\": $(mono), \"wall_utc\": \"$(date -u +%FT%T.%3NZ)\"${2:-}}" >> $X/${NAME}_markers.jsonl; }
marker samplers_started            # kept as a timing marker; no samplers in the lean script
echo "START $(date -u +%T) $NAME"; free -m | sed -n 2,3p
marker stack_start_command
memlog & MEMLOG=$!
.venv/bin/python tools/isolated_sitl.py start --profile sar_oakd --split-rendering $WORLD_ARGS --run-dir $R || { echo "START FAILED"; kill $MEMLOG; exit 2; }
.venv/bin/python -u tools/telemetry_recorder.py $L/telemetry_full.jsonl 3600 > $L/recorder.out 2>&1 &
REC=$!
echo "recorder pid $REC"
gates_ok=1
probe_ok=0
for i in 1 2 3; do
  if timeout 60 .venv/bin/python tools/verify_isolated_topics.py --duration 8 > $L/gate_topics_$i.txt 2>&1; then probe_ok=1; break; fi
done
echo "== gate_topics (attempt $i, ok=$probe_ok)"; tail -6 $L/gate_topics_$i.txt
[ $probe_ok -eq 1 ] || gates_ok=0
timeout 60 .venv/bin/python tools/verify_world_target.py > $L/gate_world_target.txt 2>&1; wt=$?
echo "== world target rc=$wt"; tail -4 $L/gate_world_target.txt
[ $wt -eq 0 ] || gates_ok=0
timeout 30 .venv/bin/python tools/verify_world_consistency.py --run-dir $R > $L/gate_world_consistency.txt 2>&1; wc_rc=$?
echo "== world consistency rc=$wc_rc"; cat $L/gate_world_consistency.txt
[ $wc_rc -eq 0 ] || gates_ok=0
# >>> fmu_in gate (extracted and tested offline by guardcheck3/test_fmu_in_gate.py)
echo "== /fmu/in publishers (strict: --no-daemon --spin-time 5 -v; safe only with 0 publishers and the PX4-side reader identified)"
# Per topic, fmu_in_gate_parse.py accepts an attempt only if: exit 0; one Type line with the expected px4_msgs type;
# one well-formed "Publisher count: 0" and no PUBLISHER endpoint; one well-formed "Subscription count: N" matching N
# complete SUBSCRIPTION records; and >= 1 subscription from a non-ROS DDS participant (node and namespace
# _CREATED_BY_BARE_DDS_APP_, type hash INVALID): the PX4 XRCE-DDS agent's reader. Our recorder's ROS subscription
# never qualifies. Any publisher -> fail at once (no retry). Anything else -> unknown; at most 3 attempts, then fail
# closed. Finally, the three PX4-side readers must share one DDS participant. Main shell only (no pipelines).
FA=$L/gate_fmu_in_attempts.txt; FJ=$L/gate_fmu_in_parsed.jsonl
: > $L/gate_fmu_in.txt; : > $FJ
fmu_ok=1
for t in offboard_control_mode vehicle_command trajectory_setpoint; do
  topic=/fmu/in/$t; verdict=unknown
  for a in 1 2 3; do
    o=$L/gate_fmu_in_${t}_a$a.stdout; e=$L/gate_fmu_in_${t}_a$a.stderr
    t_start=$(date +%s.%N)
    timeout 30 ros2 topic info --no-daemon --spin-time 5 -v $topic > $o 2> $e; rc=$?
    elapsed=$(echo "$(date +%s.%N) - $t_start" | bc)
    cls=$(/usr/bin/python3 -I $FMU_PARSE attempt $topic $a $rc $elapsed $o $e $FJ 2>&1)
    case "$cls" in "safe "*|"publisher "*|"unknown "*) ;; *) cls="unknown parser_failed: $cls" ;; esac
    { echo "### $topic attempt $a rc=$rc elapsed_s=$elapsed result: $cls"
      echo "--- stdout:"; cat $o; echo "--- stderr:"; cat $e; echo; } >> $FA
    verdict=${cls%% *}
    [ "$verdict" = unknown ] || break
  done
  echo "$topic publishers=$([ "$verdict" = safe ] && echo 0 || echo "$cls") attempts=$a" | tee -a $L/gate_fmu_in.txt
  if [ "$verdict" != safe ]; then fmu_ok=0; gates_ok=0; break; fi
done
if [ $fmu_ok -eq 1 ]; then
  cross=$(/usr/bin/python3 -I $FMU_PARSE final $FJ 2>&1)
  echo "PX4-side reader participant check: $cross" | tee -a $L/gate_fmu_in.txt
  case "$cross" in "safe "*) ;; *) fmu_ok=0 ;; esac
fi
[ $fmu_ok -eq 1 ] && [ $(grep -c ' publishers=0 attempts=' $L/gate_fmu_in.txt) -eq 3 ] || { fmu_ok=0; gates_ok=0; }
echo "/fmu/in gate: $([ $fmu_ok -eq 1 ] && echo 'PASS (zero publishers and PX4-side reader identified on all three)' || echo FAIL)"
# <<< fmu_in gate
SRV=$(ps -eo pid,args | awk '$2=="gz" && $3=="sim" && $4=="--verbose=1" {print $1}')
GUI=$(ps -eo pid,args | awk '$2=="gz" && $3=="sim" && $4=="-g" {print $1}')
cp ~/.gz/rendering/ogre2.log $L/ogre2.log 2>/dev/null   # shared by server and GUI: evidence only, not the gate
echo "== render-check"
.venv/bin/python tools/isolated_sitl.py render-check --run-dir $R > $L/gate_renderer.json 2>&1; rc_render=$?
cat $L/gate_renderer.json; echo "render-check exit $rc_render"
[ $rc_render -eq 0 ] || gates_ok=0
shot() {  # read-only GUI service; saves under ~/.gz/gui/pictures, copied into the run logs
  before=$(ls -t ~/.gz/gui/pictures/*.png 2>/dev/null | head -1)
  GZ_PARTITION=$GZ_PARTITION timeout 15 gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 10000 --req 'data: ""' > $L/screenshot_$1.txt 2>&1
  sleep 3; after=$(ls -t ~/.gz/gui/pictures/*.png 2>/dev/null | head -1)
  if [ -n "$after" ] && [ "$after" != "$before" ]; then cp "$after" $L/gui_$1.png; echo "screenshot $1: $L/gui_$1.png"; else echo "screenshot $1: none ($(tr '\n' ' ' < $L/screenshot_$1.txt))"; fi
}
rec_done() { grep -q '"k": "end"' $L/guard_observer.jsonl 2>/dev/null; }   # written on completion and on abort
outcome="FAIL"; why="readiness gates failed"
if [ $gates_ok -eq 1 ]; then
  marker readiness_gates_passed
  echo "== passive guard observer (${OBS_S} s, dropouts disabled) + six stability windows $(date -u +%T)"
  .venv/bin/python -u $D/guard_observer.py $L/guard_observer.jsonl $OBS_S 1000000 > $L/guard_observer.out 2>&1 &
  CLK=$!
  marker observer_started ", \"pid\": $CLK"
  PIDS="gz_server=$SRV"; [ -n "$GUI" ] && PIDS="$PIDS gz_gui=$GUI"
# >>> rehearsal block (extracted and tested offline by test_early_start.py)
  aborted=""; wpass=0
  for w in 1 2 3 4 5 6; do
    [ -e $ABORT ] && { aborted="memory abort flag"; break; }
    rec_done && { aborted="observer ended during windows"; break; }
    timeout 60 .venv/bin/python $D/measure_light.py ${NAME}_w$w 25 $L/stability_light_w$w.json $PIDS > /dev/null 2>&1 &
    MW=$!
    while kill -0 $MW 2>/dev/null; do
      if rec_done || [ -e $ABORT ]; then kill -INT $MW 2>/dev/null; wait $MW 2>/dev/null; aborted="abort during window $w"; break; fi
      sleep 1
    done
    [ -n "$aborted" ] && break
    wait $MW
    r=$(python3 $D/evaluate_window.py $L/stability_light_w$w.json 2>&1)
    echo "w$w: $r" | tee -a $L/stability_check_light.txt
    case "$r" in *"-> PASS") wpass=$((wpass + 1));; esac
  done
  marker windows_done ", \"passed\": $wpass, \"aborted\": \"$(echo $aborted | tr -d '"')\""
  if [ -n "$aborted" ]; then
    why="aborted: $aborted"
  elif [ $wpass -ne 6 ]; then
    why="readiness windows passed $wpass/6"
  else
    marker T_r
    echo "== T_r $(date -u +%T): readiness passed; stopping the passive observer (pid $CLK) before the flight"
# >>> flight block (tested offline by test_flight_block.py)
    kill -INT $CLK 2>/dev/null; wait $CLK; obs_rc=$?; CLK_REAPED=1
    marker observer_stopped ", \"rc\": $obs_rc"
    pre=$(python3 $D/preflight_state.py $L/guard_observer.jsonl 2>&1); pre_rc=$?
    echo "preflight state: $pre"
    if [ -e $ABORT ]; then aborted="memory abort flag before flight"
    elif [ $pre_rc -ne 0 ]; then aborted="preflight state check: $pre"
    fi
    if [ -z "$aborted" ] && [ "$MODE" = passive ]; then
      outcome="PASSIVE"; why="readiness, world gates and pre-flight state passed; no flight (passive)"
    elif [ -z "$aborted" ]; then
      marker runner_start
      echo "== SUPERVISED DEMO FLIGHT $(date -u +%T) (mode $MODE, world $VARIANT, offset $OFFSET; one episode, no retry)"
      .venv/bin/python -u run_target_reaching_episode.py --log $L/episode.jsonl --i-confirm-supervised-flight $RUNNER_ARGS > $L/episode.out 2>&1 &
      RUN=$!
      echo $RUN > $L/runner.pid
      marker runner_started ", \"pid\": $RUN"
      wait $RUN; run_rc=$?
      marker runner_end ", \"rc\": $run_rc"
      res=$(grep -o 'EPISODE RESULT: .*' $L/episode.out | tail -1)
      echo "runner exit $run_rc: ${res:-no result line}"; tail -5 $L/episode.out
      outcome="FLOWN"; why="runner exit $run_rc: ${res:-no result line}"
    else
      why="aborted before flight: $aborted"
    fi
# <<< flight block
  fi
# <<< rehearsal block
  # Normal run: let the observer finish its own duration (it writes its "end" record). Abort or failed readiness:
  # if it is still running, SIGINT its exact PID.
  # Signal the observer only while it is still our unreaped child, never after the flight block reaped it.
  if [ -n "${CLK_REAPED:-}" ]; then
    echo "guard observer stopped before flight (rc $obs_rc): $(tail -1 $L/guard_observer.out)"
  else
    if rec_done; then wait $CLK; else kill -INT $CLK 2>/dev/null; wait $CLK; fi
    echo "guard observer exit $?: $(tail -1 $L/guard_observer.out)"
  fi
  shot after_rehearsal
  GZ_PARTITION=$GZ_PARTITION timeout 10 gz topic -l 2>/dev/null | grep -c '^/gui/' | sed 's/^/gui topics: /' | tee $L/gui_topics.txt
  timeout 20 gz topic -e -n 5 -t /world/sar_walls/stats 2>/dev/null | grep real_time_factor > $L/gz_stats_rtf.txt; echo "gz /stats RTF (unreliable, recorded only): $(tr '\n' ' ' < $L/gz_stats_rtf.txt)"
  .venv/bin/python tools/isolated_sitl.py render-check --run-dir $R > $L/gate_renderer_end.json 2>&1; echo "render-check (end) exit $?"
else
  echo "READINESS GATE FAILED - no observer, no stability windows"
fi
echo "FLIGHT RUN: $outcome ($why)"
marker rehearsal_end ", \"outcome\": \"$outcome\", \"why\": \"$(echo $why | tr -d '"')\""
echo "== gated stop $(date -u +%T)"
for a in 1 2 3 4 5; do
  echo "gated stop attempt $a $(date -u +%T)"
  timeout 150 .venv/bin/python tools/isolated_sitl.py stop --run-dir $R && { echo STOPPED; break; }
done 2>&1 | tee $L/gated_stop.txt
kill -INT $REC 2>/dev/null; wait $REC 2>/dev/null
kill $MEMLOG 2>/dev/null
ps -eo pid,args | grep -E "gz sim|bin/px4|MicroXRCE|parameter_bridge" | grep -v grep && echo "WARNING: stack processes remain" || echo "stack stopped"
free -m | sed -n 1,3p
echo "END $(date -u +%T)"
