"""Offline tests for tools/demo (the supervised demo kit). No ROS/Gazebo/PX4.

The flight block is extracted verbatim from run_demo.sh and run with a stub .venv/bin/python standing in for
run_target_reaching_episode.py, and a SIGINT-handling child standing in for the passive observer (as rclpy does).
Plus: argument validation before anything starts, runner arguments per mode/offset, the pinned policy, static safety
properties, parity with the flown lean script, the episode checker on synthetic and (if present) real flown logs,
and the series driver's usage refusal."""
import json
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
DEMO = ROOT / "tools" / "demo"
SCRIPT = DEMO / "run_demo.sh"
FLOWN_LEAN = Path("/home/hp/uav_ros_experiments/stagec_prep_20261025/run_stagec_lean.sh")
RUNS = ROOT / "runs" / "sar_demo_20261018"

STUB = '''#!/usr/bin/env python3
import os, sys
open(os.environ["CALLS"], "a").write(" ".join(sys.argv[1:]) + "\\n")
print("EPISODE RESULT: " + os.environ["RESULT"], flush=True)
sys.exit(int(os.environ["RUN_RC"]))
'''


def state(t=10.0, arming=1, failsafe=False, preflight=True, landed=True, fmu=0, status=True):
    return {"k": "state", "t": t, "mem_available_mib": 1700.0, "swap_used_mib": 13.3, "fmu_in_publishers": fmu,
            "status": [arming, 4, failsafe, preflight] if status else None, "land": [landed, True] if status else None}


GOOD_LOG = [{"k": "start", "t": 1.0}, state(9.0), {"k": "guard", "t": 10.0}, state(10.0)]


def text_between(a, b):
    t = SCRIPT.read_text()
    return t[t.index(a):t.index(b)]


def line(prefix):
    return next(l for l in SCRIPT.read_text().splitlines() if l.startswith(prefix))


def run_block(tmp_path, log=GOOD_LOG, run_rc=0, result="success", abort_flag=False, mode="ppo",
              runner_args="--policy models/e1r_300k_s2/final_policy_numpy.npz"):
    (tmp_path / ".venv" / "bin").mkdir(parents=True)
    stub = tmp_path / ".venv" / "bin" / "python"
    stub.write_text(STUB)
    stub.chmod(0o755)
    logs = tmp_path / "logs"
    logs.mkdir()
    (logs / "guard_observer.jsonl").write_text(log if isinstance(log, str) else "".join(json.dumps(r) + "\n" for r in log))
    if abort_flag:
        (tmp_path / "MEMORY_ABORT").touch()
    bash = "\n".join([
        "set -u", "NAME=t", f"D={DEMO}", f"X={tmp_path}", f"L={logs}", f"ABORT={tmp_path}/MEMORY_ABORT", 'aborted=""',
        'outcome="FAIL"; why="readiness gates failed"', line("mono() {"), line("marker() {"),
        f"MODE={mode}; VARIANT=default; OFFSET=0", f"RUNNER_ARGS='{runner_args}'",
        "python3 -c 'import signal,sys,time; signal.signal(signal.SIGINT, lambda *a: sys.exit(1)); "
        "open(sys.argv[1], \"w\").close(); time.sleep(300)' observer_ready & CLK=$!",
        "until [ -e observer_ready ]; do sleep 0.05; done",
        text_between("# >>> flight block", "# <<< flight block"),
        'kill -0 $CLK 2>/dev/null && echo OBSERVER_ALIVE || echo OBSERVER_GONE',
        'echo "OUTCOME=[$outcome] WHY=[$why] REAPED=[${CLK_REAPED:-}]"'])
    env = {"PATH": "/usr/bin:/bin", "CALLS": str(tmp_path / "calls.txt"), "RUN_RC": str(run_rc), "RESULT": result}
    p = subprocess.run(["bash", "-c", bash], capture_output=True, text=True, timeout=60, cwd=tmp_path, env=env)
    mk = tmp_path / "t_markers.jsonl"
    kinds = [json.loads(l)["k"] for l in mk.read_text().splitlines()] if mk.exists() else []
    calls = (tmp_path / "calls.txt").read_text().splitlines() if (tmp_path / "calls.txt").exists() else []
    return p, kinds, calls


# ---------- flight block ----------

def test_good_state_stops_observer_then_runs_the_runner_exactly_once(tmp_path):
    p, kinds, calls = run_block(tmp_path)
    assert "OBSERVER_GONE" in p.stdout and "REAPED=[1]" in p.stdout, p.stdout + p.stderr
    assert kinds == ["observer_stopped", "runner_start", "runner_started", "runner_end"]
    assert calls == [f"-u run_target_reaching_episode.py --log {tmp_path}/logs/episode.jsonl --i-confirm-supervised-flight "
                     "--policy models/e1r_300k_s2/final_policy_numpy.npz"]
    assert "OUTCOME=[FLOWN] WHY=[runner exit 0: EPISODE RESULT: success]" in p.stdout


def test_passive_mode_never_starts_the_runner(tmp_path):
    p, kinds, calls = run_block(tmp_path, mode="passive", runner_args="")
    assert calls == [] and kinds == ["observer_stopped"] and "OUTCOME=[PASSIVE]" in p.stdout, p.stdout + p.stderr


def test_failed_episode_is_reported_and_never_retried(tmp_path):
    p, kinds, calls = run_block(tmp_path, run_rc=1, result="guard_abort")
    assert len(calls) == 1 and "WHY=[runner exit 1: EPISODE RESULT: guard_abort]" in p.stdout


@pytest.mark.parametrize("log, reason", [
    (GOOD_LOG + [{"k": "abort", "t": 11.0, "reason": "vehicle not landed"}], "observer abort: vehicle not landed"),
    ([{"k": "start", "t": 1.0}, state(10.0, arming=2)], "arming=2"),
    ([{"k": "start", "t": 1.0}, state(10.0, landed=False)], "not landed"),
    ([{"k": "start", "t": 1.0}, state(10.0, fmu=1)], "/fmu/in publishers 1"),
    ([{"k": "start", "t": 1.0}, state(3.0), {"k": "guard", "t": 10.0}], "s old"),
    ("{not json\n", "unreadable log"),
])
def test_bad_state_never_starts_the_runner(tmp_path, log, reason):
    p, kinds, calls = run_block(tmp_path, log=log)
    assert calls == [] and "runner_start" not in kinds and reason in p.stdout, p.stdout


def test_memory_abort_flag_never_starts_the_runner(tmp_path):
    p, kinds, calls = run_block(tmp_path, abort_flag=True)
    assert calls == [] and "memory abort flag before flight" in p.stdout


# ---------- arguments, policy, safety properties ----------

@pytest.mark.parametrize("args", [[], ["Demo1", "ppo"], ["d1", "fly"], ["d1", "ppo", "c3_x"], ["d1", "ppo", "default", "45"],
                                  ["d1", "ppo", "default", "0", "agl"], ["d1", "ppo", "gen_Bad"], ["d1", "ppo", "gen_x;y"],
                                  ["d1; rm -rf /", "ppo"], ["d1", "ppo", "../c2_sw"], ["d1", "PPO"]])
def test_arguments_are_validated_before_anything_starts(args):
    t = SCRIPT.read_text()
    head = t[t.index("NAME=${1:-}"):t.index("REPO=$(")]
    p = subprocess.run(["bash", "-c", head + "echo REACHED", "x", *args], capture_output=True, text=True, timeout=30)
    assert p.returncode == 3 and "REACHED" not in p.stdout and "REFUSED" in p.stdout


@pytest.mark.parametrize("mode, offset, height, expect", [
    ("ppo", "0", "flat", "--policy p.npz"), ("ppo", "180", "flat", "--policy p.npz --start-yaw-offset-deg 180"),
    ("baseline", "0", "flat", ""), ("baseline", "-90", "flat", " --start-yaw-offset-deg -90"), ("passive", "0", "flat", ""),
    ("ppo", "0", "tf", "--policy p.npz --terrain-following"), ("baseline", "90", "tf", " --start-yaw-offset-deg 90 --terrain-following"),
    ("explore", "0", "tf", "--policy p.npz --explorer --max-steps 2000 --terrain-following")])
def test_runner_args_per_mode_offset_and_height(mode, offset, height, expect):
    t = SCRIPT.read_text()
    block = t[t.index('RUNNER_ARGS=""'):t.index('echo "demo:')]
    p = subprocess.run(["bash", "-c", f"MODE={mode}; OFFSET={offset}; HEIGHT={height}; POLICY=p.npz\n{block}echo \"[$RUNNER_ARGS]\""],
                       capture_output=True, text=True, timeout=30)
    assert p.stdout.strip() == f"[{expect}]"


def test_world_selection_exports_the_variant_only_for_variants():
    t = SCRIPT.read_text()
    block = line("if [ \"$VARIANT\" = default ]")
    for variant, env, args in [("default", "", ""), ("c2_sw", "c2_sw", "--world-variant c2_sw")]:
        p = subprocess.run(["bash", "-c", f"SAR_WORLD_VARIANT=stale; VARIANT={variant}\n{block}\n"
                            'echo "[${SAR_WORLD_VARIANT:-}] [$WORLD_ARGS]"'], capture_output=True, text=True, timeout=30)
        assert p.stdout.strip() == f"[{env}] [{args}]"
    assert "$WORLD_ARGS --run-dir $R" in t


def test_the_pinned_policy_is_in_the_repo_and_matches():
    import hashlib
    t = SCRIPT.read_text()
    sha = line("POLICY_SHA256=").split("=", 1)[1]
    path = ROOT / line("POLICY=").split("=", 1)[1]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == sha
    src = ROOT / "models/e1r_300k_s2/final_model.zip"
    import numpy_policy
    assert numpy_policy.NumpyPolicy.load(path).meta["source_sha256"] == numpy_policy.file_sha256(src)


def test_script_safety_properties():
    t = SCRIPT.read_text()
    code = "\n".join(l.split("#", 1)[0] for l in t.splitlines())
    for bad in ("SIGKILL", "kill -9", "kill -KILL", "pkill", "killall", "ros2 topic pub", "GALLIUM_DRIVER",
                "--software-rendering", "train_ppo", "stable_baselines3", "final_model.zip",
                "--no-safety", "--no-filter", "uav_ros_experiments", "typeperf", "run_sampler"):
        assert bad not in code, bad
    # the episode length changes only for explore mode (approved 2026-10-25), nowhere else
    assert [l.strip() for l in code.splitlines() if "--max-steps" in l] == \
        ['[ "$MODE" = explore ] && RUNNER_ARGS="--policy $POLICY --explorer --max-steps 2000"']
    assert code.count("run_target_reaching_episode.py") == 1
    i_run = code.index("run_target_reaching_episode.py")
    assert code.index("if [ $gates_ok -eq 1 ]") < code.index("elif [ $wpass -ne 6 ]") < code.index("marker T_r") \
        < code.index("kill -INT $CLK") < code.index("preflight_state.py") < i_run < code.index('echo "== gated stop')
    assert code.index("POLICY_SHA256=") < code.index("sha256sum $POLICY") < code.index('echo "START')
    assert code.index("isolated_sitl.py start") < code.index("verify_world_consistency.py --run-dir $R") < code.index("if [ $gates_ok -eq 1 ]")
    assert "memlog & MEMLOG=$!" in code and "touch $ABORT" in code
    assert code.index("source /opt/ros/jazzy/setup.bash") < code.index("set -u")      # ROS setup is not set -u clean


@pytest.mark.skipif(not FLOWN_LEAN.exists(), reason="flown experiment script not present")
def test_procedure_is_the_flown_lean_script_line_for_line():
    """Every code line of run_demo.sh is a line of the flown run_stagec_lean.sh, except the documented ports
    (argument handling, repo-relative paths, the policy now in models/, the start-offset runner argument)."""
    strip = lambda s: {l.split("  #", 1)[0].strip() for l in s.splitlines() if l.strip() and not l.lstrip().startswith("#")}  # noqa: E731
    extra = strip(SCRIPT.read_text()) - strip(FLOWN_LEAN.read_text())
    allowed_markers = ("NAME=", "case \"$", "[[ \"$NAME\"", "REPO=", "D=$REPO", "if [ \"$VARIANT\" = default ]", "cd \"$REPO\"",
                       "source /opt/ros", "R=runs/", "X=runs/", "FMU_PARSE=$D", "test ! -e $R", "mkdir -p $X",
                       "POLICY=models/", "[ \"$OFFSET\" != 0 ]", "[ \"$HEIGHT\" = tf ]", "[ \"$MODE\" = explore ]", "echo \"demo:", ".venv/bin/python tools/isolated_sitl.py start",
                       ".venv/bin/python -u $D/guard_observer.py", "timeout 60 .venv/bin/python $D/measure_light.py",
                       "r=$(python3 $D/evaluate_window.py", "pre=$(python3 $D/preflight_state.py", "echo \"== SUPERVISED DEMO FLIGHT")
    unexplained = [l for l in extra if not l.startswith(allowed_markers)]
    assert unexplained == [], unexplained


# ---------- checker ----------

def checker(*args):
    return subprocess.run([str(ROOT / ".venv/bin/python"), str(DEMO / "check_episode.py"), *map(str, args)],
                          capture_output=True, text=True, timeout=60).stdout.strip()


FLOWN = [("stagec_prep_20261025", "restart_20261025_stagec_c2_sw_ppo", "c2_sw", "ppo", 0),
         ("stagec_prep_20261025", "restart_20261025_stagec_c2_sw_baseline", "c2_sw", "baseline", 0),
         ("stageb_prep_20261025", "restart_20261025_stageb2", None, "ppo", 180),
         ("stagec_prep_20261025", "restart_20261025_stagec_c5_w_passive", "c5_w", "passive", 0)]


@pytest.mark.parametrize("folder, run, world, mode, offset", FLOWN)
def test_checker_accepts_the_real_flown_runs(folder, run, world, mode, offset):
    log = Path("/home/hp/uav_ros_experiments") / folder / f"{run}.log"
    if not log.exists():
        pytest.skip("flown logs not present")
    if world is None:
        pytest.skip("Stage B ran before world recording; its log has no world gates")
    assert checker(log, RUNS / run / "logs/episode.jsonl", world, mode, offset).startswith("ok"), run


def test_checker_refuses_a_wrong_world_or_offset():
    log = Path("/home/hp/uav_ros_experiments/stagec_prep_20261025/restart_20261025_stagec_c2_sw_ppo.log")
    if not log.exists():
        pytest.skip("flown logs not present")
    ep = RUNS / "restart_20261025_stagec_c2_sw_ppo/logs/episode.jsonl"
    assert checker(log, ep, "c5_w", "ppo", 0).startswith("stop")
    assert checker(log, ep, "c2_sw", "ppo", 180).startswith("stop")
    assert checker(log, ep, "default", "ppo", 0).startswith("stop")


def test_checker_usage():
    assert checker().startswith("usage")


def test_series_driver_refuses_bad_usage():
    for args in ([], ["Bad Name", "ppo default 0"], ["ok"]):
        p = subprocess.run(["bash", str(DEMO / "run_demo_series.sh"), *args], capture_output=True, text=True, timeout=30)
        assert p.returncode == 2 and "usage" in p.stdout


def test_series_driver_has_no_host_check_or_waits():
    """Requested 2026-10-25: no pre-run host check and no waits between runs (speed is gated by the stability windows)."""
    t = (DEMO / "run_demo_series.sh").read_text()
    code = "\n".join(l.split("#", 1)[0] for l in t.splitlines())
    assert "host_check" not in code and "typeperf" not in code and "sleep" not in code
    assert not (DEMO / "host_check.sh").exists()


RUN_LOG_OK = """FLIGHT RUN: FLOWN (runner exit 0: EPISODE RESULT: success)
PASS: live Gazebo target matches sim/worlds/sar_walls.sdf (sha256 x)
PASS: world variant None, path
PASS: fresh landed, at-rest, disarmed telemetry
stack stopped
"""


def run_runner(*args):
    """The runner in a fresh process, so torch imported by other tests cannot leak into its torch_loaded record."""
    return subprocess.run([str(ROOT / ".venv/bin/python"), str(ROOT / "run_target_reaching_episode.py"), *args],
                          cwd=ROOT, capture_output=True, text=True, timeout=600).returncode


def test_checker_accepts_an_explorer_episode_and_reports_its_phases(tmp_path):
    ep = tmp_path / "episode.jsonl"
    assert run_runner("--standin", "--explorer", "--policy", "models/e1r_300k_s2/final_policy_numpy.npz",
                      "--max-steps", "2000", "--log", str(ep)) == 0
    log = tmp_path / "run.log"
    log.write_text(RUN_LOG_OK)
    out = checker(log, ep, "default", "explore", 0)
    assert out.startswith("ok success") and "explore_steps=" in out and "first_approach_step=" in out, out
    assert checker(log, ep, "default", "ppo", 0).startswith("ok")          # filter reasons are read the same way


def test_checker_refuses_explore_mode_without_an_explorer(tmp_path):
    ep = tmp_path / "episode.jsonl"
    run_runner("--standin", "--policy", "models/e1r_300k_s2/final_policy_numpy.npz", "--log", str(ep))
    log = tmp_path / "run.log"
    log.write_text(RUN_LOG_OK)
    assert "no explorer" in checker(log, ep, "default", "explore", 0)


def test_max_steps_reaches_the_env_and_is_bounded(tmp_path):
    import json
    import run_target_reaching_episode as runner
    ep = tmp_path / "e.jsonl"
    runner.main(["--standin", "--max-steps", "1500", "--log", str(ep)])
    w = next(json.loads(l) for l in ep.read_text().splitlines() if '"event": "world"' in l)
    assert w["max_steps"] == 1500
    for bad in ("0", "6000"):
        with pytest.raises(SystemExit):
            runner.main(["--standin", "--max-steps", bad, "--log", str(tmp_path / f"x{bad}.jsonl")])
