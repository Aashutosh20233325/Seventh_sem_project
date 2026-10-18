import json

import pytest

import run_target_reaching_episode as runner


def test_standin_dry_run_reaches_target_and_logs_every_step(tmp_path):
    log = tmp_path / "episode.jsonl"
    assert runner.main(["--standin", "--log", str(log)]) == 0
    events = [json.loads(line) for line in log.read_text().splitlines()]
    kinds = [e["event"] for e in events]
    assert kinds[0] == "reset_start" and kinds[-2:] == ["close_start", "close_returned"]
    steps = [e for e in events if e["event"] == "step"]
    assert steps and steps[-1]["info"]["reason"] == "success"
    assert all("wall_utc" in e and "mono" in e for e in events)
    assert all("distance_to_target" not in e["fields"] for e in steps)


def test_sitl_run_requires_explicit_supervision_flag(tmp_path):
    with pytest.raises(SystemExit):
        runner.main(["--log", str(tmp_path / "x.jsonl")])


def test_existing_log_is_never_overwritten(tmp_path):
    log = tmp_path / "episode.jsonl"
    log.write_text("keep\n")
    with pytest.raises(FileExistsError):
        runner.main(["--standin", "--log", str(log)])
    assert log.read_text() == "keep\n"
