"""tools/mission_map.py on a tiny fabricated flight log: the GIF and summary card are written, the target is marked only
after it was seen, the summary numbers come from the log."""
import json
import subprocess
import sys
from pathlib import Path

from PIL import Image

REPO = Path(__file__).resolve().parent


def fake_run(tmp: Path) -> Path:
    run = tmp / "restart_fake"
    (run / "logs").mkdir(parents=True)
    rows = [{"event": "world", "variant": "gen_mine_s1", "target_ned": [-9.0, 8.0]}]
    tel = []
    for k in range(12):
        mono = 100.0 + k * 0.1
        mode = "EXPLORE" if k < 8 else "APPROACH"
        info = {"privileged": {"x": 0.3 * k, "y": 0.0, "heading": 0.0}}
        if k == 11:
            info["supply_drop_ned"] = [-9.8, 8.0]
        rows.append({"event": "step", "step": k, "mono": mono, "info": info,
                     "policy": {"mode": mode, "goal": [5.0, 0.0]}})
        tel.append({"event": "lidar_sectors", "mono": mono - 0.05, "msg": {"data": [6.0] * 49}})
    rows.append({"event": "episode_end", "reason": "success"})
    (run / "logs/episode.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (run / "logs/telemetry_full.jsonl").write_text("".join(json.dumps(r) + "\n" for r in tel))
    return run


def test_mission_map_writes_the_gif_and_the_summary(tmp_path):
    run = fake_run(tmp_path)
    p = subprocess.run([sys.executable, str(REPO / "tools/mission_map.py"), str(run), "--every", "3"],
                       capture_output=True, text=True, timeout=300)
    assert p.returncode == 0, p.stderr
    gif, card = run / "presentation/mission_map.gif", run / "presentation/mission_summary.png"
    assert Image.open(gif).n_frames >= 4 and Image.open(card).size == (1400, 700)
    assert "Miner first seen: 0.8 s (step 8)" in p.stdout
    assert "Supply packet: dropped beside the miner" in p.stdout
    assert "9.0 m south, 8.0 m east of the borehole" in p.stdout
