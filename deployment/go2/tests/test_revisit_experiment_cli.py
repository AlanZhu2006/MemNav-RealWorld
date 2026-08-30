from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "deployment" / "go2" / "offboard" / "revisit_experiment.sh"


def _run(*arguments: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(SCRIPT), *arguments],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def test_formal_start_requires_scene_bound_identity():
    result = _run("formal-start", "survey01", "--arm", "mono_cec")

    assert result.returncode != 0
    assert "formal-start requires --scene-id" in result.stderr


def test_formal_start_rejects_goal_not_matching_registered_sha(tmp_path: Path):
    goal = tmp_path / "goal.jpg"
    goal.write_bytes(b"frozen goal bytes")
    actual = hashlib.sha256(goal.read_bytes()).hexdigest()
    wrong = ("0" if actual[0] != "0" else "1") + actual[1:]
    result = _run(
        "formal-start",
        "survey01",
        "--scene-id",
        "scene01",
        "--run-id",
        "scene01_pair01_cec",
        "--arm",
        "mono_cec",
        "--goal",
        str(goal),
        "--expected-goal-sha256",
        wrong,
        "--expected-dataset-sha256",
        "a" * 64,
    )

    assert result.returncode != 0
    assert "frozen goal SHA mismatch" in result.stderr


def test_help_exposes_role_hidden_frozen_goal_contract():
    result = _run("--help")

    assert result.returncode == 0
    assert "--expected-goal-sha256" in result.stdout
    assert "No Novel/Revisit label is passed to runtime" in result.stdout
