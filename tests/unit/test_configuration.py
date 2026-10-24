from pathlib import Path

import yaml

ROOT = Path(__file__).parents[2]


def test_safety_conscious_defaults():
    config = yaml.safe_load((ROOT / "src/vgm_bringup/config/task.yaml").read_text())
    assert config["task_manager"]["ros__parameters"]["plan_only"] is True
    assert config["motion_server"]["ros__parameters"]["velocity_scaling"] <= 0.2

