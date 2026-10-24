import ast
from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def test_launch_file_is_valid_python():
    ast.parse((PACKAGE / "launch" / "demo.launch.py").read_text())


def test_task_configuration_defaults_to_plan_only():
    config = yaml.safe_load((PACKAGE / "config" / "task.yaml").read_text())
    assert config["task_manager"]["ros__parameters"]["plan_only"] is True
    assert config["motion_server"]["ros__parameters"]["execution_enabled"] is False

