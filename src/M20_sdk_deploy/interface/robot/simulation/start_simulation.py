#!/usr/bin/env python3
"""Entry point for M20 MuJoCo simulation with optional YAML config."""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from simulation_config import SimulationConfig


def _parse_args():
    import argparse

    parser = argparse.ArgumentParser(description="Start the M20 MuJoCo simulator")
    parser.add_argument("--config", default=None, help="Path to simulation YAML config")
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="PATH=VALUE",
        help="Override a simulation config field using dotted syntax, e.g. sensors.lidar_2d.enabled=true",
    )
    return parser.parse_known_args()


def _parse_overrides(items: list[str]) -> dict[str, object]:
    overrides: dict[str, object] = {}
    for item in items:
        key, separator, raw_value = item.partition("=")
        if not separator:
            raise SystemExit(f"Invalid simulation override '{item}', expected PATH=VALUE")
        dotted_key = key.strip()
        if not dotted_key:
            raise SystemExit(f"Invalid simulation override '{item}', PATH cannot be empty")
        overrides[dotted_key] = yaml.safe_load(raw_value)
    return overrides


def main():
    args, ros_args = _parse_args()
    config = SimulationConfig.load(args.config).with_overrides(_parse_overrides(args.set))
    errors = config.validate()
    if errors:
        raise SystemExit("Invalid simulation config:\n- " + "\n- ".join(errors))

    from mujoco_simulation_ros2 import run_mujoco
    run_mujoco(config, ros_args)


if __name__ == "__main__":
    main()
