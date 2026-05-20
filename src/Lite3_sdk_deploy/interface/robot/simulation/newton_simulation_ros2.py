#!/usr/bin/env python3

import importlib.util
import sys
from pathlib import Path


def main():
    newton_dir = Path(__file__).resolve().parent / "newton"
    if str(newton_dir) not in sys.path:
        sys.path.insert(0, str(newton_dir))

    module_path = newton_dir / "newton_simulation_ros2.py"
    spec = importlib.util.spec_from_file_location("lite3_newton_simulation_ros2", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.main()


if __name__ == "__main__":
    main()
