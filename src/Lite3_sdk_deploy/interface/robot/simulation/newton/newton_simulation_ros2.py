#!/usr/bin/env python3

import argparse
import sys
import time
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

import rclpy

from ros_bridge import NewtonRosBridge
from simulation import DT, ODOM_EVERY_STEPS, PUBLISH_EVERY_STEPS, ROS_SPIN_EVERY_STEPS, NewtonSimulation, create_newton_viewer, default_mjcf_path

RENDER_EVERY_STEPS = 50

def run_loop(sim: NewtonSimulation, ros: NewtonRosBridge):
    next_step_time = time.perf_counter()
    while rclpy.ok() and not ros.should_exit():
        sleep_time = next_step_time - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_step_time = time.perf_counter()
        next_step_time += DT

        if sim.step_count % ROS_SPIN_EVERY_STEPS == 0:
            ros.spin_once()
        sim.set_command(ros.read_latest_action())
        sim.step()

        state = None
        if sim.step_count % PUBLISH_EVERY_STEPS == 0:
            state = sim.state_snapshot()
            ros.publish_state(sim.timestamp, state, sim.last_tau)
        if sim.step_count % ODOM_EVERY_STEPS == 0:
            state = state or sim.state_snapshot()
            ros.publish_odom_and_tf(sim.timestamp, state)
        
        if sim.step_count % RENDER_EVERY_STEPS == 0:
            sim.render()


def main():
    parser = argparse.ArgumentParser(description="Run Lite3 Newton ROS2 simulation")
    parser.add_argument("--mjcf", default=default_mjcf_path(), help="Path to Lite3 MJCF robot description")
    parser.add_argument("--usd", default=None, help="Deprecated: path to Lite3 USD robot description")
    parser.add_argument("--headless", dest="headless", action="store_true", default=True, help="Run without a Newton viewer")
    parser.add_argument("--viewer", dest="headless", action="store_false", help="Show the Newton viewer")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    viewer = None if args.headless else create_newton_viewer()
    model_path = args.usd if args.usd is not None else args.mjcf
    ros = NewtonRosBridge(headless=args.headless, model_path=model_path)
    sim = NewtonSimulation(model_path=model_path, headless=args.headless, viewer=viewer, logger=ros.get_logger())

    try:
        run_loop(sim, ros)
    finally:
        ros.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
