import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


MIN_DISTANCE_M = 4.5
TEST_TIMEOUT_SEC = 120.0
MAX_ODOM_STEP_M = 1.0
TEST_VIDEO_PATH = './lite3_rail_target_follow_distance.mp4'


@pytest.mark.launch_test
def generate_test_description():
    if os.path.exists(TEST_VIDEO_PATH):
        os.remove(TEST_VIDEO_PATH)

    launch_file = (
        get_package_share_directory('lite3_sdk_deploy')
        + '/launch/sim_rail_target_follow.launch.py'
    )

    rail_follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'scene_type': 'railroad',
            'procedural_env_seed': '123',
            'headless': 'false',
            'use_rviz': 'false',
            'use_keyboard_teleop': 'false',
            'use_joy_teleop': 'false',
            'enable_lidar': 'false',
            'enable_mid360': 'true',
            'enable_depth': 'false',
            'enable_color': 'false',
            'enable_pointcloud': 'false',
            'enable_heightmap': 'true',
            'follow_distance': '1.5',
            'min_linear_x': '0.35',
            'max_linear_x': '0.45',
            'stale_timeout_sec': '0.75',
            'enable_follow_camera': 'true',
            'follow_camera_video_path': TEST_VIDEO_PATH,
        }.items(),
    )

    return LaunchDescription([
        rail_follow_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestRailTargetFollowDistance(unittest.TestCase):

    def test_robot_travels_minimum_distance(self):
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node('test_sim_rail_target_follow_distance', context=context)
        executor = SingleThreadedExecutor(context=context)
        state = {
            'distance_m': 0.0,
            'message_count': 0,
            'previous_xy': None,
            'last_xy': None,
        }

        def odom_callback(msg: Odometry):
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            xy = (x, y)
            state['message_count'] += 1
            state['last_xy'] = xy

            previous_xy = state['previous_xy']
            state['previous_xy'] = xy
            if previous_xy is None:
                return

            step_m = math.hypot(x - previous_xy[0], y - previous_xy[1])
            if step_m <= MAX_ODOM_STEP_M:
                state['distance_m'] += step_m

        subscription = node.create_subscription(Odometry, '/odom', odom_callback, 20)
        deadline = time.monotonic() + TEST_TIMEOUT_SEC

        try:
            while time.monotonic() < deadline and state['distance_m'] < MIN_DISTANCE_M:
                rclpy.spin_once(node, executor=executor, timeout_sec=0.1)

            elapsed_sec = TEST_TIMEOUT_SEC - max(0.0, deadline - time.monotonic())
            self.assertGreaterEqual(
                state['distance_m'],
                MIN_DISTANCE_M,
                msg=(
                    f"robot travelled {state['distance_m']:.3f} m in {elapsed_sec:.1f} s; "
                    f"expected at least {MIN_DISTANCE_M:.3f} m "
                    f"from {state['message_count']} odom messages, last_xy={state['last_xy']}"
                ),
            )
        finally:
            executor.shutdown(timeout_sec=0.0)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown(context=context)


@launch_testing.post_shutdown_test()
class TestRailTargetFollowVideo(unittest.TestCase):

    def test_follow_camera_video_created(self):
        self.assertTrue(
            os.path.exists(TEST_VIDEO_PATH),
            msg=f"expected follow-camera video at {TEST_VIDEO_PATH}",
        )
        self.assertGreater(
            os.path.getsize(TEST_VIDEO_PATH),
            1024,
            msg=f"follow-camera video at {TEST_VIDEO_PATH} is empty or too small",
        )
