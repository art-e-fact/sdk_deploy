"""Launch M20 MuJoCo sim + RL deploy with twist control + optional teleop.

Usage:
  ros2 launch m20_sdk_deploy sim_policy_teleop.launch.py
  ros2 launch m20_sdk_deploy sim_policy_teleop.launch.py simulation_config:=src/M20_sdk_deploy/config/simulations/mujoco_rgbd_lidar.yaml
  ros2 launch m20_sdk_deploy sim_policy_teleop.launch.py headless:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_simulation_config(package_share: str) -> str:
    return f"{package_share}/config/simulations/mujoco_default.yaml"


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration("headless")
    simulation_config = LaunchConfiguration("simulation_config").perform(context).strip()
    use_keyboard_teleop = LaunchConfiguration("use_keyboard_teleop")

    package_share = FindPackageShare("m20_sdk_deploy").perform(context)
    headless_value = headless.perform(context).strip().lower()
    selected_config = simulation_config or _default_simulation_config(package_share)

    return [
        Node(
            package="m20_sdk_deploy",
            executable="start_simulation.py",
            output="screen",
            arguments=[
                "--config", selected_config,
                *(["--set", f"headless={headless_value}"] if headless_value in {"true", "false"} else []),
            ],
        ),

        Node(
            package="m20_sdk_deploy",
            executable="rl_deploy",
            output="screen",
            arguments=["--twist"],
        ),

        Node(
            package="teleop_twist_keyboard",
            executable="teleop_twist_keyboard",
            name="teleop_twist_keyboard",
            output="screen",
            remappings=[("cmd_vel", "/cmd_vel")],
            emulate_tty=True,
            condition=IfCondition(
                PythonExpression([
                    "'", use_keyboard_teleop, "' == 'true' and '", headless, "' != 'true'"
                ])
            ),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value=""),
        DeclareLaunchArgument("simulation_config", default_value=""),
        DeclareLaunchArgument(
            "use_keyboard_teleop",
            default_value="false",
            description="Launch teleop_twist_keyboard (interactive terminal required)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
