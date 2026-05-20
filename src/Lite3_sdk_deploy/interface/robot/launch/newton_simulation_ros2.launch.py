from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    control_type = int(LaunchConfiguration("control_type").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    mjcf_path = LaunchConfiguration("mjcf").perform(context).strip()
    usd_path = LaunchConfiguration("usd").perform(context).strip()

    rl_deploy_prefix = ""
    if control_type == 0:
        rl_deploy_args = ["--twist"]
    elif control_type == 1:
        rl_deploy_args = []
        rl_deploy_prefix = "xterm -e"
    elif control_type == 2:
        rl_deploy_args = ["--gamepad"]
    else:
        raise ValueError("control_type must be 0 (twist), 1 (keyboard), or 2 (gamepad)")

    newton_args = []
    if mjcf_path:
        newton_args.extend(["--mjcf", mjcf_path])
    if usd_path:
        newton_args.extend(["--usd", usd_path])
    if headless:
        newton_args.append("--headless")
    else:
        newton_args.append("--viewer")

    return [
        Node(
            package="lite3_sdk_deploy",
            executable="newton_simulation_ros2.py",
            name="newton_simulation",
            output="screen",
            arguments=newton_args,
            parameters=[{"headless": headless, "model_path": usd_path if usd_path else mjcf_path}],
        ),
        Node(
            package="lite3_sdk_deploy",
            executable="rl_deploy",
            name="rl_deploy",
            output="screen",
            arguments=rl_deploy_args,
            prefix=rl_deploy_prefix,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "control_type",
                default_value="0",
                description="Control type: 0 (twist), 1 (keyboard), 2 (gamepad).",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run Newton simulation without viewer integration.",
            ),
            DeclareLaunchArgument(
                "mjcf",
                default_value="",
                description="Optional absolute path to a Lite3 MJCF robot description.",
            ),
            DeclareLaunchArgument(
                "usd",
                default_value="",
                description="Deprecated optional absolute path to a Lite3 USD robot description.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )