from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    control_type = int(LaunchConfiguration("control_type").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    mjcf_path = LaunchConfiguration("mjcf").perform(context).strip()
    usd_path = LaunchConfiguration("usd").perform(context).strip()
    enable_lidar = LaunchConfiguration("enable_lidar").perform(context).lower() == "true"
    enable_depth = LaunchConfiguration("enable_depth").perform(context).lower() == "true"
    enable_color = LaunchConfiguration("enable_color").perform(context).lower() == "true"
    enable_pointcloud = LaunchConfiguration("enable_pointcloud").perform(context).lower() == "true"
    enable_mid360 = LaunchConfiguration("enable_mid360").perform(context).lower() == "true"
    rviz_config = LaunchConfiguration("rviz_config")

    nav2_package_share = FindPackageShare("nav2_bringup").perform(context)
    rviz_launch = f"{nav2_package_share}/launch/rviz_launch.py"

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
            parameters=[{
                "headless": headless,
                "model_path": usd_path if usd_path else mjcf_path,
                "enable_lidar": enable_lidar,
                "enable_mid360": enable_mid360,
                "enable_depth": enable_depth,
                "enable_color": enable_color,
                "enable_pointcloud": enable_pointcloud,
            }],
        ),
        Node(
            package="lite3_sdk_deploy",
            executable="rl_deploy",
            name="rl_deploy",
            output="screen",
            arguments=rl_deploy_args,
            prefix=rl_deploy_prefix,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_launch]),
            launch_arguments=[
                ("rviz_config", rviz_config),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="false",
                description="Publish 2D lidar scans.",
            ),
            DeclareLaunchArgument(
                "enable_depth",
                default_value="false",
                description="Publish RealSense depth images.",
            ),
            DeclareLaunchArgument(
                "enable_color",
                default_value="false",
                description="Publish RealSense color images.",
            ),
            DeclareLaunchArgument(
                "enable_pointcloud",
                default_value="false",
                description="Publish RealSense pointcloud (debug; off by default).",
            ),
            DeclareLaunchArgument(  
                "enable_mid360",
                default_value="false",
                description="Publish Mid360 pointcloud (off by default).",
            ),
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
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([
                    get_package_share_directory("lite3_sdk_deploy"),
                    "config",
                    "sensors.rviz",
                ]),
                description="RViz config file to load.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )