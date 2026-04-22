from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_procedural_scene = LaunchConfiguration("use_procedural_scene")
    procedural_env_seed = LaunchConfiguration("procedural_env_seed")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    database_path = LaunchConfiguration("database_path")

    package_share = FindPackageShare("lite3_sdk_deploy")

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "rtabmap.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "localization": "false",
            "database_path": database_path,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_procedural_scene", default_value="true"),
            DeclareLaunchArgument("procedural_env_seed", default_value="-1"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("database_path", default_value="~/.ros/rtabmap.db"),
            Node(
                package="lite3_sdk_deploy",
                executable="mujoco_simulation_ros2.py",
                output="screen",
                parameters=[
                    {
                        "use_procedural_scene": use_procedural_scene,
                        "procedural_env_seed": procedural_env_seed,
                        "headless": headless,
                    }
                ],
            ),
            Node(
                package="lite3_sdk_deploy",
                executable="rl_deploy",
                output="screen",
                arguments=["--twist"],
            ),
            Node(
                package="lite3_sdk_deploy",
                executable="auto_waypoint_navigator.py",
                output="screen",
            ),
            rtabmap_launch,
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", PathJoinSubstitution([package_share, "config", "mapping.rviz"])],
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_rviz, "' == 'true' and '", headless, "' != 'true'"]
                    )
                ),
            ),
        ]
    )
