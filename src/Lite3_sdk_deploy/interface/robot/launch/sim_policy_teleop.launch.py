from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_procedural_scene = LaunchConfiguration("use_procedural_scene")
    procedural_env_seed = LaunchConfiguration("procedural_env_seed")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_mid360 = LaunchConfiguration("enable_mid360")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_color = LaunchConfiguration("enable_color")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")
    enable_heightmap = LaunchConfiguration("enable_heightmap")

    enable_mid360_value = enable_mid360.perform(context).lower() == "true"
    enable_depth_value = enable_depth.perform(context).lower() == "true"
    enable_pointcloud_value = enable_pointcloud.perform(context).lower() == "true"
    enable_heightmap_value = enable_heightmap.perform(context).lower() == "true"

    use_keyboard_teleop = LaunchConfiguration("use_keyboard_teleop")
    use_joy_teleop = LaunchConfiguration("use_joy_teleop")

    joy_config = PathJoinSubstitution([
        FindPackageShare("lite3_sdk_deploy"), "config", "f310_holonomic.yaml"
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("lite3_sdk_deploy"), "config", "teleop.rviz"
    ])

    heightmap_cloud_topic = None
    if enable_mid360_value:
        heightmap_cloud_topic = "/mid360/points"
    elif enable_depth_value and enable_pointcloud_value:
        heightmap_cloud_topic = "/camera/depth/color/points"

    actions = [
        Node(
            package="lite3_sdk_deploy",
            executable="mujoco_simulation_ros2.py",
            output="screen",
            parameters=[{
                "use_procedural_scene": use_procedural_scene,
                "procedural_env_seed": procedural_env_seed,
                "headless": headless,
                "enable_lidar": enable_lidar,
                "enable_mid360": enable_mid360,
                "enable_depth": enable_depth,
                "enable_color": enable_color,
                "enable_pointcloud": enable_pointcloud,
            }],
        ),
    ]

    if enable_heightmap_value and heightmap_cloud_topic is not None:
        actions.append(
            Node(
                package="simple_local_heightmap",
                executable="local_heightmap_node",
                name="local_heightmap_node",
                output="screen",
                parameters=[{
                    "cloud_topic": heightmap_cloud_topic,
                    "map_frame": "base_link",
                    "resolution": 0.025,
                    "length_x": 8.0,
                    "length_y": 8.0,
                }],
            )
        )
    elif enable_heightmap_value:
        actions.append(
            LogInfo(
                msg=(
                    "simple_local_heightmap not started: enable_mid360:=true or "
                    "enable_depth:=true enable_pointcloud:=true is required"
                )
            )
        )

    actions.extend([
        Node(
            package="lite3_sdk_deploy",
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

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_joy_teleop")),
        ),

        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            output="screen",
            parameters=[joy_config],
            remappings=[("cmd_vel", "/cmd_vel")],
            condition=IfCondition(LaunchConfiguration("use_joy_teleop")),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(
                PythonExpression([
                    "'", use_rviz, "' == 'true' and '", headless, "' != 'true'"
                ])
            ),
        ),
    ])

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_procedural_scene", default_value="true"),
        DeclareLaunchArgument("procedural_env_seed", default_value="-1"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        DeclareLaunchArgument("enable_lidar", default_value="false"),
        DeclareLaunchArgument("enable_mid360", default_value="false"),
        DeclareLaunchArgument("enable_depth", default_value="false"),
        DeclareLaunchArgument("enable_color", default_value="false"),
        DeclareLaunchArgument("enable_pointcloud", default_value="false"),
        DeclareLaunchArgument(
            "enable_heightmap",
            default_value="false",
            description="Launch the simple local heightmap node for Mid360 testing",
        ),

        DeclareLaunchArgument(
            "use_keyboard_teleop",
            default_value="false",
            description="Launch teleop_twist_keyboard (interactive terminal required)",
        ),
        DeclareLaunchArgument(
            "use_joy_teleop",
            default_value="false",
            description="Launch joy_node + teleop_twist_joy",
        ),

        OpaqueFunction(function=launch_setup),
    ])
