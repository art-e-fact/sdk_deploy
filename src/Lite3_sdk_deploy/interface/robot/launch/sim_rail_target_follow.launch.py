from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    scene_type = LaunchConfiguration('scene_type')
    procedural_env_seed = LaunchConfiguration('procedural_env_seed')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_mid360 = LaunchConfiguration('enable_mid360')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_color = LaunchConfiguration('enable_color')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')
    enable_heightmap = LaunchConfiguration('enable_heightmap')

    enable_mid360_value = enable_mid360.perform(context).lower() == 'true'
    enable_depth_value = enable_depth.perform(context).lower() == 'true'
    enable_pointcloud_value = enable_pointcloud.perform(context).lower() == 'true'
    enable_heightmap_value = enable_heightmap.perform(context).lower() == 'true'

    use_keyboard_teleop = LaunchConfiguration('use_keyboard_teleop')
    use_joy_teleop = LaunchConfiguration('use_joy_teleop')
    follow_distance = LaunchConfiguration('follow_distance')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_linear_y = LaunchConfiguration('max_linear_y')
    max_angular_z = LaunchConfiguration('max_angular_z')
    k_distance = LaunchConfiguration('k_distance')
    k_center = LaunchConfiguration('k_center')
    k_heading = LaunchConfiguration('k_heading')
    stale_timeout_sec = LaunchConfiguration('stale_timeout_sec')

    joy_config = PathJoinSubstitution([
        FindPackageShare('lite3_sdk_deploy'), 'config', 'f310_holonomic.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('lite3_sdk_deploy'), 'config', 'teleop.rviz'
    ])

    heightmap_cloud_topic = None
    if enable_mid360_value:
        heightmap_cloud_topic = '/mid360/points'
    elif enable_depth_value and enable_pointcloud_value:
        heightmap_cloud_topic = '/camera/depth/color/points'

    actions = [
        Node(
            package='lite3_sdk_deploy',
            executable='mujoco_simulation_ros2.py',
            output='screen',
            parameters=[{
                'scene_type': scene_type,
                'procedural_env_seed': procedural_env_seed,
                'headless': headless,
                'enable_lidar': enable_lidar,
                'enable_mid360': enable_mid360,
                'enable_depth': enable_depth,
                'enable_color': enable_color,
                'enable_pointcloud': enable_pointcloud,
            }],
        ),
    ]

    if enable_heightmap_value and heightmap_cloud_topic is not None:
        actions.append(
            Node(
                package='simple_local_heightmap',
                executable='local_heightmap_node',
                name='local_heightmap_node',
                output='screen',
                parameters=[{
                    'cloud_topic': heightmap_cloud_topic,
                    'map_frame': 'odom',
                    'robot_frame': 'base_link',
                    'resolution': 0.025,
                    'length_x': 8.0,
                    'length_y': 8.0,
                    'front_clear_enabled': True,
                    'front_clear_length': 1.5,
                    'front_clear_width': 1.0,
                    'front_clear_offset_x': 0.25,
                    'front_stale_time_sec': 0.75,
                }],
            )
        )
        actions.append(
            Node(
                package='simple_local_heightmap',
                executable='rail_detector_node',
                name='rail_detector_node',
                output='screen',
                parameters=[{
                    'heightmap_topic': '/local_heightmap',
                    'odom_topic': '/odom',
                    'marker_topic': '/rail_detector/markers',
                    'center_offset_topic': '/rail_detector/center_offset',
                    'tangent_yaw_topic': '/rail_detector/tangent_yaw',
                    'target_distance_topic': '/rail_detector/target_distance',
                    'track_gauge': 1.067,
                }],
            )
        )
        actions.append(
            Node(
                package='simple_local_heightmap',
                executable='rail_target_follower_node',
                name='rail_target_follower_node',
                output='screen',
                parameters=[{
                    'cmd_vel_topic': '/cmd_vel',
                    'odom_topic': '/odom',
                    'center_offset_topic': '/rail_detector/center_offset',
                    'tangent_yaw_topic': '/rail_detector/tangent_yaw',
                    'target_distance_topic': '/rail_detector/target_distance',
                    'follow_distance': follow_distance,
                    'max_linear_x': max_linear_x,
                    'max_linear_y': max_linear_y,
                    'max_angular_z': max_angular_z,
                    'k_distance': k_distance,
                    'k_center': k_center,
                    'k_heading': k_heading,
                    'stale_timeout_sec': stale_timeout_sec,
                }],
            )
        )
    elif enable_heightmap_value:
        actions.append(
            LogInfo(
                msg=(
                    'simple_local_heightmap not started: enable_mid360:=true or '
                    'enable_depth:=true enable_pointcloud:=true is required'
                )
            )
        )

    actions.extend([
        Node(
            package='lite3_sdk_deploy',
            executable='rl_deploy',
            output='screen',
            arguments=['--twist'],
        ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            remappings=[('cmd_vel', '/cmd_vel')],
            emulate_tty=True,
            condition=IfCondition(
                PythonExpression([
                    "'", use_keyboard_teleop, "' == 'true' and '", headless, "' != 'true'"
                ])
            ),
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_joy_teleop')),
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_config],
            remappings=[('cmd_vel', '/cmd_vel')],
            condition=IfCondition(LaunchConfiguration('use_joy_teleop')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
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
        DeclareLaunchArgument('scene_type', default_value='railroad'),
        DeclareLaunchArgument('procedural_env_seed', default_value='123'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        DeclareLaunchArgument('enable_lidar', default_value='false'),
        DeclareLaunchArgument('enable_mid360', default_value='true'),
        DeclareLaunchArgument('enable_depth', default_value='false'),
        DeclareLaunchArgument('enable_color', default_value='false'),
        DeclareLaunchArgument('enable_pointcloud', default_value='false'),
        DeclareLaunchArgument(
            'enable_heightmap',
            default_value='true',
            description='Launch the simple local heightmap, rail detector, and rail follower nodes',
        ),

        DeclareLaunchArgument(
            'follow_distance',
            default_value='1.5',
            description='Desired stand-off distance to the detected target in meters',
        ),
        DeclareLaunchArgument(
            'max_linear_x',
            default_value='0.55',
            description='Maximum forward body-frame speed command in meters per second',
        ),
        DeclareLaunchArgument(
            'max_linear_y',
            default_value='0.4',
            description='Maximum lateral body-frame speed command in meters per second',
        ),
        DeclareLaunchArgument(
            'max_angular_z',
            default_value='0.5',
            description='Maximum yaw-rate command in radians per second',
        ),
        DeclareLaunchArgument(
            'k_distance',
            default_value='0.7',
            description='Gain that converts target distance error into along-rail speed',
        ),
        DeclareLaunchArgument(
            'k_center',
            default_value='1.0',
            description='Gain that converts rail center offset into lateral correction speed',
        ),
        DeclareLaunchArgument(
            'k_heading',
            default_value='1.2',
            description='Gain that converts rail tangent yaw error into angular speed',
        ),
        DeclareLaunchArgument(
            'stale_timeout_sec',
            default_value='0.5',
            description='Maximum wall-time age accepted for detector and odometry inputs',
        ),

        DeclareLaunchArgument(
            'use_keyboard_teleop',
            default_value='false',
            description='Launch teleop_twist_keyboard (interactive terminal required)',
        ),
        DeclareLaunchArgument(
            'use_joy_teleop',
            default_value='false',
            description='Launch joy_node + teleop_twist_joy',
        ),

        OpaqueFunction(function=launch_setup),
    ])