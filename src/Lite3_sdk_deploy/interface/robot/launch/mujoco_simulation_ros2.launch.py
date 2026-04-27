from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    mode = int(LaunchConfiguration('mode').perform(context))
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'True' or localization == 'true'
    control_type = int(LaunchConfiguration('control_type').perform(context))
    nav2_params_filepath_launch_arg = LaunchConfiguration('nav2_params_filepath')
    use_sim_time = LaunchConfiguration("use_sim_time")
    database_path = LaunchConfiguration("database_path")

    nav2_package_share = FindPackageShare("nav2_bringup").perform(context)
    lite3_package_share = FindPackageShare("lite3_sdk_deploy").perform(context)

    nav2_launch = (
        f"{nav2_package_share}"
        "/launch/navigation_launch.py"
    )
    rviz_launch = (
        f"{nav2_package_share}"
        "/launch/rviz_launch.py"
    )

    if mode == 0:
        rtabmap_mode = "lidar"
        rviz_filepath = f"{lite3_package_share}/config/mapping_lidar_costmaps.rviz"
    elif mode == 1:
        rtabmap_mode = "rgbd"
        rviz_filepath = f"{lite3_package_share}/config/mapping_rgbd_costmaps.rviz"
    else:
        rtabmap_mode = "rgbd_lidar"
        rviz_filepath = f"{lite3_package_share}/config/mapping_rgbd_lidar_costmaps.rviz"

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{lite3_package_share}/launch/rtabmap_{rtabmap_mode}.launch.py"),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "localization": localization,
            "database_path": database_path,
        }.items(),
    )

    rl_deploy_prefix = ''
    if control_type == 0:
        rl_deploy_args = ["--twist"]
    elif control_type == 1:
        rl_deploy_args = []
        rl_deploy_prefix = 'xterm -e'
    else:
        rl_deploy_args = ["--gamepad"]

    return [

        # MuJoCo simulation
        Node(
            package='lite3_sdk_deploy', 
            executable='mujoco_simulation_ros2.py',
            output='screen',
            parameters = [{
                'use_procedural_scene': True,
                'procedural_env_seed': 1234,
            }],
        ),

        # RL controller with twist input
        Node(
            package='lite3_sdk_deploy', 
            executable='rl_deploy',
            output='screen',
            arguments=rl_deploy_args,
            prefix=rl_deploy_prefix,
        ),

        # RTAB-Map
        rtabmap_launch, 

        # Navigation (planner + controller)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch]),
            launch_arguments=[
                ('params_file', nav2_params_filepath_launch_arg)
            ]
        ),   

        # RViz2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_launch]),
            launch_arguments=[
                ('rviz_config', rviz_filepath)
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("database_path", default_value="~/.ros/rtabmap.db"),

        DeclareLaunchArgument(
            'mode', default_value='2',
            description='RTAB-Map mode: 0 (lidar), 1 (rgbd), 2 (lidar+rgbd)'
        ),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'
        ),

        DeclareLaunchArgument(
            'control_type', default_value='0',
            description='Joints control type: 0 (twist), 1 (keyboard), 2 (gamepad)'
        ),

        DeclareLaunchArgument(
            'nav2_params_filepath',
            default_value=PathJoinSubstitution([
                get_package_share_directory('lite3_sdk_deploy'),
                'config', 'nav2_params.yaml'
            ]),
            description='the file path to Nav2 params'
        ),

        DeclareLaunchArgument(
            'nav2_rviz_filepath',
            default_value=PathJoinSubstitution([
                get_package_share_directory('lite3_sdk_deploy'),
                'config', 'mapping2.rviz'
            ]),
            description='the file path to Nav2 rviz config'
        ),

        OpaqueFunction(function=launch_setup)
    ])
