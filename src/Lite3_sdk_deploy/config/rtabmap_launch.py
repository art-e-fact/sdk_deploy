from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Set to true to run in localization mode (reuse existing map)'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db',
                              description='Path to RTAB-Map database file'),

        # RTAB-Map SLAM node (2D lidar only, ICP odometry)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_scan': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_odom_info': False,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                'use_action_for_goal': True,
                'database_path': database_path,
                'queue_size': 30,
                'approx_sync': True,

                # RTAB-Map parameters
                'Mem/IncrementalMemory': 'true',  # set to false for localization
                'Mem/InitWMWithAllNodes': 'false',  # set to true for localization

                # Grid map from laser scans
                'Grid/FromDepth': 'false',
                'Grid/RangeMin': '0.3',  # ignore laser hits on the robot body
                'Grid/RangeMax': '8.0',  # match RPLiDAR A2M8 max range
                'Grid/CellSize': '0.05',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '2.0',
                'Grid/Sensor': '0',  # laser scan

                # Registration / ICP
                'Reg/Strategy': '1',  # 1 = ICP
                'Reg/Force3DoF': 'true',  # 2D mode
                'Icp/VoxelSize': '0.05',  # downsample scans before ICP
                'Icp/MaxCorrespondenceDistance': '0.1',
                'Icp/PM': 'true',
                'Icp/PMOutlierRatio': '0.65',
                'Icp/CorrespondenceRatio': '0.2',

                # Loop closure
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityMaxGraphDepth': '0',
                'RGBD/AngularUpdate': '0.05',
                'RGBD/LinearUpdate': '0.05',
                'RGBD/OptimizeFromGraphEnd': 'false',

                # Optimizer
                'Optimizer/GravitySigma': '0',  # disable IMU constraints (2D mode)

                # Memory management
                'Rtabmap/DetectionRate': '1.0',
            }],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
                ('grid_map', '/map'),
            ],
        ),
    ])
