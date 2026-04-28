from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration("headless")
    use_rtabmap_rviz = LaunchConfiguration("use_rtabmap_rviz")
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path').perform(context)
    localization = LaunchConfiguration('localization').perform(context)
    localization = localization.lower() == 'true'
    max_ground_height = LaunchConfiguration('max_ground_height').perform(context)
    max_ground_angle = LaunchConfiguration('max_ground_angle').perform(context)
    arguments = []

    parameters = {
        'use_sim_time': use_sim_time,
        'subscribe_scan': True,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom_info': False,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'publish_tf': True,
        'use_action_for_goal': True,
        'database_path': database_path,
        'queue_size': 30,
        'approx_sync': True,

        ### Grid map
        'Grid/Sensor': '1', # Create occupancy grid from selected sensor: 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s)
        'Grid/RangeMin': '0.3',  # ignore laser hits on the robot body
        'Grid/RangeMax': '12.0',  # match RPLiDAR A2M8 max range
        'Grid/CellSize': '0.05', # Resolution of the occupancy grid.
        
        'Grid/MaxGroundHeight': max_ground_height, # Anything below this limit won't be considered as obstacles
        'Grid/MaxGroundAngle': max_ground_angle, # Maximum angle (degrees) between point's normal to ground's normal to label it as ground. Points with higher angle difference are considered as obstacles.
        
        # 'Grid/MaxObstacleHeight': '5.0', # Maximum obstacles height (0=disabled).     
        # 'Grid/3D': 'true',
        

        ### Registration
        'Reg/Strategy': '0', # registration: 0=Vis, 1=Icp, 2=VisIcp
        
        'Reg/Force3DoF': 'false', #'false',  # 3D mode (due to Lite3 erractic, non-planar movement)

        ### Lidar
        # 'Icp/VoxelSize': '0.05',  # Uniform sampling voxel size (0=disabled). Default: 0.05
        # 'Icp/MaxCorrespondenceDistance': '0.1', # Max distance for point correspondences. Default: 0.1

        # ### RGBD / Loop closure
        # 'RGBD/NeighborLinkRefining': 'True', # When a new node is added to the graph, the transformation of its neighbor link to the previous node is refined using registration approach selected
        # 'RGBD/ProximityBySpace': 'true', # Detection over locations (in Working Memory) near in space.
        # 'RGBD/ProximityMaxGraphDepth': '0', # Maximum depth from the current/last loop closure location and the local loop closure hypotheses. Set 0 to ignore.
        # 'RGBD/AngularUpdate': '0.05',#'0.05', # Minimum angular displacement (rad) to update the map.
        # 'RGBD/LinearUpdate': '0.05',#'0.05', # Minimum linear displacement (m) to update the map.
        # 'RGBD/OptimizeFromGraphEnd': 'false', # Optimize graph from the newest node. If false, the graph is optimized from the oldest node of the current graph

        ### Optimizer
        # 'Optimizer/GravitySigma': '0',  # disable IMU constraints (2D mode)

        ### Memory management
        # 'Rtabmap/DetectionRate': '1.0',
    }

    if localization:
        parameters['Mem/IncrementalMemory'] = 'false'
        parameters['Mem/InitWMWithAllNodes'] = 'true'
    else:
        arguments.append('-d')

    remappings = [
        ('scan', '/scan'),
        ('odom', '/odom'),
        ('grid_map', '/map'),
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_rect_raw'),
    ]

    return [
        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=arguments
        ),

        # (Optional) RTAB-Map Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=arguments,
            condition=IfCondition(
                PythonExpression(["'", use_rtabmap_rviz, "' == 'true' and '", headless, "' != 'true'"])
            ),
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rtabmap_rviz", default_value="false"),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Set to true to run in localization mode (reuse existing map)'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db',
                              description='Path to RTAB-Map database file'),
        DeclareLaunchArgument('max_ground_height', default_value='0.0'),
        DeclareLaunchArgument('max_ground_angle', default_value='45'),
        OpaqueFunction(function=launch_setup)
    ])
