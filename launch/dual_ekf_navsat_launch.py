from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = join(
        get_package_share_directory('kitti_launches'), 'params', 'dual_ekf_navsat.yaml'
    )

    bag_exec = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', '1.0',
             '/data/kitti/raw/2011_09_29_drive_0071_sync_bag' , '--clock']
    )

    # The TF and URDF of the vehicle
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kitti_urdf'), 'launch', 'kitti_urdf_launch.py'
            ])
        ])
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(get_package_share_directory('kitti_launches'), 'rviz', 'dual_ekf.rviz')]
    )

    # Localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kitti_launches'), 'launch', 'localization_launch.py'
            ])
        ])
    )

    trajectory_server_gps_node = Node(
        package='trajectory_server',
        executable='trajectory_server_node',
        name='trajectory_server_node',
        namespace='oxts',
        parameters=[{
            'target_frame_name': 'map',
            'source_frame_name': 'oxts_link',
            'trajectory_update_rate': 10.0,
            'trajectory_publish_rate': 10.0
        }]
    )

    trajectory_server_ekf_node = Node(
        package='trajectory_server',
        executable='trajectory_server_node',
        name='trajectory_server_node',
        namespace='ekf',
        parameters=[{
            'target_frame_name': 'map',
            'source_frame_name': 'ekf_link',
            'trajectory_update_rate': 10.0,
            'trajectory_publish_rate': 10.0
        }]
    )

    wheel_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wheel_odom_node'), 'launch', 'wheel_odom_launch.py'
            ])
        ])
    )

    output_position_arg = DeclareLaunchArgument(
        'output_final_position',
        default_value='false')

    output_location_arg = DeclareLaunchArgument(
        'output_location',
        default_value='~/dual_ekf_navsat_example_debug.txt')

    ekf_filter_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[params],
        remappings=[
            # remap subscriber topic
            ('imu/data', 'kitti/oxts/imu_rotated'),
            ('odometry/wheel', 'kitti/wheel_odom'),
            # remap publisher topic
            ('odometry/filtered', 'odometry/local')]
    )

    ekf_filter_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[params],
        remappings=[
            # remap subscriber topic
            ('imu/data', 'kitti/oxts/imu_rotated'),
            ('odometry/wheel', 'kitti/wheel_odom'),
            # remap publisher topic
            ('odometry/filtered', 'odometry/global')]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[params],
        remappings=[
            # remap subscriber
            ('imu', 'kitti/oxts/imu_rotated'),
            ('gps/fix', 'kitti/gps/fix'),
            ('odometry/filtered', 'odometry/global')]
            # remap publisher
            #('odometry/gps', 'odometry/gps'),
            #('gps/filtered', 'gps/filtered')]
        )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        bag_exec,
        robot_state_publisher_launch,
        rviz_node,
        output_position_arg,
        output_location_arg,
        TimerAction(
            period=1.0, # dely localization for 1.0 seconds
            actions=[
                ekf_filter_odom_node,
                ekf_filter_map_node,
                navsat_transform_node,
                localization_launch,
                trajectory_server_gps_node,
                trajectory_server_ekf_node,
                wheel_odom_launch,
            ]
        )
    ])
