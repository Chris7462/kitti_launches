from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('kitti_launches')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bagfile/CARLA) clock if true'
    )

    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        description='Input lidar point cloud topic name. Required - no '
                     'default. Passed through to localization_launch.py, '
                     'which will refuse to start its node(s) if this is '
                     'not provided.'
    )

    declare_odometry_package = DeclareLaunchArgument(
        'odometry_package',
        default_value='mad_icp',
        description='Package providing lidar odometry. Alternative: floam. '
                     'Passed through to localization_launch.py.'
    )

    declare_odometry_launch_file = DeclareLaunchArgument(
        'odometry_launch_file',
        default_value='mad_icp_launch.py',
        description='Launch file, relative to <odometry_package>/launch, '
                     'that starts lidar odometry. Alternative: '
                     'floam_launch.py. Passed through to '
                     'localization_launch.py.'
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('input_topic'),
            'odometry_package': LaunchConfiguration('odometry_package'),
            'odometry_launch_file': LaunchConfiguration('odometry_launch_file'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(pkg_share, 'rviz', 'localization.rviz')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_input_topic,
        declare_odometry_package,
        declare_odometry_launch_file,
        localization_launch,
        rviz_node
    ])
