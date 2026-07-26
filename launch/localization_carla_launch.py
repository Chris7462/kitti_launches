from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('kitti_launches')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (CARLA) clock if true'
    )

    declare_odometry_package = DeclareLaunchArgument(
        'odometry_package',
        default_value='mad_icp',
        description='Package providing lidar odometry. Alternative: floam.'
    )

    declare_odometry_launch_file = DeclareLaunchArgument(
        'odometry_launch_file',
        default_value='mad_icp_launch.py',
        description='Launch file, relative to <odometry_package>/launch. '
                     'Alternative: floam_launch.py.'
    )

    localization_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'localization_rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/carla/hero/lidar/point_cloud',
            'odometry_package': LaunchConfiguration('odometry_package'),
            'odometry_launch_file': LaunchConfiguration('odometry_launch_file'),
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_odometry_package,
        declare_odometry_launch_file,
        localization_rviz_launch,
    ])
