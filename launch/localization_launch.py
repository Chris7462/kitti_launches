from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bagfile) clock if true'
    )

    local_gps_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('local_gps_imu'),
                 'launch', 'local_gps_imu_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    #   ekf_localizer_launch = IncludeLaunchDescription(
    #       PythonLaunchDescriptionSource(
    #           join(get_package_share_directory()'ekf_localizer'),
    #                'launch', 'ekf_localizer_launch.py')
    #       ),
    #       launch_arguments={
    #           'use_sim_time': LaunchConfiguration('use_sim_time')
    #       }.items()
    #   )

    iekf_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('iekf_localizer'),
                 'launch', 'iekf_localizer_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        local_gps_imu_launch,
        # ekf_localizer_launch
        iekf_localizer_launch
    ])
