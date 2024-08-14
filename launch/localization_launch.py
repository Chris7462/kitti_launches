from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    local_gps_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('local_gps_imu'), 'launch', 'local_gps_imu_launch.py'
            ])
        ])
    )

    #   ekf_localizer_launch = IncludeLaunchDescription(
    #       PythonLaunchDescriptionSource([
    #           PathJoinSubstitution([
    #               FindPackageShare('ekf_localizer'), 'launch', 'ekf_localizer_launch.py'
    #           ])
    #       ])
    #   )

    iekf_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iekf_localizer'), 'launch', 'iekf_localizer_launch.py'
            ])
        ])
    )

    return LaunchDescription([
        local_gps_imu_launch,
        # ekf_localizer_launch
        iekf_localizer_launch
    ])
