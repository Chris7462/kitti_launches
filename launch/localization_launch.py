from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gps_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gps_imu_node"), "launch", "gps_imu_launch.py"
            ])
        ])
    )

    ekf_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ekf_localizer"), "launch", "ekf_localizer_launch.py"
            ])
        ])
    )

    return LaunchDescription([
        gps_imu_launch,
        ekf_localizer_launch
    ])
