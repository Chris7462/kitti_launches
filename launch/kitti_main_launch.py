from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
    bag_exec = ExecuteProcess(
        cmd=["ros2", "bag", "play", "-r", "1.0", "/data/Kitti/raw/2011_09_29_drive_0071_sync_bag" , "--clock"]
    )

    # The TF and URDF of the vehicle
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("kitti_urdf"), "launch", "kitti_urdf_launch.py"
            ])
        ])
    )

    # Localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("kitti_launches"), "launch", "localization_launch.py"
            ])
        ])
    )

    # Perception launch


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", join(get_package_share_directory("kitti_launches"), "rviz", "kitti.rviz")]
    )

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        bag_exec,
        robot_state_publisher_launch,
        localization_launch,
        rviz_node
    ])