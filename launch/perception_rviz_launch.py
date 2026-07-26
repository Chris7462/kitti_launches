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
        description='Input image topic name. Required - no default. '
                     'Passed through to perception_launch.py, which will '
                     'refuse to start its nodes if this is not provided.'
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'perception_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('input_topic'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(pkg_share, 'rviz', 'perception.rviz')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_input_topic,
        perception_launch,
        rviz_node
    ])
