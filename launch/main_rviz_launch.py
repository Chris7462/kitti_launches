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

    declare_camera_topic = DeclareLaunchArgument(
        'camera_topic',
        description='Input camera image topic name. Required - no '
                     'default. Passed through to main_launch.py.'
    )

    declare_lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        description='Input lidar point cloud topic name. Required - no '
                     'default. Passed through to main_launch.py.'
    )

    declare_tf_source_package = DeclareLaunchArgument(
        'tf_source_package',
        default_value='kitti_urdf',
        description='Package providing the vehicle/sensor TF and URDF. '
                     'Alternative: carla_urdf. Passed through to '
                     'main_launch.py.'
    )

    declare_tf_source_launch_file = DeclareLaunchArgument(
        'tf_source_launch_file',
        default_value='kitti_urdf_launch.py',
        description='Launch file, relative to <tf_source_package>/launch. '
                     'Alternative: carla_urdf_launch.py. Passed through '
                     'to main_launch.py.'
    )

    declare_odometry_package = DeclareLaunchArgument(
        'odometry_package',
        default_value='mad_icp',
        description='Package providing lidar odometry. Alternative: floam. '
                     'Passed through to main_launch.py.'
    )

    declare_odometry_launch_file = DeclareLaunchArgument(
        'odometry_launch_file',
        default_value='mad_icp_launch.py',
        description='Launch file, relative to <odometry_package>/launch. '
                     'Alternative: floam_launch.py. Passed through to '
                     'main_launch.py.'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        description='Path to the .rviz config file. The PointCloud2/TF '
                     'displays are baked into this file, so it must match '
                     'the data source (KITTI vs CARLA vs ...). Required - '
                     'no default, since a mismatched RViz topic fails '
                     'silently (blank display) rather than erroring.'
    )

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'main_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'tf_source_package': LaunchConfiguration('tf_source_package'),
            'tf_source_launch_file': LaunchConfiguration('tf_source_launch_file'),
            'odometry_package': LaunchConfiguration('odometry_package'),
            'odometry_launch_file': LaunchConfiguration('odometry_launch_file'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_camera_topic,
        declare_lidar_topic,
        declare_tf_source_package,
        declare_tf_source_launch_file,
        declare_odometry_package,
        declare_odometry_launch_file,
        declare_rviz_config,
        main_launch,
        rviz_node,
    ])
