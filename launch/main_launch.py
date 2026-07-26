from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
                     'default, since it differs per data source '
                     '(e.g. /kitti/camera/color/left/image_raw or '
                     '/carla/hero/cam2/image). Forwarded to '
                     'perception_launch.py.'
    )

    declare_lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        description='Input lidar point cloud topic name. Required - no '
                     'default, since it differs per data source '
                     '(e.g. /kitti/velo or /carla/hero/lidar/point_cloud). '
                     'Forwarded to localization_launch.py.'
    )

    declare_tf_source_package = DeclareLaunchArgument(
        'tf_source_package',
        default_value='kitti_urdf',
        description='Package providing the vehicle/sensor TF and URDF. '
                     'Alternative: carla_urdf. Must accept use_sim_time '
                     'as a launch argument.'
    )

    declare_tf_source_launch_file = DeclareLaunchArgument(
        'tf_source_launch_file',
        default_value='kitti_urdf_launch.py',
        description='Launch file, relative to <tf_source_package>/launch, '
                     'that publishes the vehicle/sensor TF. Alternative: '
                     'carla_urdf_launch.py.'
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
        description='Launch file, relative to <odometry_package>/launch. '
                     'Alternative: floam_launch.py. Passed through to '
                     'localization_launch.py.'
    )

    tf_source_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('tf_source_package')),
                'launch',
                LaunchConfiguration('tf_source_launch_file')
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'perception_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('camera_topic'),
        }.items()
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('lidar_topic'),
            'odometry_package': LaunchConfiguration('odometry_package'),
            'odometry_launch_file': LaunchConfiguration('odometry_launch_file'),
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_camera_topic,
        declare_lidar_topic,
        declare_tf_source_package,
        declare_tf_source_launch_file,
        declare_odometry_package,
        declare_odometry_launch_file,
        tf_source_launch,
        perception_launch,
        localization_launch,
    ])
