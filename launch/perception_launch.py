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

    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        description='Input image topic name. Required - no default, since '
                     'it differs per data source '
                     '(e.g. /kitti/camera/color/left/image_raw or '
                     '/carla/hero/cam2/image). Forwarded to all perception '
                     'sub-launches, which will refuse to start their nodes '
                     'if this is not provided.'
    )

    fcn_segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('fcn_segmentation'),
                'launch', 'fcn_segmentation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('input_topic')
        }.items()
    )

    fcos_object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('fcos_object_detection'),
                 'launch', 'fcos_object_detection_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('input_topic')
        }.items()
    )

    # sort_tracker's detection_input_topic must match fcos_object_detection's
    # output topic above - they're paired by convention here, not by any
    # enforced link between the two includes. If this stack ever switches
    # detectors, this string needs to change to match.
    sort_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('sort_tracker'),
                 'launch', 'sort_tracker_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('input_topic'),
            'detection_input_topic': 'fcos_object_detection/detection_array'
        }.items()
    )

    scnn_lane_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('scnn_lane_detection'),
                 'launch', 'scnn_lane_detection_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('input_topic')
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_input_topic,
        fcn_segmentation_launch,
        fcos_object_detection_launch,
        sort_tracker_launch,
        scnn_lane_detection_launch
    ])
