import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_path = str(os.path.join(get_package_share_directory("zed2_chasing_utils"), 'rviz_config', 'utils.rviz'))
    utils_param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('zed2_chasing_utils'),
            'param',
            'abc.yaml')
    )
    DeclareLaunchArgument(
        'param_dir',
        default_value=utils_param_dir,
        description='YAML FILE',
    )
    return LaunchDescription([
        Node(
            package='zed2_chasing_utils',
            executable='zed2_chasing_server_node',
            name='zed2_chasing_server_node',
            output='screen',
            parameters=[utils_param_dir],
            emulate_tty=True,
            remappings=[("~/depth_compressed_image", "/zed2/zed_node/depth/depth_registered/compressedDepth"),
                        ("~/camera_info", "/zed2/zed_node/rgb/camera_info"),
                        ("~/objects", "/zed2/zed_node/obj_det/objects")]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_path]
        )
    ])
