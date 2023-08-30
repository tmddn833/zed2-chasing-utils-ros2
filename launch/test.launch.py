import os

from launch import LaunchDescription
from launch_ros.actions import Node

current_directory = os.path.dirname(os.path.abspath(__file__))
parameters = [os.path.join(current_directory, os.pardir, 'param', 'abc.yaml')]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zed2_chasing_utils',
            namespace='zed2_chasing_utils',
            executable='zed2_chasing_server_node',
            name='zed2_chasing_server_node',
            output='screen',
            parameters=parameters,
            remappings=[("~/depth_compressed_image", "/zed2/zed_node/depth/depth_registered/compressedDepth"),
                        ("~/camera_info", "/zed2/zed_node/rgb/camera_info"),
                        ("~/objects", "/zed2/zed_node/obj_det/objects")]
        ),
    ])
