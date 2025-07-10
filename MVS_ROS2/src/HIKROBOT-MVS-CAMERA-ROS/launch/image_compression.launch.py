#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的路径
    package_dir = get_package_share_directory('hikrobot_camera')
    
    # 参数文件路径
    config_file = os.path.join(package_dir, 'config', 'image_compression_params.yaml')
    
    return LaunchDescription([
        Node(
            package='hikrobot_camera',
            executable='image_compression_node',
            name='image_compression_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                # 可以在这里重新映射话题名称
                # ('/hikrobot_camera/rgb', '/camera/image_raw'),
            ]
        )
    ])
