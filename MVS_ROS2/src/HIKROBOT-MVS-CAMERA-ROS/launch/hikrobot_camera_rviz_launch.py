from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('hikrobot_camera')
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'camera.yaml')
    
    # rviz配置文件路径
    rviz_config = os.path.join(pkg_share, 'rviz_config', 'hikrobot.rviz')
    
    # 相机节点
    camera_node = Node(
        package='hikrobot_camera',
        executable='hikrobot_camera',
        name='hikrobot_camera',
        respawn=True,
        output='screen',
        parameters=[config_file]  # 加载配置文件
    )
    
    # rviz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config]
    )
    
    return LaunchDescription([
        camera_node,
        rviz_node
    ])