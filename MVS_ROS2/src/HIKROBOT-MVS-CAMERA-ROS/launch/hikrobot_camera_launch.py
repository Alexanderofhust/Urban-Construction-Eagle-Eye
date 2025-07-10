from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('hikrobot_camera')
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'camera.yaml')
    
    return LaunchDescription([
        # 声明可配置的参数
        DeclareLaunchArgument(
            'camera_name', 
            default_value='hikrobot_camera',
            description='相机节点名称'
        ),
        
        # 相机节点
        Node(
            package='hikrobot_camera',
            executable='hikrobot_camera',
            name=LaunchConfiguration('camera_name'),
            output='screen',
            respawn=True,
            parameters=[config_file],
            respawn_delay=2,  # 重启延迟(秒)
        )
    ])