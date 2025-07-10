from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'model_path',
            default_value='yolov8s-seg.onnx',
            description='YOLOv8分割模型文件路径'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.5',
            description='推理置信度阈值'
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/hikrobot_camera/rgb',
            description='输入图像话题'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/inference/results',
            description='推理结果输出话题'
        ),
        
        # 启动图像推理节点
        Node(
            package='hikrobot_camera',
            executable='image_inference_node',
            name='image_inference_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
            }],
            remappings=[
                # 如果需要重映射话题名称，可以在这里添加
            ]
        )
    ])
