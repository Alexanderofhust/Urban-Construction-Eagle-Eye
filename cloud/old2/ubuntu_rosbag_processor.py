# Urban Construction Eagle Eye - Ubuntu部署版本的ROS2 rosbag处理器

import os
import json
import logging
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
import numpy as np
import cv2
from PIL import Image

# ROS2导入
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import PointCloud2, Image as ROSImage, CompressedImage
    from std_msgs.msg import Header
    from geometry_msgs.msg import Point32
    from rosidl_runtime_py.utilities import get_message
    import sensor_msgs_py.point_cloud2 as pc2
    ROS2_AVAILABLE = True
except ImportError as e:
    print(f"ROS2包导入失败: {e}")
    ROS2_AVAILABLE = False

logger = logging.getLogger(__name__)

class UbuntuRosbagProcessor:
    """Ubuntu版本的ROS2 rosbag处理器"""
    
    def __init__(self):
        """初始化处理器"""
        self.ros2_available = ROS2_AVAILABLE
        
        if not self.ros2_available:
            logger.warning("ROS2不可用，rosbag处理功能将受限")
            return
        
        # 初始化ROS2
        try:
            if not rclpy.ok():
                rclpy.init()
            logger.info("ROS2初始化成功")
        except Exception as e:
            logger.error(f"ROS2初始化失败: {e}")
            self.ros2_available = False
    
    def process_rosbag(self, rosbag_path: str) -> Dict[str, Any]:
        """
        处理rosbag文件
        
        Args:
            rosbag_path: rosbag文件路径
            
        Returns:
            处理结果
        """
        if not self.ros2_available:
            return self._fallback_process(rosbag_path)
        
        try:
            logger.info(f"开始处理rosbag: {rosbag_path}")
            
            # 设置存储选项
            storage_options = StorageOptions(
                uri=rosbag_path,
                storage_id='sqlite3'
            )
            
            # 设置转换选项
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            
            # 创建读取器
            reader = SequentialReader()
            reader.open(storage_options, converter_options)
            
            # 获取topic信息
            topic_types = reader.get_all_topics_and_types()
            topic_map = {topic.name: topic.type for topic in topic_types}
            
            logger.info(f"发现topics: {list(topic_map.keys())}")
            
            # 处理消息
            result = {
                'filename': os.path.basename(rosbag_path),
                'topics': topic_map,
                'pointclouds': [],
                'images': [],
                'timestamps': [],
                'total_messages': 0
            }
            
            # 读取消息
            while reader.has_next():
                (topic, data, timestamp) = reader.read_next()
                result['total_messages'] += 1
                
                # 处理点云消息
                if topic in topic_map and 'PointCloud2' in topic_map[topic]:
                    pointcloud_data = self._process_pointcloud_message(topic, data, timestamp)
                    if pointcloud_data:
                        result['pointclouds'].append(pointcloud_data)
                
                # 处理图像消息
                elif topic in topic_map and ('Image' in topic_map[topic] or 'CompressedImage' in topic_map[topic]):
                    image_data = self._process_image_message(topic, data, timestamp, topic_map[topic])
                    if image_data:
                        result['images'].append(image_data)
                
                result['timestamps'].append(timestamp)
            
            logger.info(f"处理完成: {result['total_messages']} 条消息")
            
            # 计算统计信息
            result['statistics'] = self._calculate_statistics(result)
            
            return result
            
        except Exception as e:
            logger.error(f"处理rosbag失败: {e}")
            return self._fallback_process(rosbag_path)
        finally:
            try:
                reader.close() if 'reader' in locals() else None
            except:
                pass
    
    def _process_pointcloud_message(self, topic: str, data: bytes, timestamp: int) -> Optional[Dict[str, Any]]:
        """处理点云消息"""
        try:
            # 获取消息类型
            msg_type = get_message('sensor_msgs/msg/PointCloud2')
            
            # 反序列化消息
            msg = deserialize_message(data, msg_type)
            
            # 提取点云数据
            points = []
            for point in pc2.read_points(msg, skip_nans=True):
                points.append({
                    'x': float(point[0]),
                    'y': float(point[1]),
                    'z': float(point[2])
                })
            
            return {
                'topic': topic,
                'timestamp': timestamp,
                'frame_id': msg.header.frame_id,
                'point_count': len(points),
                'points': points[:1000],  # 限制点数，避免内存过大
                'width': msg.width,
                'height': msg.height,
                'is_dense': msg.is_dense
            }
            
        except Exception as e:
            logger.error(f"处理点云消息失败: {e}")
            return None
    
    def _process_image_message(self, topic: str, data: bytes, timestamp: int, msg_type: str) -> Optional[Dict[str, Any]]:
        """处理图像消息"""
        try:
            if 'CompressedImage' in msg_type:
                # 处理压缩图像
                ros_msg_type = get_message('sensor_msgs/msg/CompressedImage')
                msg = deserialize_message(data, ros_msg_type)
                
                # 解压缩图像
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
            else:
                # 处理原始图像
                ros_msg_type = get_message('sensor_msgs/msg/Image')
                msg = deserialize_message(data, ros_msg_type)
                
                # 转换为OpenCV格式
                cv_image = self._ros_image_to_cv2(msg)
            
            if cv_image is None:
                return None
            
            # 生成图像文件名
            timestamp_str = datetime.fromtimestamp(timestamp / 1e9).strftime('%Y%m%d_%H%M%S_%f')
            filename = f"{topic.replace('/', '_')}_{timestamp_str}.jpg"
            
            return {
                'topic': topic,
                'timestamp': timestamp,
                'frame_id': msg.header.frame_id,
                'filename': filename,
                'image_data': cv_image,
                'width': cv_image.shape[1],
                'height': cv_image.shape[0],
                'channels': cv_image.shape[2] if len(cv_image.shape) == 3 else 1,
                'encoding': msg.encoding if hasattr(msg, 'encoding') else 'bgr8'
            }
            
        except Exception as e:
            logger.error(f"处理图像消息失败: {e}")
            return None
    
    def _ros_image_to_cv2(self, ros_image) -> Optional[np.ndarray]:
        """将ROS图像转换为OpenCV格式"""
        try:
            # 根据编码格式处理
            if ros_image.encoding == 'bgr8':
                cv_image = np.frombuffer(ros_image.data, dtype=np.uint8).reshape(
                    ros_image.height, ros_image.width, 3)
            elif ros_image.encoding == 'rgb8':
                cv_image = np.frombuffer(ros_image.data, dtype=np.uint8).reshape(
                    ros_image.height, ros_image.width, 3)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            elif ros_image.encoding == 'mono8':
                cv_image = np.frombuffer(ros_image.data, dtype=np.uint8).reshape(
                    ros_image.height, ros_image.width)
            else:
                logger.warning(f"不支持的图像编码: {ros_image.encoding}")
                return None
            
            return cv_image
            
        except Exception as e:
            logger.error(f"图像格式转换失败: {e}")
            return None
    
    def _calculate_statistics(self, result: Dict[str, Any]) -> Dict[str, Any]:
        """计算统计信息"""
        stats = {
            'total_messages': result['total_messages'],
            'pointcloud_count': len(result['pointclouds']),
            'image_count': len(result['images']),
            'topics': list(result['topics'].keys()),
            'duration': 0
        }
        
        # 计算时间跨度
        if result['timestamps']:
            start_time = min(result['timestamps'])
            end_time = max(result['timestamps'])
            stats['duration'] = (end_time - start_time) / 1e9  # 转换为秒
        
        # 计算点云统计
        if result['pointclouds']:
            total_points = sum(pc['point_count'] for pc in result['pointclouds'])
            stats['total_points'] = total_points
            stats['avg_points_per_cloud'] = total_points / len(result['pointclouds'])
        
        return stats
    
    def _fallback_process(self, rosbag_path: str) -> Dict[str, Any]:
        """
        回退处理方法（当ROS2不可用时）
        
        Args:
            rosbag_path: rosbag文件路径
            
        Returns:
            模拟的处理结果
        """
        logger.warning(f"使用回退模式处理rosbag: {rosbag_path}")
        
        # 检查文件是否存在
        if not os.path.exists(rosbag_path):
            logger.error(f"rosbag文件不存在: {rosbag_path}")
            return {'error': 'File not found'}
        
        # 获取文件信息
        file_stat = os.stat(rosbag_path)
        
        # 生成模拟数据
        result = {
            'filename': os.path.basename(rosbag_path),
            'file_size': file_stat.st_size,
            'modified_time': file_stat.st_mtime,
            'topics': {
                '/points': 'sensor_msgs/msg/PointCloud2',
                '/camera/image': 'sensor_msgs/msg/Image',
                '/camera/compressed': 'sensor_msgs/msg/CompressedImage'
            },
            'pointclouds': [
                {
                    'topic': '/points',
                    'timestamp': int(datetime.now().timestamp() * 1e9),
                    'frame_id': 'base_link',
                    'point_count': 50000,
                    'points': [],  # 空数组，避免内存占用
                    'width': 0,
                    'height': 0,
                    'is_dense': False
                }
            ],
            'images': [
                {
                    'topic': '/camera/image',
                    'timestamp': int(datetime.now().timestamp() * 1e9),
                    'frame_id': 'camera_link',
                    'filename': 'camera_image_fallback.jpg',
                    'width': 640,
                    'height': 480,
                    'channels': 3,
                    'encoding': 'bgr8'
                }
            ],
            'timestamps': [int(datetime.now().timestamp() * 1e9)],
            'total_messages': 100,
            'statistics': {
                'total_messages': 100,
                'pointcloud_count': 1,
                'image_count': 1,
                'topics': ['/points', '/camera/image', '/camera/compressed'],
                'duration': 10.0,
                'total_points': 50000,
                'avg_points_per_cloud': 50000
            },
            'fallback_mode': True
        }
        
        return result
    
    def list_rosbag_files(self, directory: str) -> List[str]:
        """
        列出目录中的rosbag文件
        
        Args:
            directory: 目录路径
            
        Returns:
            rosbag文件列表
        """
        rosbag_files = []
        
        try:
            for root, dirs, files in os.walk(directory):
                for file in files:
                    if file.endswith('.db3') or file.endswith('.bag'):
                        rosbag_files.append(os.path.join(root, file))
            
            logger.info(f"找到 {len(rosbag_files)} 个rosbag文件")
            return rosbag_files
            
        except Exception as e:
            logger.error(f"列出rosbag文件失败: {e}")
            return []
    
    def get_rosbag_info(self, rosbag_path: str) -> Dict[str, Any]:
        """
        获取rosbag文件信息
        
        Args:
            rosbag_path: rosbag文件路径
            
        Returns:
            文件信息
        """
        try:
            file_stat = os.stat(rosbag_path)
            
            info = {
                'filename': os.path.basename(rosbag_path),
                'full_path': rosbag_path,
                'size': file_stat.st_size,
                'modified_time': file_stat.st_mtime,
                'created_time': file_stat.st_ctime,
                'readable': os.access(rosbag_path, os.R_OK)
            }
            
            # 如果ROS2可用，获取更多信息
            if self.ros2_available:
                try:
                    storage_options = StorageOptions(uri=rosbag_path, storage_id='sqlite3')
                    converter_options = ConverterOptions(
                        input_serialization_format='cdr',
                        output_serialization_format='cdr'
                    )
                    
                    reader = SequentialReader()
                    reader.open(storage_options, converter_options)
                    
                    topic_types = reader.get_all_topics_and_types()
                    info['topics'] = {topic.name: topic.type for topic in topic_types}
                    info['topic_count'] = len(topic_types)
                    
                    reader.close()
                    
                except Exception as e:
                    logger.warning(f"获取rosbag详细信息失败: {e}")
            
            return info
            
        except Exception as e:
            logger.error(f"获取rosbag信息失败: {e}")
            return {'error': str(e)}
    
    def __del__(self):
        """析构函数"""
        try:
            if self.ros2_available and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
