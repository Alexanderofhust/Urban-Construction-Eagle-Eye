"""
ROS2 Bag数据处理器
功能：
1. 解析rosbag文件
2. 提取点云、图像、mask数据
3. 数据预处理和格式转换
4. 生成复检图像
"""

import rosbag2_py
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import cv2
import json
import os
from pathlib import Path
import logging
from typing import List, Dict, Any, Optional
import open3d as o3d
from sensor_msgs.msg import PointCloud2, Image, CompressedImage
from std_msgs.msg import Header
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

logger = logging.getLogger(__name__)

class RosbagProcessor:
    """ROS2 Bag数据处理器"""
    
    def __init__(self):
        self.bridge = CvBridge()
        self.pointcloud_data = []
        self.image_data = []
        self.mask_data = []
        
        # 话题名称配置
        self.topics = {
            'pointcloud': '/cloud_registered',
            'image': '/hikrobot_camera/rgb',
            'mask': '/unet_result',
            'imu': '/imu_data'
        }
    
    async def process_rosbag(self, rosbag_path: str, session_id: str) -> Dict[str, Any]:
        """处理rosbag文件"""
        try:
            logger.info(f"开始处理rosbag: {rosbag_path}")
            
            # 打开rosbag
            storage_options = rosbag2_py.StorageOptions(
                uri=rosbag_path,
                storage_id='sqlite3'
            )
            
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            
            reader = rosbag2_py.SequentialReader()
            reader.open(storage_options, converter_options)
            
            # 获取话题信息
            topic_types = reader.get_all_topics_and_types()
            topic_type_map = {topic.name: topic.type for topic in topic_types}
            
            # 处理消息
            processed_data = {
                'pointclouds': [],
                'images': [],
                'masks': [],
                'sync_data': []
            }
            
            while reader.has_next():
                (topic, data, timestamp) = reader.read_next()
                
                if topic in topic_type_map:
                    msg_type = get_message(topic_type_map[topic])
                    msg = deserialize_message(data, msg_type)
                    
                    # 处理不同类型的消息
                    if topic == self.topics['pointcloud']:
                        await self._process_pointcloud_message(msg, timestamp, processed_data)
                    elif topic == self.topics['image']:
                        await self._process_image_message(msg, timestamp, processed_data)
                    elif topic == self.topics['mask']:
                        await self._process_mask_message(msg, timestamp, processed_data)
            
            # 同步数据
            await self._synchronize_data(processed_data, session_id)
            
            # 保存处理结果
            await self._save_processed_data(processed_data, session_id)
            
            logger.info(f"rosbag处理完成: {rosbag_path}")
            return processed_data
            
        except Exception as e:
            logger.error(f"处理rosbag失败: {str(e)}")
            raise
    
    async def _process_pointcloud_message(self, msg: PointCloud2, timestamp: int, processed_data: Dict):
        """处理点云消息"""
        try:
            # 转换点云数据
            points = list(pc2.read_points(msg, field_names=['x', 'y', 'z', 'rgb'], skip_nans=True))
            
            if points:
                pointcloud_data = {
                    'timestamp': timestamp,
                    'points': points,
                    'header': {
                        'frame_id': msg.header.frame_id,
                        'stamp': msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
                    }
                }
                
                processed_data['pointclouds'].append(pointcloud_data)
                
        except Exception as e:
            logger.error(f"处理点云消息失败: {str(e)}")
    
    async def _process_image_message(self, msg: Image, timestamp: int, processed_data: Dict):
        """处理图像消息"""
        try:
            # 转换图像数据
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            image_data = {
                'timestamp': timestamp,
                'image': cv_image,
                'header': {
                    'frame_id': msg.header.frame_id,
                    'stamp': msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
                },
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding
            }
            
            processed_data['images'].append(image_data)
            
        except Exception as e:
            logger.error(f"处理图像消息失败: {str(e)}")
    
    async def _process_mask_message(self, msg: Image, timestamp: int, processed_data: Dict):
        """处理mask消息"""
        try:
            # 转换mask数据
            cv_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            mask_data = {
                'timestamp': timestamp,
                'mask': cv_mask,
                'header': {
                    'frame_id': msg.header.frame_id,
                    'stamp': msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
                },
                'width': msg.width,
                'height': msg.height
            }
            
            processed_data['masks'].append(mask_data)
            
        except Exception as e:
            logger.error(f"处理mask消息失败: {str(e)}")
    
    async def _synchronize_data(self, processed_data: Dict, session_id: str):
        """同步图像和mask数据"""
        try:
            sync_data = []
            
            # 按时间戳匹配图像和mask
            for image_data in processed_data['images']:
                image_timestamp = image_data['timestamp']
                
                # 找到最接近的mask
                closest_mask = None
                min_time_diff = float('inf')
                
                for mask_data in processed_data['masks']:
                    time_diff = abs(mask_data['timestamp'] - image_timestamp)
                    if time_diff < min_time_diff:
                        min_time_diff = time_diff
                        closest_mask = mask_data
                
                # 如果时间差小于阈值，认为是同步的
                if closest_mask and min_time_diff < 1e8:  # 100ms阈值
                    sync_data.append({
                        'timestamp': image_timestamp,
                        'image': image_data,
                        'mask': closest_mask,
                        'defect_area': self._calculate_defect_area(closest_mask['mask']),
                        'severity': self._calculate_severity(closest_mask['mask'])
                    })
            
            processed_data['sync_data'] = sync_data
            
        except Exception as e:
            logger.error(f"数据同步失败: {str(e)}")
    
    def _calculate_defect_area(self, mask: np.ndarray) -> Dict[str, float]:
        """计算缺陷面积"""
        try:
            # 转换为HSV颜色空间
            hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)
            
            # 定义颜色范围
            color_ranges = {
                'red': ([0, 100, 100], [10, 255, 255]),      # 红色缺陷1
                'blue': ([100, 100, 100], [130, 255, 255]),  # 蓝色缺陷2
                'green': ([40, 100, 100], [80, 255, 255])    # 绿色缺陷3
            }
            
            defect_areas = {}
            total_pixels = mask.shape[0] * mask.shape[1]
            
            for color, (lower, upper) in color_ranges.items():
                lower = np.array(lower)
                upper = np.array(upper)
                
                # 创建颜色掩码
                color_mask = cv2.inRange(hsv, lower, upper)
                defect_pixels = cv2.countNonZero(color_mask)
                
                defect_areas[color] = {
                    'pixels': defect_pixels,
                    'percentage': (defect_pixels / total_pixels) * 100
                }
            
            return defect_areas
            
        except Exception as e:
            logger.error(f"计算缺陷面积失败: {str(e)}")
            return {}
    
    def _calculate_severity(self, mask: np.ndarray) -> str:
        """计算严重程度"""
        try:
            defect_areas = self._calculate_defect_area(mask)
            
            # 计算总缺陷面积
            total_defect_percentage = sum(
                area['percentage'] for area in defect_areas.values()
            )
            
            # 根据面积百分比判断严重程度
            if total_defect_percentage > 10:
                return "severe"
            elif total_defect_percentage > 5:
                return "moderate"
            elif total_defect_percentage > 1:
                return "mild"
            else:
                return "minor"
                
        except Exception as e:
            logger.error(f"计算严重程度失败: {str(e)}")
            return "unknown"
    
    async def _save_processed_data(self, processed_data: Dict, session_id: str):
        """保存处理后的数据"""
        try:
            # 创建会话目录
            session_dir = Path(f"data/rosbags/{session_id}")
            session_dir.mkdir(parents=True, exist_ok=True)
            
            # 保存点云数据
            if processed_data['pointclouds']:
                await self._save_pointcloud_data(processed_data['pointclouds'], session_id)
            
            # 保存图像数据
            if processed_data['images']:
                await self._save_image_data(processed_data['images'], session_id)
            
            # 保存同步数据
            if processed_data['sync_data']:
                await self._save_sync_data(processed_data['sync_data'], session_id)
            
        except Exception as e:
            logger.error(f"保存处理数据失败: {str(e)}")
    
    async def _save_pointcloud_data(self, pointclouds: List[Dict], session_id: str):
        """保存点云数据"""
        try:
            # 合并所有点云
            all_points = []
            for pc_data in pointclouds:
                all_points.extend(pc_data['points'])
            
            if all_points:
                # 转换为Open3D点云格式
                points = np.array([[p[0], p[1], p[2]] for p in all_points])
                colors = np.array([[((p[3] >> 16) & 0xFF) / 255.0, 
                                  ((p[3] >> 8) & 0xFF) / 255.0, 
                                  (p[3] & 0xFF) / 255.0] for p in all_points])
                
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                pcd.colors = o3d.utility.Vector3dVector(colors)
                
                # 保存PCD文件
                pcd_file = f"data/pointclouds/{session_id}_pointcloud.pcd"
                o3d.io.write_point_cloud(pcd_file, pcd)
                
                # 保存JSON格式用于Web显示
                pointcloud_json = {
                    'points': points.tolist(),
                    'colors': colors.tolist(),
                    'count': len(points)
                }
                
                json_file = f"data/pointclouds/{session_id}_pointcloud.json"
                with open(json_file, 'w') as f:
                    json.dump(pointcloud_json, f)
                
                logger.info(f"点云数据已保存: {pcd_file}")
                
        except Exception as e:
            logger.error(f"保存点云数据失败: {str(e)}")
    
    async def _save_image_data(self, images: List[Dict], session_id: str):
        """保存图像数据"""
        try:
            image_dir = Path(f"data/images/{session_id}")
            image_dir.mkdir(parents=True, exist_ok=True)
            
            for i, image_data in enumerate(images):
                image_file = image_dir / f"image_{i:06d}.jpg"
                cv2.imwrite(str(image_file), image_data['image'])
            
            logger.info(f"图像数据已保存: {image_dir}")
            
        except Exception as e:
            logger.error(f"保存图像数据失败: {str(e)}")
    
    async def _save_sync_data(self, sync_data: List[Dict], session_id: str):
        """保存同步数据"""
        try:
            sync_file = f"data/results/{session_id}_sync_data.json"
            
            # 转换数据格式用于保存
            save_data = []
            for data in sync_data:
                save_data.append({
                    'timestamp': data['timestamp'],
                    'defect_area': data['defect_area'],
                    'severity': data['severity']
                })
            
            with open(sync_file, 'w', encoding='utf-8') as f:
                json.dump(save_data, f, ensure_ascii=False, indent=2)
            
            logger.info(f"同步数据已保存: {sync_file}")
            
        except Exception as e:
            logger.error(f"保存同步数据失败: {str(e)}")
    
    async def generate_review_images(self, session_id: str, top_n: int = 10) -> List[Dict]:
        """生成复检图像"""
        try:
            # 读取同步数据
            sync_file = f"data/results/{session_id}_sync_data.json"
            with open(sync_file, 'r', encoding='utf-8') as f:
                sync_data = json.load(f)
            
            # 按严重程度排序
            severity_order = {"severe": 4, "moderate": 3, "mild": 2, "minor": 1}
            sorted_data = sorted(sync_data, 
                               key=lambda x: severity_order.get(x['severity'], 0), 
                               reverse=True)
            
            # 选择前top_n个最严重的
            top_data = sorted_data[:top_n]
            
            review_images = []
            image_dir = Path(f"data/images/{session_id}")
            review_dir = Path(f"data/images/{session_id}/review")
            review_dir.mkdir(parents=True, exist_ok=True)
            
            for i, data in enumerate(top_data):
                # 查找对应的图像文件
                image_files = list(image_dir.glob("image_*.jpg"))
                if i < len(image_files):
                    original_image = cv2.imread(str(image_files[i]))
                    
                    # 创建复检图像信息
                    review_info = {
                        'id': f"review_{i:03d}",
                        'original_image': f"/static/images/{session_id}/image_{i:06d}.jpg",
                        'overlay_image': f"/static/images/{session_id}/review/overlay_{i:03d}.jpg",
                        'severity': data['severity'],
                        'defect_area': data['defect_area'],
                        'timestamp': data['timestamp']
                    }
                    
                    # 生成叠加图像（原图 + mask叠加）
                    # 这里需要根据实际mask数据生成叠加图像
                    overlay_image = original_image.copy()  # 临时处理
                    
                    overlay_file = review_dir / f"overlay_{i:03d}.jpg"
                    cv2.imwrite(str(overlay_file), overlay_image)
                    
                    review_images.append(review_info)
            
            return review_images
            
        except Exception as e:
            logger.error(f"生成复检图像失败: {str(e)}")
            return []
