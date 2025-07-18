# Urban Construction Eagle Eye - 基于文件系统的数据处理器

import os
import json
import logging
import shutil
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
import numpy as np
import cv2
from PIL import Image
import glob
import time
import hashlib

logger = logging.getLogger(__name__)

class FileSystemDataProcessor:
    """基于文件系统的数据处理器
    
    数据流程：
    1. ROS节点自动保存原图和mask到指定文件夹
    2. 建图完成后保存最终点云PCD数据
    3. Python程序监控文件变化并处理
    """
    
    def __init__(self, config: Dict[str, Any]):
        """初始化数据处理器
        
        Args:
            config: 配置信息
        """
        self.config = config
        
        # 文件监控路径
        self.watch_dirs = {
            'images': Path(config.get('image_dir', './data/images')),
            'masks': Path(config.get('mask_dir', './data/masks')),
            'pointclouds': Path(config.get('pointcloud_dir', './data/pointclouds')),
            'metadata': Path(config.get('metadata_dir', './data/metadata'))
        }
        
        # 创建监控目录
        for dir_path in self.watch_dirs.values():
            dir_path.mkdir(parents=True, exist_ok=True)
            
        # 处理状态
        self.processed_files = set()
        self.processing_sessions = {}
        
        # 文件监控间隔
        self.monitor_interval = config.get('monitor_interval', 2.0)
        
        logger.info(f"文件系统数据处理器初始化完成")
        logger.info(f"监控目录: {dict(self.watch_dirs)}")
    
    def start_monitoring(self, session_id: str) -> bool:
        """开始监控指定会话的数据
        
        Args:
            session_id: 会话ID
            
        Returns:
            bool: 是否成功开始监控
        """
        try:
            if session_id in self.processing_sessions:
                logger.warning(f"会话 {session_id} 已在监控中")
                return True
            
            # 创建会话目录
            session_dirs = {}
            for name, base_path in self.watch_dirs.items():
                session_dir = base_path / session_id
                session_dir.mkdir(parents=True, exist_ok=True)
                session_dirs[name] = session_dir
            
            # 记录会话信息
            self.processing_sessions[session_id] = {
                'start_time': datetime.now(),
                'session_dirs': session_dirs,
                'processed_files': set(),
                'last_update': datetime.now(),
                'status': 'monitoring'
            }
            
            logger.info(f"开始监控会话 {session_id}")
            return True
            
        except Exception as e:
            logger.error(f"启动监控失败: {str(e)}")
            return False
    
    def stop_monitoring(self, session_id: str) -> bool:
        """停止监控指定会话
        
        Args:
            session_id: 会话ID
            
        Returns:
            bool: 是否成功停止监控
        """
        try:
            if session_id in self.processing_sessions:
                self.processing_sessions[session_id]['status'] = 'stopped'
                logger.info(f"停止监控会话 {session_id}")
                return True
            else:
                logger.warning(f"会话 {session_id} 不在监控中")
                return False
                
        except Exception as e:
            logger.error(f"停止监控失败: {str(e)}")
            return False
    
    def scan_new_files(self, session_id: str) -> Dict[str, List[str]]:
        """扫描新的文件
        
        Args:
            session_id: 会话ID
            
        Returns:
            Dict[str, List[str]]: 新文件列表
        """
        if session_id not in self.processing_sessions:
            return {}
        
        session = self.processing_sessions[session_id]
        new_files = {
            'images': [],
            'masks': [],
            'pointclouds': [],
            'metadata': []
        }
        
        try:
            for data_type, session_dir in session['session_dirs'].items():
                if not session_dir.exists():
                    continue
                
                # 扫描文件
                if data_type == 'images':
                    patterns = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
                elif data_type == 'masks':
                    patterns = ['*.png', '*.jpg', '*.mask']
                elif data_type == 'pointclouds':
                    patterns = ['*.pcd', '*.ply', '*.pts']
                elif data_type == 'metadata':
                    patterns = ['*.json', '*.txt', '*.yaml']
                else:
                    continue
                
                for pattern in patterns:
                    for file_path in session_dir.glob(pattern):
                        if file_path.is_file():
                            file_key = str(file_path.relative_to(session_dir))
                            if file_key not in session['processed_files']:
                                new_files[data_type].append(str(file_path))
                                session['processed_files'].add(file_key)
            
            # 更新最后扫描时间
            session['last_update'] = datetime.now()
            
        except Exception as e:
            logger.error(f"扫描新文件失败: {str(e)}")
        
        return new_files
    
    def process_image_pair(self, image_path: str, mask_path: str = None) -> Dict[str, Any]:
        """处理图像和mask对
        
        Args:
            image_path: 图像路径
            mask_path: mask路径（可选）
            
        Returns:
            Dict[str, Any]: 处理结果
        """
        try:
            # 读取图像
            image = cv2.imread(image_path)
            if image is None:
                raise ValueError(f"无法读取图像: {image_path}")
            
            # 基本信息
            height, width = image.shape[:2]
            file_size = os.path.getsize(image_path)
            
            result = {
                'image_path': image_path,
                'width': width,
                'height': height,
                'file_size': file_size,
                'timestamp': datetime.now().isoformat(),
                'processed': True
            }
            
            # 处理mask（如果存在）
            if mask_path and os.path.exists(mask_path):
                mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
                if mask is not None:
                    # 分析mask
                    mask_area = np.sum(mask > 0)
                    total_area = mask.shape[0] * mask.shape[1]
                    coverage_ratio = mask_area / total_area if total_area > 0 else 0
                    
                    result.update({
                        'mask_path': mask_path,
                        'mask_area': int(mask_area),
                        'coverage_ratio': float(coverage_ratio),
                        'has_defects': coverage_ratio > 0.01  # 阈值可配置
                    })
            
            return result
            
        except Exception as e:
            logger.error(f"处理图像对失败: {str(e)}")
            return {
                'image_path': image_path,
                'error': str(e),
                'processed': False,
                'timestamp': datetime.now().isoformat()
            }
    
    def process_pointcloud(self, pcd_path: str) -> Dict[str, Any]:
        """处理点云数据
        
        Args:
            pcd_path: 点云文件路径
            
        Returns:
            Dict[str, Any]: 处理结果
        """
        try:
            if not os.path.exists(pcd_path):
                raise FileNotFoundError(f"点云文件不存在: {pcd_path}")
            
            # 基本信息
            file_size = os.path.getsize(pcd_path)
            
            result = {
                'pcd_path': pcd_path,
                'file_size': file_size,
                'timestamp': datetime.now().isoformat(),
                'processed': True
            }
            
            # 尝试读取点云头信息
            try:
                with open(pcd_path, 'r') as f:
                    lines = f.readlines()
                    
                point_count = 0
                for line in lines:
                    if line.startswith('POINTS'):
                        point_count = int(line.split()[1])
                        break
                
                result['point_count'] = point_count
                
            except Exception as e:
                logger.warning(f"读取点云头信息失败: {str(e)}")
                result['point_count'] = 0
            
            return result
            
        except Exception as e:
            logger.error(f"处理点云失败: {str(e)}")
            return {
                'pcd_path': pcd_path,
                'error': str(e),
                'processed': False,
                'timestamp': datetime.now().isoformat()
            }
    
    def get_session_status(self, session_id: str) -> Dict[str, Any]:
        """获取会话状态
        
        Args:
            session_id: 会话ID
            
        Returns:
            Dict[str, Any]: 会话状态
        """
        if session_id not in self.processing_sessions:
            return {'status': 'not_found'}
        
        session = self.processing_sessions[session_id]
        
        # 统计文件数量
        file_counts = {}
        for data_type, session_dir in session['session_dirs'].items():
            if session_dir.exists():
                if data_type == 'images':
                    patterns = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
                elif data_type == 'masks':
                    patterns = ['*.png', '*.jpg', '*.mask']
                elif data_type == 'pointclouds':
                    patterns = ['*.pcd', '*.ply', '*.pts']
                elif data_type == 'metadata':
                    patterns = ['*.json', '*.txt', '*.yaml']
                else:
                    continue
                
                count = 0
                for pattern in patterns:
                    count += len(list(session_dir.glob(pattern)))
                file_counts[data_type] = count
            else:
                file_counts[data_type] = 0
        
        return {
            'session_id': session_id,
            'status': session['status'],
            'start_time': session['start_time'].isoformat(),
            'last_update': session['last_update'].isoformat(),
            'file_counts': file_counts,
            'processed_files_count': len(session['processed_files'])
        }
    
    def get_processed_data(self, session_id: str) -> Dict[str, Any]:
        """获取已处理的数据
        
        Args:
            session_id: 会话ID
            
        Returns:
            Dict[str, Any]: 已处理的数据
        """
        if session_id not in self.processing_sessions:
            return {'error': 'Session not found'}
        
        session = self.processing_sessions[session_id]
        processed_data = {
            'images': [],
            'pointclouds': [],
            'session_info': self.get_session_status(session_id)
        }
        
        try:
            # 获取图像数据
            image_dir = session['session_dirs']['images']
            mask_dir = session['session_dirs']['masks']
            
            for image_path in image_dir.glob('*.jpg'):
                # 查找对应的mask
                mask_path = None
                mask_name = image_path.stem + '.png'
                potential_mask = mask_dir / mask_name
                if potential_mask.exists():
                    mask_path = str(potential_mask)
                
                # 处理图像对
                result = self.process_image_pair(str(image_path), mask_path)
                processed_data['images'].append(result)
            
            # 获取点云数据
            pcd_dir = session['session_dirs']['pointclouds']
            for pcd_path in pcd_dir.glob('*.pcd'):
                result = self.process_pointcloud(str(pcd_path))
                processed_data['pointclouds'].append(result)
            
        except Exception as e:
            logger.error(f"获取处理数据失败: {str(e)}")
            processed_data['error'] = str(e)
        
        return processed_data
    
    def cleanup_session(self, session_id: str, remove_files: bool = False) -> bool:
        """清理会话数据
        
        Args:
            session_id: 会话ID
            remove_files: 是否删除文件
            
        Returns:
            bool: 是否成功清理
        """
        try:
            if session_id not in self.processing_sessions:
                return True
            
            session = self.processing_sessions[session_id]
            
            # 删除文件（如果需要）
            if remove_files:
                for session_dir in session['session_dirs'].values():
                    if session_dir.exists():
                        shutil.rmtree(session_dir)
                        logger.info(f"删除会话目录: {session_dir}")
            
            # 删除会话记录
            del self.processing_sessions[session_id]
            logger.info(f"清理会话 {session_id} 完成")
            return True
            
        except Exception as e:
            logger.error(f"清理会话失败: {str(e)}")
            return False
    
    def export_session_data(self, session_id: str, export_path: str) -> bool:
        """导出会话数据
        
        Args:
            session_id: 会话ID
            export_path: 导出路径
            
        Returns:
            bool: 是否成功导出
        """
        try:
            if session_id not in self.processing_sessions:
                return False
            
            export_dir = Path(export_path)
            export_dir.mkdir(parents=True, exist_ok=True)
            
            # 导出处理结果
            processed_data = self.get_processed_data(session_id)
            
            # 保存JSON报告
            report_path = export_dir / f"{session_id}_report.json"
            with open(report_path, 'w', encoding='utf-8') as f:
                json.dump(processed_data, f, indent=2, ensure_ascii=False)
            
            logger.info(f"会话 {session_id} 数据导出完成: {export_path}")
            return True
            
        except Exception as e:
            logger.error(f"导出会话数据失败: {str(e)}")
            return False
