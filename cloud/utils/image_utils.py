import cv2
import numpy as np
import os
import logging
from typing import List, Dict, Tuple
from PIL import Image
import asyncio

logger = logging.getLogger(__name__)

class ImageAnalyzer:
    def __init__(self):
        # 定义缺陷颜色范围 (BGR格式)
        self.color_ranges = {
            'red': {
                'lower': np.array([0, 0, 200]),
                'upper': np.array([50, 50, 255])
            },
            'blue': {
                'lower': np.array([200, 0, 0]),
                'upper': np.array([255, 100, 100])
            },
            'green': {
                'lower': np.array([0, 200, 0]),
                'upper': np.array([100, 255, 100])
            }
        }
    
    async def analyze_defects(self, session_id: str) -> List[Dict]:
        """分析缺陷图像，返回按严重程度排序的缺陷列表"""
        try:
            images_dir = os.path.join("data", "sessions", session_id, "images")
            masks_dir = os.path.join("data", "sessions", session_id, "masks")
            
            if not os.path.exists(images_dir) or not os.path.exists(masks_dir):
                raise FileNotFoundError("图像或mask目录不存在")
            
            # 获取所有图像文件
            image_files = [f for f in os.listdir(images_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
            mask_files = [f for f in os.listdir(masks_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
            
            defects = []
            
            # 分析每个mask文件
            for mask_file in mask_files:
                # 找到对应的原始图像
                base_name = os.path.splitext(mask_file)[0]
                image_file = None
                
                for img_file in image_files:
                    if os.path.splitext(img_file)[0] == base_name:
                        image_file = img_file
                        break
                
                if not image_file:
                    continue
                
                # 分析单个缺陷
                defect_info = await self._analyze_single_defect(
                    session_id, image_file, mask_file
                )
                
                if defect_info:
                    defects.append(defect_info)
            
            # 按严重程度排序
            defects.sort(key=lambda x: x['severity_score'], reverse=True)
            
            logger.info(f"分析完成，共找到 {len(defects)} 个缺陷")
            return defects
            
        except Exception as e:
            logger.error(f"缺陷分析错误: {str(e)}")
            raise
    
    async def _analyze_single_defect(self, session_id: str, image_file: str, mask_file: str) -> Dict:
        """分析单个缺陷"""
        try:
            images_dir = os.path.join("data", "sessions", session_id, "images")
            masks_dir = os.path.join("data", "sessions", session_id, "masks")
            
            # 加载图像
            image_path = os.path.join(images_dir, image_file)
            mask_path = os.path.join(masks_dir, mask_file)
            
            image = cv2.imread(image_path)
            mask = cv2.imread(mask_path)
            
            if image is None or mask is None:
                return None
            
            # 分析缺陷
            defect_analysis = await asyncio.get_event_loop().run_in_executor(
                None, self._process_defect_mask, mask
            )
            
            # 计算严重程度分数
            severity_score = self._calculate_severity_score(defect_analysis, image.shape)
            
            return {
                "defect_id": f"{session_id}_{os.path.splitext(mask_file)[0]}",
                "image_file": image_file,
                "mask_file": mask_file,
                "severity_score": severity_score,
                "defect_types": defect_analysis['defect_types'],
                "total_defect_area": defect_analysis['total_area'],
                "defect_percentage": defect_analysis['defect_percentage'],
                "analysis": defect_analysis
            }
            
        except Exception as e:
            logger.error(f"单个缺陷分析错误: {str(e)}")
            return None
    
    def _process_defect_mask(self, mask: np.ndarray) -> Dict:
        """处理缺陷mask"""
        total_pixels = mask.shape[0] * mask.shape[1]
        defect_analysis = {
            'defect_types': {},
            'total_area': 0,
            'defect_percentage': 0.0
        }
        
        # 分析每种颜色的缺陷
        for color_name, color_range in self.color_ranges.items():
            # 创建颜色mask
            color_mask = cv2.inRange(mask, color_range['lower'], color_range['upper'])
            
            # 计算面积
            defect_pixels = cv2.countNonZero(color_mask)
            
            if defect_pixels > 0:
                defect_analysis['defect_types'][color_name] = {
                    'pixels': defect_pixels,
                    'area_percentage': (defect_pixels / total_pixels) * 100
                }
                defect_analysis['total_area'] += defect_pixels
        
        # 计算总缺陷百分比
        defect_analysis['defect_percentage'] = (defect_analysis['total_area'] / total_pixels) * 100
        
        return defect_analysis
    
    def _calculate_severity_score(self, defect_analysis: Dict, image_shape: Tuple) -> float:
        """计算严重程度分数"""
        total_pixels = image_shape[0] * image_shape[1]
        
        # 基础分数：基于缺陷面积百分比
        base_score = defect_analysis['defect_percentage']
        
        # 缺陷类型权重
        type_weights = {'red': 3.0, 'blue': 2.0, 'green': 1.5}
        
        # 计算加权分数
        weighted_score = 0
        for defect_type, info in defect_analysis['defect_types'].items():
            weight = type_weights.get(defect_type, 1.0)
            weighted_score += info['area_percentage'] * weight
        
        # 最终分数（0-100）
        final_score = min(weighted_score, 100.0)
        
        return round(final_score, 2)
    
    def create_overlay_image(self, session_id: str, defect_id: str) -> str:
        """创建叠加图像"""
        try:
            # 从defect_id解析文件名
            mask_file = defect_id.replace(f"{session_id}_", "") + ".png"
            image_file = mask_file  # 假设同名
            
            images_dir = os.path.join("data", "sessions", session_id, "images")
            masks_dir = os.path.join("data", "sessions", session_id, "masks")
            
            # 加载图像
            image_path = os.path.join(images_dir, image_file)
            mask_path = os.path.join(masks_dir, mask_file)
            
            image = cv2.imread(image_path)
            mask = cv2.imread(mask_path)
            
            if image is None or mask is None:
                raise FileNotFoundError("图像或mask文件不存在")
            
            # 创建叠加图像
            overlay = cv2.addWeighted(image, 0.7, mask, 0.3, 0)
            
            # 保存叠加图像
            overlay_path = os.path.join("data", "sessions", session_id, f"overlay_{defect_id}.png")
            cv2.imwrite(overlay_path, overlay)
            
            return overlay_path
            
        except Exception as e:
            logger.error(f"创建叠加图像错误: {str(e)}")
            raise
