"""
点云分析器
功能：
1. 点云数据分析和处理
2. 缺陷区域识别和量化
3. 点云可视化渲染
4. 统计分析
"""

import numpy as np
import cv2
import json
import logging
from typing import Dict, List, Tuple, Optional
from pathlib import Path
import matplotlib.pyplot as plt
from PIL import Image
import io
import base64

# 由于Open3D在Windows上可能有依赖问题，我们使用基础的numpy处理
# import open3d as o3d

logger = logging.getLogger(__name__)

class PointCloudAnalyzer:
    """点云分析器"""
    
    def __init__(self):
        self.red_threshold = {
            "r_min": 0.6,
            "g_max": 0.8,
            "b_max": 0.8
        }
        
        self.defect_colors = {
            'red': [1.0, 0.0, 0.0],     # 红色缺陷1
            'blue': [0.0, 0.0, 1.0],    # 蓝色缺陷2
            'green': [0.0, 1.0, 0.0]    # 绿色缺陷3
        }
    
    def analyze_pointcloud(self, pointcloud_file: str) -> Dict:
        """分析点云数据"""
        try:
            logger.info(f"开始分析点云: {pointcloud_file}")
            
            # 读取点云数据
            points, colors = self._load_pointcloud(pointcloud_file)
            
            if points is None or len(points) == 0:
                return {"error": "点云数据为空"}
            
            # 基础统计
            analysis_result = {
                "total_points": len(points),
                "bounds": self._calculate_bounds(points),
                "defect_analysis": self._analyze_defects(points, colors),
                "statistics": self._calculate_statistics(points, colors),
                "severity_assessment": None
            }
            
            # 评估严重程度
            analysis_result["severity_assessment"] = self._assess_severity(analysis_result["defect_analysis"])
            
            logger.info(f"点云分析完成: {pointcloud_file}")
            return analysis_result
            
        except Exception as e:
            logger.error(f"点云分析失败: {str(e)}")
            return {"error": str(e)}
    
    def _load_pointcloud(self, pointcloud_file: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """加载点云数据"""
        try:
            # 首先尝试加载JSON格式
            json_file = pointcloud_file.replace('.pcd', '.json')
            if Path(json_file).exists():
                with open(json_file, 'r') as f:
                    data = json.load(f)
                    points = np.array(data['points'])
                    colors = np.array(data['colors'])
                    return points, colors
            
            # 如果JSON文件不存在，尝试解析PCD文件
            return self._parse_pcd_file(pointcloud_file)
            
        except Exception as e:
            logger.error(f"加载点云数据失败: {str(e)}")
            return None, None
    
    def _parse_pcd_file(self, pcd_file: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """解析PCD文件（简化版本）"""
        try:
            points = []
            colors = []
            
            with open(pcd_file, 'r') as f:
                lines = f.readlines()
                
                # 查找DATA行
                data_start = -1
                for i, line in enumerate(lines):
                    if line.startswith('DATA'):
                        data_start = i + 1
                        break
                
                if data_start > 0:
                    for line in lines[data_start:]:
                        parts = line.strip().split()
                        if len(parts) >= 6:  # x, y, z, r, g, b
                            x, y, z = map(float, parts[:3])
                            r, g, b = map(float, parts[3:6])
                            
                            points.append([x, y, z])
                            colors.append([r, g, b])
            
            return np.array(points), np.array(colors)
            
        except Exception as e:
            logger.error(f"解析PCD文件失败: {str(e)}")
            return None, None
    
    def _calculate_bounds(self, points: np.ndarray) -> Dict:
        """计算点云边界"""
        try:
            min_coords = np.min(points, axis=0)
            max_coords = np.max(points, axis=0)
            
            return {
                "min": min_coords.tolist(),
                "max": max_coords.tolist(),
                "size": (max_coords - min_coords).tolist(),
                "center": ((min_coords + max_coords) / 2).tolist()
            }
            
        except Exception as e:
            logger.error(f"计算边界失败: {str(e)}")
            return {}
    
    def _analyze_defects(self, points: np.ndarray, colors: np.ndarray) -> Dict:
        """分析缺陷"""
        try:
            defect_analysis = {
                "defect_types": {},
                "total_defect_points": 0,
                "defect_percentage": 0.0,
                "defect_distribution": {}
            }
            
            total_points = len(points)
            total_defect_points = 0
            
            for defect_type, target_color in self.defect_colors.items():
                # 查找匹配的颜色点
                defect_mask = self._find_defect_points(colors, target_color)
                defect_points = np.sum(defect_mask)
                
                if defect_points > 0:
                    defect_percentage = (defect_points / total_points) * 100
                    
                    defect_analysis["defect_types"][defect_type] = {
                        "count": int(defect_points),
                        "percentage": defect_percentage,
                        "color": target_color,
                        "severity": self._classify_defect_severity(defect_percentage)
                    }
                    
                    total_defect_points += defect_points
            
            defect_analysis["total_defect_points"] = int(total_defect_points)
            defect_analysis["defect_percentage"] = (total_defect_points / total_points) * 100
            
            return defect_analysis
            
        except Exception as e:
            logger.error(f"分析缺陷失败: {str(e)}")
            return {}
    
    def _find_defect_points(self, colors: np.ndarray, target_color: List[float], tolerance: float = 0.3) -> np.ndarray:
        """查找缺陷点"""
        try:
            # 计算颜色距离
            color_diff = np.linalg.norm(colors - np.array(target_color), axis=1)
            return color_diff < tolerance
            
        except Exception as e:
            logger.error(f"查找缺陷点失败: {str(e)}")
            return np.array([])
    
    def _classify_defect_severity(self, percentage: float) -> str:
        """分类缺陷严重程度"""
        if percentage > 5.0:
            return "severe"
        elif percentage > 2.0:
            return "moderate"
        elif percentage > 0.5:
            return "mild"
        else:
            return "minor"
    
    def _calculate_statistics(self, points: np.ndarray, colors: np.ndarray) -> Dict:
        """计算统计数据"""
        try:
            stats = {
                "point_density": len(points) / 1000,  # 点密度 (点/千立方米)
                "color_distribution": {},
                "spatial_distribution": {},
                "quality_metrics": {}
            }
            
            # 计算颜色分布
            stats["color_distribution"] = {
                "mean_color": np.mean(colors, axis=0).tolist(),
                "std_color": np.std(colors, axis=0).tolist(),
                "unique_colors": len(np.unique(colors.view(np.void), axis=0))
            }
            
            # 计算空间分布
            stats["spatial_distribution"] = {
                "mean_position": np.mean(points, axis=0).tolist(),
                "std_position": np.std(points, axis=0).tolist(),
                "volume": self._calculate_volume(points)
            }
            
            # 质量指标
            stats["quality_metrics"] = {
                "completeness": self._calculate_completeness(points),
                "uniformity": self._calculate_uniformity(points),
                "noise_level": self._calculate_noise_level(points)
            }
            
            return stats
            
        except Exception as e:
            logger.error(f"计算统计数据失败: {str(e)}")
            return {}
    
    def _calculate_volume(self, points: np.ndarray) -> float:
        """计算点云体积（简化版本）"""
        try:
            min_coords = np.min(points, axis=0)
            max_coords = np.max(points, axis=0)
            volume = np.prod(max_coords - min_coords)
            return float(volume)
            
        except Exception as e:
            logger.error(f"计算体积失败: {str(e)}")
            return 0.0
    
    def _calculate_completeness(self, points: np.ndarray) -> float:
        """计算完整性分数"""
        # 简化的完整性计算
        return min(1.0, len(points) / 100000)  # 假设100k点为完整
    
    def _calculate_uniformity(self, points: np.ndarray) -> float:
        """计算均匀性分数"""
        # 简化的均匀性计算
        try:
            std_dev = np.std(points, axis=0)
            uniformity = 1.0 / (1.0 + np.mean(std_dev))
            return float(uniformity)
        except:
            return 0.5
    
    def _calculate_noise_level(self, points: np.ndarray) -> float:
        """计算噪声水平"""
        # 简化的噪声水平计算
        return 0.1  # 假设固定噪声水平
    
    def _assess_severity(self, defect_analysis: Dict) -> Dict:
        """评估整体严重程度"""
        try:
            total_percentage = defect_analysis.get("defect_percentage", 0.0)
            defect_types = defect_analysis.get("defect_types", {})
            
            # 计算严重程度分数
            severity_score = 0
            for defect_type, info in defect_types.items():
                percentage = info.get("percentage", 0.0)
                if defect_type == "red":  # 红色缺陷更严重
                    severity_score += percentage * 2
                else:
                    severity_score += percentage
            
            # 分类严重程度
            if severity_score > 10:
                level = "critical"
                description = "严重缺陷，需要立即维修"
            elif severity_score > 5:
                level = "high"
                description = "较严重缺陷，需要尽快维修"
            elif severity_score > 2:
                level = "medium"
                description = "中等缺陷，需要计划维修"
            elif severity_score > 0.5:
                level = "low"
                description = "轻微缺陷，需要监控"
            else:
                level = "minimal"
                description = "缺陷很少，状态良好"
            
            return {
                "level": level,
                "score": severity_score,
                "description": description,
                "recommendations": self._generate_recommendations(level, defect_types)
            }
            
        except Exception as e:
            logger.error(f"评估严重程度失败: {str(e)}")
            return {"level": "unknown", "score": 0, "description": "无法评估"}
    
    def _generate_recommendations(self, severity_level: str, defect_types: Dict) -> List[str]:
        """生成维修建议"""
        recommendations = []
        
        if severity_level == "critical":
            recommendations.append("立即停止使用，进行紧急维修")
            recommendations.append("联系专业维修团队")
        elif severity_level == "high":
            recommendations.append("在1个月内进行维修")
            recommendations.append("加强监控，防止恶化")
        elif severity_level == "medium":
            recommendations.append("在3个月内进行维修")
            recommendations.append("定期检查缺陷发展情况")
        elif severity_level == "low":
            recommendations.append("在6个月内进行维修")
            recommendations.append("继续监控，记录变化")
        else:
            recommendations.append("保持现状，定期检查")
        
        # 根据缺陷类型添加特定建议
        for defect_type, info in defect_types.items():
            if defect_type == "red" and info["percentage"] > 1.0:
                recommendations.append("重点关注红色缺陷区域")
            elif defect_type == "blue" and info["percentage"] > 1.0:
                recommendations.append("检查蓝色缺陷区域的结构完整性")
            elif defect_type == "green" and info["percentage"] > 1.0:
                recommendations.append("处理绿色缺陷区域的表面问题")
        
        return recommendations
    
    def render_pointcloud_image(self, pointcloud_file: str, output_size: Tuple[int, int] = (800, 600)) -> Optional[Image.Image]:
        """渲染点云图像用于AI分析"""
        try:
            logger.info(f"渲染点云图像: {pointcloud_file}")
            
            # 加载点云数据
            points, colors = self._load_pointcloud(pointcloud_file)
            
            if points is None or len(points) == 0:
                return None
            
            # 创建3D散点图
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            
            # 绘制点云
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                      c=colors, s=1, alpha=0.6)
            
            # 设置标题和标签
            ax.set_title('Building Point Cloud Analysis', fontsize=16)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            
            # 设置视角
            ax.view_init(elev=20, azim=45)
            
            # 保存为图像
            buffer = io.BytesIO()
            plt.savefig(buffer, format='png', dpi=100, bbox_inches='tight')
            buffer.seek(0)
            
            image = Image.open(buffer)
            plt.close()
            
            return image
            
        except Exception as e:
            logger.error(f"渲染点云图像失败: {str(e)}")
            return None
    
    def set_red_threshold(self, r_min: float, g_max: float, b_max: float):
        """设置红色点云检测阈值"""
        self.red_threshold = {
            "r_min": r_min,
            "g_max": g_max,
            "b_max": b_max
        }
