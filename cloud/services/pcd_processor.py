import open3d as o3d
import numpy as np
import json
import os
import logging
from typing import Dict, List, Tuple, Optional
import asyncio
from PIL import Image
import io
import base64

logger = logging.getLogger(__name__)

class PCDProcessor:
    def __init__(self):
        self.point_clouds = {}
        
    async def process_pointcloud(self, session_id: str) -> Dict:
        """处理点云数据"""
        try:
            pcd_path = os.path.join("data", "sessions", session_id, "pointclouds.pcd")
            
            if not os.path.exists(pcd_path):
                raise FileNotFoundError(f"点云文件不存在: {pcd_path}")
            
            # 在线程池中加载点云
            pcd = await asyncio.get_event_loop().run_in_executor(
                None,
                o3d.io.read_point_cloud,
                pcd_path
            )
            
            if len(pcd.points) == 0:
                raise ValueError("点云数据为空")
            
            # 转换为numpy数组
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors) if len(pcd.colors) > 0 else None
            
            # 计算统计信息
            stats = self._calculate_stats(points)
            
            # 降采样以减少数据量（用于web显示）
            if len(points) > 50000:
                pcd_downsampled = pcd.voxel_down_sample(voxel_size=0.1)
                points = np.asarray(pcd_downsampled.points)
                colors = np.asarray(pcd_downsampled.colors) if len(pcd_downsampled.colors) > 0 else None
            
            # 准备返回数据
            result = {
                "points": points.tolist(),
                "colors": colors.tolist() if colors is not None else None,
                "stats": stats,
                "bounds": {
                    "min": points.min(axis=0).tolist(),
                    "max": points.max(axis=0).tolist()
                }
            }
            
            # 缓存处理结果
            self.point_clouds[session_id] = result
            
            logger.info(f"Point cloud processed for session {session_id}: {len(points)} points")
            return result
            
        except Exception as e:
            logger.error(f"Point cloud processing error: {str(e)}")
            raise
    
    def _calculate_stats(self, points: np.ndarray) -> Dict:
        """计算点云统计信息"""
        return {
            "total_points": len(points),
            "bounds": {
                "x": [float(points[:, 0].min()), float(points[:, 0].max())],
                "y": [float(points[:, 1].min()), float(points[:, 1].max())],
                "z": [float(points[:, 2].min()), float(points[:, 2].max())]
            },
            "center": points.mean(axis=0).tolist(),
            "density": len(points) / self._calculate_volume(points)
        }
    
    def _calculate_volume(self, points: np.ndarray) -> float:
        """计算点云包围盒体积"""
        min_bound = points.min(axis=0)
        max_bound = points.max(axis=0)
        volume = np.prod(max_bound - min_bound)
        return max(volume, 1.0)  # 避免除零
    
    async def generate_render_image(self, session_id: str) -> str:
        """生成点云渲染图像（用于AI分析）"""
        try:
            pcd_path = os.path.join("data", "sessions", session_id, "pointclouds.pcd")
            
            if not os.path.exists(pcd_path):
                raise FileNotFoundError(f"点云文件不存在: {pcd_path}")
            
            # 加载点云
            pcd = await asyncio.get_event_loop().run_in_executor(
                None,
                o3d.io.read_point_cloud,
                pcd_path
            )
            
            # 创建可视化器
            vis = o3d.visualization.Visualizer()
            vis.create_window(visible=False, width=800, height=600)
            vis.add_geometry(pcd)
            
            # 设置视角
            view_control = vis.get_view_control()
            view_control.set_front([0, 0, -1])
            view_control.set_up([0, -1, 0])
            view_control.set_lookat([0, 0, 0])
            
            # 渲染图像
            vis.poll_events()
            vis.update_renderer()
            
            # 捕获图像
            image = vis.capture_screen_float_buffer(False)
            vis.destroy_window()
            
            # 转换为PIL图像
            image_np = np.asarray(image)
            image_pil = Image.fromarray((image_np * 255).astype(np.uint8))
            
            # 保存图像
            render_path = os.path.join("data", "sessions", session_id, "render.png")
            image_pil.save(render_path)
            
            # 转换为base64
            buffer = io.BytesIO()
            image_pil.save(buffer, format='PNG')
            image_base64 = base64.b64encode(buffer.getvalue()).decode()
            
            logger.info(f"Render image generated for session {session_id}")
            return image_base64
            
        except Exception as e:
            logger.error(f"Render image generation error: {str(e)}")
            raise
