import os
import json
import logging
from typing import Dict, List, Optional, Any
from datetime import datetime
import shutil

logger = logging.getLogger(__name__)

class DataManager:
    def __init__(self):
        self.data_dir = "data"
        self.sessions_dir = os.path.join(self.data_dir, "sessions")
        self.results_dir = os.path.join(self.data_dir, "results")
        
        # 创建必要目录
        os.makedirs(self.sessions_dir, exist_ok=True)
        os.makedirs(self.results_dir, exist_ok=True)
    
    def get_all_sessions(self) -> List[Dict]:
        """获取所有会话信息"""
        sessions = []
        
        try:
            for session_dir in os.listdir(self.sessions_dir):
                session_path = os.path.join(self.sessions_dir, session_dir)
                if os.path.isdir(session_path):
                    session_info = self._get_session_info(session_dir)
                    if session_info:
                        sessions.append(session_info)
        except Exception as e:
            logger.error(f"获取会话列表错误: {str(e)}")
        
        return sorted(sessions, key=lambda x: x.get('created_time', ''), reverse=True)
    
    def _get_session_info(self, session_id: str) -> Optional[Dict]:
        """获取单个会话信息"""
        try:
            session_path = os.path.join(self.sessions_dir, session_id)
            
            # 检查必要文件
            pcd_file = os.path.join(session_path, "pointclouds.pcd")
            location_file = os.path.join(session_path, "location.txt")
            images_dir = os.path.join(session_path, "images")
            masks_dir = os.path.join(session_path, "masks")
            
            info = {
                "session_id": session_id,
                "created_time": datetime.fromtimestamp(os.path.getctime(session_path)).isoformat(),
                "has_pointcloud": os.path.exists(pcd_file),
                "has_location": os.path.exists(location_file),
                "has_images": os.path.exists(images_dir),
                "has_masks": os.path.exists(masks_dir),
                "image_count": len(os.listdir(images_dir)) if os.path.exists(images_dir) else 0,
                "mask_count": len(os.listdir(masks_dir)) if os.path.exists(masks_dir) else 0
            }
            
            return info
            
        except Exception as e:
            logger.error(f"获取会话信息错误 {session_id}: {str(e)}")
            return None
    
    def get_location_data(self, session_id: str) -> Dict:
        """获取GPS位置数据"""
        try:
            location_file = os.path.join(self.sessions_dir, session_id, "location.txt")
            
            if not os.path.exists(location_file):
                raise FileNotFoundError(f"位置文件不存在: {location_file}")
            
            with open(location_file, 'r') as f:
                lines = f.read().strip().split('\n')
            
            if len(lines) < 1:
                raise ValueError("位置文件格式错误")
            
            # 解析所有坐标点
            points = []
            for line in lines:
                if line.strip():
                    coords = line.split(',')
                    if len(coords) >= 2:
                        points.append({
                            "lon": float(coords[0]),
                            "lat": float(coords[1])
                        })
            
            if len(points) < 1:
                raise ValueError("没有有效的坐标点")
            
            # 计算包围框
            lats = [p["lat"] for p in points]
            lons = [p["lon"] for p in points]
            
            min_lat, max_lat = min(lats), max(lats)
            min_lon, max_lon = min(lons), max(lons)
            
            # 添加一些边距
            margin = 0.001  # 约100米的边距
            
            return {
                "points": points,
                "bounds": {
                    "southwest": {"lat": min_lat + margin, "lon": min_lon + margin},
                    "northeast": {"lat": max_lat - margin, "lon": max_lon - margin}
                },
                "center": {
                    "lat": (min_lat + max_lat) / 2,
                    "lon": (min_lon + max_lon) / 2
                },
                "total_points": len(points)
            }
            
        except Exception as e:
            logger.error(f"获取位置数据错误: {str(e)}")
            raise
    
    def save_review_result(self, session_id: str, defect_id: str, approved: bool) -> Dict:
        """保存复检结果"""
        try:
            review_file = os.path.join(self.results_dir, f"{session_id}_reviews.json")
            
            # 加载现有复检结果
            reviews = {}
            if os.path.exists(review_file):
                with open(review_file, 'r', encoding='utf-8') as f:
                    reviews = json.load(f)
            
            # 添加新的复检结果
            reviews[defect_id] = {
                "approved": approved,
                "timestamp": datetime.now().isoformat(),
                "session_id": session_id
            }
            
            # 保存到文件
            with open(review_file, 'w', encoding='utf-8') as f:
                json.dump(reviews, f, ensure_ascii=False, indent=2)
            
            logger.info(f"复检结果已保存: {session_id}/{defect_id} = {approved}")
            return reviews[defect_id]
            
        except Exception as e:
            logger.error(f"保存复检结果错误: {str(e)}")
            raise
    
    def get_review_results(self, session_id: str) -> Dict:
        """获取复检结果"""
        try:
            review_file = os.path.join(self.results_dir, f"{session_id}_reviews.json")
            
            if not os.path.exists(review_file):
                return {}
            
            with open(review_file, 'r', encoding='utf-8') as f:
                return json.load(f)
                
        except Exception as e:
            logger.error(f"获取复检结果错误: {str(e)}")
            return {}
    
    def save_analysis_result(self, session_id: str, analysis: Dict) -> None:
        """保存AI分析结果"""
        try:
            analysis_file = os.path.join(self.results_dir, f"{session_id}_analysis.json")
            
            with open(analysis_file, 'w', encoding='utf-8') as f:
                json.dump(analysis, f, ensure_ascii=False, indent=2)
            
            logger.info(f"AI分析结果已保存: {session_id}")
            
        except Exception as e:
            logger.error(f"保存AI分析结果错误: {str(e)}")
            raise
    
    def get_analysis_result(self, session_id: str) -> Optional[Dict]:
        """获取AI分析结果"""
        try:
            analysis_file = os.path.join(self.results_dir, f"{session_id}_analysis.json")
            
            if not os.path.exists(analysis_file):
                return None
            
            with open(analysis_file, 'r', encoding='utf-8') as f:
                return json.load(f)
                
        except Exception as e:
            logger.error(f"获取AI分析结果错误: {str(e)}")
            return None
    
    def get_statistics(self, session_id: str) -> Dict:
        """获取统计数据"""
        try:
            session_path = os.path.join(self.sessions_dir, session_id)
            
            # 基础统计
            stats = {
                "session_id": session_id,
                "total_images": 0,
                "total_masks": 0,
                "defect_types": {"red": 0, "blue": 0, "green": 0},
                "total_defects": 0,
                "reviewed_count": 0,
                "approved_count": 0,
                "rejected_count": 0
            }
            
            # 计算图像和mask数量
            images_dir = os.path.join(session_path, "images")
            masks_dir = os.path.join(session_path, "masks")
            
            if os.path.exists(images_dir):
                stats["total_images"] = len([f for f in os.listdir(images_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
            
            if os.path.exists(masks_dir):
                stats["total_masks"] = len([f for f in os.listdir(masks_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
            
            # 获取复检统计
            reviews = self.get_review_results(session_id)
            stats["reviewed_count"] = len(reviews)
            stats["approved_count"] = sum(1 for r in reviews.values() if r.get("approved", False))
            stats["rejected_count"] = stats["reviewed_count"] - stats["approved_count"]
            
            # 获取AI分析结果
            analysis = self.get_analysis_result(session_id)
            if analysis:
                stats["ai_analysis"] = {
                    "condition_score": analysis.get("condition_score", 0),
                    "overall_condition": analysis.get("overall_condition", "未知"),
                    "urgency_level": analysis.get("urgency_level", "未知")
                }
            
            return stats
            
        except Exception as e:
            logger.error(f"获取统计数据错误: {str(e)}")
            return {}
    
    def get_image_path(self, session_id: str, image_type: str, filename: str) -> str:
        """获取图像文件路径"""
        image_path = os.path.join(self.sessions_dir, session_id, image_type, filename)
        
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"图像文件不存在: {image_path}")
        
        return image_path
    
    def cleanup_old_sessions(self, keep_days: int = 30) -> None:
        """清理旧会话数据"""
        try:
            cutoff_time = datetime.now().timestamp() - (keep_days * 24 * 3600)
            
            for session_dir in os.listdir(self.sessions_dir):
                session_path = os.path.join(self.sessions_dir, session_dir)
                if os.path.isdir(session_path):
                    if os.path.getctime(session_path) < cutoff_time:
                        shutil.rmtree(session_path)
                        logger.info(f"清理旧会话: {session_dir}")
                        
        except Exception as e:
            logger.error(f"清理旧会话错误: {str(e)}")
