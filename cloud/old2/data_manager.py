# Urban Construction Eagle Eye - 数据管理器

import os
import json
import sqlite3
import logging
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any
import hashlib
import shutil

logger = logging.getLogger(__name__)

class DataManager:
    """数据管理器"""
    
    def __init__(self, data_dir: str = "./data"):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(exist_ok=True)
        
        # 创建子目录
        self.rosbag_dir = self.data_dir / "rosbags"
        self.pointcloud_dir = self.data_dir / "pointclouds"
        self.image_dir = self.data_dir / "images"
        self.cache_dir = self.data_dir / "cache"
        
        for dir_path in [self.rosbag_dir, self.pointcloud_dir, self.image_dir, self.cache_dir]:
            dir_path.mkdir(exist_ok=True)
        
        # 初始化数据库
        self.db_path = self.data_dir / "eagle_eye.db"
        self.init_database()
    
    def init_database(self):
        """初始化数据库"""
        try:
            conn = sqlite3.connect(str(self.db_path))
            cursor = conn.cursor()
            
            # 创建会话表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS sessions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id TEXT UNIQUE NOT NULL,
                    ip_address TEXT NOT NULL,
                    start_time DATETIME NOT NULL,
                    end_time DATETIME,
                    status TEXT DEFAULT 'active',
                    rosbag_count INTEGER DEFAULT 0,
                    pointcloud_count INTEGER DEFAULT 0,
                    defect_count INTEGER DEFAULT 0,
                    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
                )
            ''')
            
            # 创建点云数据表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS pointclouds (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id TEXT NOT NULL,
                    filename TEXT NOT NULL,
                    timestamp DATETIME NOT NULL,
                    point_count INTEGER,
                    file_size INTEGER,
                    gps_lat REAL,
                    gps_lon REAL,
                    defect_severity INTEGER DEFAULT 0,
                    analysis_result TEXT,
                    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (session_id) REFERENCES sessions (session_id)
                )
            ''')
            
            # 创建图像数据表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS images (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id TEXT NOT NULL,
                    pointcloud_id INTEGER,
                    filename TEXT NOT NULL,
                    image_type TEXT NOT NULL,
                    timestamp DATETIME NOT NULL,
                    file_size INTEGER,
                    defect_area REAL DEFAULT 0,
                    defect_severity INTEGER DEFAULT 0,
                    ai_analysis TEXT,
                    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (session_id) REFERENCES sessions (session_id),
                    FOREIGN KEY (pointcloud_id) REFERENCES pointclouds (id)
                )
            ''')
            
            # 创建缺陷数据表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS defects (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id TEXT NOT NULL,
                    pointcloud_id INTEGER,
                    image_id INTEGER,
                    defect_type TEXT NOT NULL,
                    severity INTEGER NOT NULL,
                    area REAL,
                    position_x REAL,
                    position_y REAL,
                    position_z REAL,
                    description TEXT,
                    created_at DATETIME DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (session_id) REFERENCES sessions (session_id),
                    FOREIGN KEY (pointcloud_id) REFERENCES pointclouds (id),
                    FOREIGN KEY (image_id) REFERENCES images (id)
                )
            ''')
            
            conn.commit()
            conn.close()
            
            logger.info("数据库初始化完成")
            
        except Exception as e:
            logger.error(f"数据库初始化失败: {str(e)}")
    
    def create_session(self, ip_address: str) -> str:
        """创建新会话"""
        try:
            session_id = self.generate_session_id(ip_address)
            
            conn = sqlite3.connect(str(self.db_path))
            cursor = conn.cursor()
            
            cursor.execute('''
                INSERT INTO sessions (session_id, ip_address, start_time)
                VALUES (?, ?, ?)
            ''', (session_id, ip_address, datetime.now()))
            
            conn.commit()
            conn.close()
            
            # 创建会话目录
            session_dir = self.data_dir / session_id
            session_dir.mkdir(exist_ok=True)
            
            logger.info(f"会话创建成功: {session_id}")
            return session_id
            
        except Exception as e:
            logger.error(f"会话创建失败: {str(e)}")
            raise
    
    def generate_session_id(self, ip_address: str) -> str:
        """生成会话ID"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        ip_hash = hashlib.md5(ip_address.encode()).hexdigest()[:8]
        return f"session_{timestamp}_{ip_hash}"
    
    def get_session(self, session_id: str) -> Optional[Dict[str, Any]]:
        """获取会话信息"""
        try:
            conn = sqlite3.connect(str(self.db_path))
            cursor = conn.cursor()
            
            cursor.execute('''
                SELECT * FROM sessions WHERE session_id = ?
            ''', (session_id,))
            
            row = cursor.fetchone()
            conn.close()
            
            if row:
                columns = [desc[0] for desc in cursor.description]
                return dict(zip(columns, row))
            
            return None
            
        except Exception as e:
            logger.error(f"获取会话失败: {str(e)}")
            return None
    
    def update_session_status(self, session_id: str, status: str):
        """更新会话状态"""
        try:
            conn = sqlite3.connect(str(self.db_path))
            cursor = conn.cursor()
            
            cursor.execute('''
                UPDATE sessions SET status = ?, end_time = ?
                WHERE session_id = ?
            ''', (status, datetime.now(), session_id))
            
            conn.commit()
            conn.close()
            
            logger.info(f"会话状态更新: {session_id} -> {status}")
            
        except Exception as e:
            logger.error(f"会话状态更新失败: {str(e)}")
    
    def save_pointcloud(self, session_id: str, filename: str, data: Dict[str, Any]) -> int:
        """保存点云数据"""
        try:
            conn = sqlite3.connect(str(self.db_path))
            cursor = conn.cursor()
            
            cursor.execute('''
                INSERT INTO pointclouds (session_id, filename, timestamp, point_count, 
                                       file_size, gps_lat, gps_lon, defect_severity)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                session_id,
                filename,
                data.get('timestamp', datetime.now()),
                data.get('point_count', 0),
                data.get('file_size', 0),
                data.get('gps_lat'),
                data.get('gps_lon'),
                data.get('defect_severity', 0)
            ))
            
            pointcloud_id = cursor.lastrowid
            
            conn.commit()
            conn.close()
            
            logger.info(f"点云数据保存成功: {filename}")
            return pointcloud_id
            
        except Exception as e:
            logger.error(f"点云数据保存失败: {str(e)}")
            raise
    
    def save_image(self, session_id: str, pointcloud_id: int, filename: str, 
                   image_type: str, data: Dict[str, Any]) -> int:
        """保存图像数据"""
        try:
            conn = sqlite3.connect(str(self.db_path))
            cursor = conn.cursor()
            
            cursor.execute('''
                INSERT INTO images (session_id, pointcloud_id, filename, image_type, 
                                  timestamp, file_size, defect_area, defect_severity, ai_analysis)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                session_id,
                pointcloud_id,
                filename,
                image_type,
                data.get('timestamp', datetime.now()),
                data.get('file_size', 0),
                data.get('defect_area', 0),
                data.get('defect_severity', 0),
                json.dumps(data.get('ai_analysis', {}))
            ))
            
            image_id = cursor.lastrowid
            
            conn.commit()
            conn.close()
            
            logger.info(f"图像数据保存成功: {filename}")
            return image_id
            
        except Exception as e:
            logger.error(f"图像数据保存失败: {str(e)}")
            raise
    
    def get_session_images(self, session_id: str, limit: int = 10) -> List[Dict[str, Any]]:
        """获取会话中的图像（按缺陷严重程度排序）"""
        try:
            conn = sqlite3.connect(str(self.db_path))
            cursor = conn.cursor()
            
            cursor.execute('''
                SELECT * FROM images 
                WHERE session_id = ? 
                ORDER BY defect_severity DESC, defect_area DESC
                LIMIT ?
            ''', (session_id, limit))
            
            rows = cursor.fetchall()
            columns = [desc[0] for desc in cursor.description]
            
            conn.close()
            
            images = []
            for row in rows:
                image_data = dict(zip(columns, row))
                # 解析AI分析结果
                if image_data.get('ai_analysis'):
                    try:
                        image_data['ai_analysis'] = json.loads(image_data['ai_analysis'])
                    except:
                        image_data['ai_analysis'] = {}
                images.append(image_data)
            
            return images
            
        except Exception as e:
            logger.error(f"获取会话图像失败: {str(e)}")
            return []
    
    def get_file_path(self, session_id: str, filename: str, file_type: str) -> Path:
        """获取文件路径"""
        session_dir = self.data_dir / session_id
        
        if file_type == 'rosbag':
            return session_dir / 'rosbags' / filename
        elif file_type == 'pointcloud':
            return session_dir / 'pointclouds' / filename
        elif file_type == 'image':
            return session_dir / 'images' / filename
        else:
            return session_dir / filename
    
    def save_file(self, session_id: str, filename: str, content: bytes, file_type: str) -> str:
        """保存文件"""
        try:
            file_path = self.get_file_path(session_id, filename, file_type)
            file_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(file_path, 'wb') as f:
                f.write(content)
            
            logger.info(f"文件保存成功: {file_path}")
            return str(file_path)
            
        except Exception as e:
            logger.error(f"文件保存失败: {str(e)}")
            raise
    
    def cleanup_old_sessions(self, max_sessions: int = 100):
        """清理旧会话"""
        try:
            conn = sqlite3.connect(str(self.db_path))
            cursor = conn.cursor()
            
            # 获取需要删除的会话
            cursor.execute('''
                SELECT session_id FROM sessions 
                ORDER BY created_at DESC 
                LIMIT -1 OFFSET ?
            ''', (max_sessions,))
            
            old_sessions = [row[0] for row in cursor.fetchall()]
            
            # 删除数据库记录
            for session_id in old_sessions:
                cursor.execute('DELETE FROM sessions WHERE session_id = ?', (session_id,))
                cursor.execute('DELETE FROM pointclouds WHERE session_id = ?', (session_id,))
                cursor.execute('DELETE FROM images WHERE session_id = ?', (session_id,))
                cursor.execute('DELETE FROM defects WHERE session_id = ?', (session_id,))
                
                # 删除文件
                session_dir = self.data_dir / session_id
                if session_dir.exists():
                    shutil.rmtree(session_dir)
            
            conn.commit()
            conn.close()
            
            logger.info(f"清理了 {len(old_sessions)} 个旧会话")
            
        except Exception as e:
            logger.error(f"清理旧会话失败: {str(e)}")
    
    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        try:
            conn = sqlite3.connect(str(self.db_path))
            cursor = conn.cursor()
            
            # 会话统计
            cursor.execute('SELECT COUNT(*) FROM sessions')
            session_count = cursor.fetchone()[0]
            
            # 点云统计
            cursor.execute('SELECT COUNT(*), SUM(point_count) FROM pointclouds')
            pointcloud_stats = cursor.fetchone()
            
            # 图像统计
            cursor.execute('SELECT COUNT(*), SUM(file_size) FROM images')
            image_stats = cursor.fetchone()
            
            # 缺陷统计
            cursor.execute('SELECT COUNT(*), AVG(severity) FROM defects')
            defect_stats = cursor.fetchone()
            
            conn.close()
            
            return {
                'sessions': session_count,
                'pointclouds': pointcloud_stats[0] or 0,
                'total_points': pointcloud_stats[1] or 0,
                'images': image_stats[0] or 0,
                'total_image_size': image_stats[1] or 0,
                'defects': defect_stats[0] or 0,
                'average_severity': defect_stats[1] or 0
            }
            
        except Exception as e:
            logger.error(f"获取统计信息失败: {str(e)}")
            return {}

# 全局数据管理器实例
data_manager = DataManager()
