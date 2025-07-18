"""
Urban Construction Eagle Eye - 建筑缺陷检测可视化系统
主要功能：
1. SSH连接下位机获取rosbag数据
2. 解析rosbag提取点云、图像、mask数据
3. 3D点云可视化展示
4. 地图位置关联
5. 缺陷量化分析
6. AI分析报告生成
7. 人工复检功能
"""

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, HTMLResponse
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import os
import json
import asyncio
import logging
from datetime import datetime
import subprocess
from pathlib import Path

# 导入自定义模块
from rosbag_processor import RosbagProcessor
from pointcloud_analyzer import PointCloudAnalyzer
from ssh_client import SSHClient
from gemini_client import GeminiClient
from baidu_map_client import BaiduMapClient

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 创建FastAPI应用
app = FastAPI(
    title="Urban Construction Eagle Eye",
    description="建筑缺陷检测可视化系统",
    version="1.0.0"
)

# 配置CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 静态文件配置
app.mount("/static", StaticFiles(directory="static"), name="static")

# 数据存储路径
DATA_DIR = Path("data")
ROSBAG_DIR = DATA_DIR / "rosbags"
RESULTS_DIR = DATA_DIR / "results"
IMAGES_DIR = DATA_DIR / "images"
POINTCLOUDS_DIR = DATA_DIR / "pointclouds"

# 创建必要目录
for dir_path in [DATA_DIR, ROSBAG_DIR, RESULTS_DIR, IMAGES_DIR, POINTCLOUDS_DIR]:
    dir_path.mkdir(parents=True, exist_ok=True)

# 全局变量
current_session = None
processing_status = {"status": "idle", "progress": 0, "message": ""}

# 数据模型
class ConnectionConfig(BaseModel):
    host: str
    username: str
    password: str
    rosbag_path: str = "/home/user/rosbags"  # 默认rosbag路径

class ReviewResult(BaseModel):
    image_id: str
    result: str  # "pass" or "fail"
    timestamp: str

class AnalysisRequest(BaseModel):
    session_id: str
    analysis_type: str = "building_defect"

# 初始化客户端
ssh_client = SSHClient()
gemini_client = GeminiClient()
baidu_map_client = BaiduMapClient()

@app.get("/")
async def read_root():
    """返回主页面"""
    return FileResponse("templates/index.html")

@app.get("/api/status")
async def get_status():
    """获取当前处理状态"""
    return processing_status

@app.post("/api/connect")
async def connect_to_device(config: ConnectionConfig, background_tasks: BackgroundTasks):
    """连接到下位机并下载rosbag数据"""
    global current_session, processing_status
    
    try:
        # 生成会话ID
        session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        current_session = session_id
        
        # 更新状态
        processing_status = {"status": "connecting", "progress": 10, "message": "连接到下位机..."}
        
        # 在后台任务中处理数据下载
        background_tasks.add_task(download_and_process_data, config, session_id)
        
        return {"session_id": session_id, "status": "connecting"}
        
    except Exception as e:
        logger.error(f"连接失败: {str(e)}")
        processing_status = {"status": "error", "progress": 0, "message": f"连接失败: {str(e)}"}
        raise HTTPException(status_code=500, detail=str(e))

async def download_and_process_data(config: ConnectionConfig, session_id: str):
    """下载并处理rosbag数据"""
    global processing_status
    
    try:
        # 创建会话目录
        session_dir = ROSBAG_DIR / session_id
        session_dir.mkdir(exist_ok=True)
        
        # 1. 连接SSH并下载rosbag
        processing_status = {"status": "downloading", "progress": 20, "message": "正在下载rosbag文件..."}
        
        ssh_client.connect(config.host, config.username, config.password)
        rosbag_files = ssh_client.download_rosbags(config.rosbag_path, str(session_dir))
        
        # 2. 处理rosbag数据
        processing_status = {"status": "processing", "progress": 40, "message": "正在处理rosbag数据..."}
        
        processor = RosbagProcessor()
        for rosbag_file in rosbag_files:
            await processor.process_rosbag(rosbag_file, session_id)
        
        # 3. 生成点云分析
        processing_status = {"status": "analyzing", "progress": 60, "message": "正在分析点云数据..."}
        
        analyzer = PointCloudAnalyzer()
        pointcloud_file = session_dir / "pointcloud_merged.pcd"
        if pointcloud_file.exists():
            analysis_result = analyzer.analyze_pointcloud(str(pointcloud_file))
            
            # 保存分析结果
            result_file = RESULTS_DIR / f"{session_id}_analysis.json"
            with open(result_file, 'w', encoding='utf-8') as f:
                json.dump(analysis_result, f, ensure_ascii=False, indent=2)
        
        # 4. 生成复检图像
        processing_status = {"status": "generating", "progress": 80, "message": "正在生成复检图像..."}
        
        await generate_review_images(session_id)
        
        # 5. 完成处理
        processing_status = {"status": "completed", "progress": 100, "message": "数据处理完成"}
        
        ssh_client.disconnect()
        
    except Exception as e:
        logger.error(f"数据处理失败: {str(e)}")
        processing_status = {"status": "error", "progress": 0, "message": f"处理失败: {str(e)}"}

@app.get("/api/pointcloud/{session_id}")
async def get_pointcloud_data(session_id: str):
    """获取点云数据"""
    try:
        pointcloud_file = POINTCLOUDS_DIR / f"{session_id}_pointcloud.json"
        if not pointcloud_file.exists():
            raise HTTPException(status_code=404, detail="点云数据不存在")
        
        with open(pointcloud_file, 'r', encoding='utf-8') as f:
            pointcloud_data = json.load(f)
        
        return pointcloud_data
        
    except Exception as e:
        logger.error(f"获取点云数据失败: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/analysis/{session_id}")
async def get_analysis_result(session_id: str):
    """获取分析结果"""
    try:
        result_file = RESULTS_DIR / f"{session_id}_analysis.json"
        if not result_file.exists():
            raise HTTPException(status_code=404, detail="分析结果不存在")
        
        with open(result_file, 'r', encoding='utf-8') as f:
            analysis_data = json.load(f)
        
        return analysis_data
        
    except Exception as e:
        logger.error(f"获取分析结果失败: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/review_images/{session_id}")
async def get_review_images(session_id: str):
    """获取复检图像列表"""
    try:
        review_file = RESULTS_DIR / f"{session_id}_review_images.json"
        if not review_file.exists():
            raise HTTPException(status_code=404, detail="复检图像不存在")
        
        with open(review_file, 'r', encoding='utf-8') as f:
            review_images = json.load(f)
        
        return review_images
        
    except Exception as e:
        logger.error(f"获取复检图像失败: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/review_submit")
async def submit_review(result: ReviewResult):
    """提交复检结果"""
    try:
        # 保存复检结果
        review_results_file = RESULTS_DIR / "review_results.json"
        
        if review_results_file.exists():
            with open(review_results_file, 'r', encoding='utf-8') as f:
                review_results = json.load(f)
        else:
            review_results = []
        
        review_results.append(result.dict())
        
        with open(review_results_file, 'w', encoding='utf-8') as f:
            json.dump(review_results, f, ensure_ascii=False, indent=2)
        
        return {"status": "success", "message": "复检结果已保存"}
        
    except Exception as e:
        logger.error(f"提交复检结果失败: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/ai_analysis")
async def generate_ai_analysis(request: AnalysisRequest):
    """生成AI分析报告"""
    try:
        # 获取点云渲染图片
        pointcloud_image = await generate_pointcloud_render(request.session_id)
        
        # 调用Gemini API分析
        analysis_result = await gemini_client.analyze_building_defects(
            pointcloud_image, 
            request.analysis_type
        )
        
        # 保存AI分析结果
        ai_result_file = RESULTS_DIR / f"{request.session_id}_ai_analysis.json"
        with open(ai_result_file, 'w', encoding='utf-8') as f:
            json.dump(analysis_result, f, ensure_ascii=False, indent=2)
        
        return analysis_result
        
    except Exception as e:
        logger.error(f"AI分析失败: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/map_data/{session_id}")
async def get_map_data(session_id: str):
    """获取地图数据"""
    try:
        # 读取GPS坐标文件
        gps_file = ROSBAG_DIR / session_id / "gps_coordinates.txt"
        if not gps_file.exists():
            raise HTTPException(status_code=404, detail="GPS坐标文件不存在")
        
        with open(gps_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            if len(lines) >= 2:
                lat1, lon1 = map(float, lines[0].strip().split(','))
                lat2, lon2 = map(float, lines[1].strip().split(','))
                
                map_data = {
                    "center": {"lat": (lat1 + lat2) / 2, "lng": (lon1 + lon2) / 2},
                    "bounds": {
                        "southwest": {"lat": min(lat1, lat2), "lng": min(lon1, lon2)},
                        "northeast": {"lat": max(lat1, lat2), "lng": max(lon1, lon2)}
                    }
                }
                
                return map_data
        
        raise HTTPException(status_code=400, detail="GPS坐标格式错误")
        
    except Exception as e:
        logger.error(f"获取地图数据失败: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

async def generate_review_images(session_id: str):
    """生成复检图像"""
    try:
        # 从处理结果中筛选出最严重的10张图像
        processor = RosbagProcessor()
        review_images = await processor.generate_review_images(session_id, top_n=10)
        
        # 保存复检图像信息
        review_file = RESULTS_DIR / f"{session_id}_review_images.json"
        with open(review_file, 'w', encoding='utf-8') as f:
            json.dump(review_images, f, ensure_ascii=False, indent=2)
        
    except Exception as e:
        logger.error(f"生成复检图像失败: {str(e)}")
        raise

async def generate_pointcloud_render(session_id: str):
    """生成点云渲染图片用于AI分析"""
    try:
        analyzer = PointCloudAnalyzer()
        pointcloud_file = POINTCLOUDS_DIR / f"{session_id}_pointcloud.pcd"
        
        if pointcloud_file.exists():
            render_image = analyzer.render_pointcloud_image(str(pointcloud_file))
            
            # 保存渲染图片
            render_file = IMAGES_DIR / f"{session_id}_render.png"
            render_image.save(render_file)
            
            return str(render_file)
        
        return None
        
    except Exception as e:
        logger.error(f"生成点云渲染图片失败: {str(e)}")
        return None

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
