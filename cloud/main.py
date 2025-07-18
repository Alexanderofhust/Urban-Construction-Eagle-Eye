import os
import sys
import base64
from datetime import datetime
from fastapi import FastAPI, HTTPException, Request, Form, File, UploadFile
from fastapi.responses import HTMLResponse, FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import json
import asyncio
from typing import Optional, List
import logging
import uvicorn
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import custom modules
from config import Config
from services.ssh_client import SSHClient
from services.pcd_processor import PCDProcessor
from services.gemini_client import GeminiClient
from services.data_manager import DataManager
from utils.image_utils import ImageAnalyzer

# Load configuration
Config.load_from_env()

# 确保日志目录存在
os.makedirs(Config.LOGS_DIR, exist_ok=True)

# Configure logging
logging.basicConfig(
    level=getattr(logging, Config.LOG_CONFIG["level"]),
    format=Config.LOG_CONFIG["format"],
    handlers=[
        logging.FileHandler(Config.LOG_CONFIG["file"], encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(title="Urban Construction Eagle Eye", description="Building Defect Detection Visualization System")

# Get the directory of the current script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Mount static files with absolute path
static_dir = os.path.join(current_dir, "static")
templates_dir = os.path.join(current_dir, "templates")

app.mount("/static", StaticFiles(directory=static_dir), name="static")

# Initialize templates
templates = Jinja2Templates(directory=templates_dir)

# Initialize services
ssh_client = SSHClient()
pcd_processor = PCDProcessor()
gemini_client = GeminiClient()
data_manager = DataManager()
image_analyzer = ImageAnalyzer()

# Global state
app_state = {
    "current_session": None,
    "connection_status": "disconnected",
    "download_progress": 0,
    "analysis_results": None
}

# 页面路由
@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    """主页面"""
    return templates.TemplateResponse("index.html", {
        "request": request,
        "amap_api_key": Config.AMAP_API_KEY,
        "amap_security_key": os.getenv("AMAP_SECURITY_KEY", "your_security_key_here")
    })

@app.get("/review/{session_id}", response_class=HTMLResponse)
async def review_page(request: Request, session_id: str):
    """缺陷复检页面"""
    return templates.TemplateResponse("review.html", {"request": request, "session_id": session_id})

@app.get("/api/status")
async def get_status():
    """获取应用状态"""
    return {
        "status": "running",
        "connection_status": app_state["connection_status"],
        "current_session": app_state["current_session"],
        "download_progress": app_state["download_progress"]
    }

@app.post("/api/connect")
async def connect_to_device(
    host: str = Form(...),
    username: str = Form(default=""),
    password: str = Form(default=""),
    data_path: str = Form(...)
):
    """连接到下位机并下载数据"""
    try:
        app_state["connection_status"] = "connecting"
        
        # 使用配置文件中的SSH信息（如果未提供的话）
        ssh_host = host if host else Config.SSH_CONFIG["hostname"]
        ssh_username = username if username else Config.SSH_CONFIG["username"]
        ssh_password = password if password else Config.SSH_CONFIG["password"]
        ssh_port = Config.SSH_CONFIG["port"]
        ssh_timeout = Config.SSH_CONFIG["timeout"]
        
        logger.info(f"尝试连接SSH: {ssh_username}@{ssh_host}:{ssh_port}")
        
        # 建立SSH连接
        success = await ssh_client.connect(ssh_host, ssh_username, ssh_password, ssh_port, ssh_timeout)
        if not success:
            app_state["connection_status"] = "error"
            error_msg = f"SSH连接失败: 无法连接到 {ssh_username}@{ssh_host}:{ssh_port}"
            logger.error(error_msg)
            raise HTTPException(status_code=400, detail=error_msg)
        
        app_state["connection_status"] = "connected"
        logger.info("SSH连接成功，开始下载数据...")
        
        # 下载数据
        session_id = await ssh_client.download_data(data_path, progress_callback=update_progress)
        app_state["current_session"] = session_id
        app_state["connection_status"] = "completed"
        
        return {"status": "success", "session_id": session_id}
        
    except HTTPException:
        raise
    except Exception as e:
        app_state["connection_status"] = "error"
        error_msg = f"连接过程中发生错误: {str(e)}"
        logger.error(error_msg)
        logger.error(f"连接错误: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

def update_progress(progress: int):
    """更新下载进度"""
    app_state["download_progress"] = progress

@app.get("/api/sessions")
async def get_sessions():
    """获取所有会话"""
    sessions = data_manager.get_all_sessions()
    return {"sessions": sessions}

@app.get("/api/pointcloud/{session_id}")
async def get_pointcloud(session_id: str):
    """获取点云数据"""
    try:
        pointcloud_data = await pcd_processor.process_pointcloud(session_id)
        return {"pointcloud": pointcloud_data}
    except Exception as e:
        logger.error(f"点云处理错误: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/location/{session_id}")
async def get_location(session_id: str):
    """获取GPS位置数据"""
    try:
        location_data = data_manager.get_location_data(session_id)
        return {"location": location_data}
    except Exception as e:
        logger.error(f"位置数据获取错误: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/defects/{session_id}")
async def get_defects(session_id: str):
    """获取缺陷数据（前10个最严重的）"""
    try:
        defects = await image_analyzer.analyze_defects(session_id)
        # 按严重程度排序，取前10个
        top_defects = sorted(defects, key=lambda x: x['severity_score'], reverse=True)[:10]
        return {"defects": top_defects}
    except Exception as e:
        logger.error(f"缺陷分析错误: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/review/{session_id}/{defect_id}")
async def review_defect(session_id: str, defect_id: str, approved: bool = Form(...)):
    """复检缺陷"""
    try:
        result = data_manager.save_review_result(session_id, defect_id, approved)
        return {"status": "success", "result": result}
    except Exception as e:
        logger.error(f"复检保存错误: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/analyze/{session_id}")
async def analyze_with_ai(session_id: str):
    """使用AI分析建筑"""
    try:
        # 获取最严重的2张缺陷图像
        defects = await image_analyzer.analyze_defects(session_id)
        top_defects = sorted(defects, key=lambda x: x.get('severity_score', 0), reverse=True)[:2]
        
        logger.info(f"AI分析 - 会话: {session_id}, 总缺陷: {len(defects)}, 选择前2严重: {len(top_defects)}")
        
        # 准备缺陷图像的base64编码
        defect_images = []
        for defect in top_defects:
            image_path = data_manager.get_image_path(session_id, 'images', defect['image_file'])
            with open(image_path, 'rb') as f:
                image_data = f.read()
                defect_images.append(base64.b64encode(image_data).decode('utf-8'))
                logger.info(f"AI分析 - 加载缺陷图像: {defect['image_file']}, 大小: {len(image_data)} bytes")
        
        # 生成点云渲染图
        pointcloud_b64 = await pcd_processor.generate_render_image(session_id)
        logger.info(f"AI分析 - 点云渲染图生成完成, 大小: {len(pointcloud_b64)} chars")
        
        # 准备图像数据
        images = {
            'pointcloud': pointcloud_b64,
            'defect_images': defect_images
        }
        
        logger.info(f"AI分析 - 开始调用Gemini API...")
        
        # 调用Gemini分析
        analysis_result = await gemini_client.analyze_building(images, session_id)
        
        # DEBUG: 输出原始返回数据
        logger.info("="*50)
        logger.info("AI分析原始返回数据:")
        logger.info(f"类型: {type(analysis_result)}")
        logger.info(f"内容: {analysis_result}")
        logger.info("="*50)
        
        # 保存分析结果
        app_state["analysis_results"] = analysis_result
        data_manager.save_analysis_result(session_id, analysis_result)
        
        return {"status": "success", "analysis": analysis_result, "debug_info": {
            "total_defects": len(defects),
            "selected_defects": len(top_defects),
            "images_processed": len(defect_images) + 1,
            "analysis_type": type(analysis_result).__name__
        }}
        
    except Exception as e:
        logger.error(f"AI分析错误: {str(e)}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/3d-demo")
async def show_3d_demo():
    """显示3D地图演示页面"""
    try:
        return FileResponse("templates/3d_map_demo.html")
    except Exception as e:
        logger.error(f"3D演示页面错误: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/demo/location")
async def get_demo_location():
    """获取演示位置数据"""
    try:
        # 创建演示数据 - 模拟真实建筑物轮廓
        demo_data = {
            "center": {"lat": 39.90923, "lon": 116.397428},
            "bounds": {
                "southwest": {"lat": 39.90800, "lon": 116.39600},
                "northeast": {"lat": 39.91046, "lon": 116.39886}
            },
            "points": [
                {"lat": 39.90850, "lon": 116.39650},  # 建筑物角点1
                {"lat": 39.90950, "lon": 116.39680},  # 建筑物角点2
                {"lat": 39.90980, "lon": 116.39780},  # 建筑物角点3
                {"lat": 39.90920, "lon": 116.39820},  # 建筑物角点4
                {"lat": 39.90860, "lon": 116.39790},  # 建筑物角点5
                {"lat": 39.90830, "lon": 116.39720},  # 建筑物角点6
                {"lat": 39.90850, "lon": 116.39650}   # 闭合到起点
            ],
            "total_points": 7,
            "building_name": "示例建筑物",
            "detection_status": "活跃",
            "last_updated": datetime.now().isoformat()
        }
        
        return {"location": demo_data}
        
    except Exception as e:
        logger.error(f"演示位置数据获取错误: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))
    """获取统计数据"""
    try:
        # 获取基础统计数据
        stats = data_manager.get_statistics(session_id)
        
        # 获取缺陷详细数据
        defects = await image_analyzer.analyze_defects(session_id)
        
        # 统计各类缺陷数量（根据文件名解析类别）
        defect_class_counts = {}
        class_severity_stats = {}
        
        for defect in defects:
            image_file = defect.get('image_file', '')
            
            # 从文件名解析类别 (例如: cls01_001.jpg -> cls01)
            if image_file.startswith('cls'):
                class_name = image_file.split('_')[0]  # 提取cls01, cls03, cls05等
                
                if class_name not in defect_class_counts:
                    defect_class_counts[class_name] = 0
                    class_severity_stats[class_name] = {
                        'total': 0,
                        'high_confidence': 0,
                        'avg_confidence': 0
                    }
                
                defect_class_counts[class_name] += 1
                class_severity_stats[class_name]['total'] += 1
                
                # 统计高置信度缺陷
                confidence = defect.get('confidence', 0)
                if confidence > 0.8:
                    class_severity_stats[class_name]['high_confidence'] += 1
        
        # 计算平均置信度
        for class_name in class_severity_stats:
            class_defects = [d for d in defects if d.get('image_file', '').startswith(class_name)]
            if class_defects:
                avg_conf = sum(d.get('confidence', 0) for d in class_defects) / len(class_defects)
                class_severity_stats[class_name]['avg_confidence'] = round(avg_conf, 3)
        
        # 重新计算统计数据
        stats.update({
            "total_defects": len(defects),
            "defect_class_counts": defect_class_counts,
            "class_severity_stats": class_severity_stats,
            "defect_distribution": {
                "cls01": defect_class_counts.get('cls01', 0),  # 类别1缺陷
                "cls03": defect_class_counts.get('cls03', 0),  # 类别3缺陷  
                "cls05": defect_class_counts.get('cls05', 0),  # 类别5缺陷
                "其他": sum(v for k, v in defect_class_counts.items() if k not in ['cls01', 'cls03', 'cls05'])
            },
            "confidence_distribution": {
                "高置信度(>0.8)": sum(1 for d in defects if d.get('confidence', 0) > 0.8),
                "中置信度(0.5-0.8)": sum(1 for d in defects if 0.5 < d.get('confidence', 0) <= 0.8),
                "低置信度(≤0.5)": sum(1 for d in defects if d.get('confidence', 0) <= 0.5)
            }
        })
        
        # 获取复检完成率
        total_defects = len(defects)
        reviewed_count = stats.get("reviewed_count", 0)
        stats["review_progress"] = {
            "total": total_defects,
            "reviewed": reviewed_count,
            "completion_rate": (reviewed_count / total_defects * 100) if total_defects > 0 else 0
        }
        
        # 添加时间统计
        import datetime
        stats["generated_time"] = datetime.datetime.now().isoformat()
        
        logger.info(f"统计数据生成完成 - 总缺陷: {len(defects)}, 类别分布: {defect_class_counts}")
        
        return {"statistics": stats}
        
    except Exception as e:
        logger.error(f"统计数据获取错误: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/image/{session_id}/{image_type}/{filename}")
async def get_image(session_id: str, image_type: str, filename: str):
    """获取图像文件"""
    try:
        image_path = data_manager.get_image_path(session_id, image_type, filename)
        return FileResponse(image_path)
    except Exception as e:
        logger.error(f"图像获取错误: {str(e)}")
        raise HTTPException(status_code=404, detail="图像未找到")

if __name__ == "__main__":
    try:
        # 创建必要的目录
        os.makedirs(Config.DATA_DIR, exist_ok=True)
        os.makedirs(Config.SESSIONS_DIR, exist_ok=True)
        os.makedirs(Config.RESULTS_DIR, exist_ok=True)
        os.makedirs(Config.LOGS_DIR, exist_ok=True)
        
        # 验证配置
        config_errors = Config.validate_config()
        if config_errors:
            print("配置警告:")
            for error in config_errors:
                print(f"  - {error}")
            print()
        
        # 打印配置信息
        print("========================================")
        print("         城市建设鹰眼系统启动")
        print("========================================")
        Config.print_config()
        print("========================================")
        
        # 启动服务器
        print(f"正在启动服务器 http://{Config.HOST}:{Config.PORT}")
        print("按 Ctrl+C 停止服务")
        print("========================================")
        
        # 修复reload问题：当直接运行main.py时，不能使用reload
        if Config.DEBUG:
            # 在调试模式下，不使用reload以避免导入错误
            uvicorn.run(
                app, 
                host=Config.HOST, 
                port=Config.PORT, 
                reload=False,
                log_level="info"
            )
        else:
            uvicorn.run(
                app, 
                host=Config.HOST, 
                port=Config.PORT, 
                reload=False,
                log_level="info"
            )
        
    except KeyboardInterrupt:
        print("\n服务器已停止")
    except Exception as e:
        print(f"启动失败: {str(e)}")
        import traceback
        traceback.print_exc()
        input("按Enter键退出...")
        sys.exit(1)
