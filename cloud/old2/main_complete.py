# Urban Construction Eagle Eye - 完整的主程序

import uvicorn
from fastapi import FastAPI, HTTPException, Request, Depends, BackgroundTasks
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import Dict, List, Optional, Any
import asyncio
import os
from pathlib import Path
import logging

# 导入自定义模块
from config import config
from data_manager import data_manager
from error_handler import (
    logger, handle_errors, log_api_call, log_performance,
    EagleEyeError, ConnectionError, DataProcessingError, APIError,
    get_error_response, get_success_response, error_collector,
    get_system_health
)
from ssh_client import SSHClient
from rosbag_processor import RosbagProcessor
from pointcloud_analyzer import PointCloudAnalyzer
from gemini_client import GeminiClient
from baidu_map_client import BaiduMapClient

# 创建FastAPI应用
app = FastAPI(
    title="Urban Construction Eagle Eye",
    description="城市建设鹰眼系统 - 智能建筑缺陷检测",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc"
)

# 配置CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 配置静态文件和模板
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")

# 全局变量
ssh_client = None
rosbag_processor = None
pointcloud_analyzer = None
gemini_client = None
baidu_map_client = None

# Pydantic模型
class ConnectionRequest(BaseModel):
    ip_address: str = Field(..., description="Ubuntu系统IP地址")
    username: str = Field(default="user", description="SSH用户名")
    password: str = Field(..., description="SSH密码")

class ProcessRequest(BaseModel):
    session_id: str = Field(..., description="会话ID")
    rosbag_path: str = Field(default="/home/user/rosbags", description="rosbag文件路径")

class AnalysisRequest(BaseModel):
    session_id: str = Field(..., description="会话ID")
    analysis_type: str = Field(default="defect", description="分析类型")
    image_ids: List[int] = Field(default=[], description="图像ID列表")

# 依赖注入
def get_ssh_client():
    """获取SSH客户端"""
    global ssh_client
    if not ssh_client:
        raise HTTPException(status_code=400, detail="SSH连接未建立")
    return ssh_client

def get_session_id(request: Request) -> str:
    """从请求中获取会话ID"""
    session_id = request.headers.get("X-Session-ID")
    if not session_id:
        raise HTTPException(status_code=400, detail="缺少会话ID")
    return session_id

# 初始化客户端
@app.on_event("startup")
async def startup_event():
    """应用启动事件"""
    global rosbag_processor, pointcloud_analyzer, gemini_client, baidu_map_client
    
    logger.info("正在启动Urban Construction Eagle Eye系统...")
    
    try:
        # 验证配置
        if not config.validate_config():
            logger.warning("配置验证失败，某些功能可能无法正常工作")
        
        # 初始化处理器
        rosbag_processor = RosbagProcessor()
        pointcloud_analyzer = PointCloudAnalyzer()
        
        # 初始化API客户端
        gemini_api_key = config.get('GEMINI_API_KEY')
        if gemini_api_key:
            gemini_client = GeminiClient(gemini_api_key)
        else:
            logger.warning("未配置Gemini API密钥")
        
        baidu_map_ak = config.get('BAIDU_MAP_AK')
        if baidu_map_ak:
            baidu_map_client = BaiduMapClient(baidu_map_ak)
        else:
            logger.warning("未配置百度地图API密钥")
        
        # 清理旧数据
        data_manager.cleanup_old_sessions()
        
        logger.info("系统启动完成")
        
    except Exception as e:
        logger.error(f"系统启动失败: {str(e)}")
        raise

@app.on_event("shutdown")
async def shutdown_event():
    """应用关闭事件"""
    logger.info("正在关闭Urban Construction Eagle Eye系统...")
    
    global ssh_client
    if ssh_client:
        ssh_client.disconnect()
    
    logger.info("系统关闭完成")

# 路由处理
@app.get("/")
async def index(request: Request):
    """主页"""
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/api/health")
@log_api_call("/api/health", "GET")
async def health_check():
    """健康检查"""
    return get_success_response(get_system_health())

@app.post("/api/connect")
@log_api_call("/api/connect", "POST")
@handle_errors
async def connect_to_system(request: ConnectionRequest):
    """连接到Ubuntu系统"""
    global ssh_client
    
    try:
        # 创建SSH连接
        ssh_client = SSHClient(
            hostname=request.ip_address,
            username=request.username,
            password=request.password
        )
        
        # 测试连接
        await asyncio.get_event_loop().run_in_executor(
            None, ssh_client.connect
        )
        
        # 创建会话
        session_id = data_manager.create_session(request.ip_address)
        
        logger.info(f"SSH连接成功，会话ID: {session_id}")
        
        return get_success_response({
            'session_id': session_id,
            'ip_address': request.ip_address,
            'connection_status': 'connected'
        }, "连接成功")
        
    except Exception as e:
        error_collector.add_error(ConnectionError(f"连接失败: {str(e)}", request.ip_address))
        raise ConnectionError(f"连接失败: {str(e)}", request.ip_address)

@app.post("/api/disconnect")
@log_api_call("/api/disconnect", "POST")
@handle_errors
async def disconnect_from_system(session_id: str = Depends(get_session_id)):
    """断开连接"""
    global ssh_client
    
    try:
        if ssh_client:
            ssh_client.disconnect()
            ssh_client = None
        
        # 更新会话状态
        data_manager.update_session_status(session_id, 'disconnected')
        
        return get_success_response(message="断开连接成功")
        
    except Exception as e:
        raise EagleEyeError(f"断开连接失败: {str(e)}")

@app.post("/api/process")
@log_api_call("/api/process", "POST")
@handle_errors
async def process_rosbag_data(
    request: ProcessRequest,
    background_tasks: BackgroundTasks,
    ssh_client: SSHClient = Depends(get_ssh_client)
):
    """处理rosbag数据"""
    try:
        # 验证会话
        session = data_manager.get_session(request.session_id)
        if not session:
            raise HTTPException(status_code=404, detail="会话不存在")
        
        # 启动后台处理任务
        background_tasks.add_task(
            process_rosbag_background,
            request.session_id,
            request.rosbag_path,
            ssh_client
        )
        
        return get_success_response({
            'session_id': request.session_id,
            'status': 'processing',
            'rosbag_path': request.rosbag_path
        }, "数据处理已启动")
        
    except Exception as e:
        error_collector.add_error(DataProcessingError(f"处理失败: {str(e)}", request.rosbag_path))
        raise DataProcessingError(f"处理失败: {str(e)}", request.rosbag_path)

@app.get("/api/pointcloud/{session_id}")
@log_api_call("/api/pointcloud", "GET")
@handle_errors
async def get_pointcloud_data(session_id: str):
    """获取点云数据"""
    try:
        # 获取会话信息
        session = data_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="会话不存在")
        
        # 获取点云数据文件
        pointcloud_file = data_manager.get_file_path(session_id, "pointcloud_data.json", "pointcloud")
        
        if not pointcloud_file.exists():
            return get_success_response({
                'session_id': session_id,
                'status': 'no_data',
                'message': '点云数据尚未生成'
            })
        
        # 读取点云数据
        import json
        with open(pointcloud_file, 'r', encoding='utf-8') as f:
            pointcloud_data = json.load(f)
        
        return get_success_response(pointcloud_data, "获取点云数据成功")
        
    except Exception as e:
        raise DataProcessingError(f"获取点云数据失败: {str(e)}")

@app.get("/api/review/{session_id}")
@log_api_call("/api/review", "GET")
@handle_errors
async def get_review_images(session_id: str, limit: int = 10):
    """获取审核图像"""
    try:
        # 获取会话信息
        session = data_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="会话不存在")
        
        # 获取图像列表
        images = data_manager.get_session_images(session_id, limit)
        
        # 处理图像数据
        processed_images = []
        for image in images:
            image_data = {
                'id': image['id'],
                'filename': image['filename'],
                'defect_severity': image['defect_severity'],
                'defect_area': image['defect_area'],
                'timestamp': image['timestamp'],
                'ai_analysis': image.get('ai_analysis', {})
            }
            
            # 添加图像URL
            image_url = f"/api/image/{session_id}/{image['filename']}"
            image_data['image_url'] = image_url
            image_data['overlay_url'] = f"{image_url}?overlay=true"
            
            processed_images.append(image_data)
        
        return get_success_response({
            'session_id': session_id,
            'images': processed_images,
            'total_count': len(processed_images)
        }, "获取审核图像成功")
        
    except Exception as e:
        raise DataProcessingError(f"获取审核图像失败: {str(e)}")

@app.get("/api/image/{session_id}/{filename}")
@handle_errors
async def get_image(session_id: str, filename: str, overlay: bool = False):
    """获取图像文件"""
    try:
        # 获取图像文件路径
        image_path = data_manager.get_file_path(session_id, filename, "image")
        
        if not image_path.exists():
            raise HTTPException(status_code=404, detail="图像文件不存在")
        
        # 如果需要叠加效果，生成叠加图像
        if overlay:
            overlay_path = data_manager.get_file_path(session_id, f"overlay_{filename}", "image")
            if overlay_path.exists():
                return FileResponse(str(overlay_path))
        
        return FileResponse(str(image_path))
        
    except Exception as e:
        raise DataProcessingError(f"获取图像失败: {str(e)}")

@app.post("/api/analysis")
@log_api_call("/api/analysis", "POST")
@handle_errors
async def ai_analysis(
    request: AnalysisRequest,
    background_tasks: BackgroundTasks
):
    """AI分析"""
    try:
        # 验证会话
        session = data_manager.get_session(request.session_id)
        if not session:
            raise HTTPException(status_code=404, detail="会话不存在")
        
        if not gemini_client:
            raise APIError("Gemini API未配置")
        
        # 启动后台分析任务
        background_tasks.add_task(
            ai_analysis_background,
            request.session_id,
            request.analysis_type,
            request.image_ids
        )
        
        return get_success_response({
            'session_id': request.session_id,
            'analysis_type': request.analysis_type,
            'status': 'analyzing'
        }, "AI分析已启动")
        
    except Exception as e:
        error_collector.add_error(APIError(f"AI分析失败: {str(e)}", "gemini"))
        raise APIError(f"AI分析失败: {str(e)}", "gemini")

@app.get("/api/map/{session_id}")
@log_api_call("/api/map", "GET")
@handle_errors
async def get_map_data(session_id: str):
    """获取地图数据"""
    try:
        # 获取会话信息
        session = data_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="会话不存在")
        
        if not baidu_map_client:
            raise APIError("百度地图API未配置")
        
        # 获取GPS数据
        gps_file = data_manager.get_file_path(session_id, "gps_data.json", "cache")
        
        if not gps_file.exists():
            return get_success_response({
                'session_id': session_id,
                'status': 'no_data',
                'message': 'GPS数据尚未生成'
            })
        
        # 读取GPS数据
        import json
        with open(gps_file, 'r', encoding='utf-8') as f:
            gps_data = json.load(f)
        
        return get_success_response(gps_data, "获取地图数据成功")
        
    except Exception as e:
        raise APIError(f"获取地图数据失败: {str(e)}", "baidu_map")

@app.get("/api/statistics/{session_id}")
@log_api_call("/api/statistics", "GET")
@handle_errors
async def get_statistics(session_id: str):
    """获取统计数据"""
    try:
        # 获取会话信息
        session = data_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="会话不存在")
        
        # 获取统计数据
        stats = data_manager.get_statistics()
        
        return get_success_response(stats, "获取统计数据成功")
        
    except Exception as e:
        raise DataProcessingError(f"获取统计数据失败: {str(e)}")

# 后台任务
@log_performance("rosbag_processing")
async def process_rosbag_background(session_id: str, rosbag_path: str, ssh_client: SSHClient):
    """后台处理rosbag数据"""
    try:
        logger.info(f"开始处理rosbag数据: {session_id}")
        
        # 1. 下载rosbag文件
        rosbag_files = await asyncio.get_event_loop().run_in_executor(
            None, ssh_client.download_rosbags, rosbag_path
        )
        
        logger.info(f"下载了 {len(rosbag_files)} 个rosbag文件")
        
        # 2. 处理每个rosbag文件
        for rosbag_file in rosbag_files:
            try:
                # 解析rosbag
                pointcloud_data = await asyncio.get_event_loop().run_in_executor(
                    None, rosbag_processor.process_rosbag, rosbag_file
                )
                
                # 分析点云
                analysis_result = await asyncio.get_event_loop().run_in_executor(
                    None, pointcloud_analyzer.analyze_pointcloud, pointcloud_data
                )
                
                # 保存结果
                pointcloud_id = data_manager.save_pointcloud(
                    session_id, 
                    os.path.basename(rosbag_file), 
                    {
                        'point_count': len(pointcloud_data.get('points', [])),
                        'file_size': os.path.getsize(rosbag_file),
                        'defect_severity': analysis_result.get('severity', 0),
                        'timestamp': pointcloud_data.get('timestamp')
                    }
                )
                
                # 保存图像
                for image_data in analysis_result.get('images', []):
                    data_manager.save_image(
                        session_id, 
                        pointcloud_id, 
                        image_data['filename'],
                        image_data['type'],
                        image_data
                    )
                
                logger.info(f"处理完成: {rosbag_file}")
                
            except Exception as e:
                logger.error(f"处理rosbag文件失败: {rosbag_file} - {str(e)}")
                continue
        
        # 3. 更新会话状态
        data_manager.update_session_status(session_id, 'processed')
        
        logger.info(f"rosbag数据处理完成: {session_id}")
        
    except Exception as e:
        logger.error(f"后台处理失败: {str(e)}")
        data_manager.update_session_status(session_id, 'error')

@log_performance("ai_analysis")
async def ai_analysis_background(session_id: str, analysis_type: str, image_ids: List[int]):
    """后台AI分析"""
    try:
        logger.info(f"开始AI分析: {session_id}")
        
        # 获取图像数据
        images = data_manager.get_session_images(session_id, 100)
        
        if image_ids:
            # 过滤指定的图像
            images = [img for img in images if img['id'] in image_ids]
        
        # 分析每个图像
        for image in images:
            try:
                # 读取图像文件
                image_path = data_manager.get_file_path(session_id, image['filename'], "image")
                
                if not image_path.exists():
                    continue
                
                # 调用AI分析
                analysis_result = await asyncio.get_event_loop().run_in_executor(
                    None, 
                    gemini_client.analyze_image_defects, 
                    str(image_path)
                )
                
                # 更新数据库
                # 这里应该更新图像的AI分析结果
                logger.info(f"AI分析完成: {image['filename']}")
                
            except Exception as e:
                logger.error(f"AI分析失败: {image['filename']} - {str(e)}")
                continue
        
        logger.info(f"AI分析完成: {session_id}")
        
    except Exception as e:
        logger.error(f"后台AI分析失败: {str(e)}")

# 异常处理器
@app.exception_handler(EagleEyeError)
async def eagle_eye_error_handler(request: Request, exc: EagleEyeError):
    """自定义异常处理"""
    error_collector.add_error(exc)
    return JSONResponse(
        status_code=400,
        content=get_error_response(exc)
    )

@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """HTTP异常处理"""
    return JSONResponse(
        status_code=exc.status_code,
        content={
            'success': False,
            'error': {
                'error_code': 'HTTP_ERROR',
                'message': exc.detail,
                'status_code': exc.status_code
            }
        }
    )

@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """通用异常处理"""
    logger.error(f"未处理的异常: {str(exc)}")
    error = EagleEyeError(f"服务器内部错误: {str(exc)}", "INTERNAL_ERROR")
    error_collector.add_error(error)
    
    return JSONResponse(
        status_code=500,
        content=get_error_response(error)
    )

# 主函数
if __name__ == "__main__":
    # 获取服务器配置
    server_config = config.get_server_config()
    
    # 启动服务器
    uvicorn.run(
        app,
        host=server_config['host'],
        port=server_config['port'],
        reload=server_config['debug'],
        log_level="info"
    )
