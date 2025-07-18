# Urban Construction Eagle Eye - Ubuntu部署版本主程序

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
import platform
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
from pointcloud_analyzer import PointCloudAnalyzer
from gemini_client import GeminiClient
from amap_client import AMapClient

# 导入文件系统数据处理器
from filesystem_data_processor import FileSystemDataProcessor

# 创建FastAPI应用
app = FastAPI(
    title="Urban Construction Eagle Eye",
    description="城市建设鹰眼系统 - 智能建筑缺陷检测 (Ubuntu版)",
    version="1.2.0",
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
try:
    app.mount("/static", StaticFiles(directory="static"), name="static")
    templates = Jinja2Templates(directory="templates")
except Exception as e:
    logger.warning(f"静态文件或模板目录配置失败: {str(e)}")
    templates = None

# 全局变量
ssh_client = None
data_processor = None
pointcloud_analyzer = None
gemini_client = None
amap_client = None

# Pydantic模型
class ConnectionRequest(BaseModel):
    ip_address: str = Field(..., description="Ubuntu系统IP地址")
    username: str = Field(default="user", description="SSH用户名")
    password: str = Field(..., description="SSH密码")

class ProcessRequest(BaseModel):
    session_id: str = Field(..., description="会话ID")
    data_path: str = Field(default="./data", description="数据目录路径")

class AnalysisRequest(BaseModel):
    session_id: str = Field(..., description="会话ID")
    analysis_type: str = Field(default="defect", description="分析类型")
    image_ids: List[int] = Field(default=[], description="图像ID列表")

# 依赖注入
def get_ssh_client():
    """获取SSH客户端"""
    global ssh_client
    if not ssh_client:
        # 在Ubuntu版本中，SSH客户端是可选的
        logger.info("SSH客户端未配置，使用本地模式")
        return None
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
    global rosbag_processor, pointcloud_analyzer, gemini_client, amap_client
    
    logger.info("正在启动Urban Construction Eagle Eye系统 (Ubuntu版)...")
    
    try:
        # 检查系统环境
        system_info = {
            'os': platform.system(),
            'platform': platform.platform(),
            'python_version': platform.python_version(),
            'architecture': platform.architecture()
        }
        logger.info(f"系统信息: {system_info}")
        
        # 检查数据目录
        data_dirs_ready = check_data_directories()
        logger.info(f"数据目录: {'就绪' if data_dirs_ready else '未就绪'}")
        
        # 验证配置
        if not config.validate_config():
            logger.warning("配置验证失败，某些功能可能无法正常工作")
        
        # 初始化处理器
        data_processor = FileSystemDataProcessor(config.get_all())
        pointcloud_analyzer = PointCloudAnalyzer()
        
        # 初始化API客户端
        gemini_api_key = config.get('GEMINI_API_KEY')
        if gemini_api_key:
            gemini_client = GeminiClient(gemini_api_key)
        else:
            logger.warning("未配置Gemini API密钥")
        
        amap_api_key = config.get('AMAP_API_KEY')
        amap_security_code = config.get('AMAP_SECURITY_CODE')
        if amap_api_key:
            amap_client = AMapClient(amap_api_key, amap_security_code)
        else:
            logger.warning("未配置高德地图API密钥")
        
        # 清理旧数据
        data_manager.cleanup_old_sessions()
        
        # 创建默认会话用于本地处理
        default_session = create_local_session()
        logger.info(f"创建本地会话: {default_session}")
        
        logger.info("系统启动完成")
        
    except Exception as e:
        logger.error(f"系统启动失败: {str(e)}")
        # 不抛出异常，让系统继续启动

def check_data_directories() -> bool:
    """检查数据目录是否准备就绪"""
    try:
        required_dirs = [
            config.get('image_dir', './data/images'),
            config.get('mask_dir', './data/masks'),
            config.get('pointcloud_dir', './data/pointclouds'),
            config.get('metadata_dir', './data/metadata')
        ]
        
        for dir_path in required_dirs:
            Path(dir_path).mkdir(parents=True, exist_ok=True)
            logger.info(f"数据目录检查: {dir_path}")
        
        return True
    except Exception as e:
        logger.error(f"数据目录检查失败: {str(e)}")
        return False

def create_local_session() -> str:
    """创建本地处理会话"""
    try:
        # 获取本机IP地址
        import socket
        hostname = socket.gethostname()
        local_ip = socket.gethostbyname(hostname)
        
        # 创建会话
        session_id = data_manager.create_session(local_ip)
        return session_id
        
    except Exception as e:
        logger.error(f"创建本地会话失败: {str(e)}")
        return "local_session_default"

@app.on_event("shutdown")
async def shutdown_event():
    """应用关闭事件"""
    logger.info("正在关闭Urban Construction Eagle Eye系统...")
    
    global ssh_client
    if ssh_client:
        try:
            ssh_client.disconnect()
        except:
            pass
    
    logger.info("系统关闭完成")

# 路由处理
@app.get("/")
async def index(request: Request):
    """主页"""
    if templates:
        return templates.TemplateResponse("index.html", {"request": request})
    else:
        return JSONResponse({
            "message": "Urban Construction Eagle Eye System (Ubuntu版)",
            "version": "1.2.0",
            "status": "running",
            "api_docs": "/api/docs",
            "platform": platform.system()
        })

@app.get("/api/health")
@log_api_call("/api/health", "GET")
async def health_check():
    """健康检查"""
    health_info = get_system_health()
    
    # 添加Ubuntu特有的健康检查
    health_info['data_dirs_ready'] = check_data_directories()
    health_info['platform'] = platform.system()
    health_info['rosbag_processor'] = rosbag_processor.__class__.__name__
    
    return get_success_response(health_info)

@app.get("/api/system/info")
@log_api_call("/api/system/info", "GET")
async def get_system_info():
    """获取系统信息"""
    try:
        import psutil
        
        info = {
            'platform': platform.platform(),
            'processor': platform.processor(),
            'memory': {
                'total': psutil.virtual_memory().total,
                'available': psutil.virtual_memory().available,
                'percent': psutil.virtual_memory().percent
            },
            'disk': {
                'total': psutil.disk_usage('/').total,
                'used': psutil.disk_usage('/').used,
                'free': psutil.disk_usage('/').free
            },
            'data_dirs_ready': check_data_directories(),
            'rosbag_processor': rosbag_processor.__class__.__name__
        }
        
        return get_success_response(info, "获取系统信息成功")
        
    except Exception as e:
        raise APIError(f"获取系统信息失败: {str(e)}")

@app.post("/api/connect")
@log_api_call("/api/connect", "POST")
@handle_errors
async def connect_to_system(request: ConnectionRequest):
    """连接到Ubuntu系统（或创建本地会话）"""
    global ssh_client
    
    try:
        # 检查是否是本地连接
        if request.ip_address in ['localhost', '127.0.0.1', '0.0.0.0']:
            # 本地连接模式
            session_id = create_local_session()
            
            return get_success_response({
                'session_id': session_id,
                'ip_address': request.ip_address,
                'connection_status': 'local',
                'mode': 'local'
            }, "本地连接成功")
        
        # 远程SSH连接
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
            'connection_status': 'connected',
            'mode': 'remote'
        }, "连接成功")
        
    except Exception as e:
        error_collector.add_error(ConnectionError(f"连接失败: {str(e)}", request.ip_address))
        raise ConnectionError(f"连接失败: {str(e)}", request.ip_address)

@app.post("/api/process")
@log_api_call("/api/process", "POST")
@handle_errors
async def process_data(
    request: ProcessRequest,
    background_tasks: BackgroundTasks
):
    """处理数据"""
    try:
        # 验证会话
        session = data_manager.get_session(request.session_id)
        if not session:
            raise HTTPException(status_code=404, detail="会话不存在")
        
        # 启动数据监控
        if data_processor:
            success = data_processor.start_monitoring(request.session_id)
            if not success:
                raise DataProcessingError("启动数据监控失败", request.session_id)
        
        # 启动后台处理任务
        background_tasks.add_task(
            process_data_background,
            request.session_id,
            request.data_path
        )
        
        return get_success_response({
            'session_id': request.session_id,
            'status': 'monitoring',
            'data_path': request.data_path
        }, "数据处理已启动")
        
    except Exception as e:
        error_collector.add_error(DataProcessingError(f"处理失败: {str(e)}", request.session_id))
        raise DataProcessingError(f"处理失败: {str(e)}", request.session_id)

@app.get("/api/data")
@log_api_call("/api/data", "GET")
@handle_errors
async def list_data_files(session_id: str, data_type: str = "all"):
    """列出数据文件"""
    try:
        if not data_processor:
            raise APIError("数据处理器未初始化")
        
        # 获取会话状态
        status = data_processor.get_session_status(session_id)
        if status.get('status') == 'not_found':
            raise HTTPException(status_code=404, detail="会话不存在")
        
        # 获取处理的数据
        processed_data = data_processor.get_processed_data(session_id)
        
        # 根据数据类型过滤
        if data_type == "images":
            result = {'images': processed_data.get('images', [])}
        elif data_type == "pointclouds":
            result = {'pointclouds': processed_data.get('pointclouds', [])}
        else:
            result = processed_data
        
        return get_success_response(result, "数据获取成功")
        
    except Exception as e:
        error_collector.add_error(APIError(f"获取数据失败: {str(e)}"))
        raise APIError(f"获取数据失败: {str(e)}")
        file_info = []
        
        for file_path in files:
            info = rosbag_processor.get_rosbag_info(file_path)
            file_info.append(info)
        
        return get_success_response({
            'directory': directory,
            'files': file_info,
            'total_count': len(file_info)
        }, "获取rosbag文件列表成功")
        
    except Exception as e:
        raise APIError(f"获取rosbag文件列表失败: {str(e)}")

# 继续使用之前的其他路由...
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

# 后台任务
@log_performance("data_processing")
async def process_data_background(session_id: str, data_path: str):
    """后台处理数据"""
    try:
        logger.info(f"开始处理数据: {session_id}")
        
        # 持续监控文件变化
        while True:
            try:
                # 扫描新文件
                new_files = data_processor.scan_new_files(session_id)
                
                if any(new_files.values()):
                    logger.info(f"发现新文件: {new_files}")
                    
                    # 处理新图像
                    for image_path in new_files.get('images', []):
                        try:
                            # 查找对应的mask
                            mask_path = None
                            image_name = Path(image_path).stem
                            for mask_path_candidate in new_files.get('masks', []):
                                if Path(mask_path_candidate).stem == image_name:
                                    mask_path = mask_path_candidate
                                    break
                            
                            # 处理图像对
                            result = data_processor.process_image_pair(image_path, mask_path)
                            
                            # 保存处理结果
                            data_manager.save_processed_data(
                                session_id, 
                                f"image_{image_name}.json", 
                                result, 
                                "processed"
                            )
                            
                            logger.info(f"处理图像完成: {image_path}")
                            
                        except Exception as e:
                            logger.error(f"处理图像失败 {image_path}: {str(e)}")
                    
                    # 处理新点云
                    for pcd_path in new_files.get('pointclouds', []):
                        try:
                            result = data_processor.process_pointcloud(pcd_path)
                            
                            # 分析点云
                            if pointcloud_analyzer:
                                analysis_result = await asyncio.get_event_loop().run_in_executor(
                                    None, pointcloud_analyzer.analyze_pointcloud, pcd_path
                                )
                                result.update(analysis_result)
                            
                            # 保存处理结果
                            pcd_name = Path(pcd_path).stem
                            data_manager.save_processed_data(
                                session_id, 
                                f"pointcloud_{pcd_name}.json", 
                                result, 
                                "processed"
                            )
                            
                            logger.info(f"处理点云完成: {pcd_path}")
                            
                        except Exception as e:
                            logger.error(f"处理点云失败 {pcd_path}: {str(e)}")
                
                # 检查会话状态
                status = data_processor.get_session_status(session_id)
                if status.get('status') == 'stopped':
                    logger.info(f"会话 {session_id} 已停止监控")
                    break
                
                # 等待下一次扫描
                await asyncio.sleep(data_processor.monitor_interval)
                
            except Exception as e:
                logger.error(f"数据处理循环出错: {str(e)}")
                await asyncio.sleep(5)  # 出错后等待5秒再重试
        
        logger.info(f"数据处理完成: {session_id}")
        
    except Exception as e:
        logger.error(f"后台数据处理失败: {str(e)}")
        # 标记会话为错误状态
        data_manager.update_session_status(session_id, "error", str(e))
                        analysis_result = await asyncio.get_event_loop().run_in_executor(
                            None, pointcloud_analyzer.analyze_pointcloud, pointcloud
                        )
                        
                        # 保存结果
                        pointcloud_id = data_manager.save_pointcloud(
                            session_id, 
                            os.path.basename(rosbag_file), 
                            {
                                'point_count': pointcloud.get('point_count', 0),
                                'file_size': os.path.getsize(rosbag_file),
                                'defect_severity': analysis_result.get('severity', 0),
                                'timestamp': pointcloud.get('timestamp')
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
                
                total_processed += 1
                logger.info(f"处理完成: {rosbag_file}")
                
            except Exception as e:
                logger.error(f"处理rosbag文件失败: {rosbag_file} - {str(e)}")
                continue
        
        # 保存处理结果
        result_data = {
            'session_id': session_id,
            'processed_files': total_processed,
            'total_files': len(rosbag_files),
            'processing_time': '完成',
            'status': 'completed'
        }
        
        result_file = data_manager.get_file_path(session_id, "pointcloud_data.json", "pointcloud")
        result_file.parent.mkdir(parents=True, exist_ok=True)
        
        import json
        with open(result_file, 'w', encoding='utf-8') as f:
            json.dump(result_data, f, ensure_ascii=False, indent=2)
        
        # 更新会话状态
        data_manager.update_session_status(session_id, 'processed')
        
        logger.info(f"rosbag数据处理完成: {session_id}")
        
    except Exception as e:
        logger.error(f"后台处理失败: {str(e)}")
        data_manager.update_session_status(session_id, 'error')

# 复用之前的其他路由和异常处理器...
@app.get("/api/map/config")
@log_api_call("/api/map/config", "GET")
@handle_errors
async def get_map_config():
    """获取地图配置"""
    try:
        if not amap_client:
            raise APIError("高德地图API未配置")
        
        return get_success_response({
            'api_key': config.get('AMAP_API_KEY'),
            'security_code': config.get('AMAP_SECURITY_CODE'),
            'js_config': amap_client.create_js_api_config()
        }, "获取地图配置成功")
        
    except Exception as e:
        raise APIError(f"获取地图配置失败: {str(e)}", "amap")

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
