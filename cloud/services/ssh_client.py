import paramiko
import os
import asyncio
from typing import Optional, Callable
import logging
import uuid
from datetime import datetime
import zipfile
import shutil

logger = logging.getLogger(__name__)

class SSHClient:
    def __init__(self):
        self.client = None
        self.sftp = None
        self.connected = False
        
    async def connect(self, host: str, username: str, password: str, port: int = 22, timeout: int = 30) -> bool:
        """连接到SSH服务器"""
        try:
            logger.info(f"正在连接SSH服务器: {username}@{host}:{port}")
            
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            
            # 创建连接函数
            def connect_ssh():
                self.client.connect(
                    hostname=host,
                    port=port,
                    username=username,
                    password=password,
                    timeout=timeout,
                    allow_agent=False,
                    look_for_keys=False
                )
            
            # 在线程池中执行连接
            await asyncio.get_event_loop().run_in_executor(None, connect_ssh)
            
            logger.info("SSH连接成功，正在建立SFTP连接...")
            self.sftp = self.client.open_sftp()
            self.connected = True
            logger.info(f"SSH和SFTP连接成功: {username}@{host}:{port}")
            return True
            
        except paramiko.AuthenticationException as e:
            logger.error(f"SSH身份验证失败: 用户名或密码错误 - {str(e)}")
            return False
        except paramiko.SSHException as e:
            logger.error(f"SSH连接错误: {str(e)}")
            return False
        except Exception as e:
            logger.error(f"SSH连接失败: {str(e)}")
            return False
    
    async def download_data(self, remote_path: str, progress_callback: Optional[Callable] = None) -> str:
        """下载数据并返回session_id"""
        if not self.connected:
            raise Exception("SSH not connected")
        
        # 生成唯一的session_id
        session_id = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}"
        local_base_path = os.path.join("data", "sessions", session_id)
        os.makedirs(local_base_path, exist_ok=True)
        
        try:
            # 获取远程文件列表
            remote_files = []
            
            # 递归获取所有文件
            def get_files_recursive(path):
                try:
                    for item in self.sftp.listdir_attr(path):
                        item_path = os.path.join(path, item.filename).replace('\\', '/')
                        if item.st_mode & 0o040000:  # 是目录
                            get_files_recursive(item_path)
                        else:
                            remote_files.append(item_path)
                except Exception as e:
                    logger.error(f"Error listing directory {path}: {str(e)}")
            
            get_files_recursive(remote_path)
            
            # 下载文件
            total_files = len(remote_files)
            downloaded = 0
            
            for remote_file in remote_files:
                try:
                    # 计算相对路径
                    rel_path = os.path.relpath(remote_file, remote_path)
                    local_file = os.path.join(local_base_path, rel_path)
                    
                    # 创建本地目录
                    os.makedirs(os.path.dirname(local_file), exist_ok=True)
                    
                    # 下载文件
                    await asyncio.get_event_loop().run_in_executor(
                        None,
                        self.sftp.get,
                        remote_file,
                        local_file
                    )
                    
                    downloaded += 1
                    progress = int((downloaded / total_files) * 100)
                    
                    if progress_callback:
                        progress_callback(progress)
                    
                    logger.info(f"Downloaded: {rel_path} ({downloaded}/{total_files})")
                    
                except Exception as e:
                    logger.error(f"Error downloading {remote_file}: {str(e)}")
                    continue
            
            logger.info(f"Download completed for session: {session_id}")
            return session_id
            
        except Exception as e:
            logger.error(f"Download failed: {str(e)}")
            raise
    
    def disconnect(self):
        """断开SSH连接"""
        if self.sftp:
            self.sftp.close()
        if self.client:
            self.client.close()
        self.connected = False
        logger.info("SSH connection closed")
