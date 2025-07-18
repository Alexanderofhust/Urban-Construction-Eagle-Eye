"""
SSH客户端
功能：
1. 连接到Ubuntu下位机
2. 下载rosbag文件和GPS坐标文件
3. 文件传输进度管理
"""

import paramiko
import os
import logging
from pathlib import Path
from typing import List, Optional
import time
from scp import SCPClient

logger = logging.getLogger(__name__)

class SSHClient:
    """SSH客户端用于连接下位机"""
    
    def __init__(self):
        self.ssh_client = None
        self.scp_client = None
        self.connected = False
    
    def connect(self, host: str, username: str, password: str, port: int = 22) -> bool:
        """连接到SSH服务器"""
        try:
            logger.info(f"连接到 {host}:{port}")
            
            self.ssh_client = paramiko.SSHClient()
            self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            
            self.ssh_client.connect(
                hostname=host,
                port=port,
                username=username,
                password=password,
                timeout=30
            )
            
            self.scp_client = SCPClient(self.ssh_client.get_transport())
            self.connected = True
            
            logger.info("SSH连接成功")
            return True
            
        except Exception as e:
            logger.error(f"SSH连接失败: {str(e)}")
            self.connected = False
            return False
    
    def disconnect(self):
        """断开SSH连接"""
        try:
            if self.scp_client:
                self.scp_client.close()
            if self.ssh_client:
                self.ssh_client.close()
            self.connected = False
            logger.info("SSH连接已断开")
            
        except Exception as e:
            logger.error(f"断开SSH连接失败: {str(e)}")
    
    def list_rosbags(self, remote_path: str) -> List[str]:
        """列出远程rosbag文件"""
        try:
            if not self.connected:
                raise Exception("SSH未连接")
            
            # 执行远程命令列出rosbag文件
            stdin, stdout, stderr = self.ssh_client.exec_command(f"find {remote_path} -name '*.db3' -o -name '*.bag'")
            
            rosbag_files = []
            for line in stdout.readlines():
                file_path = line.strip()
                if file_path:
                    rosbag_files.append(file_path)
            
            logger.info(f"找到 {len(rosbag_files)} 个rosbag文件")
            return rosbag_files
            
        except Exception as e:
            logger.error(f"列出rosbag文件失败: {str(e)}")
            return []
    
    def download_rosbags(self, remote_path: str, local_path: str) -> List[str]:
        """下载rosbag文件"""
        try:
            if not self.connected:
                raise Exception("SSH未连接")
            
            logger.info(f"开始下载rosbag文件从 {remote_path} 到 {local_path}")
            
            # 确保本地目录存在
            Path(local_path).mkdir(parents=True, exist_ok=True)
            
            # 获取rosbag文件列表
            rosbag_files = self.list_rosbags(remote_path)
            
            if not rosbag_files:
                logger.warning("未找到rosbag文件")
                return []
            
            downloaded_files = []
            
            for remote_file in rosbag_files:
                try:
                    # 构造本地文件路径
                    filename = os.path.basename(remote_file)
                    local_file = os.path.join(local_path, filename)
                    
                    # 下载文件
                    logger.info(f"下载文件: {remote_file}")
                    self.scp_client.get(remote_file, local_file)
                    
                    downloaded_files.append(local_file)
                    logger.info(f"文件下载完成: {local_file}")
                    
                except Exception as e:
                    logger.error(f"下载文件失败 {remote_file}: {str(e)}")
                    continue
            
            # 下载GPS坐标文件
            gps_files = self.download_gps_files(remote_path, local_path)
            
            logger.info(f"共下载 {len(downloaded_files)} 个rosbag文件")
            return downloaded_files
            
        except Exception as e:
            logger.error(f"下载rosbag文件失败: {str(e)}")
            return []
    
    def download_gps_files(self, remote_path: str, local_path: str) -> List[str]:
        """下载GPS坐标文件"""
        try:
            if not self.connected:
                raise Exception("SSH未连接")
            
            logger.info("搜索GPS坐标文件")
            
            # 搜索GPS坐标文件（.txt文件）
            stdin, stdout, stderr = self.ssh_client.exec_command(f"find {remote_path} -name '*.txt' -type f")
            
            gps_files = []
            for line in stdout.readlines():
                file_path = line.strip()
                if file_path and 'gps' in file_path.lower():
                    gps_files.append(file_path)
            
            downloaded_gps_files = []
            
            for remote_file in gps_files:
                try:
                    filename = os.path.basename(remote_file)
                    local_file = os.path.join(local_path, filename)
                    
                    logger.info(f"下载GPS文件: {remote_file}")
                    self.scp_client.get(remote_file, local_file)
                    
                    downloaded_gps_files.append(local_file)
                    logger.info(f"GPS文件下载完成: {local_file}")
                    
                except Exception as e:
                    logger.error(f"下载GPS文件失败 {remote_file}: {str(e)}")
                    continue
            
            # 如果没有找到GPS文件，创建默认的GPS坐标文件
            if not downloaded_gps_files:
                logger.warning("未找到GPS坐标文件，创建默认坐标")
                self.create_default_gps_file(local_path)
            
            return downloaded_gps_files
            
        except Exception as e:
            logger.error(f"下载GPS文件失败: {str(e)}")
            return []
    
    def create_default_gps_file(self, local_path: str):
        """创建默认GPS坐标文件"""
        try:
            # 创建默认GPS坐标（北京地区示例）
            default_gps = "39.9042,116.4074\n39.9142,116.4174\n"
            
            gps_file = os.path.join(local_path, "gps_coordinates.txt")
            with open(gps_file, 'w', encoding='utf-8') as f:
                f.write(default_gps)
            
            logger.info(f"创建默认GPS坐标文件: {gps_file}")
            
        except Exception as e:
            logger.error(f"创建默认GPS文件失败: {str(e)}")
    
    def execute_command(self, command: str) -> tuple:
        """执行远程命令"""
        try:
            if not self.connected:
                raise Exception("SSH未连接")
            
            stdin, stdout, stderr = self.ssh_client.exec_command(command)
            
            output = stdout.read().decode('utf-8')
            error = stderr.read().decode('utf-8')
            
            return output, error
            
        except Exception as e:
            logger.error(f"执行远程命令失败: {str(e)}")
            return "", str(e)
    
    def get_system_info(self) -> dict:
        """获取系统信息"""
        try:
            if not self.connected:
                raise Exception("SSH未连接")
            
            # 获取系统信息
            commands = {
                'hostname': 'hostname',
                'os': 'cat /etc/os-release | grep PRETTY_NAME | cut -d= -f2 | tr -d \'"\'',
                'kernel': 'uname -r',
                'uptime': 'uptime -p',
                'disk_usage': 'df -h /',
                'memory': 'free -h',
                'ros_version': 'echo $ROS_DISTRO'
            }
            
            system_info = {}
            
            for key, command in commands.items():
                try:
                    output, error = self.execute_command(command)
                    if not error:
                        system_info[key] = output.strip()
                    else:
                        system_info[key] = "Unknown"
                except:
                    system_info[key] = "Unknown"
            
            return system_info
            
        except Exception as e:
            logger.error(f"获取系统信息失败: {str(e)}")
            return {}
    
    def check_rosbag_size(self, remote_path: str) -> dict:
        """检查rosbag文件大小"""
        try:
            if not self.connected:
                raise Exception("SSH未连接")
            
            # 获取文件大小信息
            output, error = self.execute_command(f"du -sh {remote_path}/*")
            
            if error:
                logger.error(f"检查文件大小失败: {error}")
                return {}
            
            file_sizes = {}
            for line in output.split('\n'):
                if line.strip():
                    parts = line.split('\t')
                    if len(parts) == 2:
                        size, path = parts
                        filename = os.path.basename(path)
                        file_sizes[filename] = size
            
            return file_sizes
            
        except Exception as e:
            logger.error(f"检查rosbag大小失败: {str(e)}")
            return {}
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
