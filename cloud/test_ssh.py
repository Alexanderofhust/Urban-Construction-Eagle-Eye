"""
SSH连接测试脚本
用于诊断SSH连接问题
"""

import asyncio
import logging
import sys
import os
from dotenv import load_dotenv

# 加载环境变量
load_dotenv()

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# 导入配置和SSH客户端
from config import Config
from services.ssh_client import SSHClient

async def test_ssh_connection():
    """测试SSH连接"""
    print("========================================")
    print("           SSH连接测试")
    print("========================================")
    
    # 加载配置
    Config.load_from_env()
    
    # 显示配置信息
    ssh_config = Config.SSH_CONFIG
    print(f"SSH主机: {ssh_config['hostname']}")
    print(f"SSH端口: {ssh_config['port']}")
    print(f"SSH用户名: {ssh_config['username']}")
    print(f"SSH密码: {'*' * len(ssh_config['password'])}")
    print(f"连接超时: {ssh_config['timeout']}秒")
    print()
    
    # 创建SSH客户端
    ssh_client = SSHClient()
    
    try:
        print("正在测试SSH连接...")
        success = await ssh_client.connect(
            ssh_config['hostname'],
            ssh_config['username'], 
            ssh_config['password'],
            ssh_config['port'],
            ssh_config['timeout']
        )
        
        if success:
            print("✅ SSH连接成功!")
            
            # 测试简单命令
            print("\n正在测试远程命令执行...")
            try:
                stdin, stdout, stderr = ssh_client.client.exec_command('pwd')
                output = stdout.read().decode().strip()
                error = stderr.read().decode().strip()
                
                if output:
                    print(f"✅ 远程命令执行成功: {output}")
                if error:
                    print(f"⚠️ 命令错误输出: {error}")
                    
            except Exception as e:
                print(f"❌ 远程命令执行失败: {str(e)}")
            
            # 测试SFTP
            print("\n正在测试SFTP...")
            try:
                sftp = ssh_client.sftp
                remote_files = sftp.listdir('.')
                print(f"✅ SFTP连接成功，远程目录文件数: {len(remote_files)}")
                print(f"远程目录内容: {remote_files[:5]}...")  # 显示前5个文件
                
            except Exception as e:
                print(f"❌ SFTP测试失败: {str(e)}")
                
        else:
            print("❌ SSH连接失败")
            return False
            
    except Exception as e:
        print(f"❌ SSH连接异常: {str(e)}")
        return False
    
    finally:
        # 关闭连接
        if ssh_client.connected:
            ssh_client.disconnect()
            print("\n🔌 SSH连接已关闭")
    
    return success

def test_network_connectivity():
    """测试网络连通性"""
    print("========================================")
    print("           网络连通性测试")
    print("========================================")
    
    import subprocess
    
    Config.load_from_env()
    host = Config.SSH_CONFIG['hostname']
    port = Config.SSH_CONFIG['port']
    
    # 测试ping
    print(f"正在ping {host}...")
    try:
        result = subprocess.run(['ping', '-n', '4', host], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("✅ Ping成功")
        else:
            print("❌ Ping失败")
            print(result.stdout)
    except Exception as e:
        print(f"❌ Ping测试异常: {str(e)}")
    
    # 测试端口连通性
    print(f"\n正在测试端口 {host}:{port}...")
    try:
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex((host, port))
        sock.close()
        
        if result == 0:
            print("✅ 端口连通")
        else:
            print(f"❌ 端口不通，错误代码: {result}")
            
    except Exception as e:
        print(f"❌ 端口测试异常: {str(e)}")

async def main():
    """主函数"""
    print("城市建设鹰眼系统 - SSH连接诊断工具")
    print("=" * 50)
    
    # 测试网络连通性
    test_network_connectivity()
    
    print("\n")
    
    # 测试SSH连接
    success = await test_ssh_connection()
    
    print("\n" + "=" * 50)
    if success:
        print("🎉 SSH连接测试完成，连接正常！")
    else:
        print("❌ SSH连接测试失败，请检查:")
        print("1. 网络连通性")
        print("2. SSH服务是否运行")
        print("3. 用户名密码是否正确")
        print("4. 防火墙设置")
    
    input("\n按Enter键退出...")

if __name__ == "__main__":
    asyncio.run(main())
