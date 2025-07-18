#!/usr/bin/env python3
"""
启动脚本 - 确保在正确的目录中运行
"""
import os
import sys
import subprocess

def main():
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 切换到脚本目录
    os.chdir(script_dir)
    
    print(f"当前工作目录: {os.getcwd()}")
    
    # 检查main.py是否存在
    main_py = os.path.join(script_dir, "main.py")
    if not os.path.exists(main_py):
        print(f"错误: 找不到 {main_py}")
        sys.exit(1)
    
    # 启动main.py
    try:
        subprocess.run([sys.executable, main_py], check=True)
    except KeyboardInterrupt:
        print("\n服务器已停止")
    except subprocess.CalledProcessError as e:
        print(f"启动失败: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
