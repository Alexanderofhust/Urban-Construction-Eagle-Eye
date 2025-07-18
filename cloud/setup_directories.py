#!/usr/bin/env python3
"""
创建必要的目录结构
"""

import os
from pathlib import Path

def create_directories():
    """创建必要的目录"""
    directories = [
        'data',
        'data/sessions',
        'data/results',
        'templates',
        'static',
        'static/css',
        'static/js',
        'static/images',
        'services',
        'utils',
        'logs'
    ]
    
    for directory in directories:
        dir_path = Path(directory)
        if not dir_path.exists():
            dir_path.mkdir(parents=True, exist_ok=True)
            print(f"✅ 创建目录: {directory}")
        else:
            print(f"📁 目录已存在: {directory}")

def create_gitignore():
    """创建.gitignore文件"""
    gitignore_content = """
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
build/
develop-eggs/
dist/
downloads/
eggs/
.eggs/
lib/
lib64/
parts/
sdist/
var/
wheels/
*.egg-info/
.installed.cfg
*.egg
MANIFEST

# Virtual Environment
venv/
env/
ENV/

# IDE
.vscode/
.idea/
*.swp
*.swo
*~

# OS
.DS_Store
Thumbs.db

# Project specific
data/sessions/
data/results/
logs/
*.log
config.py

# API Keys
.env
*.key
"""
    
    gitignore_path = Path('.gitignore')
    if not gitignore_path.exists():
        with open(gitignore_path, 'w', encoding='utf-8') as f:
            f.write(gitignore_content.strip())
        print("✅ 创建 .gitignore 文件")
    else:
        print("📄 .gitignore 文件已存在")

def create_empty_init_files():
    """创建空的__init__.py文件"""
    init_dirs = ['services', 'utils']
    
    for directory in init_dirs:
        init_file = Path(directory) / '__init__.py'
        if not init_file.exists():
            init_file.touch()
            print(f"✅ 创建 {init_file}")
        else:
            print(f"📄 {init_file} 已存在")

if __name__ == "__main__":
    print("=" * 50)
    print("    创建项目目录结构")
    print("=" * 50)
    
    create_directories()
    create_gitignore()
    create_empty_init_files()
    
    print("\n" + "=" * 50)
    print("✅ 目录结构创建完成！")
    print("=" * 50)
