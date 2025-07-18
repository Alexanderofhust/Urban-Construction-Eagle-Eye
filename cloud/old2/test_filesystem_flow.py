#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件系统数据处理流程测试脚本
用于验证新的数据处理流程是否正常工作
"""

import os
import sys
import time
import json
import cv2
import numpy as np
from pathlib import Path
from datetime import datetime
import requests
import shutil

def create_test_data(output_dir, session_id, num_images=5):
    """创建测试数据"""
    print(f"创建测试数据到: {output_dir}/{session_id}")
    
    # 创建目录
    session_dir = Path(output_dir) / session_id
    image_dir = session_dir / 'images'
    mask_dir = session_dir / 'masks'
    pointcloud_dir = session_dir / 'pointclouds'
    metadata_dir = session_dir / 'metadata'
    
    for dir_path in [image_dir, mask_dir, pointcloud_dir, metadata_dir]:
        dir_path.mkdir(parents=True, exist_ok=True)
    
    # 创建测试图像和mask
    for i in range(num_images):
        timestamp = datetime.now()
        filename_base = f"{timestamp.strftime('%Y%m%d_%H%M%S')}_{i:06d}"
        
        # 创建测试图像
        image = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
        image_path = image_dir / f"{filename_base}.jpg"
        cv2.imwrite(str(image_path), image)
        
        # 创建测试mask
        mask = np.random.randint(0, 2, (480, 640), dtype=np.uint8) * 255
        mask_path = mask_dir / f"{filename_base}_mask.png"
        cv2.imwrite(str(mask_path), mask)
        
        # 创建元数据
        metadata = {
            'filename': f"{filename_base}.jpg",
            'mask_filename': f"{filename_base}_mask.png",
            'timestamp': timestamp.isoformat(),
            'test_data': True,
            'image_size': {'width': 640, 'height': 480, 'channels': 3}
        }
        
        metadata_path = metadata_dir / f"{filename_base}_metadata.json"
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"创建测试图像: {filename_base}.jpg")
        
        # 延时以模拟实时数据
        time.sleep(0.5)
    
    # 创建测试点云
    create_test_pointcloud(pointcloud_dir, metadata_dir)
    
    print(f"测试数据创建完成")

def create_test_pointcloud(pointcloud_dir, metadata_dir):
    """创建测试点云"""
    print("创建测试点云...")
    
    # 生成随机点云数据
    num_points = 10000
    points = np.random.rand(num_points, 3) * 10 - 5  # -5 to 5 range
    
    # 保存PCD文件
    timestamp = datetime.now()
    filename = f"test_pointcloud_{timestamp.strftime('%Y%m%d_%H%M%S')}.pcd"
    pcd_path = pointcloud_dir / filename
    
    with open(pcd_path, 'w') as f:
        # PCD头
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        
        # 点数据
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
    
    # 保存点云元数据
    metadata = {
        'filename': filename,
        'timestamp': timestamp.isoformat(),
        'point_count': num_points,
        'test_data': True,
        'file_size': os.path.getsize(pcd_path)
    }
    
    metadata_path = metadata_dir / f"{Path(filename).stem}_metadata.json"
    with open(metadata_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"测试点云创建完成: {filename}")

def test_api_endpoints(base_url, session_id):
    """测试API端点"""
    print(f"测试API端点: {base_url}")
    
    try:
        # 测试健康检查
        response = requests.get(f"{base_url}/api/health")
        print(f"健康检查: {response.status_code}")
        if response.status_code == 200:
            print(f"  响应: {response.json()}")
        
        # 测试会话状态
        response = requests.get(f"{base_url}/api/session/{session_id}/status")
        print(f"会话状态: {response.status_code}")
        if response.status_code == 200:
            print(f"  响应: {response.json()}")
        
        # 测试数据获取
        response = requests.get(f"{base_url}/api/data", params={'session_id': session_id})
        print(f"数据获取: {response.status_code}")
        if response.status_code == 200:
            data = response.json()
            if data.get('success'):
                result = data.get('data', {})
                print(f"  图像数量: {len(result.get('images', []))}")
                print(f"  点云数量: {len(result.get('pointclouds', []))}")
        
        # 测试系统信息
        response = requests.get(f"{base_url}/api/system/info")
        print(f"系统信息: {response.status_code}")
        if response.status_code == 200:
            print(f"  响应: {response.json()}")
        
    except Exception as e:
        print(f"API测试失败: {str(e)}")

def test_filesystem_processor():
    """测试文件系统处理器"""
    print("测试文件系统处理器...")
    
    try:
        # 导入处理器
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        from filesystem_data_processor import FileSystemDataProcessor
        
        # 创建配置
        config = {
            'image_dir': './test_data/images',
            'mask_dir': './test_data/masks',
            'pointcloud_dir': './test_data/pointclouds',
            'metadata_dir': './test_data/metadata',
            'monitor_interval': 1.0
        }
        
        # 创建处理器
        processor = FileSystemDataProcessor(config)
        
        # 测试会话
        session_id = "test_session"
        
        # 启动监控
        success = processor.start_monitoring(session_id)
        print(f"启动监控: {success}")
        
        # 获取状态
        status = processor.get_session_status(session_id)
        print(f"会话状态: {status}")
        
        # 扫描文件
        new_files = processor.scan_new_files(session_id)
        print(f"新文件: {new_files}")
        
        # 获取处理数据
        processed_data = processor.get_processed_data(session_id)
        print(f"处理数据: {len(processed_data.get('images', []))} 图像, {len(processed_data.get('pointclouds', []))} 点云")
        
        # 停止监控
        processor.stop_monitoring(session_id)
        print("监控已停止")
        
    except Exception as e:
        print(f"文件系统处理器测试失败: {str(e)}")

def cleanup_test_data(output_dir):
    """清理测试数据"""
    print(f"清理测试数据: {output_dir}")
    
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
        print("测试数据已清理")

def main():
    """主函数"""
    print("========================================")
    print("文件系统数据处理流程测试")
    print("========================================")
    
    # 配置
    test_data_dir = "./test_data"
    session_id = f"test_session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    api_base_url = "http://localhost:8000"
    
    try:
        # 清理旧数据
        cleanup_test_data(test_data_dir)
        
        # 1. 创建测试数据
        print("\n1. 创建测试数据...")
        create_test_data(test_data_dir, session_id, num_images=3)
        
        # 2. 测试文件系统处理器
        print("\n2. 测试文件系统处理器...")
        test_filesystem_processor()
        
        # 3. 测试API端点（如果服务器在运行）
        print("\n3. 测试API端点...")
        try:
            test_api_endpoints(api_base_url, session_id)
        except Exception as e:
            print(f"API测试跳过（服务器可能未运行）: {str(e)}")
        
        print("\n========================================")
        print("测试完成!")
        print("========================================")
        
        # 询问是否保留测试数据
        keep_data = input("\n是否保留测试数据? (y/n): ").lower().strip()
        if keep_data != 'y':
            cleanup_test_data(test_data_dir)
    
    except KeyboardInterrupt:
        print("\n测试被用户中断")
        cleanup_test_data(test_data_dir)
    
    except Exception as e:
        print(f"\n测试失败: {str(e)}")
        cleanup_test_data(test_data_dir)

if __name__ == "__main__":
    main()
