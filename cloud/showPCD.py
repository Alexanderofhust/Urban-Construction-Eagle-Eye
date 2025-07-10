import open3d as o3d
import numpy as np
import os
import glob
import sys
import time
from typing import List, Optional
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk

class PointCloudViewer:
    """交互式点云/图片查看器，支持键盘控制"""
    
    def __init__(self, pcd_folder: str = "./pcd", img_folder: str = "./img"):
        """
        初始化查看器
        
        参数:
            pcd_folder: 存放点云文件的文件夹路径
            img_folder: 存放图片文件的文件夹路径
        """
        self.pcd_folder = pcd_folder
        self.img_folder = img_folder
        self.pcd_files = self._get_pcd_files()
        self.img_files = self._get_img_files()
        self.current_index = 0
        self.current_mode = "pcd"  # 初始模式为点云查看器
        self.visualizer = None
        self.image_window = None
        self.canvas = None
        self.fig = None
        self.ax = None
        self.is_running = True
        
        # 检查文件
        if not self.pcd_files and not self.img_files:
            print(f"错误: 在 {pcd_folder} 中未找到PCD文件，且在 {img_folder} 中未找到图片文件!")
            sys.exit(1)
            
        print(f"找到 {len(self.pcd_files)} 个点云文件")
        print(f"找到 {len(self.img_files)} 个图片文件")
            
    def _get_pcd_files(self) -> List[str]:
        """获取指定文件夹中所有PCD文件的路径"""
        return sorted(glob.glob(os.path.join(self.pcd_folder, "*.pcd")))
    
    def _get_img_files(self) -> List[str]:
        """获取指定文件夹中所有图片文件的路径"""
        return sorted(glob.glob(os.path.join(self.img_folder, "*.png")) + 
                      glob.glob(os.path.join(self.img_folder, "*.jpg")) +
                      glob.glob(os.path.join(self.img_folder, "*.jpeg")))
    
    def _load_point_cloud(self, index: int) -> Optional[o3d.geometry.PointCloud]:
        """加载指定索引的点云文件"""
        if 0 <= index < len(self.pcd_files):
            file_path = self.pcd_files[index]
            print(f"加载点云: {os.path.basename(file_path)} ({index+1}/{len(self.pcd_files)})")
            try:
                return o3d.io.read_point_cloud(file_path)
            except Exception as e:
                print(f"加载点云失败: {e}")
        return None
    
    def _load_image(self, index: int):
        """加载指定索引的图片文件"""
        if 0 <= index < len(self.img_files):
            file_path = self.img_files[index]
            print(f"加载图片: {os.path.basename(file_path)} ({index+1}/{len(self.img_files)})")
            try:
                img = plt.imread(file_path)
                return img
            except Exception as e:
                print(f"加载图片失败: {e}")
        return None
    
    def _key_callback(self, vis, key):
        """键盘事件回调函数"""
        # 按 'q' 键退出程序
        if key in [ord('Q'), ord('q')]:
            self.is_running = False
            self._close_all_windows()
            return False
        
        # 按 'e' 键切换模式
        elif key in [ord('E'), ord('e')]:
            self._switch_mode()
            return True
        
        # 在点云模式下
        if self.current_mode == "pcd" and self.pcd_files:
            # 按 'a' 键切换到上一个点云
            if key in [ord('A'), ord('a')]:
                self.current_index = (self.current_index - 1) % len(self.pcd_files)
                self._update_point_cloud(vis)
                return True
            
            # 按 'd' 键切换到下一个点云
            elif key in [ord('D'), ord('d')]:
                self.current_index = (self.current_index + 1) % len(self.pcd_files)
                self._update_point_cloud(vis)
                return True
        
        # 按 'h' 键显示帮助信息
        if key in [ord('H'), ord('h')]:
            self._print_help()
            return True
            
        return False
    
    def _image_key_callback(self, event):
        """图片窗口的键盘事件回调"""
        key = event.keysym
        
        # 按 'q' 键退出程序
        if key.lower() == 'q':
            self.is_running = False
            self._close_all_windows()
        
        # 按 'e' 键切换模式
        elif key.lower() == 'e':
            self._switch_mode()
        
        # 在图片模式下
        elif self.current_mode == "img" and self.img_files:
            # 按 'a' 键切换到上一个图片
            if key.lower() == 'a':
                self.current_index = (self.current_index - 1) % len(self.img_files)
                self._update_image()
            
            # 按 'd' 键切换到下一个图片
            elif key.lower() == 'd':
                self.current_index = (self.current_index + 1) % len(self.img_files)
                self._update_image()
    
    def _update_point_cloud(self, vis):
        """更新可视化器中的点云"""
        if not self.pcd_files:
            print("没有点云文件可供显示")
            return False
            
        point_cloud = self._load_point_cloud(self.current_index)
        if point_cloud:
            # 清除当前几何图形并添加新的点云
            vis.clear_geometries()
            vis.add_geometry(point_cloud)
            vis.update_renderer()
            return True
        return False
    
    def _update_image(self):
        """更新图片显示并Resize为窗口大小"""
        if not self.img_files:
            print("没有图片文件可供显示")
            return
            
        if self.image_window is None or not self.image_window.winfo_exists():
            self._create_image_window()
            
        img = self._load_image(self.current_index)
        if img is not None:
            self.ax.clear()
            
            # 强制Resize图片以填充窗口
            self.ax.imshow(img, extent=[0, 1, 0, 1], aspect='auto')  # 使用extent填满坐标轴
            self.ax.set_title(os.path.basename(self.img_files[self.current_index]))
            self.ax.axis('off')
            
            self.canvas.draw()
    
    def _create_image_window(self):
        """创建图片显示窗口"""
        self.image_window = tk.Tk()
        self.image_window.title("图片查看器")
        self.image_window.geometry("1600x900")  # 设置窗口大小
        
        # 创建无边距的matplotlib图形
        self.fig = plt.figure(figsize=(16, 9), dpi=100)  # 16x9英寸，100DPI=1600x900像素
        self.fig.patch.set_facecolor('black')  # 背景色设为黑色
        self.ax = self.fig.add_axes([0, 0, 1, 1])  # 坐标范围[0,0]到[1,1]，填满整个图形
        self.ax.axis('off')  # 隐藏坐标轴
        
        # 创建Tkinter画布
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.image_window)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 绑定窗口大小变化事件，实时Resize图片
        self.image_window.bind("<Configure>", self._on_window_resize)
        
        # 绑定键盘事件
        self.image_window.bind("<Key>", self._image_key_callback)
        
        # 配置窗口关闭事件
        self.image_window.protocol("WM_DELETE_WINDOW", self._on_image_window_close)
    
    def _on_window_resize(self, event):
        """窗口大小变化时重新Resize图片"""
        if self.current_mode == "img" and self.img_files:
            self._update_image()
    
    def _on_image_window_close(self):
        """处理图片窗口关闭事件"""
        self.image_window.destroy()
        self.image_window = None
        # 切换回点云模式
        self.current_mode = "pcd"
        print("已切换回点云查看器模式")
    
    def _switch_mode(self):
        """切换查看器模式"""
        if self.current_mode == "pcd" and self.img_files:
            self.current_mode = "img"
            self._close_all_windows()
            self._create_image_window()
            self._update_image()
            print("已切换到图片查看器模式")
        elif self.current_mode == "img" and self.pcd_files:
            self.current_mode = "pcd"
            if self.image_window:
                self.image_window.destroy()
                self.image_window = None
            print("已切换到点云查看器模式")
        else:
            print("没有其他类型的文件可供切换")
    
    def _close_all_windows(self):
        """关闭所有打开的窗口"""
        if self.visualizer:
            self.visualizer.destroy_window()
        if self.image_window:
            self.image_window.destroy()
            self.image_window = None
    
    def _print_help(self):
        """打印帮助信息"""
        print("\n===== 键盘控制帮助 =====")
        print("a: 显示上一个项目")
        print("d: 显示下一个项目")
        print("e: 切换点云/图片查看器")
        print("q: 退出程序")
        print("h: 显示帮助信息")
        print("=======================\n")
    
    def run(self):
        """运行交互式查看器"""
        # 打印帮助信息
        self._print_help()
        
        # 如果有PCD文件，初始化点云查看器
        if self.pcd_files:
            self.visualizer = o3d.visualization.VisualizerWithKeyCallback()
            self.visualizer.create_window(window_name="点云查看器")
            
            # 注册键盘回调函数
            self.visualizer.register_key_callback(ord('Q'), lambda vis: self._key_callback(vis, ord('Q')))
            self.visualizer.register_key_callback(ord('q'), lambda vis: self._key_callback(vis, ord('q')))
            self.visualizer.register_key_callback(ord('A'), lambda vis: self._key_callback(vis, ord('A')))
            self.visualizer.register_key_callback(ord('a'), lambda vis: self._key_callback(vis, ord('a')))
            self.visualizer.register_key_callback(ord('D'), lambda vis: self._key_callback(vis, ord('D')))
            self.visualizer.register_key_callback(ord('d'), lambda vis: self._key_callback(vis, ord('d')))
            self.visualizer.register_key_callback(ord('E'), lambda vis: self._key_callback(vis, ord('E')))
            self.visualizer.register_key_callback(ord('e'), lambda vis: self._key_callback(vis, ord('e')))
            self.visualizer.register_key_callback(ord('H'), lambda vis: self._key_callback(vis, ord('H')))
            self.visualizer.register_key_callback(ord('h'), lambda vis: self._key_callback(vis, ord('h')))
            
            # 加载并显示第一个点云
            point_cloud = self._load_point_cloud(self.current_index)
            if point_cloud:
                self.visualizer.add_geometry(point_cloud)
        else:
            # 如果没有PCD文件，直接切换到图片查看器
            self.current_mode = "img"
            self._create_image_window()
            self._update_image()
        
        # 开始主循环
        while self.is_running:
            if self.current_mode == "pcd" and self.visualizer:
                self.visualizer.poll_events()
                self.visualizer.update_renderer()
            elif self.current_mode == "img" and self.image_window:
                self.image_window.update_idletasks()
                self.image_window.update()
                
            time.sleep(0.01)  # 减少CPU使用率
        
        # 清理资源
        self._close_all_windows()

if __name__ == "__main__":
    # 默认从当前目录的pcd和img文件夹加载文件
    # 也可以通过命令行参数指定其他文件夹
    pcd_folder = "./pcd"
    img_folder = "./img"
    
    if len(sys.argv) > 1:
        pcd_folder = sys.argv[1]
    if len(sys.argv) > 2:
        img_folder = sys.argv[2]
    
    viewer = PointCloudViewer(pcd_folder, img_folder)
    viewer.run()