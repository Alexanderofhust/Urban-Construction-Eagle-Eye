import open3d as o3d
import numpy as np
import os
import glob
import sys
import time
from typing import List, Optional


class PointCloudViewer:
    """简化版点云查看器，仅支持点云文件查看和切换"""

    def __init__(self, pcd_folder: str = "./pcd"):
        """
        初始化点云查看器

        参数:
            pcd_folder: 存放点云文件的文件夹路径
        """
        self.pcd_folder = pcd_folder
        self.pcd_files = self._get_pcd_files()
        self.current_index = 0
        self.visualizer = None
        self.is_running = True

        # 检查点云文件
        if not self.pcd_files:
            print(f"错误: 在 {pcd_folder} 中未找到PCD文件!")
            sys.exit(1)

        print(f"找到 {len(self.pcd_files)} 个点云文件")

    def _get_pcd_files(self) -> List[str]:
        """获取指定文件夹中所有PCD文件的路径"""
        return sorted(glob.glob(os.path.join(self.pcd_folder, "*.pcd")))

    def _load_point_cloud(self, index: int) -> Optional[o3d.geometry.PointCloud]:
        """加载指定索引的点云文件"""
        if 0 <= index < len(self.pcd_files):
            file_path = self.pcd_files[index]
            print(f"加载点云: {os.path.basename(file_path)} ({index + 1}/{len(self.pcd_files)})")
            try:
                return o3d.io.read_point_cloud(file_path)
            except Exception as e:
                print(f"加载点云失败: {e}")
        return None

    def _key_callback(self, vis, key):
        """键盘事件回调函数"""
        # 按 'q' 键退出程序
        if key in [ord('Q'), ord('q')]:
            self.is_running = False
            self.visualizer.destroy_window()
            return False

        # 按 'a' 键切换到上一个点云
        if key in [ord('A'), ord('a')]:
            self.current_index = (self.current_index - 1) % len(self.pcd_files)
            self._update_point_cloud(vis)
            return True

        # 按 'd' 键切换到下一个点云
        if key in [ord('D'), ord('d')]:
            self.current_index = (self.current_index + 1) % len(self.pcd_files)
            self._update_point_cloud(vis)
            return True

        # 按 'h' 键显示帮助信息
        if key in [ord('H'), ord('h')]:
            self._print_help()
            return True

        return False

    def _update_point_cloud(self, vis):
        """更新可视化器中的点云"""
        point_cloud = self._load_point_cloud(self.current_index)
        if point_cloud:
            # 清除当前几何图形并添加新的点云
            vis.clear_geometries()
            vis.add_geometry(point_cloud)
            vis.update_renderer()
            return True
        return False

    def _print_help(self):
        """打印帮助信息"""
        print("\n===== 键盘控制帮助 =====")
        print("a: 显示上一个点云")
        print("d: 显示下一个点云")
        print("q: 退出程序")
        print("h: 显示帮助信息")
        print("=======================\n")

    def run(self):
        """运行交互式点云查看器"""
        # 打印帮助信息
        self._print_help()

        # 初始化点云查看器
        self.visualizer = o3d.visualization.VisualizerWithKeyCallback()
        self.visualizer.create_window(window_name="点云查看器")

        # 注册键盘回调函数
        self.visualizer.register_key_callback(ord('Q'), lambda vis: self._key_callback(vis, ord('Q')))
        self.visualizer.register_key_callback(ord('q'), lambda vis: self._key_callback(vis, ord('q')))
        self.visualizer.register_key_callback(ord('A'), lambda vis: self._key_callback(vis, ord('A')))
        self.visualizer.register_key_callback(ord('a'), lambda vis: self._key_callback(vis, ord('a')))
        self.visualizer.register_key_callback(ord('D'), lambda vis: self._key_callback(vis, ord('D')))
        self.visualizer.register_key_callback(ord('d'), lambda vis: self._key_callback(vis, ord('d')))
        self.visualizer.register_key_callback(ord('H'), lambda vis: self._key_callback(vis, ord('H')))
        self.visualizer.register_key_callback(ord('h'), lambda vis: self._key_callback(vis, ord('h')))

        # 加载并显示第一个点云
        point_cloud = self._load_point_cloud(self.current_index)
        if point_cloud:
            self.visualizer.add_geometry(point_cloud)

        # 主循环
        while self.is_running:
            self.visualizer.poll_events()
            self.visualizer.update_renderer()
            time.sleep(0.01)  # 减少CPU使用率

        # 清理资源
        self.visualizer.destroy_window()


if __name__ == "__main__":
    # 默认从当前目录的pcd文件夹加载文件
    # 可通过命令行参数指定其他文件夹
    pcd_folder = "./pcd"
    if len(sys.argv) > 1:
        pcd_folder = sys.argv[1]

    viewer = PointCloudViewer(pcd_folder)
    viewer.run()