import open3d as o3d
import numpy as np


class PointCloudAnalyzer:
    def __init__(self):
        """初始化点云分析器，设置红色点云判断阈值"""
        # 红色点云判断阈值（RGB范围0-1），可通过外部调整
        self.red_threshold = {
            "r_min": 0.6,  # 红色通道最小值
            "g_max": 0.8,  # 绿色通道最大值
            "b_max": 0.8  # 蓝色通道最大值
        }
        # 存储分析结果
        self.analysis_result = None

    def load_point_cloud(self, file_path):
        """
        加载点云文件
        :param file_path: 点云文件路径
        :return: 加载的点云对象，加载失败返回None
        """
        try:
            pcd = o3d.io.read_point_cloud(file_path)
            print(f"成功加载点云文件: {file_path}")
            return pcd
        except Exception as e:
            print(f"加载文件失败: {str(e)}")
            return None

    def _is_red_point(self, colors):
        """
        判断点云中的红色点
        :param colors: 点云颜色数组 (N, 3)
        :return: 红色点掩码 (N,)
        """
        r = colors[:, 0]
        g = colors[:, 1]
        b = colors[:, 2]
        return (r > self.red_threshold["r_min"]) & \
            (g < self.red_threshold["g_max"]) & \
            (b < self.red_threshold["b_max"])

    def _extract_red_points(self, pcd, red_mask):
        """
        提取红色点云
        :param pcd: 原始点云对象
        :param red_mask: 红色点掩码
        :return: 红色点云对象
        """
        points = np.asarray(pcd.points)[red_mask]
        colors = np.asarray(pcd.colors)[red_mask]

        red_pcd = o3d.geometry.PointCloud()
        red_pcd.points = o3d.utility.Vector3dVector(points)
        red_pcd.colors = o3d.utility.Vector3dVector(colors)
        return red_pcd

    def _save_red_point_cloud(self, red_pcd, original_path):
        """
        保存红色点云到文件
        :param red_pcd: 红色点云对象
        :param original_path: 原始文件路径
        :return: 保存路径，失败返回None
        """
        try:
            output_path = original_path.replace('.pcd', '_red.pcd')
            o3d.io.write_point_cloud(output_path, red_pcd)
            print(f"已提取红色点云并保存至: {output_path}")
            return output_path
        except Exception as e:
            print(f"保存红色点云失败: {str(e)}")
            return None

    def analyze(self, file_path):
        """
        执行点云分析主流程
        :param file_path: 点云文件路径
        :return: 分析结果字典，失败返回None
        """
        # 重置分析结果
        self.analysis_result = None

        # 1. 加载点云
        pcd = self.load_point_cloud(file_path)
        if pcd is None:
            return None

        # 2. 获取点云总数
        total_points = len(pcd.points)
        print(f"点云总数: {total_points}")

        # 3. 检查颜色信息
        if not pcd.has_colors():
            print("点云不包含颜色信息，无法分析红色点云占比。")
            return None

        # 4. 转换颜色数据
        colors = np.asarray(pcd.colors)

        # 5. 识别红色点云
        red_mask = self._is_red_point(colors)
        red_points_count = np.sum(red_mask)
        red_percentage = (red_points_count / total_points) * 100 if total_points > 0 else 0

        print(f"红色点云数量: {red_points_count}")
        print(f"红色点云占比: {red_percentage:.2f}%")

        # 6. 提取并保存红色点云
        red_pcd = self._extract_red_points(pcd, red_mask)
        red_save_path = self._save_red_point_cloud(red_pcd, file_path)

        # 7. 保存分析结果
        self.analysis_result = {
            "total_points": total_points,
            "red_points_count": red_points_count,
            "red_percentage": red_percentage,
            "red_points_mask": red_mask,  # 可用于后续处理
            "red_save_path": red_save_path,
            "source_path": file_path
        }

        return self.analysis_result

    def get_result(self):
        """获取最近一次分析结果"""
        return self.analysis_result

    def set_red_threshold(self, r_min=None, g_max=None, b_max=None):
        """
        调整红色点判断阈值
        :param r_min: 红色通道最小值
        :param g_max: 绿色通道最大值
        :param b_max: 蓝色通道最大值
        """
        if r_min is not None:
            self.red_threshold["r_min"] = r_min
        if g_max is not None:
            self.red_threshold["g_max"] = g_max
        if b_max is not None:
            self.red_threshold["b_max"] = b_max
        print(f"已更新红色点阈值: {self.red_threshold}")


# 示例使用
if __name__ == "__main__":
    # 1. 创建分析器实例
    analyzer = PointCloudAnalyzer()

    # 2. 可选：调整红色点阈值
    # analyzer.set_red_threshold(r_min=0.7, g_max=0.7, b_max=0.7)

    # 3. 执行分析
    file_path = "pcd/jwp.pcd"  # 替换为实际文件路径
    result = analyzer.analyze(file_path)

    # 4. 使用分析结果
    if result:
        print("\n分析结果已准备好，可以发送给大语言模型进行进一步分析。")
        print(f"总点数: {result['total_points']}")
        print(f"红色点占比: {result['red_percentage']:.2f}%")
        print(f"红色点保存路径: {result['red_save_path']}")