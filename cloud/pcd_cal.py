import open3d as o3d
import numpy as np


def analyze_point_cloud(file_path):
    # 加载点云文件
    try:
        pcd = o3d.io.read_point_cloud(file_path)
        print(f"成功加载点云文件: {file_path}")
    except Exception as e:
        print(f"加载文件失败: {e}")
        return

    # 获取点云总数
    total_points = len(pcd.points)
    print(f"点云总数: {total_points}")

    # 检查点云是否包含颜色信息
    if not pcd.has_colors():
        print("点云不包含颜色信息，无法分析红色点云占比。")
        return

    # 将颜色信息转换为 NumPy 数组
    colors = np.asarray(pcd.colors)

    # 定义红色的阈值（RGB 范围为 0-1）
    # 这里设定为 R>0.8 且 G<0.3 且 B<0.3，可以根据实际需求调整
    red_points_mask = (colors[:, 0] > 0.6) & (colors[:, 1] < 0.8) & (colors[:, 2] < 0.8)

    # 计算红色点云数量
    red_points_count = np.sum(red_points_mask)

    # 计算占比
    red_percentage = (red_points_count / total_points) * 100

    print(f"红色点云数量: {red_points_count}")
    print(f"红色点云占比: {red_percentage:.2f}%")

    # 提取红色点云并保存
    red_pcd = o3d.geometry.PointCloud()
    red_pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[red_points_mask])
    red_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[red_points_mask])

    output_file = file_path.replace('.pcd', '_red.pcd')
    o3d.io.write_point_cloud(output_file, red_pcd)
    print(f"已提取红色点云并保存至: {output_file}")

    # 返回分析结果，便于后续交给大语言模型处理
    return {
        "total_points": total_points,
        "red_points_count": red_points_count,
        "red_percentage": red_percentage,
        "red_points_mask": red_points_mask
    }


# 示例使用
if __name__ == "__main__":
    file_path = "pcd/jwp.pcd" # 请替换为实际的文件路径
    result = analyze_point_cloud(file_path)

    if result:
        # 这里可以将 result 发送给大语言模型进行分析
        print("\n分析结果已准备好，可以发送给大语言模型进行进一步分析。")
        print("结果包含: 总点数, 红色点数量, 红色点占比, 红色点掩码")