# inference_onnx.py - 使用ONNX模型进行推理
import os
import numpy as np
from PIL import Image
import cv2
import argparse
import time

def preprocess_image(image_path, target_size=(480, 480)):
    """
    预处理图像
    
    Args:
        image_path (str): 图像路径
        target_size (tuple): 目标尺寸
        
    Returns:
        np.ndarray: 预处理后的图像
    """
    # 加载图像
    image = Image.open(image_path).convert('RGB')
    image = np.array(image)
    
    # 调整尺寸
    image = cv2.resize(image, target_size)
    
    # 归一化
    image = image.astype(np.float32) / 255.0
    
    # ImageNet标准化
    mean = np.array([0.485, 0.456, 0.406])
    std = np.array([0.229, 0.224, 0.225])
    image = (image - mean) / std
    
    # 转换为CHW格式并添加batch维度
    image = np.transpose(image, (2, 0, 1))  # HWC -> CHW
    image = np.expand_dims(image, axis=0)   # 添加batch维度
    
    return image

def postprocess_output(output, num_classes=3):
    """
    后处理模型输出
    
    Args:
        output (np.ndarray): 模型输出logits
        num_classes (int): 类别数
        
    Returns:
        np.ndarray: 预测mask
    """
    # 如果输出是logits，应用softmax
    if output.shape[1] == num_classes:  # (B, C, H, W)
        # Softmax
        exp_output = np.exp(output - np.max(output, axis=1, keepdims=True))
        probs = exp_output / np.sum(exp_output, axis=1, keepdims=True)
        
        # 获取预测类别
        pred_mask = np.argmax(probs, axis=1)[0]  # 移除batch维度
    else:
        # 如果已经是概率输出
        pred_mask = np.argmax(output, axis=1)[0]
    
    return pred_mask

def visualize_result(original_image, pred_mask, save_path=None):
    """
    可视化预测结果
    
    Args:
        original_image (np.ndarray): 原始图像
        pred_mask (np.ndarray): 预测mask
        save_path (str): 保存路径
    """
    # 类别颜色
    colors = [
        [0, 0, 0],        # 类别0: 黑色
        [255, 0, 0],      # 类别1: 红色  
        [0, 255, 0],      # 类别2: 绿色
        [0, 0, 255],      # 类别3: 蓝色
    ]
    
    # 创建彩色mask
    h, w = pred_mask.shape
    color_mask = np.zeros((h, w, 3), dtype=np.uint8)
    
    for class_id in range(len(colors)):
        mask = pred_mask == class_id
        color_mask[mask] = colors[class_id]
    
    # 调整原始图像尺寸匹配mask
    if original_image.shape[:2] != (h, w):
        original_image = cv2.resize(original_image, (w, h))
    
    # 创建叠加图像
    overlay = cv2.addWeighted(original_image, 0.7, color_mask, 0.3, 0)
    
    # 拼接显示
    result = np.hstack([original_image, color_mask, overlay])
    
    if save_path:
        cv2.imwrite(save_path, cv2.cvtColor(result, cv2.COLOR_RGB2BGR))
        print(f"结果已保存: {save_path}")
    
    return result

def inference_onnx(model_path, image_path, output_dir=None, target_size=(480, 480), num_classes=3):
    """
    使用ONNX模型进行推理
    
    Args:
        model_path (str): ONNX模型路径
        image_path (str): 输入图像路径
        output_dir (str): 输出目录
        target_size (tuple): 目标尺寸
        num_classes (int): 类别数
    """
    try:
        import onnxruntime as ort
    except ImportError:
        print("错误: 请安装onnxruntime")
        print("安装命令: pip install onnxruntime 或 pip install onnxruntime-gpu")
        return
    
    # 加载原始图像
    original_image = np.array(Image.open(image_path).convert('RGB'))
    
    # 预处理
    print("预处理图像...")
    input_data = preprocess_image(image_path, target_size)
    print(f"输入形状: {input_data.shape}")
    
    # 创建推理会话
    print(f"加载ONNX模型: {model_path}")
    
    # 设置推理提供者
    providers = ['CPUExecutionProvider']
    if ort.get_device() == 'GPU':
        providers.insert(0, 'CUDAExecutionProvider')
    
    session = ort.InferenceSession(model_path, providers=providers)
    
    # 获取输入输出信息
    input_info = session.get_inputs()[0]
    output_info = session.get_outputs()
    
    print(f"模型输入: {input_info.name}, 形状: {input_info.shape}")
    print("模型输出:")
    for out in output_info:
        print(f"  {out.name}, 形状: {out.shape}")
    
    # 推理
    print("开始推理...")
    start_time = time.time()
    
    outputs = session.run(None, {input_info.name: input_data})
    
    inference_time = time.time() - start_time
    print(f"推理时间: {inference_time*1000:.2f} ms")
    
    # 后处理
    print("后处理结果...")
    if len(outputs) == 1:
        # 单任务输出
        pred_mask = postprocess_output(outputs[0], num_classes)
    else:
        # 多任务输出，使用第一个（通常是主任务）
        pred_mask = postprocess_output(outputs[0], num_classes)
    
    print(f"预测mask形状: {pred_mask.shape}")
    print(f"预测类别: {np.unique(pred_mask)}")
    
    # 可视化和保存结果
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        
        # 保存预测mask
        mask_path = os.path.join(output_dir, "prediction_mask.png")
        Image.fromarray(pred_mask.astype(np.uint8)).save(mask_path)
        
        # 保存可视化结果
        vis_path = os.path.join(output_dir, "visualization.png")
        visualize_result(original_image, pred_mask, vis_path)
        
        print(f"结果已保存到: {output_dir}")
    
    return pred_mask

def benchmark_model(model_path, target_size=(480, 480), num_runs=100):
    """
    模型性能测试
    
    Args:
        model_path (str): ONNX模型路径
        target_size (tuple): 输入尺寸
        num_runs (int): 测试次数
    """
    try:
        import onnxruntime as ort
    except ImportError:
        print("错误: 请安装onnxruntime")
        return
    
    print(f"性能测试: {num_runs} 次推理")
    
    # 创建随机输入
    input_data = np.random.randn(1, 3, target_size[0], target_size[1]).astype(np.float32)
    
    # 创建推理会话
    providers = ['CPUExecutionProvider']
    if ort.get_device() == 'GPU':
        providers.insert(0, 'CUDAExecutionProvider')
    
    session = ort.InferenceSession(model_path, providers=providers)
    input_name = session.get_inputs()[0].name
    
    # 预热
    for _ in range(10):
        session.run(None, {input_name: input_data})
    
    # 测试
    times = []
    for i in range(num_runs):
        start_time = time.time()
        session.run(None, {input_name: input_data})
        end_time = time.time()
        times.append((end_time - start_time) * 1000)  # 转换为毫秒
    
    # 统计
    mean_time = np.mean(times)
    std_time = np.std(times)
    min_time = np.min(times)
    max_time = np.max(times)
    
    print(f"平均推理时间: {mean_time:.2f} ± {std_time:.2f} ms")
    print(f"最快: {min_time:.2f} ms, 最慢: {max_time:.2f} ms")
    print(f"FPS: {1000/mean_time:.2f}")

def main():
    parser = argparse.ArgumentParser(description='ONNX模型推理')
    parser.add_argument('--model_path', type=str, required=True,
                       help='ONNX模型路径')
    parser.add_argument('--image_path', type=str, required=True,
                       help='输入图像路径')
    parser.add_argument('--output_dir', type=str, default='./onnx_results',
                       help='输出目录')
    parser.add_argument('--target_height', type=int, default=480,
                       help='目标图像高度')
    parser.add_argument('--target_width', type=int, default=480,
                       help='目标图像宽度')
    parser.add_argument('--num_classes', type=int, default=3,
                       help='分割类别数')
    parser.add_argument('--benchmark', action='store_true',
                       help='运行性能测试')
    parser.add_argument('--num_runs', type=int, default=100,
                       help='性能测试次数')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.model_path):
        print(f"错误: 模型文件不存在: {args.model_path}")
        return
    
    if not os.path.exists(args.image_path):
        print(f"错误: 图像文件不存在: {args.image_path}")
        return
    
    target_size = (args.target_height, args.target_width)
    
    # 运行推理
    print("=" * 50)
    print("ONNX模型推理")
    print("=" * 50)
    
    pred_mask = inference_onnx(
        model_path=args.model_path,
        image_path=args.image_path,
        output_dir=args.output_dir,
        target_size=target_size,
        num_classes=args.num_classes
    )
    
    # 性能测试
    if args.benchmark:
        print("\n" + "=" * 50)
        print("性能测试")
        print("=" * 50)
        benchmark_model(args.model_path, target_size, args.num_runs)

if __name__ == "__main__":
    main()
