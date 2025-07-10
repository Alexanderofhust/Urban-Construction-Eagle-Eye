# predict_new.py - 适用于新数据集的预测脚本
import os
import torch
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import cv2
from tqdm import tqdm
import argparse

from model import PrivilegedMultiTaskModel
from dataset_new import get_new_transforms

class NewDatasetPredictor:
    """新数据集预测器"""
    
    def __init__(self, model_path, device='cuda', task_mode='single', num_classes=3):
        """
        初始化预测器
        
        Args:
            model_path (str): 模型权重路径
            device (str): 设备
            task_mode (str): 任务模式
            num_classes (int): 类别数
        """
        self.device = device
        self.task_mode = task_mode
        self.num_classes = num_classes
        
        # 加载模型
        self.model = PrivilegedMultiTaskModel(
            task_mode=task_mode,
            num_classes=num_classes
        ).to(device)
        
        # 加载权重
        checkpoint = torch.load(model_path, map_location=device)
        if 'model_state_dict' in checkpoint:
            self.model.load_state_dict(checkpoint['model_state_dict'])
        else:
            self.model.load_state_dict(checkpoint)
        
        self.model.eval()
        
        # 预处理transform
        self.transform = get_new_transforms('val', resize_shape=(480, 480))
        
        # 类别颜色映射（用于可视化）
        self.class_colors = [
            [0, 0, 0],        # 类别0: 黑色
            [255, 0, 0],      # 类别1: 红色
            [0, 255, 0],      # 类别2: 绿色
            [0, 0, 255],      # 类别3: 蓝色（如果有更多类别）
            [255, 255, 0],    # 类别4: 黄色
            [255, 0, 255],    # 类别5: 品红
            [0, 255, 255],    # 类别6: 青色
        ]
        
        print(f"预测器初始化完成: {task_mode}模式, {num_classes}类别")
    
    def predict_image(self, image_path):
        """
        预测单张图像
        
        Args:
            image_path (str): 图像路径
            
        Returns:
            tuple: (prediction_mask, confidence_map, original_image)
        """
        # 加载图像
        image = np.array(Image.open(image_path).convert('RGB'))
        original_image = image.copy()
        
        # 预处理
        augmented = self.transform(image=image)
        input_tensor = augmented['image'].unsqueeze(0).to(self.device)
        
        # 预测
        with torch.no_grad():
            predictions = self.model(input_tensor)
            
            if self.task_mode == 'single':
                logits = predictions['semantic_seg']
            else:
                logits = predictions['damage_type']  # 或其他主任务
            
            # 转换为概率
            probs = torch.softmax(logits, dim=1)
            
            # 获取预测类别
            pred_classes = torch.argmax(probs, dim=1)
            
            # 获取最大概率作为置信度
            confidence = torch.max(probs, dim=1)[0]
            
            # 转换为numpy
            pred_mask = pred_classes.cpu().numpy()[0]
            confidence_map = confidence.cpu().numpy()[0]
        
        return pred_mask, confidence_map, original_image
    
    def predict_batch(self, image_paths):
        """
        批量预测多张图像
        
        Args:
            image_paths (list): 图像路径列表
            
        Returns:
            list: 预测结果列表
        """
        results = []
        
        for image_path in tqdm(image_paths, desc="预测中"):
            try:
                pred_mask, confidence_map, original_image = self.predict_image(image_path)
                results.append({
                    'image_path': image_path,
                    'prediction': pred_mask,
                    'confidence': confidence_map,
                    'original_image': original_image
                })
            except Exception as e:
                print(f"预测失败: {image_path}, 错误: {e}")
                results.append({
                    'image_path': image_path,
                    'prediction': None,
                    'confidence': None,
                    'original_image': None,
                    'error': str(e)
                })
        
        return results
    
    def visualize_prediction(self, original_image, pred_mask, confidence_map, 
                           save_path=None, show_confidence=True):
        """
        可视化预测结果
        
        Args:
            original_image (np.ndarray): 原始图像
            pred_mask (np.ndarray): 预测mask
            confidence_map (np.ndarray): 置信度图
            save_path (str): 保存路径
            show_confidence (bool): 是否显示置信度
        """
        # 创建彩色mask
        color_mask = np.zeros((pred_mask.shape[0], pred_mask.shape[1], 3), dtype=np.uint8)
        
        for class_id in range(self.num_classes):
            mask = pred_mask == class_id
            color_mask[mask] = self.class_colors[class_id]
        
        # 确保图像尺寸匹配
        if original_image.shape[:2] != color_mask.shape[:2]:
            color_mask = cv2.resize(color_mask, (original_image.shape[1], original_image.shape[0]))
        
        # 创建叠加图像
        overlay = cv2.addWeighted(original_image, 0.7, color_mask, 0.3, 0)
        
        # 创建子图
        if show_confidence:
            fig, axes = plt.subplots(2, 2, figsize=(15, 12))
            
            # 原始图像
            axes[0, 0].imshow(original_image)
            axes[0, 0].set_title('Original Image')
            axes[0, 0].axis('off')
            
            # 预测mask
            axes[0, 1].imshow(color_mask)
            axes[0, 1].set_title('Prediction Mask')
            axes[0, 1].axis('off')
            
            # 叠加图像
            axes[1, 0].imshow(overlay)
            axes[1, 0].set_title('Overlay')
            axes[1, 0].axis('off')
            
            # 置信度图
            im = axes[1, 1].imshow(confidence_map, cmap='jet', vmin=0, vmax=1)
            axes[1, 1].set_title('Confidence Map')
            axes[1, 1].axis('off')
            plt.colorbar(im, ax=axes[1, 1])
            
        else:
            fig, axes = plt.subplots(1, 3, figsize=(15, 5))
            
            # 原始图像
            axes[0].imshow(original_image)
            axes[0].set_title('Original Image')
            axes[0].axis('off')
            
            # 预测mask
            axes[1].imshow(color_mask)
            axes[1].set_title('Prediction Mask')
            axes[1].axis('off')
            
            # 叠加图像
            axes[2].imshow(overlay)
            axes[2].set_title('Overlay')
            axes[2].axis('off')
        
        # 添加类别图例
        legend_elements = []
        for i in range(self.num_classes):
            color = np.array(self.class_colors[i]) / 255.0
            legend_elements.append(plt.Line2D([0], [0], marker='s', color='w', 
                                            markerfacecolor=color, markersize=10, 
                                            label=f'Class {i}'))
        
        fig.legend(handles=legend_elements, loc='upper right')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"可视化结果已保存: {save_path}")
        else:
            plt.show()
        
        plt.close()
    
    def predict_directory(self, input_dir, output_dir, visualize=True):
        """
        预测整个目录的图像
        
        Args:
            input_dir (str): 输入目录
            output_dir (str): 输出目录
            visualize (bool): 是否生成可视化结果
        """
        # 获取所有图像文件
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']
        image_files = []
        
        for filename in os.listdir(input_dir):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                image_files.append(os.path.join(input_dir, filename))
        
        if not image_files:
            print(f"在目录 {input_dir} 中没有找到图像文件")
            return
        
        print(f"找到 {len(image_files)} 张图像")
        
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        if visualize:
            vis_dir = os.path.join(output_dir, 'visualizations')
            os.makedirs(vis_dir, exist_ok=True)
        
        # 批量预测
        results = self.predict_batch(image_files)
        filename = os.path.basename(result['image_path'])
        name = os.path.splitext(filename)[0]
        for result in results:
            if result['prediction'] is not None:
                filename = os.path.basename(result['image_path'])
                name, ext = os.path.splitext(filename)
                
                # 保存预测mask
                pred_path = os.path.join(output_dir, f"{name}_pred.png")
                pred_image = Image.fromarray(result['prediction'].astype(np.uint8))
                pred_image.save(pred_path)
                
                # 保存置信度图
                conf_path = os.path.join(output_dir, f"{name}_conf.png")
                conf_image = (result['confidence'] * 255).astype(np.uint8)
                Image.fromarray(conf_image).save(conf_path)
                
                # 生成可视化
                if visualize:
                    vis_path = os.path.join(vis_dir, f"{name}_vis.png")
                    self.visualize_prediction(
                        result['original_image'],
                        result['prediction'],
                        result['confidence'],
                        save_path=vis_path,
                        show_confidence=True
                    )
        
        print(f"预测完成，结果保存在: {output_dir}")

def main():
    parser = argparse.ArgumentParser(description='新数据集预测脚本')
    parser.add_argument('--model_path', type=str, required=True, help='模型权重路径')
    parser.add_argument('--input', type=str, required=True, help='输入图像路径或目录')
    parser.add_argument('--output', type=str, required=True, help='输出目录')
    parser.add_argument('--device', type=str, default='cuda', help='设备')
    parser.add_argument('--task_mode', type=str, default='single', help='任务模式')
    parser.add_argument('--num_classes', type=int, default=3, help='类别数')
    parser.add_argument('--visualize', action='store_true', help='是否生成可视化')
    
    args = parser.parse_args()
    
    # 创建预测器
    predictor = NewDatasetPredictor(
        model_path=args.model_path,
        device=args.device,
        task_mode=args.task_mode,
        num_classes=args.num_classes
    )
    
    # 判断输入是文件还是目录
    if os.path.isfile(args.input):
        # 单张图像预测
        pred_mask, confidence_map, original_image = predictor.predict_image(args.input)
        filename = os.path.basename(args.input)
        name = os.path.splitext(filename)[0]
        os.makedirs(args.output, exist_ok=True)
        
        filename = os.path.basename(args.input)
        name, ext = os.path.splitext(filename)
        
        # 保存预测mask
        pred_path = os.path.join(args.output, f"{name}_pred.png")
        pred_image = Image.fromarray(pred_mask.astype(np.uint8))
        pred_image.save(pred_path)
        
        # 生成可视化
        if args.visualize:
            vis_path = os.path.join(args.output, f"{name}_vis.png")
            predictor.visualize_prediction(
                original_image, pred_mask, confidence_map,
                save_path=vis_path
            )
        
        print(f"预测完成，结果保存在: {args.output}")
        
    elif os.path.isdir(args.input):
        # 目录预测
        predictor.predict_directory(args.input, args.output, args.visualize)
    else:
        print(f"输入路径不存在: {args.input}")

if __name__ == "__main__":
    main()
