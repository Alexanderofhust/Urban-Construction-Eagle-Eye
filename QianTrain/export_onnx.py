# export_onnx.py - 将PyTorch模型导出为ONNX格式
import os
import torch
import torch.onnx
import numpy as np
import argparse
from model import PrivilegedMultiTaskModel

def export_to_onnx(model_path, output_path, device='cuda', task_mode='single', 
                   num_classes=3, input_size=(1, 3, 480, 480), 
                   opset_version=11, dynamic_axes=False):
    """
    将PyTorch模型导出为ONNX格式
    
    Args:
        model_path (str): PyTorch模型权重路径
        output_path (str): 输出ONNX模型路径
        device (str): 设备
        task_mode (str): 任务模式
        num_classes (int): 类别数
        input_size (tuple): 输入尺寸 (batch, channels, height, width)
        opset_version (int): ONNX opset版本
        dynamic_axes (bool): 是否支持动态尺寸
    """
    
    print(f"加载模型: {model_path}")
    
    # 创建模型
    model = PrivilegedMultiTaskModel(
        task_mode=task_mode,
        num_classes=num_classes
    )
    
    # 加载权重
    checkpoint = torch.load(model_path, map_location='cpu')
    if 'model_state_dict' in checkpoint:
        model.load_state_dict(checkpoint['model_state_dict'])
        print(f"从checkpoint加载权重，epoch: {checkpoint.get('epoch', 'unknown')}")
    else:
        model.load_state_dict(checkpoint)
        print("加载模型权重")
    
    # 设置为评估模式
    model.eval()
    model = model.to(device)
    
    # 创建示例输入
    dummy_input = torch.randn(input_size).to(device)
    
    print(f"输入尺寸: {input_size}")
    print(f"模型任务模式: {task_mode}")
    print(f"类别数: {num_classes}")
    
    # 测试前向传播
    with torch.no_grad():
        outputs = model(dummy_input)
        print("输出形状:")
        for key, value in outputs.items():
            print(f"  {key}: {value.shape}")
    
    # 定义输入输出名称
    input_names = ['input_image']
    
    if task_mode == 'single':
        output_names = ['semantic_seg']
    else:
        output_names = ['damage_type', 'component', 'depth', 'damage_state']
    
    # 定义动态轴（如果需要）
    dynamic_axes_dict = None
    if dynamic_axes:
        dynamic_axes_dict = {
            'input_image': {0: 'batch_size', 2: 'height', 3: 'width'}
        }
        for output_name in output_names:
            if output_name == 'depth':
                dynamic_axes_dict[output_name] = {0: 'batch_size', 2: 'height', 3: 'width'}
            else:
                dynamic_axes_dict[output_name] = {0: 'batch_size', 2: 'height', 3: 'width'}
    
    # 创建输出目录
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    print(f"开始导出ONNX模型...")
    
    try:
        # 导出ONNX模型
        torch.onnx.export(
            model,                          # 模型
            dummy_input,                    # 示例输入
            output_path,                    # 输出路径
            export_params=True,             # 存储训练后的参数权重
            opset_version=opset_version,    # ONNX版本
            do_constant_folding=True,       # 是否执行常量折叠优化
            input_names=input_names,        # 输入名称
            output_names=output_names,      # 输出名称
            dynamic_axes=dynamic_axes_dict, # 动态轴
            verbose=False                   # 是否打印详细信息
        )
        
        print(f"ONNX模型已导出: {output_path}")
        
        # 验证导出的模型
        verify_onnx_model(output_path, dummy_input.cpu().numpy())
        
    except Exception as e:
        print(f"导出失败: {e}")
        return False
    
    return True

def verify_onnx_model(onnx_path, test_input):
    """
    验证导出的ONNX模型
    
    Args:
        onnx_path (str): ONNX模型路径
        test_input (np.ndarray): 测试输入
    """
    try:
        import onnx
        import onnxruntime as ort
        
        print("验证ONNX模型...")
        
        # 检查模型
        onnx_model = onnx.load(onnx_path)
        onnx.checker.check_model(onnx_model)
        print("✓ ONNX模型格式验证通过")
        
        # 创建推理会话
        ort_session = ort.InferenceSession(onnx_path)
        
        # 获取输入输出信息
        input_info = ort_session.get_inputs()[0]
        print(f"✓ 输入信息: {input_info.name}, 形状: {input_info.shape}, 类型: {input_info.type}")
        
        outputs_info = ort_session.get_outputs()
        print("✓ 输出信息:")
        for output in outputs_info:
            print(f"  {output.name}, 形状: {output.shape}, 类型: {output.type}")
        
        # 运行推理测试
        ort_inputs = {input_info.name: test_input}
        ort_outputs = ort_session.run(None, ort_inputs)
        
        print("✓ ONNX模型推理测试通过")
        print("输出形状:")
        for i, output in enumerate(outputs_info):
            print(f"  {output.name}: {ort_outputs[i].shape}")
        
        # 计算模型大小
        model_size = os.path.getsize(onnx_path) / (1024 * 1024)  # MB
        print(f"✓ 模型大小: {model_size:.2f} MB")
        
        return True
        
    except ImportError:
        print("警告: 未安装onnx或onnxruntime，跳过验证")
        print("请安装: pip install onnx onnxruntime")
        return False
    except Exception as e:
        print(f"验证失败: {e}")
        return False

def create_optimized_model(model_path, task_mode='single', num_classes=3):
    """
    创建优化的模型（移除不必要的操作）用于导出
    """
    
    class OptimizedModel(torch.nn.Module):
        def __init__(self, original_model, task_mode):
            super().__init__()
            self.original_model = original_model
            self.task_mode = task_mode
        
        def forward(self, x):
            # 强制返回logits，避免推理时的激活函数
            outputs = self.original_model(x, return_logits=True)
            
            if self.task_mode == 'single':
                # 只返回主要输出
                return outputs['semantic_seg']
            else:
                # 返回所有输出（按顺序）
                return (
                    outputs['damage_type'],
                    outputs['component'], 
                    outputs['depth'],
                    outputs['damage_state']
                )
    
    # 加载原始模型
    original_model = PrivilegedMultiTaskModel(
        task_mode=task_mode,
        num_classes=num_classes
    )
    
    checkpoint = torch.load(model_path, map_location='cpu')
    if 'model_state_dict' in checkpoint:
        original_model.load_state_dict(checkpoint['model_state_dict'])
    else:
        original_model.load_state_dict(checkpoint)
    
    original_model.eval()
    
    # 创建优化模型
    optimized_model = OptimizedModel(original_model, task_mode)
    optimized_model.eval()
    
    return optimized_model

def main():
    parser = argparse.ArgumentParser(description='导出PyTorch模型为ONNX格式')
    parser.add_argument('--model_path', type=str, required=True, 
                       help='PyTorch模型权重路径')
    parser.add_argument('--output_path', type=str, required=True,
                       help='输出ONNX模型路径')
    parser.add_argument('--device', type=str, default='cuda',
                       help='设备 (cuda/cpu)')
    parser.add_argument('--task_mode', type=str, default='single',
                       help='任务模式 (single/multi)')
    parser.add_argument('--num_classes', type=int, default=3,
                       help='分割类别数')
    parser.add_argument('--input_height', type=int, default=480,
                       help='输入图像高度')
    parser.add_argument('--input_width', type=int, default=480,
                       help='输入图像宽度')
    parser.add_argument('--batch_size', type=int, default=1,
                       help='批次大小')
    parser.add_argument('--opset_version', type=int, default=11,
                       help='ONNX opset版本')
    parser.add_argument('--dynamic_axes', action='store_true',
                       help='支持动态输入尺寸')
    parser.add_argument('--optimized', action='store_true',
                       help='使用优化的模型（仅输出logits）')
    
    args = parser.parse_args()
    
    # 检查输入文件是否存在
    if not os.path.exists(args.model_path):
        print(f"错误: 模型文件不存在: {args.model_path}")
        return
    
    # 设置输入尺寸
    input_size = (args.batch_size, 3, args.input_height, args.input_width)
    
    print("=" * 50)
    print("PyTorch 模型导出为 ONNX")
    print("=" * 50)
    
    if args.optimized:
        print("使用优化模型进行导出...")
        
        # 创建优化模型
        model = create_optimized_model(
            args.model_path, 
            args.task_mode, 
            args.num_classes
        )
        model = model.to(args.device)
        
        # 创建示例输入
        dummy_input = torch.randn(input_size).to(args.device)
        
        # 定义输入输出名称
        input_names = ['input_image']
        if args.task_mode == 'single':
            output_names = ['semantic_seg_logits']
        else:
            output_names = ['damage_type_logits', 'component_logits', 'depth', 'damage_state_logits']
        
        # 动态轴
        dynamic_axes_dict = None
        if args.dynamic_axes:
            dynamic_axes_dict = {
                'input_image': {0: 'batch_size', 2: 'height', 3: 'width'}
            }
            for output_name in output_names:
                dynamic_axes_dict[output_name] = {0: 'batch_size', 2: 'height', 3: 'width'}
        
        # 创建输出目录
        os.makedirs(os.path.dirname(args.output_path), exist_ok=True)
        
        try:
            torch.onnx.export(
                model,
                dummy_input,
                args.output_path,
                export_params=True,
                opset_version=args.opset_version,
                do_constant_folding=True,
                input_names=input_names,
                output_names=output_names,
                dynamic_axes=dynamic_axes_dict,
                verbose=False
            )
            
            print(f"优化ONNX模型已导出: {args.output_path}")
            
            # 验证模型
            verify_onnx_model(args.output_path, dummy_input.cpu().numpy())
            
        except Exception as e:
            print(f"导出失败: {e}")
    
    else:
        # 使用标准导出
        success = export_to_onnx(
            model_path=args.model_path,
            output_path=args.output_path,
            device=args.device,
            task_mode=args.task_mode,
            num_classes=args.num_classes,
            input_size=input_size,
            opset_version=args.opset_version,
            dynamic_axes=args.dynamic_axes
        )
        
        if not success:
            print("导出失败")
            return
    
    print("=" * 50)
    print("导出完成!")
    print("=" * 50)
    
    # 提供使用示例
    print("\n使用示例:")
    print("Python推理:")
    print(f"  import onnxruntime as ort")
    print(f"  session = ort.InferenceSession('{args.output_path}')")
    print(f"  outputs = session.run(None, {{'input_image': input_data}})")

if __name__ == "__main__":
    main()
