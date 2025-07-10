# summary.py - 模型摘要工具
import torch
from torchinfo import summary
from model import PrivilegedMultiTaskModel # 确保从你优化后的模型文件导入

def get_model_summary(model, input_size):
    """
    打印模型的详细摘要，包括参数量、MACs/GFLOPs等。
    """
    print(f"--- Model Summary (Input Size: {input_size}) ---")
    
    # 使用 torchinfo.summary
    # - input_size: (batch_size, channels, height, width)
    # - col_names: 定义要显示的列
    # - device: 指定设备
    # - verbose: 0=安静, 1=摘要, 2=详细
    summary(
        model,
        input_size=input_size,
        col_names=["input_size", "output_size", "num_params", "mult_adds"],
        device="cpu",
        verbose=1,
    )

if __name__ == '__main__':
    # 定义一个示例输入尺寸 (batch_size=1, channels=3, height=480, width=480)
    # 这个尺寸应该和你训练时使用的尺寸一致
    sample_input_size = (1, 3, 480, 480)

    print("=== 测试新数据集的单任务模型 ===")
    # 实例化单任务模型（新数据集）
    model_single = PrivilegedMultiTaskModel(
        encoder="efficientnet-b0",
        task_mode="single",
        num_classes=3
    )
    model_single.eval()

    # 打印摘要
    get_model_summary(model_single, sample_input_size)
    
    # 模拟一次前向传播来检查输出形状
    print("\n--- Testing Forward Pass (Single Task Mode) ---")
    dummy_input = torch.randn(sample_input_size)
    
    # 获取单任务输出
    outputs = model_single(dummy_input)
    print("Output shapes for single task:")
    for task_name, tensor in outputs.items():
        print(f"  - {task_name}: {tensor.shape}")
        
    # 只获取主任务的输出
    main_task_output = model_single(dummy_input, return_main_task_only=True)
    print("\nOutput shape for main task only:")
    print(f"  - semantic_seg: {main_task_output.shape}")
    
    print("\n" + "="*60)
    print("=== 测试原数据集的多任务模型 ===")
    
    # 实例化多任务模型（原数据集，向后兼容）
    model_multi = PrivilegedMultiTaskModel(
        encoder="efficientnet-b0",
        task_mode="multi"
    )
    model_multi.eval()

    # 打印摘要
    get_model_summary(model_multi, sample_input_size)
    
    # 模拟一次前向传播来检查输出形状
    print("\n--- Testing Forward Pass (Multi Task Mode) ---")
    
    # 获取所有任务的输出
    outputs = model_multi(dummy_input)
    print("Output shapes for all tasks:")
    for task_name, tensor in outputs.items():
        print(f"  - {task_name}: {tensor.shape}")
        
    # 只获取主任务的输出
    main_task_output = model_multi(dummy_input, return_main_task_only=True)
    print("\nOutput shape for main task only:")
    print(f"  - damage_type: {main_task_output.shape}")