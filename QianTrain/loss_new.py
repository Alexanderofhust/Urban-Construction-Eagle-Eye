# loss_new.py - 适用于新数据集的损失函数
import torch
import torch.nn as nn
import torch.nn.functional as F
import segmentation_models_pytorch.losses as smp_losses

class SingleTaskLoss(nn.Module):
    """
    适用于新数据集的单任务语义分割损失函数
    支持CrossEntropy + Dice损失的组合
    """
    def __init__(self, num_classes=3, class_weights= None, 
                 use_dice=True, dice_weight=1.0, ce_weight=1.0,
                 ignore_index=-100, label_smoothing=0.0):
        """
        初始化损失函数
        
        Args:
            num_classes (int): 分割类别数
            class_weights (list/tensor): 各类别权重，用于处理类别不平衡
            use_dice (bool): 是否使用Dice损失
            dice_weight (float): Dice损失的权重
            ce_weight (float): CrossEntropy损失的权重
            ignore_index (int): 忽略的标签索引
            label_smoothing (float): 标签平滑参数
        """
        super().__init__()
        
        self.num_classes = num_classes
        self.use_dice = use_dice
        self.dice_weight = dice_weight
        self.ce_weight = ce_weight
        
        # CrossEntropy损失
        if class_weights is not None:
            if isinstance(class_weights, list):
                class_weights = torch.tensor(class_weights, dtype=torch.float32)
            self.register_buffer('class_weights', class_weights)
        else:
            self.class_weights = None
            
        self.ce_loss = nn.CrossEntropyLoss(
            weight=self.class_weights,
            ignore_index=ignore_index,
            label_smoothing=label_smoothing
        )
        
        # Dice损失
        if use_dice:
            self.dice_loss = smp_losses.DiceLoss(
                mode='multiclass',
                from_logits=True,
                ignore_index=ignore_index
            )
    
    def forward(self, predictions, targets):
        """
        计算损失
        
        Args:
            predictions (dict): 模型预测结果 {"semantic_seg": logits}
            targets (dict): 真实标签 {"semantic_seg": labels}
        
        Returns:
            tuple: (total_loss, loss_components)
        """
        if "semantic_seg" not in predictions or "semantic_seg" not in targets:
            raise ValueError("predictions和targets必须包含'semantic_seg'键")
        
        pred_logits = predictions["semantic_seg"]  # (B, C, H, W)
        target_labels = targets["semantic_seg"]    # (B, H, W)
        
        # 确保target是long类型
        if target_labels.dtype != torch.long:
            target_labels = target_labels.long()
        
        device = pred_logits.device
        total_loss = torch.tensor(0.0, device=device)
        loss_components = {}
        
        # CrossEntropy损失
        ce_loss_val = self.ce_loss(pred_logits, target_labels)
        total_loss += self.ce_weight * ce_loss_val
        loss_components['ce'] = ce_loss_val.item()
        
        # Dice损失
        if self.use_dice:
            dice_loss_val = self.dice_loss(pred_logits, target_labels)
            total_loss += self.dice_weight * dice_loss_val
            loss_components['dice'] = dice_loss_val.item()
        
        loss_components['total'] = total_loss.item()
        
        return total_loss, loss_components

class PrivilegedLossNew(nn.Module):
    """
    新的损失函数类，支持单任务和多任务模式
    向后兼容原有的多任务损失函数
    """
    def __init__(self, task_mode="single", **kwargs):
        """
        初始化损失函数
        
        Args:
            task_mode (str): "single" 或 "multi"
            **kwargs: 传递给具体损失函数的参数
        """
        super().__init__()
        
        self.task_mode = task_mode
        
        if task_mode == "single":
            # 单任务模式：使用SingleTaskLoss
            self.loss_fn = SingleTaskLoss(**kwargs)
        elif task_mode == "multi":
            # 多任务模式：使用原有的PrivilegedLoss（需要导入）
            from loss import PrivilegedLoss
            self.loss_fn = PrivilegedLoss(**kwargs)
        else:
            raise ValueError(f"不支持的task_mode: {task_mode}")
    
    def forward(self, predictions, targets):
        return self.loss_fn(predictions, targets)

# --- 一些辅助函数 ---
def calculate_class_weights(dataset, num_classes):
    """
    计算类别权重，用于处理类别不平衡
    
    Args:
        dataset: Dataset对象
        num_classes: 类别数
    
    Returns:
        torch.Tensor: 类别权重
    """
    print("计算类别权重...")
    
    class_counts = torch.zeros(num_classes)
    total_samples = min(1000, len(dataset))  # 最多分析1000个样本
    
    for i in range(total_samples):
        sample = dataset[i]
        labels = sample["labels"]["semantic_seg"]
        
        if isinstance(labels, torch.Tensor):
            labels = labels.numpy()
        
        for class_id in range(num_classes):
            class_counts[class_id] += (labels == class_id).sum()
    
    # 计算反频率权重
    total_pixels = class_counts.sum()
    weights = total_pixels / (num_classes * class_counts)
    
    # 避免除零
    weights[class_counts == 0] = 1.0
    
    print("类别权重:")
    for i, weight in enumerate(weights):
        print(f"  类别 {i}: {weight:.4f}")
    
    return weights

if __name__ == "__main__":
    # 测试单任务损失函数
    print("=== 测试单任务损失函数 ===")
    
    # 模拟预测和标签
    batch_size, num_classes, height, width = 2, 3, 256, 256
    
    # 预测logits
    pred_logits = torch.randn(batch_size, num_classes, height, width)
    
    # 真实标签
    target_labels = torch.randint(0, num_classes, (batch_size, height, width))
    
    predictions = {"semantic_seg": pred_logits}
    targets = {"semantic_seg": target_labels}
    
    # 测试不同配置的损失函数
    configs = [
        {"num_classes": 3, "use_dice": True},
        {"num_classes": 3, "use_dice": False},
        {"num_classes": 3, "class_weights": [1.0, 2.0, 3.0]},
    ]
    
    for i, config in enumerate(configs):
        print(f"\n配置 {i+1}: {config}")
        loss_fn = SingleTaskLoss(**config)
        
        total_loss, loss_components = loss_fn(predictions, targets)
        print(f"总损失: {total_loss:.4f}")
        print(f"损失组件: {loss_components}")
    
    print("\n=== 测试新的统一损失函数 ===")
    
    # 单任务模式
    loss_single = PrivilegedLossNew(task_mode="single", num_classes=3)
    total_loss, loss_components = loss_single(predictions, targets)
    print(f"单任务模式 - 总损失: {total_loss:.4f}, 组件: {loss_components}")
    
    # 多任务模式（如果原loss.py存在）
    try:
        loss_multi = PrivilegedLossNew(task_mode="multi")
        print("多任务模式损失函数创建成功")
        total_loss, loss_components = loss_multi(predictions, targets)
        print(f"多任务模式 - 总损失: {total_loss:.4f}, 组件: {loss_components}")
    except ImportError:
        print("多任务模式损失函数不可用，请确保原有的PrivilegedLoss已正确导入")
        total_loss, loss_components = None, None
        print(f"多任务模式 - 总损失: {total_loss}, 组件: {loss_components}")
