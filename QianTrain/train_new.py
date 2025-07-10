# train_new.py - 适用于新数据集的训练脚本
import os
import torch
import torch.optim as optim
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm
import torchmetrics
import numpy as np
from sklearn.metrics import confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns
import argparse

# 从我们的模块中导入
from model import PrivilegedMultiTaskModel
from dataset_new import NewBuildingDataset, get_new_transforms, analyze_dataset
from loss_new import PrivilegedLossNew, calculate_class_weights

# --- 配置参数 ---
class Config:
    # 路径配置
    DATA_ROOT = '/data/zdy/BuildingYOLO/newdata'  # 新数据集路径
    CHECKPOINT_DIR = './checkpoints_new'
    LOG_DIR = './logs_new'
    
    # 训练配置
    NUM_EPOCHS = 200
    BATCH_SIZE = 16
    VAL_BATCH_SIZE = 24
    LEARNING_RATE = 1e-3
    WEIGHT_DECAY = 1e-4
    IMG_SIZE = (480, 480)
    
    # 模型配置
    ENCODER = "efficientnet-b0"
    TASK_MODE = "single"  # "single" 或 "multi"
    NUM_CLASSES = 3  # 只使用3个类别 (0, 1, 2)，忽略类别3
    PRETRAINED_MODEL_PATH = None  # 预训练模型路径，None表示从头训练
    FREEZE_ENCODER = False  # 是否冻结编码器参数
    
    # 损失函数配置
    USE_DICE = True
    DICE_WEIGHT = 1.0
    CE_WEIGHT = 1.0
    USE_CLASS_WEIGHTS = True
    LABEL_SMOOTHING = 0.0  # 减少标签平滑，因为类别极不平衡
    
    # 设备和优化器
    DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
    OPTIMIZER = 'adamw'  # 'adam' 或 'adamw'
    SCHEDULER = 'cosine'  # 'cosine', 'step', 或 None
    
    # 其他配置
    SAVE_INTERVAL = 20
    TRAIN_WORKER = 4
    VAL_WORKER = 4
    USE_AMP = True  # 混合精度训练
    EARLY_STOPPING_PATIENCE = 50

def create_data_loaders(config):
    """创建训练和验证数据加载器"""
    
    # 分析数据集
    print("=== 数据集分析 ===")
    analyze_dataset(config.DATA_ROOT)
    
    # 创建transforms
    train_transform = get_new_transforms('train', config.IMG_SIZE)
    val_transform = get_new_transforms('val', config.IMG_SIZE)
    
    # 创建数据集
    # 这里简单起见，我们使用整个数据集进行训练
    # 在实际应用中，您可能需要划分训练/验证集
    print("\n=== 创建数据集 ===")
    
    # 方案1：使用整个数据集作为训练集（简单测试）
    train_dataset = NewBuildingDataset(
        config.DATA_ROOT, 
        transform=train_transform, 
        use_cache=True,
        num_workers=config.TRAIN_WORKER
    )
    
    # 使用部分数据作为验证集（取前10%）
    val_size = int(len(train_dataset) * 0.1)
    val_indices = list(range(0, val_size))
    train_indices = list(range(val_size, len(train_dataset)))
    
    # 创建子集
    train_subset = torch.utils.data.Subset(train_dataset, train_indices)
    val_subset = torch.utils.data.Subset(train_dataset, val_indices)
    
    # 为验证集创建单独的dataset with val transform
    val_dataset = NewBuildingDataset(
        config.DATA_ROOT, 
        transform=val_transform, 
        use_cache=True,
        num_workers=config.VAL_WORKER
    )
    val_subset = torch.utils.data.Subset(val_dataset, val_indices)
    
    # 计算类别权重
    if config.USE_CLASS_WEIGHTS:
        class_weights = calculate_class_weights(train_dataset, config.NUM_CLASSES)
    else:
        class_weights = None
    
    # 创建数据加载器
    train_loader = DataLoader(
        train_subset,
        batch_size=config.BATCH_SIZE,
        shuffle=True,
        num_workers=config.TRAIN_WORKER,
        pin_memory=True
    )
    
    val_loader = DataLoader(
        val_subset,
        batch_size=config.VAL_BATCH_SIZE,
        shuffle=False,
        num_workers=config.VAL_WORKER,
        pin_memory=True
    )
    
    print(f"训练集大小: {len(train_subset)}")
    print(f"验证集大小: {len(val_subset)}")
    
    return train_loader, val_loader, class_weights

def create_model_and_optimizer(config, class_weights=None):
    """创建模型、损失函数和优化器"""
    
    # 创建模型
    model = PrivilegedMultiTaskModel(
        encoder=config.ENCODER,
        task_mode=config.TASK_MODE,
        num_classes=config.NUM_CLASSES
    ).to(config.DEVICE)
    
    # 加载预训练模型权重
    if config.PRETRAINED_MODEL_PATH is not None:
        print(f"加载预训练模型: {config.PRETRAINED_MODEL_PATH}")
        
        if not os.path.exists(config.PRETRAINED_MODEL_PATH):
            raise ValueError(f"预训练模型路径不存在: {config.PRETRAINED_MODEL_PATH}")
        
        try:
            checkpoint = torch.load(config.PRETRAINED_MODEL_PATH, map_location=config.DEVICE)
            
            # 检查是否是完整的checkpoint还是仅模型权重
            if 'model_state_dict' in checkpoint:
                pretrained_dict = checkpoint['model_state_dict']
                print(f"从checkpoint中加载模型权重，epoch: {checkpoint.get('epoch', 'unknown')}")
            else:
                pretrained_dict = checkpoint
                print("加载模型权重文件")
            
            # 获取当前模型的state_dict
            model_dict = model.state_dict()
            
            # 过滤出维度匹配的权重
            matched_dict = {}
            unmatched_keys = []
            
            for k, v in pretrained_dict.items():
                if k in model_dict:
                    if model_dict[k].shape == v.shape:
                        matched_dict[k] = v
                    else:
                        unmatched_keys.append(f"{k}: {model_dict[k].shape} vs {v.shape}")
                else:
                    unmatched_keys.append(f"{k}: not found in current model")
            
            # 更新模型权重
            model_dict.update(matched_dict)
            model.load_state_dict(model_dict)
            
            print(f"成功加载 {len(matched_dict)} 个匹配的权重")
            if unmatched_keys:
                print(f"跳过 {len(unmatched_keys)} 个不匹配的权重:")
                for key in unmatched_keys[:5]:  # 只显示前5个
                    print(f"  - {key}")
                if len(unmatched_keys) > 5:
                    print(f"  - ... 还有 {len(unmatched_keys) - 5} 个")
                    
        except Exception as e:
            print(f"加载预训练模型失败: {e}")
            print("将从头开始训练")
    
    # 冻结编码器参数（如果需要）
    if config.FREEZE_ENCODER:
        print("冻结编码器参数")
        for param in model.encoder.parameters():
            param.requires_grad = False
        # 如果有ASPP，也可以选择冻结
        # for param in model.aspp.parameters():
        #     param.requires_grad = False
    
    # 创建损失函数
    loss_kwargs = {
        'num_classes': config.NUM_CLASSES,
        'class_weights': class_weights,
        'use_dice': config.USE_DICE,
        'dice_weight': config.DICE_WEIGHT,
        'ce_weight': config.CE_WEIGHT,
        'label_smoothing': config.LABEL_SMOOTHING
    }
    
    criterion = PrivilegedLossNew(
        task_mode=config.TASK_MODE,
        **loss_kwargs
    ).to(config.DEVICE)
    
    # 创建优化器
    if config.OPTIMIZER.lower() == 'adam':
        optimizer = optim.Adam(
            model.parameters(),
            lr=config.LEARNING_RATE,
            weight_decay=config.WEIGHT_DECAY
        )
    elif config.OPTIMIZER.lower() == 'adamw':
        optimizer = optim.AdamW(
            model.parameters(),
            lr=config.LEARNING_RATE,
            weight_decay=config.WEIGHT_DECAY
        )
    else:
        raise ValueError(f"不支持的优化器: {config.OPTIMIZER}")
    
    # 创建学习率调度器
    scheduler = None
    if config.SCHEDULER == 'cosine':
        scheduler = optim.lr_scheduler.CosineAnnealingLR(
            optimizer, T_max=config.NUM_EPOCHS
        )
    elif config.SCHEDULER == 'step':
        scheduler = optim.lr_scheduler.StepLR(
            optimizer, step_size=50, gamma=0.5
        )
    
    return model, criterion, optimizer, scheduler

def train_one_epoch(model, train_loader, criterion, optimizer, scaler, device, epoch):
    """训练一个epoch"""
    model.train()
    
    total_loss = 0.0
    total_samples = 0
    loss_components_sum = {}
    
    # 用于计算准确率
    correct_pixels = 0
    total_pixels = 0
    
    progress_bar = tqdm(train_loader, desc=f"Epoch {epoch+1} [Train]")
    
    for batch_idx, batch in enumerate(progress_bar):
        # 数据移动到设备
        images = batch["image"].to(device)
        targets = {k: v.to(device) for k, v in batch["labels"].items()}
        
        optimizer.zero_grad()
        
        # 前向传播（使用混合精度）
        with torch.autocast(device_type='cuda' if device == 'cuda' else 'cpu', enabled=scaler is not None):
            predictions = model(images)
            loss, loss_components = criterion(predictions, targets)
        
        # 反向传播
        if scaler is not None:
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
        else:
            loss.backward()
            optimizer.step()
        
        # 统计信息
        batch_size = images.size(0)
        total_loss += loss.item() * batch_size
        total_samples += batch_size
        
        # 累计损失组件
        for key, value in loss_components.items():
            if key in loss_components_sum:
                loss_components_sum[key] += value * batch_size
            else:
                loss_components_sum[key] = value * batch_size
        
        # 计算像素准确率
        with torch.no_grad():
            pred_classes = torch.argmax(predictions["semantic_seg"], dim=1)
            target_classes = targets["semantic_seg"]
            
            correct_pixels += (pred_classes == target_classes).sum().item()
            total_pixels += target_classes.numel()
        
        # 更新进度条
        progress_bar.set_postfix({
            'Loss': f'{loss.item():.4f}',
            'Pixel_Acc': f'{correct_pixels/total_pixels:.4f}'
        })
    
    # 计算平均值
    avg_loss = total_loss / total_samples
    avg_loss_components = {k: v / total_samples for k, v in loss_components_sum.items()}
    pixel_accuracy = correct_pixels / total_pixels
    
    return avg_loss, avg_loss_components, pixel_accuracy

def validate_one_epoch(model, val_loader, criterion, device, num_classes):
    """验证一个epoch"""
    model.eval()
    
    total_loss = 0.0
    total_samples = 0
    loss_components_sum = {}
    
    # 用于计算指标
    correct_pixels = 0
    total_pixels = 0
    all_predictions = []
    all_targets = []
    
    progress_bar = tqdm(val_loader, desc="Validation")
    
    with torch.no_grad():
        for batch in progress_bar:
            # 数据移动到设备
            images = batch["image"].to(device)
            targets = {k: v.to(device) for k, v in batch["labels"].items()}
            
            # 前向传播
            predictions = model(images)
            loss, loss_components = criterion(predictions, targets)
            
            # 统计信息
            batch_size = images.size(0)
            total_loss += loss.item() * batch_size
            total_samples += batch_size
            
            # 累计损失组件
            for key, value in loss_components.items():
                if key in loss_components_sum:
                    loss_components_sum[key] += value * batch_size
                else:
                    loss_components_sum[key] = value * batch_size
            
            # 计算像素准确率和收集预测结果
            pred_classes = torch.argmax(predictions["semantic_seg"], dim=1)
            target_classes = targets["semantic_seg"]
            
            correct_pixels += (pred_classes == target_classes).sum().item()
            total_pixels += target_classes.numel()
            
            # 收集预测和目标用于计算其他指标
            all_predictions.extend(pred_classes.cpu().numpy().flatten())
            all_targets.extend(target_classes.cpu().numpy().flatten())
            
            # 更新进度条
            progress_bar.set_postfix({
                'Loss': f'{loss.item():.4f}',
                'Pixel_Acc': f'{correct_pixels/total_pixels:.4f}'
            })
    
    # 计算平均值
    avg_loss = total_loss / total_samples
    avg_loss_components = {k: v / total_samples for k, v in loss_components_sum.items()}
    pixel_accuracy = correct_pixels / total_pixels
    
    # 计算每类IoU和mIoU
    confusion_mat = confusion_matrix(all_targets, all_predictions, labels=list(range(num_classes)))
    
    # 计算IoU
    ious = []
    for i in range(num_classes):
        tp = confusion_mat[i, i]
        fp = confusion_mat[:, i].sum() - tp
        fn = confusion_mat[i, :].sum() - tp
        
        if tp + fp + fn > 0:
            iou = tp / (tp + fp + fn)
        else:
            iou = 0.0
        
        ious.append(iou)
    
    mean_iou = np.mean(ious)
    
    metrics = {
        'avg_loss': avg_loss,
        'loss_components': avg_loss_components,
        'pixel_accuracy': pixel_accuracy,
        'mean_iou': mean_iou,
        'class_ious': ious,
        'confusion_matrix': confusion_mat
    }
    
    return metrics

def save_checkpoint(model, optimizer, scheduler, epoch, best_miou, checkpoint_dir, is_best=False):
    """保存检查点"""
    os.makedirs(checkpoint_dir, exist_ok=True)
    
    checkpoint = {
        'epoch': epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'best_miou': best_miou,
    }
    
    if scheduler is not None:
        checkpoint['scheduler_state_dict'] = scheduler.state_dict()
    
    # 保存常规检查点
    checkpoint_path = os.path.join(checkpoint_dir, f'model_epoch_{epoch}.pth')
    torch.save(checkpoint, checkpoint_path)
    
    # 保存最佳模型
    if is_best:
        best_path = os.path.join(checkpoint_dir, 'best_model.pth')
        torch.save(checkpoint, best_path)
        print(f"保存最佳模型: {best_path}")

def main():
    """主训练函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='新数据集训练脚本')
    parser.add_argument('--pretrained', type=str, default="checkpoints_new/best_model.pth", 
                       help='预训练模型权重路径')
    parser.add_argument('--freeze_encoder', action='store_true', 
                       help='是否冻结编码器参数')
    parser.add_argument('--data_root', type=str, default='/data/zdy/BuildingYOLO/newdata',
                       help='数据集根目录')
    parser.add_argument('--batch_size', type=int, default=16,
                       help='训练批次大小')
    parser.add_argument('--learning_rate', type=float, default=1e-3,
                       help='学习率')
    parser.add_argument('--num_epochs', type=int, default=200,
                       help='训练轮数')
    parser.add_argument('--num_classes', type=int, default=3,
                       help='分割类别数')
    parser.add_argument('--checkpoint_dir', type=str, default='./checkpoints_new',
                       help='检查点保存目录')
    
    args = parser.parse_args()
    
    config = Config()
    
    # 用命令行参数覆盖配置
    if args.pretrained:
        config.PRETRAINED_MODEL_PATH = args.pretrained
    config.FREEZE_ENCODER = args.freeze_encoder
    config.DATA_ROOT = args.data_root
    config.BATCH_SIZE = args.batch_size
    config.LEARNING_RATE = args.learning_rate
    config.NUM_EPOCHS = args.num_epochs
    config.NUM_CLASSES = args.num_classes
    config.CHECKPOINT_DIR = args.checkpoint_dir
    
    # 创建输出目录
    os.makedirs(config.CHECKPOINT_DIR, exist_ok=True)
    os.makedirs(config.LOG_DIR, exist_ok=True)
    
    # 创建tensorboard写入器
    run_name = f"exp_{len(os.listdir(config.LOG_DIR)) if os.path.exists(config.LOG_DIR) else 0}"
    log_run_dir = os.path.join(config.LOG_DIR, run_name)
    writer = SummaryWriter(log_run_dir)
    
    print(f"设备: {config.DEVICE}")
    print(f"任务模式: {config.TASK_MODE}")
    print(f"类别数: {config.NUM_CLASSES}")
    print(f"日志目录: {log_run_dir}")
    
    # 创建数据加载器
    train_loader, val_loader, class_weights = create_data_loaders(config)
    
    # 创建模型、损失函数和优化器
    model, criterion, optimizer, scheduler = create_model_and_optimizer(config, class_weights)
    
    # 混合精度scaler
    scaler = torch.cuda.amp.GradScaler() if config.USE_AMP and config.DEVICE == 'cuda' else None
    
    # 训练循环
    best_miou = 0.0
    patience_counter = 0
    
    print(f"\n开始训练 {config.NUM_EPOCHS} 个epochs...")
    
    for epoch in range(config.NUM_EPOCHS):
        # 训练
        train_loss, train_loss_components, train_pixel_acc = train_one_epoch(
            model, train_loader, criterion, optimizer, scaler, config.DEVICE, epoch
        )
        
        # 验证
        val_metrics = validate_one_epoch(
            model, val_loader, criterion, config.DEVICE, config.NUM_CLASSES
        )
        
        # 学习率调度
        if scheduler is not None:
            scheduler.step()
        
        # 记录到tensorboard
        writer.add_scalar('Loss/Train', train_loss, epoch)
        writer.add_scalar('Loss/Val', val_metrics['avg_loss'], epoch)
        writer.add_scalar('Accuracy/Train_Pixel', train_pixel_acc, epoch)
        writer.add_scalar('Accuracy/Val_Pixel', val_metrics['pixel_accuracy'], epoch)
        writer.add_scalar('IoU/Val_Mean', val_metrics['mean_iou'], epoch)
        
        if (epoch + 1) % 5 == 0:
            for i, iou in enumerate(val_metrics['class_ious']):
                writer.add_scalar(f'IoU/Val_Class_{i}', iou, epoch)
        
        # 打印信息
        print(f"\nEpoch {epoch+1}/{config.NUM_EPOCHS}:")
        print(f"Train Loss: {train_loss:.4f}, Train Pixel Acc: {train_pixel_acc:.4f}")
        print(f"Val Loss: {val_metrics['avg_loss']:.4f}, Val Pixel Acc: {val_metrics['pixel_accuracy']:.4f}")
        print(f"Val mIoU: {val_metrics['mean_iou']:.4f}")
        print(f"Class IoUs: {[f'{iou:.4f}' for iou in val_metrics['class_ious']]}")
        
        # 保存检查点
        is_best = val_metrics['mean_iou'] > best_miou
        if is_best:
            best_miou = val_metrics['mean_iou']
            patience_counter = 0
        else:
            patience_counter += 1
        
        if (epoch + 1) % config.SAVE_INTERVAL == 0 or is_best:
            save_checkpoint(
                model, optimizer, scheduler, epoch, best_miou, 
                config.CHECKPOINT_DIR, is_best
            )
        
        # 早停
        if patience_counter >= config.EARLY_STOPPING_PATIENCE:
            print(f"早停: {config.EARLY_STOPPING_PATIENCE} epochs内验证指标未改善")
            break
    
    writer.close()
    print(f"\n训练完成! 最佳 mIoU: {best_miou:.4f}")

if __name__ == "__main__":
    main()
