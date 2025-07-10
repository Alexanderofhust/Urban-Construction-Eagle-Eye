# dataset_new.py - 适用于新数据集的Dataset类
import os
import numpy as np
import torch
from torch.utils.data import Dataset
from PIL import Image
import albumentations as A
from albumentations.pytorch import ToTensorV2
import cv2
from tqdm import tqdm
import concurrent.futures

class NewBuildingDataset(Dataset):
    """
    为新建筑损伤数据集设计的PyTorch Dataset类
    数据结构：
    - images/: RGB图像文件 (.jpg)
    - masks/: 语义分割标签 (.png), 取值0,1,2
    """
    def __init__(self, data_dir, transform=None, use_cache=True, num_workers=None):
        """
        初始化数据集
        Args:
            data_dir (str): 数据目录，包含images和masks文件夹
            transform (albumentations.Compose, optional): 数据增强流程
            use_cache (bool): 是否将数据缓存到内存中
            num_workers (int): 并行加载数据的线程数
        """
        self.data_dir = data_dir
        self.transform = transform
        self.use_cache = use_cache
        
        # 获取图像和掩码路径
        images_dir = os.path.join(data_dir, 'images')
        masks_dir = os.path.join(data_dir, 'masks')
        
        if not os.path.exists(images_dir) or not os.path.exists(masks_dir):
            raise ValueError(f"数据目录 {data_dir} 必须包含 'images' 和 'masks' 文件夹")
        
        # 获取所有图像文件名
        image_files = [f for f in os.listdir(images_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
        
        # 检查对应的mask文件是否存在
        self.valid_files = []
        for img_file in image_files:
            base_name = os.path.splitext(img_file)[0]
            mask_file = base_name + '.png'  # mask文件假设都是png格式
            
            img_path = os.path.join(images_dir, img_file)
            mask_path = os.path.join(masks_dir, mask_file)
            
            if os.path.exists(mask_path):
                self.valid_files.append((img_path, mask_path))
            else:
                print(f"警告: 找不到对应的mask文件: {mask_path}")
        
        print(f"找到 {len(self.valid_files)} 个有效的图像-mask对")
        
        # 可选的内存缓存
        self.cache = []
        if use_cache and self.valid_files:
            self._cache_data(num_workers)
    
    def _cache_data(self, num_workers=None):
        """将所有数据加载到内存中"""
        if num_workers is None:
            num_workers = min(32, os.cpu_count() + 4)
        
        print(f"开始将数据缓存到内存中...")
        
        with concurrent.futures.ThreadPoolExecutor(max_workers=num_workers) as executor:
            future_to_file = {
                executor.submit(self._load_single_sample, img_path, mask_path): (img_path, mask_path)
                for img_path, mask_path in self.valid_files
            }
            
            for future in tqdm(concurrent.futures.as_completed(future_to_file), 
                             total=len(self.valid_files), desc="缓存数据"):
                sample = future.result()
                self.cache.append(sample)
        
        print(f"成功缓存 {len(self.cache)} 个样本到内存")
    
    def _load_single_sample(self, img_path, mask_path):
        """加载单个样本"""
        # 加载图像 (RGB)
        image = np.array(Image.open(img_path).convert("RGB"))
        
        # 加载mask (单通道，值为0,1,2,3)
        mask = np.array(Image.open(mask_path))
        
        # 如果mask是3通道的，取第一个通道
        if len(mask.shape) == 3:
            mask = mask[:, :, 0]
        
        # 忽略类别3，设置为2
        mask[mask == 3] = 2
        
        return {"image": image, "mask": mask}
    
    def __len__(self):
        if self.use_cache:
            return len(self.cache)
        else:
            return len(self.valid_files)
    
    def __getitem__(self, idx):
        if self.use_cache:
            # 从缓存中获取数据
            sample = self.cache[idx]
            image = sample["image"]
            mask = sample["mask"]
        else:
            # 直接从磁盘加载
            img_path, mask_path = self.valid_files[idx]
            sample = self._load_single_sample(img_path, mask_path)
            image = sample["image"]
            mask = sample["mask"]
        
        # 应用数据增强
        if self.transform:
            augmented = self.transform(image=image, mask=mask)
            image = augmented['image']
            mask = augmented['mask']
        else:
            # 如果没有transform，需要手动转换为tensor
            image = torch.from_numpy(image).permute(2, 0, 1).float() / 255.0
            mask = torch.from_numpy(mask).long()
        
        return {
            "image": image,
            "labels": {
                "semantic_seg": mask,  # 语义分割标签，形状为(H, W)
            }
        }

# --- 针对新数据集的数据增强流程 ---
def get_new_transforms(split='train', resize_shape=(480, 480)):
    """
    获取适用于新数据集的数据增强流程
    
    Args:
        split (str): 'train' 或 'val'
        resize_shape (tuple): 目标图像尺寸
    
    Returns:
        albumentations.Compose: 数据增强流程
    """
    if split == 'train':
        return A.Compose([
            A.Resize(height=resize_shape[0], width=resize_shape[1], interpolation=cv2.INTER_NEAREST),
            A.HorizontalFlip(p=0.5),
            A.VerticalFlip(p=0.3),  # 建筑损伤检测中垂直翻转也可能有用
            A.ShiftScaleRotate(
                shift_limit=0.0625, 
                scale_limit=0.1, 
                rotate_limit=15, 
                p=0.7, 
                border_mode=cv2.BORDER_CONSTANT
            ),
            A.RandomBrightnessContrast(p=0.5),
            A.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1, p=0.5),
            A.GaussianBlur(blur_limit=(3, 7), p=0.3),  # 模糊增强
            A.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ToTensorV2(),
        ])
    elif split in ['val', 'test']:
        return A.Compose([
            A.Resize(height=resize_shape[0], width=resize_shape[1], interpolation=cv2.INTER_NEAREST),
            A.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ToTensorV2(),
        ])
    else:
        raise ValueError(f"Unsupported split: {split}")

# --- 数据集统计工具 ---
def analyze_dataset(data_dir):
    """
    分析数据集的基本统计信息
    
    Args:
        data_dir (str): 数据目录
    """
    print(f"分析数据集: {data_dir}")
    
    # 创建临时dataset来分析
    dataset = NewBuildingDataset(data_dir, transform=None, use_cache=False)
    
    if len(dataset) == 0:
        print("数据集为空!")
        return
    
    # 分析mask的类别分布
    class_counts = {}
    total_pixels = 0
    
    print("分析mask标签分布...")
    for i in tqdm(range(min(100, len(dataset))), desc="分析样本"):  # 只分析前100个样本
        sample = dataset[i]
        mask = sample["labels"]["semantic_seg"]
        
        if isinstance(mask, torch.Tensor):
            mask = mask.numpy()
        
        unique_values, counts = np.unique(mask, return_counts=True)
        
        for val, count in zip(unique_values, counts):
            if val not in class_counts:
                class_counts[val] = 0
            class_counts[val] += count
            total_pixels += count
    
    print("\n=== 类别分布统计 ===")
    for class_id in sorted(class_counts.keys()):
        count = class_counts[class_id]
        percentage = (count / total_pixels) * 100
        print(f"类别 {class_id}: {count:,} 像素 ({percentage:.2f}%)")
    
    # 计算类别权重（用于损失函数）
    if len(class_counts) > 1:
        total_samples = sum(class_counts.values())
        n_classes = len(class_counts)
        weights = []
        
        print("\n=== 建议的类别权重 ===")
        for class_id in sorted(class_counts.keys()):
            # 反频率加权
            weight = total_samples / (n_classes * class_counts[class_id])
            weights.append(weight)
            print(f"类别 {class_id}: 权重 = {weight:.4f}")
        
        print(f"权重tensor: {weights}")

if __name__ == "__main__":
    # 测试新数据集
    data_dir = "/data/zdy/BuildingYOLO/newdata"
    
    # 分析数据集
    analyze_dataset(data_dir)
    
    # 测试dataset
    print("\n=== 测试Dataset ===")
    transform = get_new_transforms('train')
    dataset = NewBuildingDataset(data_dir, transform=transform, use_cache=True)
    
    print(f"数据集大小: {len(dataset)}")
    
    if len(dataset) > 0:
        sample = dataset[0]
        print(f"图像形状: {sample['image'].shape}")
        print(f"mask形状: {sample['labels']['semantic_seg'].shape}")
        print(f"mask数据类型: {sample['labels']['semantic_seg'].dtype}")
        print(f"mask取值范围: {sample['labels']['semantic_seg'].min()} - {sample['labels']['semantic_seg'].max()}")
