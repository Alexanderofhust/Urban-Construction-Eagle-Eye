# model_optimized.py
import torch
import torch.nn as nn
import torch.nn.functional as F
import segmentation_models_pytorch as smp
from ASPP import ASPP
from segmentation_models_pytorch.base.heads import SegmentationHead
from segmentation_models_pytorch.decoders.unet.decoder import UnetDecoder

class PrivilegedMultiTaskModel(nn.Module):
    def __init__(self, encoder="efficientnet-b0", encoder_weights="imagenet", 
                 task_mode="single", num_classes=3):
        """
        改进的多任务模型，支持新数据集的单任务分割
        
        Args:
            encoder (str): 编码器名称
            encoder_weights (str): 预训练权重
            task_mode (str): 任务模式 "single" 或 "multi"
            num_classes (int): 分割类别数，仅在single模式下使用
        """
        super().__init__()
        
        self.task_mode = task_mode
        self.num_classes = num_classes
        
        # 1. --- 共享的编码器 ---
        # 我们只从smp.Unet中借用编码器
        self.encoder = smp.UnetPlusPlus(
            encoder_name=encoder,
            encoder_weights=encoder_weights,
            in_channels=3,
            classes=1,
        ).encoder

        # 2. --- 上下文增强模块 (ASPP)，放在网络瓶颈处 ---
        encoder_out_channels = self.encoder.out_channels[-1]
        self.aspp = ASPP(
            in_channels=encoder_out_channels,
            out_channels=320, # 可以自定义ASPP的输出通道数
            atrous_rates=(6, 12, 18),
        )

        # 3. --- 共享的解码器 ---
        # 解码器现在接收来自ASPP的特征
        # 直接实例化一个Unet模型，然后提取其decoder
        self.decoder = UnetDecoder(
            encoder_channels=self.encoder.out_channels,
            decoder_channels=(256, 128, 64, 32, 16),
            # 注意：解码器的第一个输入通道数需要匹配ASPP的输出
            n_blocks=5,
            use_batchnorm=True,
            center=True, # center=True时，它会期待一个bottleneck模块，这里我们用ASPP代替
            attention_type=None,
        )
        # 解码器在center=True时，它的第一个输入是ASPP的输出
        self.decoder.center = self.aspp

        # 4. --- 任务相关的分割头 (使用 nn.ModuleDict 管理) ---
        # 解码器最后的输出通道数是16
        decoder_final_channels = 16 
        
        if task_mode == "single":
            # 单任务模式：只有语义分割
            self.heads = nn.ModuleDict({
                "semantic_seg": SegmentationHead(in_channels=decoder_final_channels, out_channels=num_classes, kernel_size=1),
            })
        else:
            # 多任务模式：保留原有架构（向后兼容）
            self.heads = nn.ModuleDict({
                "damage_type": SegmentationHead(in_channels=decoder_final_channels, out_channels=3, kernel_size=1),
                "component": SegmentationHead(in_channels=decoder_final_channels, out_channels=8, kernel_size=1),
                "depth": SegmentationHead(in_channels=decoder_final_channels, out_channels=1, kernel_size=1),
                "damage_state": SegmentationHead(in_channels=decoder_final_channels, out_channels=5, kernel_size=1),
            })

    def forward(self, x, return_main_task_only=False, return_logits=False):
        # 1) 通过共享编码器提取多层级特征
        features = self.encoder(x)
        
        # 2) 解码器接收特征并进行上采样 (内部已包含ASPP和跳跃连接)
        decoder_output = self.decoder(*features)

        # 3) 将解码器输出送入所有头
        outputs = {key: head(decoder_output) for key, head in self.heads.items()}
        
        # 4) 根据模式和任务类型返回结果
        if self.training or return_logits:
            return outputs
            
        # 推理/评估时
        if self.task_mode == "single":
            if return_main_task_only:
                # 单任务模式：返回softmax激活的分割结果
                return F.softmax(outputs["semantic_seg"], dim=1)
            else:
                return {"semantic_seg": F.softmax(outputs["semantic_seg"], dim=1)}
        else:
            # 多任务模式：保留原有逻辑
            if return_main_task_only:
                # 只返回主任务的概率图
                return torch.sigmoid(outputs["damage_type"])

            # 返回所有任务的激活后结果
            return {
                "damage_type": torch.sigmoid(outputs["damage_type"]),
                "component": F.softmax(outputs["component"], dim=1),
                "depth": outputs["depth"],  # 深度是回归值，通常不加激活
                "damage_state": F.softmax(outputs["damage_state"], dim=1),
            }
        
if __name__ == "__main__":
    # 测试新数据集的单任务模式
    print("=== 测试单任务模式 (新数据集) ===")
    model_single = PrivilegedMultiTaskModel(task_mode="single", num_classes=3)
    print(f"单任务模型 heads: {list(model_single.heads.keys())}")

    # 测试前向传播
    dummy_input = torch.randn(1, 3, 480, 480)  # 假设输入尺寸为480x480
    outputs = model_single(dummy_input)
    
    for task_name, output in outputs.items():
        print(f"{task_name} output shape: {output.shape}")
    
    print("\n=== 测试多任务模式 (原数据集) ===")
    model_multi = PrivilegedMultiTaskModel(task_mode="multi")
    print(f"多任务模型 heads: {list(model_multi.heads.keys())}")
    outputs = model_multi(dummy_input)
    
    for task_name, output in outputs.items():
        print(f"{task_name} output shape: {output.shape}")