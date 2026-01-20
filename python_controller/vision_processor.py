"""
视觉处理模块 (Vision Processor)
集成 Vision Encoder (CLIP/SigLIP) 提取图像特征
用于 '建议C: VLA模型'
"""

import numpy as np
import time
from typing import Dict, List, Optional, Union
import json
import os

# 尝试导入 transformers，如果不存在则使用 Mock
try:
    import torch
    from transformers import AutoProcessor, AutoModel
    from PIL import Image
    TRANSFORMERS_AVAILABLE = True
except ImportError:
    TRANSFORMERS_AVAILABLE = False
    print("⚠️ Transformers/Torch not found. Using MockVisionEncoder.")

class VisionEncoder:
    """
    视觉编码器
    
    功能：
    1. 加载预训练视觉模型 (SigLIP/CLIP)
    2. 将图像编码为嵌入向量 (Embedding)
    """
    
    def __init__(self, model_name: str = "google/siglip-so400m-patch14-384"):
        self.model_name = model_name
        self.device = "cpu" # Default to CPU
        self.model = None
        self.processor = None
        self.is_mock = not TRANSFORMERS_AVAILABLE
        
        if not self.is_mock:
            self._load_model()
            
    def _load_model(self):
        """加载模型 (Lazy Loading)"""
        try:
            print(f"⏳ Loading Vision Model: {self.model_name}...")
            # 这里我们使用 SigLIP 或 CLIP
            # 注意：实际下载可能很大，这里代码主要演示加载逻辑
            # 为了避免第一次运行卡死，这里加了 try-except
            self.processor = AutoProcessor.from_pretrained(self.model_name)
            self.model = AutoModel.from_pretrained(self.model_name)
            
            if torch.cuda.is_available():
                self.device = "cuda"
                self.model = self.model.to(self.device)
                
            self.model.eval()
            print(f"✅ Vision Model Loaded on {self.device}")
            
        except Exception as e:
            print(f"❌ Failed to load model: {e}")
            print("⚠️ Falling back to Mock Mode")
            self.is_mock = True
            
    def encode_image(self, image_input: Union[np.ndarray, 'Image.Image']) -> np.ndarray:
        """
        编码图像
        
        Args:
            image_input: numpy array (H, W, 3) or PIL Image
            
        Returns:
            embedding: (Embedding_Dim,) numpy vector
        """
        if self.is_mock:
            # 返回随机向量模拟 Embeddings (SigLIP usually 1152 dim, CLIP 512/768)
            time.sleep(0.05) # Simulate inference time
            return np.random.randn(768).astype(np.float32)
            
        # 处理输入
        if isinstance(image_input, np.ndarray):
            image = Image.fromarray(image_input.astype('uint8'))
        else:
            image = image_input
            
        # 推理
        with torch.no_grad():
            inputs = self.processor(images=image, return_tensors="pt").to(self.device)
            # 根据模型不同，获取 embedding
            # SigLIP/CLIP 通常有 get_image_features 或类似方法
            if hasattr(self.model, "get_image_features"):
                image_features = self.model.get_image_features(**inputs)
            else:
                # 通用 transformers output
                outputs = self.model(**inputs)
                # 使用 pooler_output 或 last_hidden_state mean
                if hasattr(outputs, "pooler_output"):
                    image_features = outputs.pooler_output
                else:
                    image_features = outputs.last_hidden_state.mean(dim=1)
                    
            return image_features.cpu().numpy().flatten()

    def encode_text(self, text_list: List[str]) -> np.ndarray:
        """
        编码文本 (用于 Zero-shot 分类或 VLA 对齐)
        """
        if self.is_mock:
            return np.random.randn(len(text_list), 768).astype(np.float32)
            
        with torch.no_grad():
            inputs = self.processor(text=text_list, return_tensors="pt", padding=True).to(self.device)
            if hasattr(self.model, "get_text_features"):
                text_features = self.model.get_text_features(**inputs)
                return text_features.cpu().numpy()
            else:
                return np.zeros((len(text_list), 768)) # Fallback


# 测试代码
if __name__ == "__main__":
    print("视觉处理器测试...")
    
    # 初始化 (大概率会由 Transformers 未安装或网络问题进入 Mock 模式，这是预期的)
    encoder = VisionEncoder()
    print(f"模式: {'MOCK' if encoder.is_mock else 'REAL'}")
    
    # 创建假图像
    fake_img = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
    
    # 编码
    start = time.time()
    emb = encoder.encode_image(fake_img)
    duration = time.time() - start
    
    print(f"编码维度: {emb.shape}")
    print(f"耗时: {duration*1000:.1f}ms")
    
    # 模拟 VLA 场景
    cmd = "Walk forward"
    print(f"指令: {cmd}")
    # 这里我们只是演示，实际VLA需要将 text emb 和 img emb 融合
    
    print("✅ 视觉模块测试完成")
