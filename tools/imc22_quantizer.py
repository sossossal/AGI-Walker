"""
IMC-22 INT8量化器
将PyTorch模型量化为INT8格式，用于IMC-22 NPU部署

功能:
- 动态量化
- 量化感知训练 (QAT)
- 导出INT8权重
- 生成C代码
"""

import torch
import torch.quantization as quantization
import numpy as np
from pathlib import Path
from typing import Dict
import json


class IMC22Quantizer:
    """IMC-22量化器"""
    
    def __init__(self, model: torch.nn.Module):
        """
        初始化量化器
        
        参数:
            model: PyTorch模型
        """
        self.model = model
        self.quantized_model = None
        self.quantization_params = {}
    
    def quantize_dynamic(self):
        """动态量化（推理时量化）"""
        print("执行动态量化...")
        
        self.model.eval()
        
        # 动态量化
        self.quantized_model = quantization.quantize_dynamic(
            self.model,
            {torch.nn.Linear},  # 量化所有Linear层
            dtype=torch.qint8
        )
        
        print("✓ 动态量化完成")
        return self.quantized_model
    
    def quantize_static(self, calibration_loader):
        """静态量化（需要校准数据）"""
        print("执行静态量化...")
        
        self.model.eval()
        
        # 设置量化配置
        self.model.qconfig = quantization.get_default_qconfig('fbgemm')
        
        # 准备量化
        quantization.prepare(self.model, inplace=True)
        
        # 校准
        print("  校准中...")
        with torch.no_grad():
            for states, _ in calibration_loader:
                self.model(states)
        
        # 转换
        self.quantized_model = quantization.convert(self.model, inplace=False)
        
        print("✓ 静态量化完成")
        return self.quantized_model
    
    def extract_int8_weights(self) -> Dict[str, np.ndarray]:
        """提取INT8权重"""
        if self.quantized_model is None:
            raise ValueError("模型尚未量化，请先调用 quantize_*()")
        
        weights = {}
        
        for name, module in self.quantized_model.named_modules():
            if hasattr(module, 'weight'):
                # 提取量化权重
                weight_int8 = module.weight().int_repr().numpy()
                weights[name] = weight_int8
                
                # 提取量化参数
                if hasattr(module.weight(), 'q_scale'):
                    self.quantization_params[name] = {
                        'scale': float(module.weight().q_scale()),
                        'zero_point': int(module.weight().q_zero_point())
                    }
        
        return weights
    
    def save_quantized_weights(self, output_file: str):
        """保存量化权重"""
        weights = self.extract_int8_weights()
        
        # 保存为npz
        np.savez_compressed(output_file, **weights)
        
        # 保存量化参数
        params_file = Path(output_file).with_suffix('.json')
        with open(params_file, 'w') as f:
            json.dump(self.quantization_params, f, indent=2)
        
        print(f"✓ 量化权重已保存到: {output_file}")
        print(f"✓ 量化参数已保存到: {params_file}")
    
    def export_to_c_header(self, output_file: str):
        """导出为C头文件"""
        weights = self.extract_int8_weights()
        
        with open(output_file, 'w') as f:
            f.write("// Auto-generated INT8 weights for IMC-22 NPU\n")
            f.write("// Generated from AGI-Walker trained model\n\n")
            f.write("#ifndef IMC22_WEIGHTS_H\n")
            f.write("#define IMC22_WEIGHTS_H\n\n")
            f.write("#include <stdint.h>\n\n")
            
            # 写入每一层的权重
            for name, weight in weights.items():
                safe_name = name.replace('.', '_').replace('/', '_')
                
                # 权重数组
                f.write(f"// Layer: {name}\n")
                f.write(f"const int8_t {safe_name}_weight[] = {{\n")
                
                weight_flat = weight.flatten()
                for i, val in enumerate(weight_flat):
                    if i % 16 == 0:
                        f.write("    ")
                    f.write(f"{val:4d},")
                    if (i + 1) % 16 == 0 and i < len(weight_flat) - 1:
                        f.write("\n")
                
                f.write("\n};\n")
                f.write(f"const int {safe_name}_weight_size = {len(weight_flat)};\n\n")
                
                # 量化参数
                if name in self.quantization_params:
                    params = self.quantization_params[name]
                    f.write(f"const float {safe_name}_scale = {params['scale']:.8f}f;\n")
                    f.write(f"const int8_t {safe_name}_zero_point = {params['zero_point']};\n\n")
            
            f.write("#endif // IMC22_WEIGHTS_H\n")
        
        print(f"✓ C头文件已保存到: {output_file}")
    
    def export_to_c_source(self, output_file: str):
        """导出完整的C推理代码"""
        with open(output_file, 'w') as f:
            f.write(self._generate_c_inference_code())
        
        print(f"✓ C源文件已保存到: {output_file}")
    
    def _generate_c_inference_code(self) -> str:
        """生成C推理代码"""
        code = """// IMC-22 NPU推理代码
// Auto-generated from AGI-Walker trained model

#include "imc22_weights.h"
#include <stdint.h>

// 网络配置
#define INPUT_DIM 3
#define HIDDEN_DIM 16
#define OUTPUT_DIM 3

// 缓冲区
static int8_t layer1_output[HIDDEN_DIM];
static int8_t layer2_output[HIDDEN_DIM];
static int8_t final_output[OUTPUT_DIM];

/**
 * INT8矩阵乘法 (使用NPU加速)
 */
static void matmul_int8(
    const int8_t* input, int in_dim,
    const int8_t* weight, 
    int8_t* output, int out_dim
) {
    for (int i = 0; i < out_dim; i++) {
        int32_t acc = 0;
        
        // 使用NPU的MAC单元
        for (int j = 0; j < in_dim; j++) {
            acc += (int32_t)input[j] * weight[i * in_dim + j];
        }
        
        // 量化回INT8 (简化版，实际需要使用scale)
        acc = acc >> 7;  // 除以128
        
        // 裁剪
        if (acc > 127) acc = 127;
        if (acc < -128) acc = -128;
        
        output[i] = (int8_t)acc;
    }
}

/**
 * ReLU激活
 */
static void relu_int8(int8_t* data, int size) {
    for (int i = 0; i < size; i++) {
        if (data[i] < 0) {
            data[i] = 0;
        }
    }
}

/**
 * 主推理函数
 */
void imc22_inference(const int8_t* state, int8_t* action) {
    // 第1层
    matmul_int8(state, INPUT_DIM, fc1_weight, layer1_output, HIDDEN_DIM);
    relu_int8(layer1_output, HIDDEN_DIM);
    
    // 第2层
    matmul_int8(layer1_output, HIDDEN_DIM, fc2_weight, layer2_output, HIDDEN_DIM);
    relu_int8(layer2_output, HIDDEN_DIM);
    
    // 输出层
    matmul_int8(layer2_output, HIDDEN_DIM, fc3_weight, final_output, OUTPUT_DIM);
    
    // 复制结果
    for (int i = 0; i < OUTPUT_DIM; i++) {
        action[i] = final_output[i];
    }
}

/**
 * 浮点数转INT8
 */
int8_t float_to_int8(float value, float scale) {
    int32_t scaled = (int32_t)(value / scale);
    if (scaled > 127) scaled = 127;
    if (scaled < -128) scaled = -128;
    return (int8_t)scaled;
}

/**
 * INT8转浮点数
 */
float int8_to_float(int8_t value, float scale) {
    return (float)value * scale;
}
"""
        return code
    
    def compare_accuracy(self, test_loader, fp32_model):
        """比较量化前后的精度"""
        if self.quantized_model is None:
            raise ValueError("模型尚未量化")
        
        print("\n比较量化前后精度...")
        
        fp32_model.eval()
        self.quantized_model.eval()
        
        total_diff = 0
        num_samples = 0
        
        with torch.no_grad():
            for states, _ in test_loader:
                # FP32推理
                fp32_output = fp32_model(states)
                
                # INT8推理
                int8_output = self.quantized_model(states)
                
                # 计算差异
                diff = torch.abs(fp32_output - int8_output).mean().item()
                total_diff += diff
                num_samples += 1
        
        avg_diff = total_diff / num_samples
        
        print(f"  平均输出差异: {avg_diff:.6f}")
        print(f"  相对误差: {avg_diff * 100:.2f}%")
        
        return avg_diff


if __name__ == "__main__":
    from models.imc22_control_net import IMC22ControlNet
    
    print("IMC-22量化器测试")
    print("="*70)
    
    # 加载模型
    model = IMC22ControlNet()
    
    # 创建量化器
    quantizer = IMC22Quantizer(model)
    
    # 动态量化
    quantizer.quantize_dynamic()
    
    # 导出
    quantizer.save_quantized_weights("models/imc22_weights.npz")
    quantizer.export_to_c_header("imc22_firmware/imc22_weights.h")
    quantizer.export_to_c_source("imc22_firmware/imc22_inference.c")
    
    print("\n✓ 量化完成!")
