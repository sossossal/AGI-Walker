"""
PEFT微调训练器
参数高效微调，支持多种方法（Prefix Tuning, Adapter等）
"""

import json
import time
from pathlib import Path
from typing import Optional
from dataclasses import dataclass
from enum import Enum


class PEFTMethod(Enum):
    """PEFT方法"""

    LORA = "lora"
    PREFIX_TUNING = "prefix_tuning"
    ADAPTER = "adapter"
    PROMPT_TUNING = "prompt_tuning"


@dataclass
class PEFTConfig:
    """PEFT配置"""

    method: PEFTMethod = PEFTMethod.PREFIX_TUNING

    # 通用参数
    learning_rate: float = 1e-4
    batch_size: int = 8
    num_epochs: int = 3
    warmup_steps: int = 100

    # LoRA参数
    lora_r: int = 8
    lora_alpha: int = 32
    lora_dropout: float = 0.1

    # Prefix Tuning参数
    prefix_length: int = 20
    prefix_projection: bool = True

    # Adapter参数
    adapter_size: int = 64
    adapter_dropout: float = 0.1


class PEFTTrainer:
    """
    参数高效微调训练器

    功能：
    1. 支持多种PEFT方法
    2. 低内存消耗
    3. 快速微调
    """

    def __init__(
        self,
        base_model: str = "microsoft/phi-2",
        config: Optional[PEFTConfig] = None,
        output_dir: Optional[str] = None,
    ):
        """
        初始化训练器

        Args:
            base_model: 基础模型名称/路径
            config: PEFT配置
            output_dir: 输出目录
        """
        self.base_model = base_model
        self.config = config or PEFTConfig()

        # 动态获取项目根目录
        project_root = Path(__file__).resolve().parent.parent
        self.output_dir = (
            Path(output_dir) if output_dir else project_root / "models" / "peft"
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # 模型和tokenizer（延迟加载）
        self._model = None
        self._tokenizer = None
        self._peft_model = None

        # 训练状态
        self.is_trained = False
        self.training_history = []

        print("✅ PEFT训练器初始化完成")
        print(f"   基础模型: {base_model}")
        print(f"   方法: {self.config.method.value}")
        print(f"   输出目录: {self.output_dir}")

    def _load_base_model(self):
        """加载基础模型"""
        try:
            from transformers import AutoModelForCausalLM, AutoTokenizer

            print(f"加载基础模型: {self.base_model}")

            self._tokenizer = AutoTokenizer.from_pretrained(
                self.base_model, trust_remote_code=True
            )

            self._model = AutoModelForCausalLM.from_pretrained(
                self.base_model,
                trust_remote_code=True,
                torch_dtype="auto",
                device_map="auto",
            )

            # 设置pad token
            if self._tokenizer.pad_token is None:
                self._tokenizer.pad_token = self._tokenizer.eos_token

            print("✅ 基础模型加载成功")

        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            raise

    def _create_peft_config(self):
        """创建PEFT配置"""
        try:
            from peft import (
                LoraConfig,
                PrefixTuningConfig,
                AdapterConfig,
                PromptTuningConfig,
                TaskType,
            )
        except ImportError:
            print("⚠️ PEFT库未安装")
            print("请运行: pip install peft")
            return None

        method = self.config.method

        if method == PEFTMethod.LORA:
            peft_config = LoraConfig(
                task_type=TaskType.CAUSAL_LM,
                r=self.config.lora_r,
                lora_alpha=self.config.lora_alpha,
                lora_dropout=self.config.lora_dropout,
                target_modules=["q_proj", "v_proj"],
            )

        elif method == PEFTMethod.PREFIX_TUNING:
            peft_config = PrefixTuningConfig(
                task_type=TaskType.CAUSAL_LM,
                num_virtual_tokens=self.config.prefix_length,
                prefix_projection=self.config.prefix_projection,
            )

        elif method == PEFTMethod.ADAPTER:
            # 注意：某些版本的PEFT可能不支持
            try:
                peft_config = AdapterConfig(
                    task_type=TaskType.CAUSAL_LM, adapter_size=self.config.adapter_size
                )
            except Exception:
                print("⚠️ Adapter配置不可用，回退到LoRA")
                peft_config = LoraConfig(
                    task_type=TaskType.CAUSAL_LM,
                    r=self.config.lora_r,
                    lora_alpha=self.config.lora_alpha,
                )

        elif method == PEFTMethod.PROMPT_TUNING:
            peft_config = PromptTuningConfig(
                task_type=TaskType.CAUSAL_LM,
                num_virtual_tokens=self.config.prefix_length,
            )

        else:
            raise ValueError(f"未知的PEFT方法: {method}")

        return peft_config

    def prepare_dataset(self, labeled_data_path: str, max_length: int = 512):
        """
        准备微调数据集

        Args:
            labeled_data_path: 标记数据路径
            max_length: 最大序列长度

        Returns:
            Dataset对象
        """
        try:
            from datasets import Dataset
        except ImportError:
            print("⚠️ datasets库未安装")
            print("请运行: pip install datasets")
            return None

        print(f"准备数据集: {labeled_data_path}")

        # 加载标记数据
        with open(labeled_data_path, "r", encoding="utf-8") as f:
            labeled_data = json.load(f)

        # 转换为训练格式
        train_examples = []

        for item in labeled_data:
            # 构建训练文本
            text = self._format_training_example(item)
            train_examples.append({"text": text})

        print(f"   样本数: {len(train_examples)}")

        # 创建Dataset
        dataset = Dataset.from_list(train_examples)

        # Tokenize
        if self._tokenizer is None:
            self._load_base_model()

        def tokenize_function(examples):
            return self._tokenizer(
                examples["text"],
                truncation=True,
                max_length=max_length,
                padding="max_length",
            )

        dataset = dataset.map(tokenize_function, batched=True)

        return dataset

    def _format_training_example(self, item: dict) -> str:
        """格式化训练样本"""
        label = item.get("label", "unknown")
        explanation = item.get("explanation", "")
        metadata = item.get("metadata", {})

        # 构建输入-输出对
        input_text = f"""分析机器人轨迹：
- 持续时间: {metadata.get("duration", 0)}步
- 最大Roll: {metadata.get("max_roll", 0):.1f}°
- 最大Pitch: {metadata.get("max_pitch", 0):.1f}°
- 最低高度: {metadata.get("min_height", 1.5):.2f}m

标签: {label}
解释: {explanation}"""

        return input_text

    def train(
        self, dataset, eval_dataset=None, resume_from_checkpoint: bool = False
    ) -> dict:
        """
        执行PEFT微调

        Args:
            dataset: 训练数据集
            eval_dataset: 验证数据集
            resume_from_checkpoint: 是否从检查点恢复

        Returns:
            训练结果
        """
        if self._model is None:
            self._load_base_model()

        try:
            from peft import get_peft_model
            from transformers import (
                Trainer,
                TrainingArguments,
                DataCollatorForLanguageModeling,
            )
        except ImportError:
            print("❌ 缺少必要的库")
            return {"error": "missing dependencies"}

        print("\n🚀 开始PEFT微调")
        print(f"   方法: {self.config.method.value}")
        print(f"   Epochs: {self.config.num_epochs}")
        print(f"   批量大小: {self.config.batch_size}")

        start_time = time.time()

        # 创建PEFT模型
        peft_config = self._create_peft_config()
        if peft_config is None:
            return {"error": "PEFT config creation failed"}

        self._peft_model = get_peft_model(self._model, peft_config)

        # 打印可训练参数
        trainable_params = sum(
            p.numel() for p in self._peft_model.parameters() if p.requires_grad
        )
        all_params = sum(p.numel() for p in self._peft_model.parameters())
        print(
            f"   可训练参数: {trainable_params:,} / {all_params:,} ({100 * trainable_params / all_params:.2f}%)"
        )

        # 训练参数
        training_args = TrainingArguments(
            output_dir=str(self.output_dir / "checkpoints"),
            num_train_epochs=self.config.num_epochs,
            per_device_train_batch_size=self.config.batch_size,
            learning_rate=self.config.learning_rate,
            warmup_steps=self.config.warmup_steps,
            logging_steps=10,
            save_steps=100,
            save_total_limit=2,
            fp16=True,
            report_to="none",
        )

        # 数据整理器
        data_collator = DataCollatorForLanguageModeling(
            tokenizer=self._tokenizer, mlm=False
        )

        # 创建Trainer
        trainer = Trainer(
            model=self._peft_model,
            args=training_args,
            train_dataset=dataset,
            eval_dataset=eval_dataset,
            data_collator=data_collator,
        )

        # 训练
        trainer.train(resume_from_checkpoint=resume_from_checkpoint)

        training_time = time.time() - start_time

        # 保存
        self._peft_model.save_pretrained(self.output_dir / "final")
        self._tokenizer.save_pretrained(self.output_dir / "final")
        print(f"✅ 模型已保存: {self.output_dir / 'final'}")

        self.is_trained = True

        result = {
            "training_time": training_time,
            "trainable_params": trainable_params,
            "trainable_ratio": trainable_params / all_params,
            "method": self.config.method.value,
            "model_path": str(self.output_dir / "final"),
        }

        self.training_history.append(result)

        return result

    def merge_and_export(self, output_path: Optional[str] = None) -> str:
        """
        合并适配器并导出完整模型

        Args:
            output_path: 输出路径

        Returns:
            导出路径
        """
        if self._peft_model is None:
            raise RuntimeError("模型未训练")

        if output_path is None:
            output_path = self.output_dir / "merged"

        output_path = Path(output_path)
        output_path.mkdir(parents=True, exist_ok=True)

        print(f"合并模型到: {output_path}")

        try:
            # 合并权重
            merged_model = self._peft_model.merge_and_unload()

            # 保存
            merged_model.save_pretrained(output_path)
            self._tokenizer.save_pretrained(output_path)

            print(f"✅ 合并完成: {output_path}")

            return str(output_path)

        except Exception as e:
            print(f"❌ 合并失败: {e}")
            return ""

    def load_peft_model(self, path: str):
        """加载已训练的PEFT模型"""
        try:
            from peft import PeftModel

            if self._model is None:
                self._load_base_model()

            self._peft_model = PeftModel.from_pretrained(self._model, path)
            self.is_trained = True

            print(f"✅ PEFT模型已加载: {path}")

        except Exception as e:
            print(f"❌ 加载失败: {e}")

    def generate(self, prompt: str, max_new_tokens: int = 100) -> str:
        """使用微调后的模型生成"""
        if self._peft_model is None:
            raise RuntimeError("模型未加载")

        inputs = self._tokenizer(prompt, return_tensors="pt")
        inputs = {k: v.to(self._peft_model.device) for k, v in inputs.items()}

        outputs = self._peft_model.generate(
            **inputs, max_new_tokens=max_new_tokens, do_sample=True, temperature=0.7
        )

        response = self._tokenizer.decode(outputs[0], skip_special_tokens=True)
        return response

    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "base_model": self.base_model,
            "method": self.config.method.value,
            "is_trained": self.is_trained,
            "training_history": self.training_history,
            "output_dir": str(self.output_dir),
        }


def create_peft_trainer(
    base_model: str = "microsoft/phi-2", method: str = "prefix_tuning"
) -> PEFTTrainer:
    """工厂函数：创建PEFT训练器"""
    config = PEFTConfig(method=PEFTMethod(method))
    return PEFTTrainer(base_model, config)


# 测试代码
if __name__ == "__main__":
    print("PEFT微调训练器测试\n")

    # 创建训练器（使用虚拟模式测试）
    print("=== 配置信息 ===")
    config = PEFTConfig(method=PEFTMethod.PREFIX_TUNING, num_epochs=1, batch_size=4)
    print(f"方法: {config.method.value}")
    print(f"Prefix长度: {config.prefix_length}")
    print(f"学习率: {config.learning_rate}")

    # 模拟统计
    print("\n=== 参数效率对比 ===")
    print("| 方法 | 可训练参数 | 内存节省 |")
    print("|------|-----------|---------|")
    print("| Full FT | 100% | 0% |")
    print("| LoRA | ~0.1% | ~90% |")
    print("| Prefix | ~0.1% | ~95% |")
    print("| Adapter | ~2% | ~80% |")
