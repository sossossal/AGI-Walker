# AGI-Walker 新手入门教程 (从0开始)

欢迎来到 AGI-Walker！这是一个专为完全零基础的新手设计的教程，我们将带您从零开始，一步步运行您的第一个机器人仿真。

**预计耗时**: 15-20 分钟

---

## 📋 准备工作

在开始之前，请确保您的电脑已经安装了以下软件。

### 1. 安装 Python
AGI-Walker 是基于 Python 开发的。如果您还没有安装 Python：
1. 访问 [Python官网](https://www.python.org/downloads/)
2. 下载最新版本 (推荐 3.8 或以上)
3. 安装时务必勾选 **"Add Python to PATH"** (将Python添加到环境变量)

### 2. 安装 Git (可选，但推荐)
Git 用于下载我们的代码库。
1. 访问 [Git官网](https://git-scm.com/downloads)
2. 下载并安装 Windows 版本

---

## 🚀 第一步：下载项目

### 方法 A: 使用 Git (推荐)
打开命令行工具 (CMD 或 PowerShell)，输入：

```bash
git clone https://github.com/sossossal/AGI-Walker.git
cd AGI-Walker
```

### 方法 B: 直接下载 ZIP
1. 在 GitHub 页面点击绿色的 **"Code"** 按钮
2. 选择 **"Download ZIP"**
3. 解压下载的文件
4. 在解压后的文件夹中打开命令行 (在文件夹地址栏输入 `cmd` 并回车)

---

## 📦 第二步：安装环境

我们需要安装一些必要的 Python 库 (如 numpy, matplotlib 等)。

在项目根目录下，运行：

```bash
pip install -r requirements.txt
```

*如果下载速度慢，可以使用国内镜像*:
```bash
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

---

## 🏃 第三步：运行第一个演示

让我们来运行一个最简单的演示：让机器人走1米。

运行以下命令：
```bash
python examples/walk_1m_demo.py
```

### 您将会看到：
1. **控制台输出**: 显示机器人的实时状态（位置、速度、剩余电量）。
2. **弹出窗口**: 如果一切正常，脚本执行完毕后可能会显示生成的图表或数据摘要。

```text
开始仿真...
Step 0: Pos=0.00m, Vel=0.00m/s, Bat=100.0%
Step 10: Pos=0.05m, Vel=0.12m/s, Bat=99.9%
...
✅ 目标达成！已行进 1.05 米
```

---

## 🎮 第四步：尝试修改参数

AGI-Walker 的核心是**参数化控制**。让我们试着修改一下机器人的参数，看看会有什么不同。

1. 用记事本或代码编辑器打开 `examples/walk_1m_demo.py`
2. 找到类似这样的代码：

```python
# 修改前
params = {
    'motor_power': 1.0,  # 电机功率
    'stiffness': 1.0     # 关节刚度
}
```

3. 尝试修改参数，例如增加功率：

```python
# 修改后
params = {
    'motor_power': 2.0,  # 增加一倍功率！
    'stiffness': 0.5     # 降低刚度变软
}
```

4. 再次运行脚本，观察机器人是否跑得更快了，或者是否更容易摔倒？

---

## 🏭 第五步：生成 AI 训练数据

如果您对人工智能感兴趣，您可以尝试生成一些训练数据。

运行批量生成演示：
```bash
python examples/batch_data_generation_demo.py
```

程序会询问您选择哪种模式，输入 `1` 选择 **小规模演示**。

程序将自动：
1. 在后台启动多个进程
2. 并行生成几百条机器人运动数据
3. 将数据保存到 `data/` 目录

---

## ❓ 常见问题

**Q: 运行报错 `ModuleNotFoundError`?**
A: 请确保您已经成功运行了 `pip install -r requirements.txt`。

**Q: 只有文字输出，没有 3D 画面？**
A: 目前 AGI-Walker 主要专注于**数据生成**和**物理计算**。如果您需要 3D 画面，需要安装 Godot 引擎并运行 `godot_project` 目录下的工程。

**Q: 如何控制机器人？**
A: 您不需要通过键盘控制它。这是一个**仿真平台**，您通过修改代码中的参数来定义机器人的行为，然后观察它的表现。

---

🎉 **恭喜！您已经成功入门 AGI-Walker！**

现在您可以探索 `examples/` 目录下的其他脚本，发现更多有趣的功能。
