# CV Implementation Plan

更新日期：`2026-04-08`

本页不是历史空想路线图，而是基于当前仓库文件整理出的一个现实实施计划。目标是说明 AGI-Walker 现在已有的视觉相关积木，以及继续接通它们的顺序。

## 1. 当前已经存在的积木

### Python 侧视觉模块

- `agi_walker/core/api/sensor/vision_processor.py`
- `agi_walker/core/controllers/vision_processor.py`
- `agi_walker/core/controllers/terrain_mapper.py`
- `agi_walker/core/controllers/vla_adapter.py`

### Godot 侧视觉资源

- `godot_project/scripts/vision_sensor.gd`
- `agi_walker/core/api/comm/godot_vision_client.py`

## 2. 当前真实状态

### 已有基础能力

- OpenCV 风格的轻量视觉处理
- 边缘检测
- 障碍物检测
- 地面区域估计
- 运动检测
- 高程图构建

### 已有但更偏原型的能力

- `VisionEncoder` 的大模型视觉编码
- VLA TaskGraph 注入
- Godot 图像采集链路

### 当前限制

- 图像链路未默认纳入主线 smoke
- 部分视觉模块缺依赖时会退回 mock / dummy
- Godot 视觉客户端并不是默认 Web 主线的一部分

## 3. 推荐实施顺序

### Phase 1: 轻量视觉处理先跑通

优先目标：

- 安装 `opencv-python`
- 跑 `create_vision_processor()`
- 在本地图像上验证障碍物检测和地面估计

### Phase 2: Godot 视觉采集接通

优先目标：

- 让 `vision_sensor.gd` 在 Godot 里正常挂载
- 用 `GodotVisionClient` 拉到图像数据
- 在 Python 侧把 Base64 图像送进视觉处理器

### Phase 3: 地形图和局部地图

优先目标：

- 把点云或深度近似信息送进 `ElevationMapBuilder`
- 输出 roughness / slope / step height

### Phase 4: 多模态规划

优先目标：

- 结合 `VisionEncoder`
- 结合 `VLAAdapter`
- 把视觉推理结果注入 `TaskGraph`

## 4. 现实建议

不要一开始就冲 VLA。更现实的顺序是：

1. 先让 OpenCV 路线稳定
2. 再接 Godot 图像采集
3. 再做地形图
4. 最后才上大模型视觉编码或 VLA

## 5. 成功标准

如果要判断某一阶段已经完成，建议看这些信号：

- 能稳定处理输入帧
- 结果结构化输出可复用
- 与 Godot 的采集链可复现
- 有最小测试或演示脚本支撑

## 结论

AGI-Walker 当前已经具备一条可落地的视觉链雏形，但主线仍停留在轻量 CV 与接口骨架阶段。先把小而稳的视觉路径接通，比提前承诺完整 VLA 更务实。
