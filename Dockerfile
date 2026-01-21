# AGI-Walker Docker 镜像
FROM python:3.12-slim

# 设置工作目录
WORKDIR /app

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# 复制项目文件
COPY requirements.txt .
COPY python_api/ ./python_api/
COPY python_controller/ ./python_controller/
COPY robot_models/ ./robot_models/
COPY examples/ ./examples/
COPY tests/ ./tests/

# 安装 Python 依赖
RUN pip install --no-cache-dir -r requirements.txt

# 安装可选依赖
RUN pip install --no-cache-dir mujoco PyQt6

# 暴露端口
EXPOSE 9090 7447

# 设置环境变量
ENV PYTHONPATH=/app
ENV ZENOH_ROUTER_MODE=peer

# 默认命令
CMD ["python", "examples/zenoh_ros2_demo.py"]
