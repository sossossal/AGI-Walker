from celery import Celery
import os

# 从环境变量获取 Redis 地址，默认指向本地
REDIS_URL = os.getenv("AGI_WALKER_REDIS_URL", "redis://localhost:6379/0")

celery_app = Celery(
    "agi_walker_worker",
    broker=REDIS_URL,
    backend=REDIS_URL,
    include=["web_panel.workflow_worker_task"]
)

# 生产级优化配置
celery_app.conf.update(
    task_track_started=True,
    task_time_limit=3600,  # 强制超时 (1小时)
    worker_prefetch_multiplier=1,  # 确保长任务不会堆积在单个 Worker
    task_serializer="json",
    accept_content=["json"],
    result_serializer="json",
    timezone="Asia/Shanghai",
    enable_utc=True,
)

if __name__ == "__main__":
    celery_app.start()
