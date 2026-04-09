from datetime import datetime
from typing import Any, Awaitable, Callable, Dict, Optional

import pydantic
from fastapi import APIRouter
from fastapi import HTTPException


class TaskCreate(pydantic.BaseModel):
    name: str
    description: Optional[str] = ""
    priority: Optional[str] = "normal"
    extra: Optional[Dict[str, Any]] = None


BroadcastFn = Callable[[Dict[str, Any]], Awaitable[None]]


def list_tasks(tasks_db: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    return {"status": "success", "tasks": list(tasks_db.values())}


async def create_task(
    tasks_db: Dict[str, Dict[str, Any]],
    task: TaskCreate,
    broadcast_all: BroadcastFn,
) -> Dict[str, Any]:
    task_id = f"task_{len(tasks_db) + 1}"
    task_data = task.model_dump()
    task_data["id"] = task_id
    task_data["status"] = "pending"
    task_data["created_at"] = datetime.now().isoformat()
    tasks_db[task_id] = task_data

    await broadcast_all({"type": "task_created", "task": task_data})
    return {"status": "success", "task_id": task_id, "task": task_data}


def get_task(tasks_db: Dict[str, Dict[str, Any]], task_id: str) -> Dict[str, Any]:
    if task_id not in tasks_db:
        raise HTTPException(status_code=404, detail="Task not found")
    return {"status": "success", "task": tasks_db[task_id]}


async def update_task(
    tasks_db: Dict[str, Dict[str, Any]],
    task_id: str,
    updates: Dict[str, Any],
    broadcast_all: BroadcastFn,
) -> Dict[str, Any]:
    if task_id not in tasks_db:
        raise HTTPException(status_code=404, detail="Task not found")

    tasks_db[task_id].update(updates)
    await broadcast_all({"type": "task_updated", "task": tasks_db[task_id]})
    return {"status": "success", "task": tasks_db[task_id]}


async def delete_task(
    tasks_db: Dict[str, Dict[str, Any]],
    task_id: str,
    broadcast_all: BroadcastFn,
) -> Dict[str, Any]:
    if task_id not in tasks_db:
        raise HTTPException(status_code=404, detail="Task not found")

    del tasks_db[task_id]
    await broadcast_all({"type": "task_deleted", "task_id": task_id})
    return {"status": "success", "message": "Task deleted"}


def build_router(
    tasks_db: Dict[str, Dict[str, Any]], broadcast_all: BroadcastFn
) -> APIRouter:
    router = APIRouter()

    @router.get("/api/tasks")
    async def get_tasks():
        """获取所有任务"""
        return list_tasks(tasks_db)

    @router.post("/api/tasks")
    async def create_task_route(task: TaskCreate):
        """创建新任务"""
        return await create_task(tasks_db, task, broadcast_all)

    @router.get("/api/tasks/{task_id}")
    async def get_task_route(task_id: str):
        """获取单个任务"""
        return get_task(tasks_db, task_id)

    @router.put("/api/tasks/{task_id}")
    async def update_task_route(task_id: str, updates: Dict[str, Any]):
        """更新任务"""
        return await update_task(tasks_db, task_id, updates, broadcast_all)

    @router.delete("/api/tasks/{task_id}")
    async def delete_task_route(task_id: str):
        """删除任务"""
        return await delete_task(tasks_db, task_id, broadcast_all)

    return router
