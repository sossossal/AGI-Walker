from datetime import datetime
from typing import List, Optional, Dict, Any
from sqlalchemy import Integer, String, Float, Boolean, DateTime, JSON, ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship
from web_panel.database import Base


class User(Base):
    """工业级用户模型。"""

    __tablename__ = "users"

    id: Mapped[int] = mapped_column(primary_key=True)
    username: Mapped[str] = mapped_column(String(64), unique=True, index=True)
    hashed_password: Mapped[str] = mapped_column(String(128))
    email: Mapped[Optional[str]] = mapped_column(String(128), nullable=True)
    is_active: Mapped[bool] = mapped_column(Boolean, default=True)
    is_admin: Mapped[bool] = mapped_column(Boolean, default=False)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.now)

    # 关联 WorkflowRuns
    workflow_runs: Mapped[List["WorkflowRun"]] = relationship(back_populates="owner")


class WorkflowRun(Base):
    """持久化 Workflow 运行记录。"""

    __tablename__ = "workflow_runs"

    id: Mapped[int] = mapped_column(primary_key=True)
    run_id: Mapped[str] = mapped_column(String(64), unique=True, index=True)

    # 租户隔离 (v2.0)
    user_id: Mapped[Optional[int]] = mapped_column(
        ForeignKey("users.id"), nullable=True
    )
    owner: Mapped[Optional["User"]] = relationship(back_populates="workflow_runs")

    workflow_name: Mapped[str] = mapped_column(String(128))
    status: Mapped[str] = mapped_column(String(32), default="running")
    mode: Mapped[str] = mapped_column(String(32), default="real")
    execution_strategy: Mapped[str] = mapped_column(String(32), default="force")
    output_root: Mapped[str] = mapped_column(String(256), nullable=True)

    # 统计数据
    total_steps: Mapped[Optional[int]] = mapped_column(Integer, nullable=True)
    completed_steps: Mapped[int] = mapped_column(Integer, default=0)
    skipped_steps: Mapped[int] = mapped_column(Integer, default=0)
    failed_steps: Mapped[int] = mapped_column(Integer, default=0)
    success_rate: Mapped[float] = mapped_column(Float, default=0.0)
    timeout_seconds: Mapped[Optional[float]] = mapped_column(Float, nullable=True)

    # 工业化增强字段 (v2.0)
    status_detail: Mapped[Optional[str]] = mapped_column(String(512), nullable=True)
    cancel_requested: Mapped[bool] = mapped_column(Boolean, default=False)
    current_step_name: Mapped[Optional[str]] = mapped_column(String(128), nullable=True)
    current_step_index: Mapped[Optional[int]] = mapped_column(Integer, nullable=True)
    last_event: Mapped[Optional[str]] = mapped_column(String(64), nullable=True)
    live_log_tail: Mapped[List[str]] = mapped_column(JSON, default=list)
    live_log_line_count: Mapped[int] = mapped_column(Integer, default=0)
    live_log_updated_at: Mapped[Optional[str]] = mapped_column(
        String(64), nullable=True
    )
    live_log_path: Mapped[Optional[str]] = mapped_column(String(512), nullable=True)
    live_log_download_url: Mapped[Optional[str]] = mapped_column(
        String(256), nullable=True
    )
    log_path: Mapped[Optional[str]] = mapped_column(String(512), nullable=True)
    log_download_url: Mapped[Optional[str]] = mapped_column(String(256), nullable=True)
    message: Mapped[Optional[str]] = mapped_column(String(1024), nullable=True)
    worker_pid: Mapped[Optional[int]] = mapped_column(Integer, nullable=True)
    exit_reason: Mapped[Optional[str]] = mapped_column(String(64), nullable=True)
    preferred_godot_transport_mode: Mapped[Optional[str]] = mapped_column(
        String(64), nullable=True
    )

    # 负载与结果
    parameters: Mapped[Dict[str, Any]] = mapped_column(JSON, default=dict)
    steps_snapshot: Mapped[List[Dict[str, Any]]] = mapped_column(JSON, default=list)
    step_errors: Mapped[List[Dict[str, Any]]] = mapped_column(JSON, default=list)
    artifacts: Mapped[List[Dict[str, Any]]] = mapped_column(JSON, default=list)
    godot_delivery: Mapped[Optional[Dict[str, Any]]] = mapped_column(
        JSON, nullable=True
    )
    result: Mapped[Optional[Dict[str, Any]]] = mapped_column(JSON, nullable=True)

    # 时间戳
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.now)
    started_at: Mapped[Optional[datetime]] = mapped_column(DateTime, nullable=True)
    finished_at: Mapped[Optional[datetime]] = mapped_column(DateTime, nullable=True)
    duration: Mapped[float] = mapped_column(Float, default=0.0)
    progress_updated_at: Mapped[Optional[str]] = mapped_column(
        String(64), nullable=True
    )

    # 错误详情
    failed_step_name: Mapped[Optional[str]] = mapped_column(String(128), nullable=True)
    failed_step_error: Mapped[Optional[str]] = mapped_column(String(512), nullable=True)
    worker_error_message: Mapped[Optional[str]] = mapped_column(
        String(1024), nullable=True
    )
    worker_traceback: Mapped[Optional[str]] = mapped_column(String(4096), nullable=True)
    diagnostic_summary: Mapped[Optional[str]] = mapped_column(
        String(1024), nullable=True
    )

    def to_dict(self) -> Dict[str, Any]:
        """将模型转换为字典，保持与原有 API 格式一致。"""
        return {
            "run_id": self.run_id,
            "workflow_name": self.workflow_name,
            "status": self.status,
            "mode": self.mode,
            "execution_strategy": self.execution_strategy,
            "output_root": self.output_root,
            "total_steps": self.total_steps,
            "completed_steps": self.completed_steps,
            "skipped_steps": self.skipped_steps,
            "failed_steps": self.failed_steps,
            "success_rate": self.success_rate,
            "timeout_seconds": self.timeout_seconds,
            "status_detail": self.status_detail,
            "cancel_requested": self.cancel_requested,
            "current_step_name": self.current_step_name,
            "current_step_index": self.current_step_index,
            "last_event": self.last_event,
            "live_log_tail": self.live_log_tail,
            "live_log_line_count": self.live_log_line_count,
            "live_log_updated_at": self.live_log_updated_at,
            "live_log_path": self.live_log_path,
            "live_log_download_url": self.live_log_download_url,
            "log_path": self.log_path,
            "log_download_url": self.log_download_url,
            "message": self.message,
            "worker_pid": self.worker_pid,
            "exit_reason": self.exit_reason,
            "preferred_godot_transport_mode": self.preferred_godot_transport_mode
            or "session_bridge",
            "parameters": self.parameters,
            "steps_snapshot": self.steps_snapshot,
            "step_errors": self.step_errors,
            "artifacts": self.artifacts,
            "godot_delivery": self.godot_delivery,
            "result": self.result,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "started_at": self.started_at.isoformat() if self.started_at else None,
            "finished_at": self.finished_at.isoformat() if self.finished_at else None,
            "duration": self.duration,
            "progress_updated_at": self.progress_updated_at,
            "failed_step_name": self.failed_step_name,
            "failed_step_error": self.failed_step_error,
            "worker_error_message": self.worker_error_message,
            "worker_traceback": self.worker_traceback,
            "diagnostic_summary": self.diagnostic_summary,
            # 动态生成契约要求的 URL
            "recommended_godot_sync_url": f"/api/workflows/runs/{self.run_id}/godot-sync",
            "poll_url": f"/api/workflows/runs/{self.run_id}",
            "status_url": f"/api/workflows/runs/{self.run_id}/status",
            "cancel_url": f"/api/workflows/runs/{self.run_id}/cancel",
        }


class OperatorHistoryEntry(Base):
    """Persisted operator history for Godot instruction control sessions."""

    __tablename__ = "operator_history_entries"

    id: Mapped[int] = mapped_column(primary_key=True)
    entry_id: Mapped[str] = mapped_column(String(96), unique=True, index=True)
    session_id: Mapped[str] = mapped_column(String(96), index=True)
    kind: Mapped[str] = mapped_column(String(32))
    route_mode: Mapped[str] = mapped_column(String(32), default="session_bridge")
    operator: Mapped[Optional[str]] = mapped_column(String(96), nullable=True, index=True)
    tag: Mapped[Optional[str]] = mapped_column(String(96), nullable=True, index=True)
    note: Mapped[Optional[str]] = mapped_column(String(512), nullable=True)
    audit_user_id: Mapped[Optional[int]] = mapped_column(Integer, nullable=True, index=True)
    audit_username: Mapped[Optional[str]] = mapped_column(String(96), nullable=True, index=True)
    audit_source: Mapped[Optional[str]] = mapped_column(String(32), nullable=True)
    payload: Mapped[Dict[str, Any]] = mapped_column(JSON, default=dict)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.now, index=True)

    def to_dict(self) -> Dict[str, Any]:
        """Return one operator-history entry in API shape."""
        return {
            "entry_id": self.entry_id,
            "schema_version": "1.0",
            "session_id": self.session_id,
            "kind": self.kind,
            "operator": self.operator,
            "tag": self.tag,
            "note": self.note,
            "audit_user_id": self.audit_user_id,
            "audit_username": self.audit_username,
            "audit_source": self.audit_source,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "payload": self.payload,
        }
