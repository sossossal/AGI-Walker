"""Expand workflow run runtime state

Revision ID: 5d6b7a8c9e10
Revises: 113c6566852e
Create Date: 2026-04-02 23:15:00.000000

"""

from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = "5d6b7a8c9e10"
down_revision: Union[str, Sequence[str], None] = "113c6566852e"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    with op.batch_alter_table("workflow_runs", schema=None) as batch_op:
        batch_op.add_column(sa.Column("timeout_seconds", sa.Float(), nullable=True))
        batch_op.add_column(
            sa.Column("status_detail", sa.String(length=512), nullable=True)
        )
        batch_op.add_column(
            sa.Column(
                "cancel_requested",
                sa.Boolean(),
                nullable=False,
                server_default=sa.false(),
            )
        )
        batch_op.add_column(
            sa.Column("current_step_name", sa.String(length=128), nullable=True)
        )
        batch_op.add_column(
            sa.Column("current_step_index", sa.Integer(), nullable=True)
        )
        batch_op.add_column(
            sa.Column("last_event", sa.String(length=64), nullable=True)
        )
        batch_op.add_column(sa.Column("live_log_tail", sa.JSON(), nullable=True))
        batch_op.add_column(
            sa.Column(
                "live_log_line_count", sa.Integer(), nullable=False, server_default="0"
            )
        )
        batch_op.add_column(
            sa.Column("live_log_updated_at", sa.String(length=64), nullable=True)
        )
        batch_op.add_column(
            sa.Column("live_log_path", sa.String(length=512), nullable=True)
        )
        batch_op.add_column(
            sa.Column("live_log_download_url", sa.String(length=256), nullable=True)
        )
        batch_op.add_column(sa.Column("log_path", sa.String(length=512), nullable=True))
        batch_op.add_column(
            sa.Column("log_download_url", sa.String(length=256), nullable=True)
        )
        batch_op.add_column(sa.Column("message", sa.String(length=1024), nullable=True))
        batch_op.add_column(sa.Column("worker_pid", sa.Integer(), nullable=True))
        batch_op.add_column(
            sa.Column("exit_reason", sa.String(length=64), nullable=True)
        )
        batch_op.add_column(
            sa.Column(
                "preferred_godot_transport_mode", sa.String(length=64), nullable=True
            )
        )
        batch_op.add_column(
            sa.Column("progress_updated_at", sa.String(length=64), nullable=True)
        )
        batch_op.add_column(sa.Column("step_errors", sa.JSON(), nullable=True))
        batch_op.add_column(
            sa.Column("failed_step_name", sa.String(length=128), nullable=True)
        )
        batch_op.add_column(
            sa.Column("failed_step_error", sa.String(length=512), nullable=True)
        )
        batch_op.add_column(
            sa.Column("diagnostic_summary", sa.String(length=1024), nullable=True)
        )
        batch_op.add_column(sa.Column("result", sa.JSON(), nullable=True))


def downgrade() -> None:
    """Downgrade schema."""
    with op.batch_alter_table("workflow_runs", schema=None) as batch_op:
        batch_op.drop_column("result")
        batch_op.drop_column("diagnostic_summary")
        batch_op.drop_column("failed_step_error")
        batch_op.drop_column("failed_step_name")
        batch_op.drop_column("step_errors")
        batch_op.drop_column("progress_updated_at")
        batch_op.drop_column("preferred_godot_transport_mode")
        batch_op.drop_column("exit_reason")
        batch_op.drop_column("worker_pid")
        batch_op.drop_column("message")
        batch_op.drop_column("log_download_url")
        batch_op.drop_column("log_path")
        batch_op.drop_column("live_log_download_url")
        batch_op.drop_column("live_log_path")
        batch_op.drop_column("live_log_updated_at")
        batch_op.drop_column("live_log_line_count")
        batch_op.drop_column("live_log_tail")
        batch_op.drop_column("last_event")
        batch_op.drop_column("current_step_index")
        batch_op.drop_column("current_step_name")
        batch_op.drop_column("cancel_requested")
        batch_op.drop_column("status_detail")
        batch_op.drop_column("timeout_seconds")
