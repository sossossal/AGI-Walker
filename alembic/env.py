from logging.config import fileConfig

from sqlalchemy import engine_from_config
from sqlalchemy import pool

from alembic import context

# this is the Alembic Config object, which provides
# access to the values within the .ini file in use.
config = context.config

# Interpret the config file for Python logging.
# This line sets up loggers basically.
if config.config_file_name is not None:
    fileConfig(config.config_file_name)

import sys
import os
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from web_panel.database import Base
from web_panel.models import WorkflowRun  # Ensure models are loaded

# target_metadata = None
target_metadata = Base.metadata


def _resolve_database_url() -> str:
    """Resolve the Alembic database URL from environment or ini, using sync drivers."""
    configured_url = os.getenv("AGI_WALKER_DATABASE_URL") or config.get_main_option("sqlalchemy.url")
    normalized = configured_url.strip()

    if normalized.startswith("postgresql+asyncpg://"):
        normalized = normalized.replace("postgresql+asyncpg://", "postgresql+psycopg2://", 1)
    elif normalized.startswith("postgresql://"):
        normalized = normalized.replace("postgresql://", "postgresql+psycopg2://", 1)
    elif normalized.startswith("sqlite+aiosqlite:///"):
        normalized = normalized.replace("sqlite+aiosqlite:///", "sqlite:///", 1)

    # Alembic's ConfigParser treats '%' as interpolation markers.
    config.set_main_option("sqlalchemy.url", normalized.replace("%", "%%"))
    return normalized


# other values from the config, defined by the needs of env.py,
# can be acquired:
# my_important_option = config.get_main_option("my_important_option")
# ... etc.


def run_migrations_offline() -> None:
    """Run migrations in 'offline' mode.

    This configures the context with just a URL
    and not an Engine, though an Engine is acceptable
    here as well.  By skipping the Engine creation
    we don't even need a DBAPI to be available.

    Calls to context.execute() here emit the given string to the
    script output.

    """
    url = _resolve_database_url()
    context.configure(
        url=url,
        target_metadata=target_metadata,
        literal_binds=True,
        dialect_opts={"paramstyle": "named"},
    )

    with context.begin_transaction():
        context.run_migrations()


def run_migrations_online() -> None:
    """Run migrations in 'online' mode."""
    _resolve_database_url()
    connectable = engine_from_config(
        config.get_section(config.config_ini_section, {}),
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )

    with connectable.connect() as connection:
        context.configure(
            connection=connection, 
            target_metadata=target_metadata,
            render_as_batch=True  # 启用 Batch Mode 以支持 SQLite 外键修改
        )

        with context.begin_transaction():
            context.run_migrations()


if context.is_offline_mode():
    run_migrations_offline()
else:
    run_migrations_online()
