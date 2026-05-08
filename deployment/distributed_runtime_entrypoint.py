from __future__ import annotations

import importlib.util
import os
import subprocess
import sys


REQUIREMENTS_FILE = "/app/deployment/requirements.distributed_runtime.txt"


def _ensure_runtime_dependencies() -> None:
    if importlib.util.find_spec("yaml") is not None:
        return
    if os.environ.get("AGI_WALKER_RUNTIME_BOOTSTRAP_INSTALL", "1") == "0":
        raise ModuleNotFoundError(
            "PyYAML is missing and AGI_WALKER_RUNTIME_BOOTSTRAP_INSTALL=0"
        )
    subprocess.check_call(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--no-cache-dir",
            "-r",
            REQUIREMENTS_FILE,
        ]
    )
    if importlib.util.find_spec("yaml") is None:
        raise ModuleNotFoundError("PyYAML is still missing after runtime bootstrap")


def main() -> int:
    if len(sys.argv) < 2:
        print("distributed_runtime_entrypoint requires a command", file=sys.stderr)
        return 64
    _ensure_runtime_dependencies()
    os.execvp(sys.argv[1], sys.argv[1:])
    return 127


if __name__ == "__main__":
    raise SystemExit(main())
