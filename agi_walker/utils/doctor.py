"""
Development Environment Diagnostic Tool (agi_walker doctor)

Provides automated environment checks for AGI-Walker dependencies,
system configuration, and common pitfalls.
"""

import os
import sys
import socket
import logging
import importlib.util
from typing import Dict, List, Tuple, Any

logger = logging.getLogger(__name__)


# Colors for terminal output
class Colors:
    HEADER = "\033[95m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    GREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def check_package(package_name: str) -> Tuple[bool, str]:
    """Check if a Python package is installed"""
    spec = importlib.util.find_spec(package_name)
    if spec is not None:
        try:
            module = importlib.import_module(package_name)
            version = getattr(module, "__version__", "unknown")
            return True, f"Found {package_name} (v{version})"
        except Exception as e:
            return False, f"Error importing {package_name}: {e}"
    return False, f"Package {package_name} not found"


def check_port(port: int, host: str = "127.0.0.1") -> Tuple[bool, str]:
    """Check if a network port is available"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind((host, port))
            return True, f"Port {port} is available"
        except socket.error:
            return False, f"Port {port} is currently in use"


def check_directory_permission(path: str) -> Tuple[bool, str]:
    """Check if a directory is accessible and writable"""
    if not os.path.exists(path):
        try:
            os.makedirs(path, exist_ok=True)
            return True, f"Directory {path} created and accessible"
        except Exception as e:
            return False, f"Cannot create directory {path}: {e}"

    if os.access(path, os.R_OK | os.W_OK | os.X_OK):
        return True, f"Directory {path} is accessible and writable"
    else:
        return False, f"Permission denied for directory {path}"


def run_diagnostics() -> Dict[str, Any]:
    """Run all environment diagnostics"""
    results = {
        "core_packages": [],
        "optional_packages": [],
        "system_checks": [],
        "network_checks": [],
        "directory_checks": [],
    }

    # 1. Core Packages (from requirements.txt and code usage)
    core_deps = ["numpy", "gymnasium", "yaml", "fastapi", "uvicorn", "lxml"]
    for dep in core_deps:
        # Standardize naming for imports
        import_name = dep if dep != "yaml" else "yaml"
        success, msg = check_package(import_name)
        results["core_packages"].append((success, msg))

    # 2. Optional Packages
    optional_deps = ["torch", "zenoh", "PyQt6", "stable_baselines3"]
    for dep in optional_deps:
        success, msg = check_package(dep)
        results["optional_packages"].append((success, msg))

    # 3. Network Ports
    ports = [8000, 8080]  # Default ports for Web Panel and Godot bridge
    for port in ports:
        success, msg = check_port(port)
        results["network_checks"].append((success, msg))

    # 4. Critical Directories
    critical_dirs = [".output/", "models/", "exports/", "tests/"]
    for d in critical_dirs:
        success, msg = check_directory_permission(d)
        results["directory_checks"].append((success, msg))

    return results


def print_report(results: Dict[str, Any]):
    """Format and print the diagnostic report"""
    print(
        f"\n{Colors.HEADER}{Colors.BOLD}AGI-Walker Environment Diagnostic Report{Colors.ENDC}"
    )
    print(f"{Colors.BLUE}{'=' * 50}{Colors.ENDC}\n")

    sections = [
        ("Core Python Packages", "core_packages"),
        ("Optional/Training Packages", "optional_packages"),
        ("Network Connectivity", "network_checks"),
        ("Directory Permissions", "directory_checks"),
    ]

    all_passed = True

    for title, key in sections:
        print(f"{Colors.BOLD}{title}:{Colors.ENDC}")
        for success, msg in results[key]:
            if success:
                print(f"  [{Colors.GREEN}PASS{Colors.ENDC}] {msg}")
            else:
                if key == "core_packages" or key == "directory_checks":
                    print(f"  [{Colors.FAIL}FAIL{Colors.ENDC}] {msg}")
                    all_passed = False
                else:
                    print(f"  [{Colors.WARNING}WARN{Colors.ENDC}] {msg}")
        print()

    print(f"{Colors.BLUE}{'=' * 50}{Colors.ENDC}")
    if all_passed:
        print(
            f"{Colors.GREEN}{Colors.BOLD}Verdict: Your environment is healthy!{Colors.ENDC}"
        )
    else:
        print(
            f"{Colors.FAIL}{Colors.BOLD}Verdict: Issues detected. Please check FAILED items above.{Colors.ENDC}"
        )
    print()


if __name__ == "__main__":
    results = run_diagnostics()
    print_report(results)
