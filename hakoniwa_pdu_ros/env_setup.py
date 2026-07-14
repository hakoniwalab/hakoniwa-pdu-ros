from __future__ import annotations

import os
import sys
from importlib import import_module
from pathlib import Path


def configure_import_paths() -> None:
    _prepend_path_from_env("HAKONIWA_PDU_PYTHON_PATH")
    _prepend_endpoint_paths_from_env("HAKONIWA_PDU_ENDPOINT_PYTHON_PATH")


def _prepend_path_from_env(env_name: str) -> None:
    raw = os.environ.get(env_name)
    if not raw:
        return

    path = str(Path(raw).expanduser().resolve())
    if path not in sys.path:
        sys.path.insert(0, path)


def _prepend_endpoint_paths_from_env(env_name: str) -> None:
    raw = os.environ.get(env_name)
    if not raw:
        return

    base = Path(raw).expanduser().resolve()
    candidate_paths = [base]
    package_dirs: list[Path] = []

    # hakoniwa-pdu-endpoint keeps the Python wrapper sources under `python/`
    # and the generated cffi extension under build directories such as
    # `build/python` or `build-zenoh-shared/python`.
    if base.name == "python":
        build_package_dir = base / "hakoniwa_pdu_endpoint"
        if build_package_dir.exists():
            package_dirs.append(build_package_dir)

        if (build_package_dir / "c_endpoint.py").exists():
            source_dir = base
        else:
            source_dir = base.parent.parent / "python"

        if source_dir.exists():
            candidate_paths.append(source_dir)
            source_package_dir = source_dir / "hakoniwa_pdu_endpoint"
            if source_package_dir.exists():
                package_dirs.append(source_package_dir)

    for path in reversed(candidate_paths):
        resolved = str(path)
        if resolved not in sys.path:
            sys.path.insert(0, resolved)

    _extend_package_path("hakoniwa_pdu_endpoint", package_dirs)


def _extend_package_path(package_name: str, package_dirs: list[Path]) -> None:
    if not package_dirs:
        return

    try:
        package = import_module(package_name)
    except ModuleNotFoundError:
        return

    package_path = getattr(package, "__path__", None)
    if package_path is None:
        return

    for package_dir in package_dirs:
        resolved = str(package_dir)
        if resolved not in package_path:
            package_path.append(resolved)
