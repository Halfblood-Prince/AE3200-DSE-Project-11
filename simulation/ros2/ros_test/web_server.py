from __future__ import annotations

import os
import runpy
from pathlib import Path


PACKAGE_NAME = "ros_test"
WEB_DIR_ENV = "AEROSENTINEL_WEB_DIR"


def candidate_web_dirs() -> list[Path]:
    candidates: list[Path] = []

    configured_dir = os.environ.get(WEB_DIR_ENV)
    if configured_dir:
        candidates.append(Path(configured_dir).expanduser())

    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception:
        pass
    else:
        try:
            candidates.append(Path(get_package_share_directory(PACKAGE_NAME)) / "website")
        except Exception:
            pass

    candidates.append(Path(__file__).resolve().parents[1] / "website")
    return candidates


def find_web_dir() -> Path:
    candidates = candidate_web_dirs()
    for candidate in candidates:
        if (candidate / "app.py").is_file():
            return candidate

    searched = ", ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"Could not find AeroSentinel Flask app.py. Searched: {searched}")


def main() -> None:
    web_dir = find_web_dir()
    os.chdir(web_dir)
    runpy.run_path(str(web_dir / "app.py"), run_name="__main__")


if __name__ == "__main__":
    main()
