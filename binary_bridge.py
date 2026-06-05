import json
import platform
import subprocess
from pathlib import Path


def native_target():
    system_names = {
        "darwin": "macos",
        "linux": "linux",
        "windows": "windows",
    }
    machine_names = {
        "aarch64": "arm64",
        "amd64": "amd64",
        "arm64": "arm64",
        "x64": "amd64",
        "x86_64": "amd64",
    }

    system = platform.system().lower()
    machine = platform.machine().lower()
    try:
        return system_names[system], machine_names[machine]
    except KeyError as error:
        raise RuntimeError(
            f"Unsupported native platform: {system}/{machine}"
        ) from error


def resolve_binary(component, binary_path=None):
    if binary_path is not None:
        path = Path(binary_path).expanduser().resolve()
    else:
        operating_system, architecture = native_target()
        extension = ".exe" if operating_system == "windows" else ""
        filename = (
            f"{component}-{operating_system}-{architecture}{extension}"
        )
        path = Path(__file__).resolve().parent / "bin" / filename

    if not path.is_file():
        raise FileNotFoundError(
            f"Compiled {component} binary not found at {path}. "
            "Run the C++ binary workflow or build cpp sources locally."
        )
    return path


def run_binary(component, command, options, binary_path=None):
    executable = resolve_binary(component, binary_path)
    arguments = [str(executable), command]
    for name, value in options.items():
        arguments.extend((f"--{name}", str(value)))

    completed = subprocess.run(
        arguments,
        check=True,
        capture_output=True,
        text=True,
    )
    try:
        return json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            f"{component} binary returned invalid JSON: {completed.stdout!r}"
        ) from error
