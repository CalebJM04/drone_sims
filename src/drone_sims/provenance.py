from __future__ import annotations

from datetime import datetime, timezone
import hashlib
from importlib import metadata
import platform
from pathlib import Path
import subprocess
from typing import Any, Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PINNED_PX4_IMAGE = (
    "px4io/px4-sitl@sha256:"
    "01866d912ac22ca6119a996b830cf628a6d47dfb60fdccc41cd9f44b62935a44"
)


def _command_version(command: list[str]) -> str | None:
    try:
        completed = subprocess.run(
            command, text=True, capture_output=True, check=False, timeout=5,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return None
    output = (completed.stdout or completed.stderr).strip()
    return output.splitlines()[0] if output else None


def _git_value(*arguments: str) -> str | None:
    try:
        completed = subprocess.run(
            ["git", "-C", str(PROJECT_ROOT), *arguments],
            text=True, capture_output=True, check=False, timeout=5,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return None
    value = completed.stdout.strip()
    return value if completed.returncode == 0 and value else None


def source_digest() -> str:
    """Hash every input that can affect generated verification evidence."""

    roots = ("src", "tools", "requirements", "config", "deploy")
    files: list[Path] = [PROJECT_ROOT / "pyproject.toml", PROJECT_ROOT / "Makefile"]
    for root in roots:
        files.extend(
            path for path in (PROJECT_ROOT / root).rglob("*")
            if path.is_file()
            and "__pycache__" not in path.parts
            and not any(part.endswith(".egg-info") for part in path.parts)
        )
    digest = hashlib.sha256()
    for path in sorted(files):
        relative = path.relative_to(PROJECT_ROOT).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(4, "big"))
        digest.update(relative)
        digest.update(path.read_bytes())
    return digest.hexdigest()


def collect_metadata(
    command: str,
    *,
    seeds: Iterable[int] = (),
    parameters: dict[str, Any] | None = None,
    px4_image: str | None = None,
) -> dict[str, Any]:
    revision = _git_value("rev-parse", "HEAD")
    status = _git_value(
        "status", "--porcelain", "--", ".", ":(exclude)results/**",
    )
    try:
        pymavlink_version = metadata.version("pymavlink")
    except metadata.PackageNotFoundError:
        pymavlink_version = None
    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "command": command,
        "parameters": parameters or {},
        "seeds": list(seeds),
        "source_sha256": source_digest(),
        "git": {
            "revision": revision,
            "dirty": None if revision is None else bool(status),
        },
        "runtime": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "pymavlink": pymavlink_version,
        },
        "px4_image": px4_image,
    }
