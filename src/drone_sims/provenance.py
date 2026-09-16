from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import platform
from pathlib import Path
import subprocess
from typing import Any, Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[2]


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
    roots = ("src",)
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
) -> dict[str, Any]:
    revision = _git_value("rev-parse", "HEAD")
    status = _git_value(
        "status", "--porcelain", "--", ".", ":(exclude)results/**",
    )
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
        },
    }
