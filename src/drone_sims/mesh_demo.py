"""Team task 2 entrypoint contract. See docs/TASK_2_NODE_PROCESSES.md."""
from typing import Callable


def run_mesh_demo(*, nodes: int = 6, duration_s: float = 15.0, seed: int = 31,
                  update: Callable[[dict[str, object]], None] | None = None) -> dict[str, object]:
    # TODO: run one CompanionService per process over a parent-owned RadioMedium.
    # The update callback publishes serializable snapshots for team task 3.
    raise NotImplementedError("Team task 2: independent node processes and mesh orchestration")
