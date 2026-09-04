from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import shutil
import socket
from typing import Any


def readiness() -> dict[str, Any]:
    docker_socket = Path("/var/run/docker.sock")
    return {
        "fpga": {
            "iverilog": shutil.which("iverilog"),
            "verilator": shutil.which("verilator"),
            "ready": bool(shutil.which("iverilog") or shutil.which("verilator")),
            "available_fallback": "Python fixed-point golden model and CSV vectors",
        },
        "px4": {
            "px4_binary": shutil.which("px4"),
            "gazebo": shutil.which("gz") or shutil.which("gazebo"),
            "ready": bool(shutil.which("px4") and (shutil.which("gz") or shutil.which("gazebo"))),
        },
        "ardupilot": {
            "sim_vehicle": shutil.which("sim_vehicle.py"),
            "ready": bool(shutil.which("sim_vehicle.py")),
        },
        "mavlink": {
            "pymavlink": importlib.util.find_spec("pymavlink") is not None,
        },
        "docker": {
            "binary": shutil.which("docker"),
            "socket_exists": docker_socket.exists(),
            "socket_accessible": os.access(docker_socket, os.R_OK | os.W_OK),
        },
    }


class JsonUdpFlightController:
    """Dependency-free UDP test double for a future MAVLink/SITL bridge.

    Messages are newline-free JSON datagrams. This exercises transport timing,
    timeout, acknowledgement, and command-state handling without claiming MAVLink
    compatibility. The real adapter can implement the same send/receive boundary.
    """

    def __init__(self, host: str, port: int, timeout: float = 0.5) -> None:
        self.address = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(timeout)

    def exchange(self, payload: bytes) -> bytes:
        self.socket.sendto(payload, self.address)
        response, _ = self.socket.recvfrom(65535)
        return response

    def close(self) -> None:
        self.socket.close()
