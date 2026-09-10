from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import shutil
import socket
from typing import Any


def readiness() -> dict[str, Any]:
    docker_socket = Path("/var/run/docker.sock")
    pymavlink_ready = importlib.util.find_spec("pymavlink") is not None
    pyserial_ready = importlib.util.find_spec("serial") is not None
    serial_devices = {
        path
        for pattern in ("serial*", "ttyUSB*", "ttyACM*")
        for path in Path("/dev").glob(pattern)
    }
    serial_devices.update(Path("/dev/serial/by-id").glob("*"))
    return {
        "companion": {
            "pymavlink": pymavlink_ready,
            "pyserial": pyserial_ready,
            "serial_devices": sorted(str(path) for path in serial_devices),
            "ready": pymavlink_ready and pyserial_ready,
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
            "pymavlink": pymavlink_ready,
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
