from __future__ import annotations

from dataclasses import replace

from .avoidance import AvoidanceConfig
from .collision import Vec3
from .faults import SensorFault
from .radio import LinkOutage, RadioConfig
from .routing import RoutingPolicy
from .simulation import Node, Simulation, SimulationConfig


def _base_config(seed: int) -> SimulationConfig:
    return SimulationConfig(seed=seed, tracked_pair=(1, 2))


def build(name: str, seed: int = 1, *, event_logging: bool = True) -> Simulation:
    config = _base_config(seed)
    radio = RadioConfig()
    outages: list[LinkOutage] = []

    if name in {"head_on", "no_avoidance", "noisy", "command_loss", "limited_dynamics"}:
        nodes = [
            Node(1, Vec3(0, 0), Vec3(0, 0), autonomous=name != "no_avoidance"),
            Node(2, Vec3(14, 0), Vec3(-2, 0)),
            Node(3, Vec3(0, 25), Vec3(0, 0)),
            Node(4, Vec3(25, 25), Vec3(0, 0)),
            Node(5, Vec3(25, 0), Vec3(0, 0)),
        ]
        radio.range_m = 27.0
        outages = [LinkOutage(1, 5, 3.0), LinkOutage(2, 5, 3.0)]
        if name == "noisy":
            config.position_noise_sigma = 1.0
            config.velocity_noise_sigma = 0.2
            config.clock_noise_sigma = 0.08
            radio.random_loss = 0.08
            radio.burst_enter = 0.04
        if name == "command_loss":
            config.avoidance = replace(config.avoidance, command_loss=1.0, command_timeout=0.4)
        if name == "limited_dynamics":
            nodes[0].max_acceleration = 0.8
    elif name == "crossing":
        nodes = [
            Node(1, Vec3(-8, 0), Vec3(2, 0), autonomous=True),
            Node(2, Vec3(0, -8), Vec3(0, 2)),
            Node(3, Vec3(-4, 12), Vec3(0, 0)),
        ]
        config.duration = 10.0
        radio.range_m = 40.0
    elif name == "vertical_clear":
        nodes = [
            Node(1, Vec3(-10, 0, 0), Vec3(2, 0, 0), autonomous=True),
            Node(2, Vec3(0, -10, 8), Vec3(0, 2, 0)),
        ]
        config.duration = 10.0
        radio.range_m = 40.0
    elif name == "partition":
        nodes = [
            Node(1, Vec3(0, 0), Vec3(0, 0), autonomous=True),
            Node(2, Vec3(12, 0), Vec3(-1, 0)),
            Node(3, Vec3(0, 24), Vec3(0, 0)),
            Node(4, Vec3(0, 48), Vec3(0, 0)),
        ]
        radio.range_m = 26.0
        outages = [LinkOutage(3, 4, 4.0, 9.0)]
    elif name == "congested":
        nodes = [Node(index, Vec3((index - 1) * 3, 0), Vec3(0, 0), autonomous=index == 1) for index in range(1, 13)]
        nodes[1].position = Vec3(14, 0)
        nodes[1].velocity = Vec3(-2, 0)
        config.telemetry_interval = 0.2
        config.duration = 8.0
        radio.range_m = 50.0
        radio.max_backoff_s = 0.02
        radio.channel_access = "aloha"
    elif name == "asymmetric":
        nodes = [
            Node(1, Vec3(0, 0), Vec3(0, 0)),
            Node(2, Vec3(20, 0), Vec3(0, 0)),
            Node(3, Vec3(40, 0), Vec3(0, 0)),
        ]
        config.tracked_pair = (1, 2)
        config.duration = 8.0
        radio.range_m = 25.0
        radio.asymmetric_loss = {(1, 2): 0.9, (2, 1): 0.0}
    elif name == "multi_threat":
        nodes = [
            Node(1, Vec3(0, 0), Vec3(0, 0), autonomous=True),
            Node(2, Vec3(12, 0), Vec3(-2, 0)),
            Node(3, Vec3(0, 12), Vec3(0, -2)),
            Node(4, Vec3(-12, 0), Vec3(2, 0)),
        ]
        config.duration = 10.0
        radio.range_m = 40.0
    elif name == "gps_jump":
        nodes = [
            Node(1, Vec3(0, 0), Vec3(0, 0), autonomous=True),
            Node(2, Vec3(14, 0), Vec3(-2, 0)),
            Node(3, Vec3(5, 15), Vec3(0, 0)),
        ]
        config.duration = 10.0
        config.sensor_faults = (
            SensorFault(2, 2.0, 3.5, position_bias=Vec3(8, -5, 0)),
        )
        radio.range_m = 40.0
    elif name == "clock_skew":
        nodes = [
            Node(1, Vec3(0, 0), Vec3(0, 0), autonomous=True),
            Node(2, Vec3(14, 0), Vec3(-2, 0)),
        ]
        config.duration = 10.0
        config.sensor_faults = (SensorFault(2, 1.0, 4.0, clock_bias=-1.2),)
        radio.range_m = 40.0
    elif name == "node_restart":
        nodes = [
            Node(1, Vec3(0, 0), Vec3(0, 0), autonomous=True),
            Node(2, Vec3(14, 0), Vec3(-2, 0), sequence=30000, boot_id=41),
            Node(3, Vec3(7, 10), Vec3(0, 0)),
        ]
        config.duration = 10.0
        config.node_reboots = ((2, 2.0),)
        radio.range_m = 40.0
    elif name == "telemetry_dropout":
        nodes = [
            Node(1, Vec3(0, 0), Vec3(0, 0), autonomous=True),
            Node(2, Vec3(14, 0), Vec3(-2, 0)),
        ]
        config.duration = 10.0
        config.sensor_faults = (SensorFault(2, 2.5, 5.0, drop_telemetry=True),)
        radio.range_m = 40.0
    elif name == "endurance":
        nodes = [
            Node(index, Vec3((index % 4) * 8, (index // 4) * 8), Vec3(0, 0))
            for index in range(1, 13)
        ]
        nodes[0].autonomous = True
        config.duration = 300.0
        config.telemetry_interval = 5.0
        config.tracked_pair = (1, 2)
        config.routing = RoutingPolicy(mode="relay", relay_nodes=(5, 6, 7, 8))
        radio.range_m = 18.0
    else:
        raise ValueError(f"unknown scenario {name!r}")
    return Simulation(nodes, config=config, radio=radio, outages=outages, event_logging=event_logging)


SCENARIOS = (
    "head_on",
    "no_avoidance",
    "noisy",
    "command_loss",
    "limited_dynamics",
    "crossing",
    "vertical_clear",
    "partition",
    "congested",
    "asymmetric",
    "multi_threat",
    "gps_jump",
    "clock_skew",
    "node_restart",
    "telemetry_dropout",
    "endurance",
)

CAMPAIGN_SCENARIOS = tuple(name for name in SCENARIOS if name != "endurance")
