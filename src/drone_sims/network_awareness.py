"""Team task 1 contracts. See docs/TASK_1_ROUTING.md."""
from dataclasses import dataclass
from .collision import KinematicState


@dataclass(frozen=True, slots=True)
class LinkPrediction:
    a: int
    b: int
    distance_m: float
    projected_distance_m: float
    margin_m: float
    time_to_loss_s: float | None
    connected_now: bool
    connected_projected: bool
    status: str


def predict_link(first: KinematicState, second: KinematicState, *, now: float,
                 range_m: float, lookahead_s: float, warning_margin_m: float) -> LinkPrediction:
    # TODO: predict weakening links from relative position and velocity.
    raise NotImplementedError("Team task 1: predictive link monitoring")


def planned_routes(states: dict[int, KinematicState], *, source: int, now: float,
                   range_m: float, lookahead_s: float, route_margin: float) -> dict[int, list[int]]:
    # TODO: select stable relay paths using present and predicted connectivity.
    raise NotImplementedError("Team task 1: proactive relay routing")
