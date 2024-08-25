from classes.node import Node
from enum import Enum

class TrafficState(Enum):
    RED = 1
    GREEN = 2
    YELLOW = 3

def get_color(state) -> str:
    if state == TrafficState.RED:
        return "red"
    elif state == TrafficState.GREEN:
        return "green"
    elif state == TrafficState.YELLOW:
        return "yellow"

    raise ValueError(f'State not recognized: {state}')

def get_state(color: str) -> TrafficState:
    if color == "red":
        return TrafficState.RED
    elif color == "green":
        return TrafficState.GREEN
    elif color == "yellow":
        return TrafficState.YELLOW
    
    raise ValueError(f'Color not recognized: {color}')

class TrafficLight:
    light_id: str
    node: Node
    prev_elapsed: float
    duration: float
    i: float
    cycle: list[tuple[TrafficState, float]]

    def __init__(self, light_id: str, node_pos: Node, cycle: list[tuple[TrafficState, float]]) -> None:
        self.light_id = light_id
        self.node = node_pos
        self.prev_elapsed = 0
        self.i = 0
        self.duration = 0
        self.cycle = cycle

def next_light_state(traffic_light: TrafficLight, elapsed_time: float) -> None:
    """Sets traffic_light to next traffic state if the time for the current traffic state has expired."""
    time_diff = elapsed_time - traffic_light.prev_elapsed
    traffic_light.duration += time_diff
    traffic_light.prev_elapsed = elapsed_time

    if get_light_limit(traffic_light) < traffic_light.duration:
        traffic_light.i += 1
        traffic_light.i %= len(traffic_light.cycle)
        traffic_light.duration = 0

def get_light_state(traffic_light: TrafficLight) -> TrafficState:
    """Get traffic light state."""
    return traffic_light.cycle[traffic_light.i][0]

def get_light_limit(traffic_light: TrafficLight) -> float:
    """Get traffic light state limit."""
    return traffic_light.cycle[traffic_light.i][1]

def reset_state(traffic_light: TrafficLight) -> None:
    """Reset traffic light state."""
    traffic_light.i = 0
    traffic_light.prev_elapsed = 0
    traffic_light.duration = 0
