from classes.node import Node
from enum import Enum
from math import isclose

class TrafficState(Enum):
    num_state: int

    RED = 1
    GREEN = 2
    YELLOW = 3

    def next(self):
        if self == TrafficState.RED:
            return TrafficState.GREEN
        elif self == TrafficState.GREEN:
            return TrafficState.YELLOW
        elif self == TrafficState.YELLOW:
            return TrafficState.RED
        
    def get_color(self) -> str:
        if self == TrafficState.RED:
            return "red"
        elif self == TrafficState.GREEN:
            return "green"
        elif self == TrafficState.YELLOW:
            return "yellow"
        
    @classmethod
    def get_num_states(cls) -> int:
        return len(cls.__members__)
        
    @classmethod
    def get_next_state(cls, current_value: int):
        next_value = current_value + 1
        
        if next_value > cls.get_num_states():
            next_value = 1
        
        for state in cls:
            if state.value == next_value:
                return state

        raise ValueError(f"No TrafficState with value {next_value}")

class TrafficLight:
    light_id: str
    node: Node
    switch_duration: float
    total_duration: float
    state: TrafficState

    def __init__(self, light_id: str, node_pos: Node, switch_duration: dict[TrafficState, float]) -> None:
        self.light_id = light_id
        self.node = node_pos
        self.state = min(switch_duration, key=switch_duration.get)
        self.switch_duration = switch_duration
        self.total_duration = max(switch_duration.values())
        self.prev = 0
        self.duration = 0

def next_state(traffic_light: TrafficLight, elapsed_time: float) -> None:
    diff = elapsed_time - traffic_light.prev
    traffic_light.duration += diff
    traffic_light.prev = elapsed_time

    if traffic_light.switch_duration[traffic_light.state] < traffic_light.duration:
        traffic_light.state = traffic_light.state.next()

    if traffic_light.duration > traffic_light.total_duration:
        traffic_light.duration = 0

def get_state(traffic_light: TrafficLight) -> TrafficState:
    return traffic_light.state

def reset_state(traffic_light: TrafficLight) -> None:
    traffic_light.state = min(traffic_light.switch_duration, key=traffic_light.switch_duration.get)
    traffic_light.time = 0
    traffic_light.prev = 0
