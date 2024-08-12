from classes.node import Node
from enum import Enum

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
    state: TrafficState
    time_in_state: int
    cycle_duration: int
    id: str
    node: Node
    time_to_switch: dict[TrafficState, int]
    prev_dt: int

    def __init__(self, id: str, node_pos: Node) -> None:
        self.id = id
        self.node = node_pos
        self.state = TrafficState.GREEN # will be changed by traffic_master according to the identifier/type
        self.time_in_state = 0 # set later by traffic_master
        self.cycle_duration = 0 # set later by traffic_master
        self.time_to_switch = {} # set later by traffic_master
        self.prev_dt = 0

def next_state(traffic_light: TrafficLight):
    traffic_light.state = traffic_light.state.next()
    reset_time_in_state(traffic_light)

def reset_time_in_state(traffic_light: TrafficLight):
    traffic_light.time_in_state = 0

def get_state(traffic_light: TrafficLight) -> TrafficState:
    return traffic_light.state

def set_state(traffic_light: TrafficLight, state: TrafficState):
    traffic_light.state = state

def set_cycle_dur(traffic_light: TrafficLight, duration: int):
    if duration < 1:
        raise ValueError(f"Cycle duration cannot be less than 1.")
    traffic_light.cycle_duration = duration

def set_tts(traffic_light: TrafficLight, tts: dict[TrafficState, int]):
    traffic_light.time_to_switch = tts
