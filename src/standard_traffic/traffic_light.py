from enum import Enum

class TrafficState(Enum):
    RED = 1
    YELLOW = 2
    GREEN = 3

class TrafficLight:
    state: TrafficState

    def __init__(self) -> None:
        self.state = TrafficState.RED
        pass
