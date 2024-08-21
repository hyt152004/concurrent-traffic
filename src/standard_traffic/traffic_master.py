from .traffic_light import TrafficLight, next_state, reset_state

class TrafficMaster:
    traffic_lights: dict[str, list[TrafficLight]] = {}  # eg "eglington": [TrafficLight_1, TrafficLight2]
    intersection_types: dict[tuple[str], dict] = []  # eg ("eglington", "towncentre") : {TrafficState.RED: 30, TrafficState.YELLOW: 2, TrafficState.GREEN:32, "duration_list": [30, 2, 32]}

    def __init__(self, traffic_lights: list[TrafficLight]) -> None:
        """Initialize all the different traffic lights in each list
        Have a central time, then send requests for each traffic light to switch light"""
        self.traffic_lights = traffic_lights

def t_master_event_loop(traffic_master: TrafficMaster, delta_time: int) -> None:
    for t in traffic_master.traffic_lights:
        next_state(t, delta_time)

def reset_traffic(traffic_master: TrafficMaster) -> None:
    for t in traffic_master.traffic_lights:
        reset_state(t)