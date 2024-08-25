from .traffic_light import TrafficLight, next_light_state, reset_state

class TrafficMaster:
    traffic_lights: list[TrafficLight]

    def __init__(self, traffic_lights: list[TrafficLight]) -> None:
        """Initialize all the different traffic lights in each list
        Have a central time, then send requests for each traffic light to switch light"""
        self.traffic_lights = traffic_lights

def traffic_event_loop(traffic_master: TrafficMaster, delta_time: int) -> None:
    """Master event loop for traffic master."""
    for traffic_light in traffic_master.traffic_lights:
        next_light_state(traffic_light, delta_time)

def reset_traffic(traffic_master: TrafficMaster) -> None:
    """reset all traffic master's traffic lights."""
    for traffic_light in traffic_master.traffic_lights:
        reset_state(traffic_light)