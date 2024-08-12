from .traffic_light import TrafficLight, next_state, set_state, set_cycle_dur, set_tts, reset_time_in_state, TrafficState
from math import floor

class TrafficMaster:
    traffic_lights: dict[str, list[TrafficLight]] = {}  # eg "eglington": [TrafficLight_1, TrafficLight2]
    intersection_types: dict[tuple[str], dict] = []  # eg ("eglington", "towncentre") : {TrafficState.RED: 30, TrafficState.YELLOW: 2, TrafficState.GREEN:32, "duration_list": [30, 2, 32]}

    def __init__(self, intersection_types: list[tuple], t_lights: list[TrafficLight]) -> None:
        """Initialize all the differnet traffic lights in each list
        Have a central time, then send requests for each traffic light to switch light"""
        self.intersection_types, self.traffic_lights = {}, {}
        for inter_type in intersection_types:
            self.intersection_types[(inter_type[0], inter_type[1])] = {TrafficState.GREEN : inter_type[2], TrafficState.YELLOW : inter_type[3], TrafficState.RED: inter_type[4], "duration_list" : [inter_type[2], inter_type[3], inter_type[4]]}
            # Assuming inter_type is a tuple or list with two elements
            for key in inter_type[:2]:
                if key not in self.traffic_lights:
                    self.traffic_lights[key] = []
        for upcycle, downcycle in self.intersection_types.keys():
            for light in t_lights:
                state = TrafficState.GREEN
                if light.id == downcycle:
                    state = TrafficState.RED
                self.traffic_lights[light.id].append(light)
                reset_time_in_state(light)
                set_state(light, state)
                set_cycle_dur(light, sum(self.intersection_types[(upcycle, downcycle)]["duration_list"]))
                intersection_data = self.intersection_types[(upcycle, downcycle)]
                set_tts(light, {k: v for k, v in intersection_data.items() if k != "duration_list"})

def t_master_event_loop(traffic_master: TrafficMaster, delta_time: int) -> None:
    for l_type, light_list in traffic_master.traffic_lights.items():
        for light in light_list:
            light.time_in_state += (delta_time - light.prev_dt)
            light.prev_dt = delta_time
            if light.time_in_state > light.time_to_switch[light.state]:
                next_state(light)