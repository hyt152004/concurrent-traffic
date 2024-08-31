import pygame
from classes.vehicle import Vehicle, vehicle_copy
from manager.manager import reset
from standard_traffic.traffic_master import reset_traffic


class Button:
    """This class is for creating Buttons with on-click functionality."""
    color: tuple[int, int, int]
    hover_color: tuple[int, int, int]
    x: tuple[int, int]
    y: tuple[int, int]
    width: int
    height: int
    text: str
    on_click: callable
    cargs: tuple

    def __init__(self, color: tuple[int, int, int], hover_color: tuple[int, int, int], start_point: tuple[int, int], dim: tuple[int, int], text: str, on_click: callable=None, cargs: tuple=()) -> None:
        self.color = color
        self.hover_color = hover_color
        self.x = start_point[0]
        self.y = start_point[1]
        self.width = dim[0]
        self.height = dim[1]
        self.text = text
        self.on_click = on_click
        self.cargs = cargs

def is_selected(button: Button) -> bool:
    """Return True if the mouse is on the button, else False"""
    mouse = pygame.mouse.get_pos()
    return button.x <= mouse[0] <= button.x + button.width and button.y <= mouse[1] <= button.y + button.height

def click(button: Button) -> None:
    """Runs self.on_click function if such a function exists and the button is selected"""
    if is_selected(button) and button.on_click:
        if isinstance(button.cargs, tuple):
            button.on_click(*button.cargs)
        else:
            button.on_click(button.cargs)

MIN_PLAYBACK_SPEED_FACTOR = 0.0625
MAX_PLAYBACK_SPEED_FACTOR = 2

def toggle_update(simulation_values: dict) -> None:
    """Toggles between resuming or pausing the simulator."""
    simulation_values["is_run"] = not simulation_values["is_run"]

def restart_func(simulation_values: dict, initial_vehicles: list[Vehicle]) -> None:
    """Resets the simulator."""

    simulation_values["vehicles"] = vehicle_copy(initial_vehicles)
    simulation_values["clock"] = pygame.time.Clock()
    simulation_values["delta_time"] = 0
    simulation_values["time_elapsed"] = 0
    reset(simulation_values["manager"])
    reset_traffic(simulation_values["traffic_master"])

def toggle_route_visibility(settings: dict) -> None:
    """Toggles route visibility."""
    settings["route_visible"] = not settings["route_visible"]

def toggle_playback_speed(settings: dict, operation: str) -> None:
    """Adjusts playback speed."""
    if operation == "+":
        settings["playback_speed_factor"] = min(MAX_PLAYBACK_SPEED_FACTOR, settings["playback_speed_factor"] * 2)
    elif operation == "-":
        settings["playback_speed_factor"] = max(MIN_PLAYBACK_SPEED_FACTOR, settings["playback_speed_factor"] / 2)
    settings["display_playback_speed"] = str(settings["playback_speed_factor"]) + "x"
    
def toggle_algorithm_selector(settings: dict, simulation_values: dict, algorithm: str) -> None:
    settings["selected_algorithm"] = algorithm
    restart_func(simulation_values, simulation_values["initial_vehicles"])