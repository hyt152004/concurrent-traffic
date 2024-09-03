ORIGINAL_SCREEN_WIDTH = 1000
ORIGINAL_SCREEN_HEIGHT = 720
WORLD_WIDTH = 160
WORLD_HEIGHT = 160
TOOLBAR_HEIGHT = 100
MIN_ZOOM_FACTOR = 1
MAX_ZOOM_FACTOR = 8

import pygame

from classes.vehicle import Vehicle, vehicle_event_loop, vehicle_copy, driver_traffic_update_command
from classes.button import Button, restart_func, toggle_update, toggle_algorithm_selector, toggle_playback_speed, toggle_route_visibility, click
from manager.manager import Manager, manager_event_loop, detect_collisions
from classes.node import Node
from classes.edge import Edge
from classes.route import Route
from standard_traffic.traffic_master import TrafficMaster, traffic_event_loop
from .render import render_world, render_manager, render_vehicles, render_toolbar, render_title, set_zoomed_render, render_traffic_lights, render_loop_times
from .update import update_world
from .helper import scroll_handler


def toggle_zoom(settings: dict) -> None:
    """Toggles between max and min zoom."""
    if settings["zoom_factor"] == 1:
        settings["zoom_factor"] = MAX_ZOOM_FACTOR
    else:
        settings["zoom_factor"] = MIN_ZOOM_FACTOR
    set_zoomed_render(settings["zoom_factor"])

def run_simulation(initial_vehicles: list[Vehicle], nodes: list[Node], edges: list[Edge], routes: list[Route], intersection_points, manager: Manager, traffic_master: TrafficMaster) -> None:
    """Initializes and runs the pygame simulator. Requires initialization of lanes, manager, vehicles."""
    pygame.init()
    screen = pygame.display.set_mode((ORIGINAL_SCREEN_WIDTH, ORIGINAL_SCREEN_HEIGHT), pygame.RESIZABLE)
    running = True

    settings = {
        "zoom_factor": 1,
        "playback_speed_factor": 1.0,
        "route_visible": True,
        "selected_algorithm": "v0",
        "display_playback_speed": "1.0x"
    }

    simulation_values = {
        "is_run": True,
        "delta_time": 0,
        "time_elapsed": 0,
        "vehicles": vehicle_copy(initial_vehicles),
        "initial_vehicles": initial_vehicles,
        "traffic_master": traffic_master,
        "manager": manager,
        "clock": pygame.time.Clock()
    }
    
    not_selected_color = (40, 40, 40)
    selected_color = (255, 50, 50)

    toggle_button = Button(not_selected_color, selected_color, (5, screen.get_height()-TOOLBAR_HEIGHT+50), (100, 30), 'toggle update', toggle_update, simulation_values)
    restart_button = Button(not_selected_color, selected_color, (110, screen.get_height()-TOOLBAR_HEIGHT+50), (100, 30), 'restart', restart_func, (simulation_values, initial_vehicles))
    routes_visibility_button = Button(not_selected_color, selected_color, (215, screen.get_height()-TOOLBAR_HEIGHT+50), (150, 30), 'toggle route visibility', toggle_route_visibility, settings)
    zoom_button = Button(not_selected_color, selected_color, (370, screen.get_height()-TOOLBAR_HEIGHT+50), (70, 30), 'zoom', toggle_zoom, settings)

    subtract_playback_speed = Button(not_selected_color, selected_color, (445, screen.get_height()-TOOLBAR_HEIGHT+50), (35, 30), '-', toggle_playback_speed, (settings, "-"))
    display_playback_speed = Button(not_selected_color, not_selected_color, (480, screen.get_height()-TOOLBAR_HEIGHT+50), (45, 30), settings["display_playback_speed"])
    add_playback_speed = Button(not_selected_color, selected_color, (525, screen.get_height()-TOOLBAR_HEIGHT+50), (35, 30), '+', toggle_playback_speed, (settings, "+"))
    
    algorithm_selector_v0 = Button(not_selected_color, selected_color, (565, screen.get_height()-TOOLBAR_HEIGHT+50), (135, 30), 'ALG 0', toggle_algorithm_selector, (settings, simulation_values, "v0"))
    algorithm_selector_standard_traffic = Button(not_selected_color, selected_color, (700, screen.get_height()-TOOLBAR_HEIGHT+50), (135, 30), 'Standard Traffic', toggle_algorithm_selector, (settings, simulation_values, "traffic"))

    buttons = [toggle_button, restart_button, routes_visibility_button, zoom_button, subtract_playback_speed, add_playback_speed, display_playback_speed, algorithm_selector_v0, algorithm_selector_standard_traffic]

    while running:
        # poll for events
        # pygame.QUIT event means the user clicked X to close your window
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
            if event.type == pygame.MOUSEBUTTONDOWN:
                [click(b) for b in buttons]

            elif event.type == pygame.MOUSEWHEEL:
                settings["zoom_factor"] = scroll_handler(event, settings["zoom_factor"])
                set_zoomed_render(settings["zoom_factor"])

        # fill the screen with a color to wipe away anything from last frame
        screen.fill(pygame.Color(150,150,150))

        # optionally render nodes and edges. for now always on
        render_world(screen, nodes, edges, settings["route_visible"], intersection_points)

        render_vehicles(screen, simulation_values["vehicles"])
        render_toolbar(screen, simulation_values["time_elapsed"], buttons)
        render_title(screen)

        # manager 'cpu' or standard traffic 
        if settings["selected_algorithm"] == "v0":
            render_manager(screen, simulation_values["manager"])
            manager_event_loop(simulation_values["manager"], simulation_values["vehicles"], simulation_values["time_elapsed"])
            algorithm_selector_v0.color = selected_color
            algorithm_selector_standard_traffic.color = not_selected_color
        else:
            driver_traffic_update_command(simulation_values["vehicles"], simulation_values["time_elapsed"])
            render_traffic_lights(screen, simulation_values["traffic_master"])
            traffic_event_loop(simulation_values["traffic_master"], simulation_values["time_elapsed"])
            algorithm_selector_standard_traffic.color = selected_color
            algorithm_selector_v0.color = not_selected_color

        display_playback_speed.text = settings["display_playback_speed"]

        # simulation_values["vehicles"] 'cpu'
        for vehicle in simulation_values["vehicles"]:
            vehicle_event_loop(vehicle, simulation_values["time_elapsed"])

        # vehicle removal 
        for vehicle in simulation_values["vehicles"]:
            if vehicle.route_position > vehicle.route.total_length:
                simulation_values["vehicles"].remove(vehicle)

        if simulation_values["is_run"]:
            # physical changes to world (updating positions, velocity, etc.)
            update_world(simulation_values["delta_time"] * settings["playback_speed_factor"], simulation_values["vehicles"])
            simulation_values["time_elapsed"] += simulation_values["delta_time"] * settings["playback_speed_factor"]

        # checks if collision has occured
        if detect_collisions(simulation_values["manager"], simulation_values["vehicles"], simulation_values["time_elapsed"]) == True:
            simulation_values["is_run"] = False


        # updates the screen
        simulation_values["delta_time"] = 1 / 60
        simulation_values["loop_time"] = simulation_values["clock"].tick(60) / 1000
        if simulation_values["delta_time"] - simulation_values["loop_time"]  > 0:
            pygame.time.wait(int((simulation_values["delta_time"] - simulation_values["loop_time"] ) * 1000)) # in milliseconds
        render_loop_times(screen, simulation_values["delta_time"], simulation_values["loop_time"] )

        pygame.display.update()
  
    pygame.quit()
