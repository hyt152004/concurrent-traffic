import pygame
import numpy as np
from pygame import Surface
from classes.vehicle import Vehicle
from classes.node import Node
from classes.edge import Edge, StraightEdge, CircularEdge
from classes.route import route_position_to_world_position, direction_at_route_position
from manager.manager import Manager, CAR_COLLISION_DISTANCE, detect_collisions
from classes.button import Button
from .helper import world_to_screen_vector, world_to_screen_scalar, create_rotation_matrix, rotate_vector
from .simulator import WORLD_WIDTH, WORLD_HEIGHT, TOOLBAR_HEIGHT

pygame.font.init()
FONT = pygame.font.SysFont('Consolas', 20)

zoom_factor = 1

def set_zoomed_render(updated_zoomed: float) -> None:
    """Set zoom_factor to updated_zoomed."""
    global zoom_factor
    zoom_factor = updated_zoomed

def render_nodes(screen: Surface, nodes: list[Node]) -> None:
    """Render function for Nodes."""
    for node in nodes:
        node_position = world_to_screen_vector(screen, node.position, zoom_factor)
        pygame.draw.circle(screen, "red", node_position, 3)

def render_edges(screen: Surface, edges: list[Edge]) -> None:
    """Render function for Edges."""
    for edge in edges:
        if isinstance(edge, StraightEdge):
            start_position = world_to_screen_vector(screen, edge.start.position, zoom_factor)
            end_position   = world_to_screen_vector(screen, edge.end.position, zoom_factor)
            pygame.draw.line(screen, "red", start_position, end_position)
        elif isinstance(edge, CircularEdge):
            # define rect
            radius = world_to_screen_scalar(screen, np.linalg.norm(edge.start.position-edge.center), zoom_factor) # norm describes distance
            diameter = radius*2
            arc_rect = pygame.Rect(0,0,diameter,diameter)
            arc_rect.center = world_to_screen_vector(screen, edge.center, zoom_factor)

            theta_start = np.arctan2(-(edge.start.position[1] - edge.center[1]), edge.start.position[0] - edge.center[0])
            theta_end = np.arctan2(-(edge.end.position[1] - edge.center[1]), edge.end.position[0] - edge.center[0])

            if edge.clockwise:
                if theta_end < theta_start:
                    theta_end += 2*np.pi
                theta_end, theta_start = theta_start, theta_end
            else:
                if theta_start < theta_end:
                    theta_start += 2*np.pi

            pygame.draw.arc(screen, "red", arc_rect, theta_start, theta_end)

def render_intersections(screen: Surface, intersection_points) -> None:
    """Render function for intersecting Routes."""
    for intersection in intersection_points:
        node_position = world_to_screen_vector(screen, np.array(list(intersection[2])), zoom_factor)
        pygame.draw.circle(screen, "blue", node_position, 3)

def render_vehicles(screen: Surface, vehicles: list[Vehicle]) -> None:
    """Render function for Vehicles."""
    for vehicle in vehicles:
        vehicle_screen_width = world_to_screen_scalar(screen, vehicle.width, zoom_factor)
        vehicle_screen_length = world_to_screen_scalar(screen, vehicle.length, zoom_factor)
        
        vehicle_center_point = route_position_to_world_position(vehicle.route, vehicle.route_position)
        vehicle_center_screen_pos = world_to_screen_vector(screen, vehicle_center_point, zoom_factor)
        img = pygame.transform.smoothscale(vehicle.image, (vehicle_screen_length, vehicle_screen_width))
        vehicle_angle = direction_at_route_position(vehicle.route, vehicle.route_position)
        img = pygame.transform.rotate(img, vehicle_angle)
        car_rect = img.get_rect()
        car_rect.center = vehicle_center_screen_pos
        screen.blit(img, car_rect)
        car_collision_screen_distance = world_to_screen_scalar(screen, CAR_COLLISION_DISTANCE/2, zoom_factor)
        pygame.draw.circle(screen, "red", vehicle_center_screen_pos, car_collision_screen_distance, 1)
        vehicle_text_font = pygame.font.SysFont('Consolas', 12)
        text_surface = vehicle_text_font.render(vehicle.name, True, (139, 69, 19))
        screen.blit(text_surface, (car_rect.center[0]-(vehicle_text_font.size(vehicle.name)[0])/2, car_rect.center[1]-vehicle_screen_length))

def render_background(screen: Surface) -> None:
    """Render function for background."""
    position = world_to_screen_vector(screen, [-WORLD_WIDTH/2,-WORLD_HEIGHT/2], zoom_factor)
    width = world_to_screen_scalar(screen, WORLD_WIDTH, zoom_factor)
    height = world_to_screen_scalar(screen, WORLD_HEIGHT, zoom_factor)
    pygame.draw.rect(screen, "grey", pygame.Rect(position[0], position[1], width, height))

def render_border(screen: Surface) -> None:
    """Render function for border."""
    position = world_to_screen_vector(screen, [-WORLD_WIDTH/2,-WORLD_HEIGHT/2], zoom_factor)
    width = world_to_screen_scalar(screen, WORLD_WIDTH, zoom_factor)
    height = world_to_screen_scalar(screen, WORLD_HEIGHT, zoom_factor)
    pygame.draw.rect(screen, "maroon", pygame.Rect(position[0]-3, position[1]-3, width+6, height+6),3)

def render_world(screen: Surface, nodes: list[Node], edges: list[Edge], route_visible: bool, intersection_points) -> None:
    """Render function for simulator."""
    render_background(screen)
    if route_visible:
        render_nodes(screen, nodes)
        render_edges(screen, edges)
        render_arrows(screen, edges)
    render_intersections(screen, intersection_points)
    render_border(screen)
    # render_scenery()

def render_manager(screen: Surface, manager: Manager) -> None:
    """Render function for Manager."""
    radius = world_to_screen_scalar(screen, manager.radius, zoom_factor)
    manager_screen_pos = world_to_screen_vector(screen, manager.position, zoom_factor)

    circle_radius = 5 # px
    diameter = radius*2
    arc_rect = pygame.Rect(0,0,diameter,diameter)
    arc_rect.center = world_to_screen_vector(screen, manager.position, zoom_factor)
    pygame.draw.arc(screen, "green", arc_rect, 0, 2*np.pi)
    pygame.draw.circle(screen, "green", manager_screen_pos, circle_radius)
    
    for i, vehicle in enumerate(manager.vehicles):
        font = pygame.font.SysFont('Segoe UI', 15)
        text_surface = font.render(f"name: {vehicle.name}, pos: {vehicle.route_position:.2f}, accel: {vehicle.acceleration:.2f}, stamps: {vehicle.command.accel_func.x}", True, (0, 0, 0))
        screen.blit(text_surface, (5,i*20 + 5))

def render_time(screen: Surface, time_elapsed) -> None: 
    """Render function for time indicator."""
    font = pygame.font.SysFont('Segoe UI', 15)
    text_surface = font.render(f"Time: {time_elapsed:.3f}", True, (255, 255, 255))
    text_rect = text_surface.get_rect()
    text_rect.right = 150
    screen.blit(text_surface, text_surface.get_rect(topright = (screen.get_width()-3, screen.get_height()-TOOLBAR_HEIGHT)))

def render_buttons(screen: Surface, buttons: list[Button]) -> None:
    """Render function for Buttons."""
    for b in buttons:
        b.y = screen.get_height()-TOOLBAR_HEIGHT+50
        pygame.draw.rect(screen, b.hover_color if b.is_selected() else b.color, [b.x , b.y, b.width, b.height])
        if b.text != '':
            font = pygame.font.SysFont('Segoe UI', 15)
            text = font.render(b.text, 1, (255, 255, 255))
            screen.blit(text, (b.x + (b.width/2 - text.get_width()/2), b.y + (b.height/2 - text.get_height()/2)))

def render_toolbar(screen, time_elapsed, buttons):
    toolbar_rect = pygame.Rect(0, screen.get_height()-TOOLBAR_HEIGHT,screen.get_width(),TOOLBAR_HEIGHT)
    pygame.draw.rect(screen, pygame.Color(80,80,80), toolbar_rect)
    render_time(screen, time_elapsed)
    render_buttons(screen, buttons)

def render_arrows(screen: Surface, edges: list[Edge]):
    
    """Render function for arrows."""
            
    for edge in edges:
        if isinstance(edge, StraightEdge):
            # converts start and end position to screen position
            start_position = world_to_screen_vector(screen, edge.start.position, zoom_factor)
            end_position = world_to_screen_vector(screen, edge.end.position, zoom_factor)
            
            # finds midpoint of edge
            midpoint_position = (start_position + end_position) / 2
            midpoint_position_world = (edge.start.position + edge.end.position) / 2

            # finds direction of edge
            edge_vector = edge.end.position - edge.start.position
            edge_unit_vector = edge_vector / np.linalg.norm(edge_vector)

        elif isinstance(edge, CircularEdge):
            # define rect
            radius = np.linalg.norm(edge.start.position-edge.center) # norm describes distance

            theta_start = np.arctan2((edge.start.position[1] - edge.center[1]), edge.start.position[0] - edge.center[0])
            theta_end = np.arctan2((edge.end.position[1] - edge.center[1]), edge.end.position[0] - edge.center[0])

            if edge.clockwise:
                if theta_end < theta_start:
                    theta_end += 2*np.pi
                theta_end, theta_start = theta_start, theta_end
            else:
                if theta_start < theta_end:
                    theta_start += 2*np.pi
            
            theta_midpoint = (theta_start + theta_end) / 2
            
            midpoint_position_world = (edge.center[0] + radius*np.cos(theta_midpoint), edge.center[1] + radius*np.sin(theta_midpoint)) # (x,y) = (a+r*cos(theta), b+r*sin(theta)) <- equation
            midpoint_position = world_to_screen_vector(screen, midpoint_position_world, zoom_factor)
            tangent_line_world = (-radius*np.sin(theta_midpoint), radius*np.cos(theta_midpoint)) # derivative of center_point_world, finding tangent line of midpoint of the arc
            
            edge_unit_vector = tangent_line_world / np.linalg.norm(tangent_line_world) # <- vector divided by its magnitude
            
        # create rotation matrix
        rotation_pos_matrix = create_rotation_matrix(140)
        rotation_neg_matrix = create_rotation_matrix(-140)
            
        # rotate unit vector to create arrows
        positive_vector = rotate_vector(0.5, rotation_pos_matrix, edge_unit_vector, midpoint_position_world)
        negative_vector = rotate_vector(0.5, rotation_neg_matrix, edge_unit_vector, midpoint_position_world)

        # convert to screen vector & draw
        positive_screen_vector = world_to_screen_vector(screen, positive_vector, zoom_factor)
        negative_screen_vector = world_to_screen_vector(screen, negative_vector, zoom_factor)
        
        # draws onto screen
        pygame.draw.aaline(screen, "red", midpoint_position, positive_screen_vector, blend=40)
        pygame.draw.aaline(screen, "red", midpoint_position, negative_screen_vector, blend=40)

def render_toolbar(screen: Surface, time_elapsed, buttons: list[Button]) -> None:
    """Render function for toolbar."""
    toolbar_rect = pygame.Rect(0, screen.get_height()-TOOLBAR_HEIGHT,screen.get_width(),TOOLBAR_HEIGHT)
    pygame.draw.rect(screen, pygame.Color(80,80,80), toolbar_rect)
    render_time(screen, time_elapsed)
    render_buttons(screen, buttons)

def render_title(screen) -> None: 
    """Render function for title."""
    FONT = pygame.font.SysFont("Segoe UI", 15, bold=True, italic=False)
    text_surface = FONT.render(f"Concurent Traffic v0.0.2", True, (255, 255, 255))
    screen.blit(text_surface, (6,screen.get_height()-TOOLBAR_HEIGHT+6))

def render_red_vehicles(screen: Surface, vehicles: list[Vehicle]) -> None:
    """Render function for Vehicles."""
    for vehicle in vehicles:
        vehicle_screen_width = world_to_screen_scalar(screen, vehicle.width, zoom_factor)
        vehicle_screen_length = world_to_screen_scalar(screen, vehicle.length, zoom_factor)

        img = pygame.transform.smoothscale(vehicle.image, (vehicle_screen_length, vehicle_screen_width))
        vehicle_angle = direction_at_route_position(vehicle.route, vehicle.route_position)
        img = pygame.transform.rotate(img, vehicle_angle)
        vehicle_center_point = route_position_to_world_position(vehicle.route, vehicle.route_position)
        vehicle_center_screen_pos = world_to_screen_vector(screen, vehicle_center_point, zoom_factor)

        img_size = img.get_size()
        
        hue_surface = pygame.Surface(img_size)
        hue_surface.fill((255,0,0))
        hue_surface.set_alpha(100)
        
        img_with_hue = img.copy()
        img_with_hue.blit(hue_surface, (0,0), special_flags=pygame.BLEND_RGBA_MULT)

        img_with_hue_rect = img_with_hue.get_rect()
        img_with_hue_rect.center = vehicle_center_screen_pos
        
        screen.blit(img_with_hue, img_with_hue_rect)



