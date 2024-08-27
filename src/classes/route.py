import numpy as np
from .edge import Edge, StraightEdge, CircularEdge, get_length
from standard_traffic.traffic_light import TrafficLight
from typing import Optional

class Route():
    """A Route consists of Edges linked together."""
    current_id: int
    edges: list[Edge]
    total_length: float = 0
    pos_to_edge_map: dict[tuple, Edge]

    def __init__(self, id: int, edges: list[Edge]) -> None:
        self.current_id = id
        self.edges = edges
        self.pos_to_edge_map = {}
        for e in self.edges:
            curr_length = get_length(e)
            self.pos_to_edge_map[(self.total_length, self.total_length + curr_length)] = e
            self.total_length += curr_length

def route_position_to_world_position(route: Route, position: float) -> Optional[np.array]:
    """Return world coordinates given a route and position along that route."""
    edge_of_position = None
    percentage_on_edge = None
    for r in route.pos_to_edge_map:
        if position < r[1] and position >= r[0]:
            edge_of_position = route.pos_to_edge_map[r]
            percentage_on_edge = (position - r[0]) / (r[1] - r[0])
            break
    if edge_of_position is None:
        return None
    if isinstance(edge_of_position, StraightEdge):
        world_x = (1-percentage_on_edge)*edge_of_position.start.position[0] + percentage_on_edge*edge_of_position.end.position[0]
        world_y = (1-percentage_on_edge)*edge_of_position.start.position[1] + percentage_on_edge*edge_of_position.end.position[1]
    elif isinstance(edge_of_position, CircularEdge):
        theta_start = np.arctan2(edge_of_position.start.position[1] - edge_of_position.center[1], edge_of_position.start.position[0] - edge_of_position.center[0])
        theta_end   = np.arctan2(edge_of_position.end.position[1] - edge_of_position.center[1], edge_of_position.end.position[0] - edge_of_position.center[0])

        if edge_of_position.clockwise:
            if theta_end < theta_start:
                theta_end += 2*np.pi
        else:
            if theta_start < theta_end:
                theta_start += 2*np.pi

        theta_of_pos = theta_start + percentage_on_edge * (theta_end - theta_start)
        world_x = edge_of_position.center[0] + edge_of_position.radius * np.cos(theta_of_pos)
        world_y = edge_of_position.center[1] + edge_of_position.radius * np.sin(theta_of_pos)
    
    return np.array([world_x, world_y])

def direction_at_route_position(route: Route, position: float) -> float:
    """Return the direction at a given position on a given route."""
    edge_of_position = None
    percentage_on_edge = None

    for r in route.pos_to_edge_map:
        if position < r[1] and position >= r[0]:
            edge_of_position = route.pos_to_edge_map[r]
            percentage_on_edge = (position - r[0]) / (r[1] - r[0])
            break

    if edge_of_position is None:
        return -1
    
    if isinstance(edge_of_position, StraightEdge):
        direction_x = edge_of_position.end.position[0] - edge_of_position.start.position[0]
        direction_y = - (edge_of_position.end.position[1] - edge_of_position.start.position[1])
        vehicle_angle =  np.arctan2(direction_y, direction_x) * (180 / np.pi)
    
    elif isinstance(edge_of_position, CircularEdge):
        theta_start = np.arctan2(edge_of_position.start.position[1] - edge_of_position.center[1], edge_of_position.start.position[0] - edge_of_position.center[0])
        theta_end   = np.arctan2(edge_of_position.end.position[1] - edge_of_position.center[1], edge_of_position.end.position[0] - edge_of_position.center[0])
        perpendicular_rotation = 0

        if edge_of_position.clockwise:
            perpendicular_rotation = np.pi/2
            if theta_end < theta_start:
                theta_end += 2*np.pi
        else:
            perpendicular_rotation = - np.pi/2
            if theta_start < theta_end:
                theta_start += 2*np.pi

        theta_of_pos = theta_start + percentage_on_edge * (theta_end - theta_start) 
        direction_x = np.cos(theta_of_pos + perpendicular_rotation)
        direction_y = - np.sin(theta_of_pos + perpendicular_rotation)
        vehicle_angle =  np.arctan2(direction_y, direction_x) * (180 / np.pi) 

    return vehicle_angle

def world_position_to_route_position(route: Route, edge: Edge, position: np.ndarray) -> float:
    """Return position along route given a route, edge and world position on the edge.
    Currently assumes the world position will be on the edge, should be enforced in the future."""
    route_position = 0
    if isinstance(edge, StraightEdge):
        arclength = np.sqrt((position[0] - edge.start.position[0])**2 + (position[1] - edge.start.position[1])**2)
        
    elif isinstance(edge, CircularEdge):
        theta_of_start = np.arctan2(edge.start.position[1] - edge.center[1], edge.start.position[0] - edge.center[0])
        theta_of_pos = np.arctan2(position[1] - edge.center[1], position[0] - edge.center[0])
        delta_theta = theta_of_pos - theta_of_start
        # adjust angle to be positive and within 2pi
        if delta_theta < 0:
            delta_theta += 2*np.pi
        arclength = edge.radius * delta_theta

    index_of_edge = route.edges.index(edge)
    for edge_index in range(index_of_edge):
        route_position += get_length(route.edges[edge_index])
    route_position += arclength
    return route_position