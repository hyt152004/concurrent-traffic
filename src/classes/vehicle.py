import numpy as np
import pygame
from pygame import Surface
from manager.command import Command
from classes.route import Route, route_position_to_world_position

class Vehicle:
    """A Vehicle is given commands that it follows along a given route."""

    id: int                  = None            # vehicle identifier
    name: str                = None            # vehicle name
    route: Route             = None            # Route
    route_position: float     = 0               # float representing position in meters along the route

    default_velocity          = 0               # float representing default velocity in meters/second along route
    velocity: float           = 0               # float representing velocity in meters/second along route
    acceleration: float       = 0               # float representing acceleration in meters/second^2 along route

    width: float              = 2.23            # float representing width of car in meters. orthogonal to direction
    length: float             = 4.90            # float representing length of car in meters. parallel to direction
    pivot_distance: float     = 1.25            # float representing distance from pivot to center.
    image: Surface
    direction_angle           = 0
    leading_vehicle             = None          # vehicle the current vehicle is trailing if any

    command: Command          = Command(np.array([0]), np.array([0]))             # Command

    def __init__(self,
                 name: str,
                 id: int,
                 route: Route,
                 route_position: float,
                 velocity: float,
                 acceleration: float,
                 width: float,
                 length: float,
                 pivot_distance: float,
                 image_source: str,
                 ) -> None:
        self.id = id
        self.name = name
        self.route = route
        self.route_position = route_position
        self.velocity = velocity
        self.acceleration = acceleration
        self.width = width
        self.length = length
        self.pivot_distance = pivot_distance
        self.image_source = image_source
        self.image = pygame.image.load(self.image_source)
        self.leading_vehicle = None
        self.default_velocity = velocity

# helpers

# def get_vehicle_center_point(vehicle: Vehicle):
    


#     vehicle_center_x = vehicle.position[0] + vehicle.direction[0] * vehicle.pivot_distance
#     vehicle_center_y = vehicle.position[1] + vehicle.direction[1] * vehicle.pivot_distance
#     return vehicle_center_x, vehicle_center_y

def vehicle_copy(vehicles: list[Vehicle]) -> list[Vehicle]:
    """Return a deep copy of a list of Vehicles."""
    return [Vehicle(v.id, v.name, v.route, v.route_position, v.velocity, v.acceleration, v.width, v.length, v.pivot_distance, v.image_source) for v in vehicles]
  
def vehicle_event_loop(vehicle: Vehicle, delta_time: float) -> None:
    """Event loop for Vehicle."""
    vehicle.acceleration = vehicle.command(delta_time)

def update_cmd(old_cmd: Command, t: np.array, a: np.array, elapsed_time: float=0) -> Command:
    """Return new Command, a concatenation of the old_cmd and new acceleration-time calculations."""
    del_index = None
    for i in range(len(old_cmd.accel_func.x)):
        if old_cmd.accel_func.x[i] >= elapsed_time:
            # print(i, len(old_cmd.accel_func.x))
            del_index = i
            break

    new_t = np.concatenate((old_cmd.accel_func.x[:del_index], t))
    new_a = np.concatenate((old_cmd.accel_func.y[:del_index], a))

    return Command(new_t, new_a)

### FUNCTIONS FOR STANDARD TRAFFIC

def driver_traffic_update_command(vehicles: list, cur_time: float) -> None:
    """Update command for standard traffic."""

    # calculate command that achieves two things
    # 1. stop before any traffic stop line if that traffic light is red
    #    traffic light red? stop before it
    #    traffic light green? speed up
    # 2. maintain distance between car with closest collision (the car in front)
    #    if conditions match to calculate new command
    #    a. is our distance to that car within X?
    #       is their acceleration lower than ours?
    #       -> calculate slow down
    #    b. is our distance to that car more than Y?
    #       is their acceleration more than ours?
    #       -> calculate speed up

    # 1 and 2 need to work together. If 2. determines that we can speed up,
    # but the car is approaching a red traffic light,
    # then the final command should be to slow down
    
    update_driver_lead(vehicles)
    for vehicle in vehicles:
        leading_vehicle = vehicle.leading_vehicle
        initial_velocity = vehicle.velocity
        
        if leading_vehicle:
            final_velocity = leading_vehicle.velocity
            distance_to_leading = abs(vehicle.route_position - leading_vehicle.route_position) 

            # ALL VEHICLES MUST BE PLACED 10m AWAY FROM EACH OTHER AT THE START
            safety_distance = 9
            distance = distance_to_leading - safety_distance
            required_deceleration = (final_velocity**2 - initial_velocity**2) / (2 * distance)
            
            new_t = np.array([cur_time, cur_time + 0.1])
            new_a = np.array([required_deceleration, leading_vehicle.acceleration])
            
            vehicle.command = update_cmd(vehicle.command, new_t, new_a, cur_time)
    
        elif abs(vehicle.velocity - vehicle.default_velocity) > 0.01:
            acceleration_distance = 10
            required_deceleration = (vehicle.default_velocity**2 - initial_velocity**2) / (2 * acceleration_distance)
            
            new_t = np.array([cur_time, cur_time + 0.1])
            new_a = np.array([required_deceleration, vehicle.acceleration])
            
            vehicle.command = update_cmd(vehicle.command, new_t, new_a, cur_time)
            
MIN_LEADING_DIST = 30

def update_driver_lead(vehicles: list) -> None:

    max_angle_diff = 80
    for i, tv in enumerate(vehicles):
        curr_lv = tv.leading_vehicle
        curr_lv_wp = route_position_to_world_position(curr_lv.route, curr_lv.route_position) if curr_lv else None
        tv_wp = route_position_to_world_position(tv.route, tv.route_position)
        distance_curr_lv = np.linalg.norm(curr_lv_wp - tv_wp) if curr_lv else None

        for j, potential_lv in enumerate(vehicles):
            if i == j:  # Avoid comparing the vehicle with itself
                continue

            if tv.route_position > potential_lv.route_position:
                continue

            if abs(tv.direction_angle - potential_lv.direction_angle) > max_angle_diff:
                continue

            potential_lv_wp = route_position_to_world_position(potential_lv.route, potential_lv.route_position)
            distance_potential_lv = np.linalg.norm(potential_lv_wp - tv_wp)
            
            if distance_curr_lv is None or distance_potential_lv < distance_curr_lv:
                curr_lv = potential_lv
                distance_curr_lv = distance_potential_lv

        # if this is the last iteration, and the vehicle is greater than 30 meters,
        # the leading vehicle will be set to None, regardless of closer cars
        # This issue is resolved
        if distance_curr_lv is None or distance_curr_lv > MIN_LEADING_DIST:
            tv.leading_vehicle = None
        else:
            tv.leading_vehicle = curr_lv