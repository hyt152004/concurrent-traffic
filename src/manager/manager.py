import numpy as np
from classes.vehicle import Vehicle, update_cmd
from classes.route import Route, route_position_to_world_position, direction_at_route_position
from itertools import combinations
from scipy.optimize import minimize_scalar
import logging

CAR_COLLISION_DISTANCE = 2.5 # meters
MINIMUM_CRUISING_SPEED = 0
DESIRED_CRUISING_SPEED = 20
MAX_ACCELERATION = 3.5

from colorama import init as colorama_init
from colorama import Fore
from colorama import Style
colorama_init()

class Collision:
    """A Collision represents a collision between two Vehicles at a given time."""
    vehicle0: Vehicle
    vehicle1: Vehicle
    time: float

    def __init__(self, vehicle0: Vehicle, vehicle1: Vehicle, time: float) -> None:
        self.vehicle0 = vehicle0
        self.vehicle1 = vehicle1
        self.time = time

class Manager:
    """A Manager controls all Vehicles within its radius, calculating and sending commands to ensure that Vehicles do
    not collide with each other."""
    position: np.ndarray
    radius: float = 25
    vehicles: list[Vehicle] = []
    intersecting_points = None
    collisions: list[Collision] = []
    run_once_for_debug = 0


    def __init__(self, position: np.ndarray, radius: float, routes: list[Route]) -> None:
        # initialize
        self.position = position
        self.radius = radius
        self.i = 0
        self.logger = logging.getLogger(self.__class__.__name__)

def reset(manager: Manager) -> None:
    """Clear manager.vehicles attribute."""
    manager.vehicles.clear()

def manager_event_loop(manager: Manager, vehicles: list[Vehicle], cur_time: float) -> None:
    """Event loop for Manager. Updates manager.vehicles if a Vehicle enters its radius. Also recalculates and sends Commands on update of manager.vehicles."""
    if _update_manager_vehicle_list(manager, vehicles, cur_time):
        _compute_and_send_acceleration_commands(manager, cur_time)

def _update_manager_vehicle_list(manager: Manager, vehicles: list[Vehicle], elapsed_time: float) -> bool:
    """Return True if new vehicles have been added to manager.vehicles."""
    new_vehicle = False
    for vehicle in vehicles:
        # vehicle within manager radius? 
        route_position_of_vehicle = route_position_to_world_position(vehicle.route, vehicle.route_position)
        if route_position_of_vehicle is None: # this vehicle is out of its route and returns no position
            continue
        distance_to_vehicle = np.linalg.norm(route_position_of_vehicle-manager.position)

        # vehicle already in list and within manager radius?
        vehicle_in_list = any(manager_vehicle.id == vehicle.id for manager_vehicle in manager.vehicles)

        # append if not in list and inside radius
        if not vehicle_in_list and distance_to_vehicle <= manager.radius:
            manager.vehicles.append(vehicle)

            # set command to get vehicle to desired cruising speed
            accel_duration = (DESIRED_CRUISING_SPEED - vehicle.velocity) / MAX_ACCELERATION
            t = [elapsed_time, elapsed_time + accel_duration]
            a = [MAX_ACCELERATION, 0]
            
            vehicle.command = update_cmd(vehicle.command, t, a, elapsed_time)

            new_vehicle = True

        # remove if in list and outside radius
        elif vehicle_in_list and distance_to_vehicle > manager.radius:
            manager.vehicles.remove(vehicle)
    return new_vehicle

def corner_vector_to_pos(v_pos: np.ndarray, v_angle: float, vector: np.ndarray):
    x =  np.cos(np.deg2rad(v_angle)) * vector[0] + np.sin(np.deg2rad(v_angle)) * vector[1]
    y = -np.sin(np.deg2rad(v_angle)) * vector[0] + np.cos(np.deg2rad(v_angle)) * vector[1]
    return np.array([v_pos[0] + x, v_pos[1] + y])

def get_collision(manager: Manager, cur_time: float) -> Collision | None:
    """Return Collision between two Vehicles in manager's radius. Return None if there are no Collisions."""
    vehicle_pairs = combinations(manager.vehicles, 2)
    
    for vehicle_pair in vehicle_pairs:
        collision = get_collisions_between_two_vehicles(vehicle_pair[0], vehicle_pair[1], cur_time)
        if collision is not None:
            return collision
    return None

def get_collisions_between_two_vehicles(vehicle0: Vehicle, vehicle1: Vehicle, cur_time: float) -> Collision | None:
    vehicle_out_of_bounds_time = int(min(time_until_end_of_route(vehicle0, cur_time), time_until_end_of_route(vehicle1, cur_time)))

    v0_angle = vehicle0.direction_angle
    v0_corner_vectors = []
    v0_corner_vectors.append(np.array([vehicle0.length/2, 0]))
    # v0_corner_vectors.append(np.array([vehicle0.length/2, -(vehicle0.width/2)]))
    v0_corner_vectors.append(np.array([-vehicle0.length/2, 0]))
    # v0_corner_vectors.append(np.array([-vehicle0.length/2, -(vehicle0.width/2)]))

    v1_angle = vehicle1.direction_angle
    v1_corner_vectors = []
    v1_corner_vectors.append(np.array([vehicle1.length/2, 0]))
    # v1_corner_vectors.append(np.array([vehicle1.length/2 + 3, -(vehicle1.width/2)]))
    v1_corner_vectors.append(np.array([-vehicle1.length/2, 0]))
    # v1_corner_vectors.append(np.array([-vehicle1.length/2, -(vehicle1.width/2)]))

    for v0_corner_vector in v0_corner_vectors:
        for v1_corner_vector in v1_corner_vectors:

            def distance_objective(t):
                rp0 = route_pos_at_delta_time(vehicle0, t, cur_time)
                rp1 = route_pos_at_delta_time(vehicle1, t, cur_time)
                wp0 = route_position_to_world_position(vehicle0.route, rp0)
                wp1 = route_position_to_world_position(vehicle1.route, rp1)
                if wp0 is None or wp1 is None:
                    return np.inf
                a0 = direction_at_route_position(vehicle0.route, rp0)
                a1 = direction_at_route_position(vehicle1.route, rp1)
                v0_corner_vector_pos = corner_vector_to_pos(wp0, a0, v0_corner_vector)
                v1_corner_vector_pos = corner_vector_to_pos(wp1, a1, v1_corner_vector)
                return np.linalg.norm(v1_corner_vector_pos-v0_corner_vector_pos)
                
            result = minimize_scalar(distance_objective, bounds=(0, vehicle_out_of_bounds_time), method='bounded')
            if result.success:
                time_of_collision = result.x + cur_time
                if distance_objective(result.x) <= CAR_COLLISION_DISTANCE:
                    rp0 = route_pos_at_delta_time(vehicle0, result.x, cur_time)
                    rp1 = route_pos_at_delta_time(vehicle1, result.x, cur_time)
                    wp0 = route_position_to_world_position(vehicle0.route, rp0)
                    wp1 = route_position_to_world_position(vehicle1.route, rp1)
                    a0 = direction_at_route_position(vehicle0.route, rp0)
                    a1 = direction_at_route_position(vehicle1.route, rp1)
                    v0_corner_vector_pos = corner_vector_to_pos(wp0, a0, v0_corner_vector)
                    v1_corner_vector_pos = corner_vector_to_pos(wp1, a1, v1_corner_vector)
                    print(f"distance was: {Fore.RED}{round(distance_objective(result.x), 2)}m{Style.RESET_ALL}")
                    print(f"v0_corner_pos: ({Fore.YELLOW}{round(v0_corner_vector_pos[0], 2)}{Style.RESET_ALL},{Fore.YELLOW}{round(v0_corner_vector_pos[1], 2)}{Style.RESET_ALL})")
                    print(f"v1_corner_pos: ({Fore.YELLOW}{round(v1_corner_vector_pos[0], 2)}{Style.RESET_ALL},{Fore.YELLOW}{round(v1_corner_vector_pos[1], 2)}{Style.RESET_ALL})")
                    return Collision(vehicle0, vehicle1, time_of_collision)
    return None
    
def route_pos_at_delta_time(vehicle: Vehicle, delta_time: float, cur_time: float) -> float:
    """Return vehicle's route position along its Route after delta_time seconds has passed."""

    if delta_time == 0:
        return vehicle.route_position
    # this allows us to assume that there will always be at least 2 elements in junction_list

    end_time = cur_time + delta_time
    # A junction is a period of time in which we can assume there is no change to acceleration
    # create junction lists
    junction_lists = [cur_time]
    for i in range(len(vehicle.command.accel_func.x)):
        if vehicle.command.accel_func.x[i] > cur_time and vehicle.command.accel_func.x[i] < end_time:
            junction_lists.append(vehicle.command.accel_func.x[i])
    junction_lists.append(end_time)

    start_junction_index = 0
    next_junction_index = 1
    velocity = vehicle.velocity
    delta_distance = 0
    while(start_junction_index != len(junction_lists)-1):
        junction_time_delta = junction_lists[next_junction_index] - junction_lists[start_junction_index]
        acceleration = vehicle.command.accel_func(junction_lists[start_junction_index])
        delta_distance += velocity*junction_time_delta + 0.5*acceleration*junction_time_delta**2
        velocity = velocity + acceleration*junction_time_delta

        start_junction_index += 1
        next_junction_index += 1

    return vehicle.route_position + delta_distance

def time_until_end_of_route(vehicle: Vehicle, cur_time: float) -> float:
    """Return time til vehicle reaches the end of its route."""
    # cur_v = vehicle.velocity

    # end_of_route = vehicle.route.total_length
    # cur_pos = vehicle.route_position
    # t = cur_time

    # i = np.where(vehicle.command.accel_func.y == vehicle.command(cur_time))[0][0]

    # while cur_pos < end_of_route and i < len(vehicle.command.accel_func.x) - 1:
    #     duration = vehicle.command.accel_func.x[i + 1] - vehicle.command.accel_func.x[i]
    #     cur_v += duration * vehicle.command.accel_func.y[i]
    #     cur_pos += (cur_v * duration) + (0.5 * vehicle.command.accel_func.y[i] * (duration ** 2))
    #     t += duration
    #     i += 1
    return 10

def _compute_and_send_acceleration_commands(manager: Manager, elapsed_time: float) -> None:
    """Compute and send commands."""

    updated_vehicles = set()
    # manager.vehicles.sort(key=lambda v: v.route_position, reverse=True)
    collision = get_collision(manager, elapsed_time)

    while collision != None:
        # initial durations
        decel = -MAX_ACCELERATION
        deceling_duration = 0
        deceled_duration = 0
        attempts_to_deter_collision = 0

        # find lower priority vehicle
        vehicle0_index = manager.vehicles.index(collision.vehicle0)
        vehicle1_index = manager.vehicles.index(collision.vehicle1)
        cur_vehicle = collision.vehicle0 if vehicle0_index > vehicle1_index else collision.vehicle1
        print(f"attempting to resolve collision between {Fore.LIGHTBLUE_EX}{collision.vehicle0.name}{Style.RESET_ALL} and {Fore.LIGHTBLUE_EX}{collision.vehicle1.name}{Style.RESET_ALL} at {Fore.GREEN}{round(collision.time, 3)}s{Style.RESET_ALL}")
        print(f"vehicle to modify: {cur_vehicle.name}")

        cur_vehicle_original_cmd = cur_vehicle.command

        while collision is not None:
            # first and foremost if we've tried 3000 times to resolve this collision, just give up
            if attempts_to_deter_collision >= 300:
                raise RuntimeError("failed to deter collision in 300 attempts :(")

            # if lower_priority_vehicle already had a command sent to it?
            if cur_vehicle in updated_vehicles:
                index = np.where(cur_vehicle.command.accel_func.x == elapsed_time)[0][0]
                cur_vehicle.command(elapsed_time)
                deceling_duration = cur_vehicle.command.accel_func.x[index + 1] - cur_vehicle.command.accel_func.x[index]
                deceled_duration = cur_vehicle.command.accel_func.x[index + 2] - cur_vehicle.command.accel_func.x[index + 1]
            
            # max decelerating duration is when vehicle will come to a complete stop
            max_deceling_duration = (MINIMUM_CRUISING_SPEED-cur_vehicle.velocity)/decel-0.05
            deceling_duration = min(deceling_duration, max_deceling_duration)

            # increase decelerating duration, and if that isn't possible, increase duration of staying decelerated
            if (deceling_duration + 0.1) < max_deceling_duration:
                deceling_duration += 0.1
            else:
                deceled_duration += 0.1

            deceled_velocity = cur_vehicle.velocity + decel * deceling_duration
            accel_duration = (DESIRED_CRUISING_SPEED - deceled_velocity) / MAX_ACCELERATION

            # compose the command
            t = [elapsed_time,
                elapsed_time + deceling_duration,
                elapsed_time + deceling_duration + deceled_duration,
                elapsed_time + deceling_duration + deceled_duration + accel_duration]
            a = [-MAX_ACCELERATION,
                0,
                MAX_ACCELERATION,
                0]
            
            route_pos_before_cmd = route_pos_at_delta_time(cur_vehicle, collision.time-elapsed_time, elapsed_time)
            cur_vehicle.command = update_cmd(cur_vehicle.command, t, a, elapsed_time)
            updated_vehicles.add(cur_vehicle)
            route_pos_after_cmd = route_pos_at_delta_time(cur_vehicle, collision.time-elapsed_time, elapsed_time)

            if cur_vehicle == collision.vehicle0:
                other_v = collision.vehicle1
            else:
                other_v = collision.vehicle0
            print(f"time of collision: {Fore.GREEN}{round(collision.time, 3)}s{Style.RESET_ALL}")
            world_pos_v0 = route_position_to_world_position(cur_vehicle.route, route_pos_after_cmd)
            world_pos_v1 = route_position_to_world_position(other_v.route, route_pos_at_delta_time(other_v, collision.time-elapsed_time, elapsed_time))
            # print(f"deceling duration: {Fore.GREEN}{round(deceling_duration, 3)}s{Style.RESET_ALL}")
            # print(f"decelled duration: {Fore.GREEN}{round(deceled_duration, 3)}s{Style.RESET_ALL}")
            print(f"position of {Fore.LIGHTBLUE_EX}{cur_vehicle.name}{Style.RESET_ALL}: ({Fore.YELLOW}{round(world_pos_v0[0], 2)}m, {round(world_pos_v0[1], 2)}m{Style.RESET_ALL})")
            print(f"position of {Fore.LIGHTBLUE_EX}{other_v.name}{Style.RESET_ALL}: ({Fore.YELLOW}{round(world_pos_v1[0], 2)}m, {round(world_pos_v1[1], 2)}m{Style.RESET_ALL})")

            # if route_pos_before_cmd == route_pos_after_cmd:
            #     # our modifications to cmd is making no difference. Let's attempt to switch the priority of the vehicles
            #     # reset what we've done:
            #     cur_vehicle.command = cur_vehicle_original_cmd
            #     updated_vehicles.remove(cur_vehicle)

            #     # swap priority of vehicles
            #     cur_vehicle = collision.vehicle1 if cur_vehicle == collision.vehicle0 else collision.vehicle0

            attempts_to_deter_collision += 1
            collision = get_collisions_between_two_vehicles(collision.vehicle0, collision.vehicle1, elapsed_time)

        manager.logger.info(f"{round(elapsed_time, 3)} - Command sent to {cur_vehicle.name}({cur_vehicle.id}) | T: {t}, A: {a}")
        collision = get_collision(manager, elapsed_time)


def detect_collisions(manager: Manager, vehicles: list[Vehicle], cur_time: float) -> list[Collision]:
    """Detects when a collision has actually occurred."""
    collision = False
    vehicle_pairs = combinations(manager.vehicles, 2)
    car_info = []

    for vehicle_pair in vehicle_pairs:
        # find two cars world position and calculate the distance between two cars
        wp0 = route_position_to_world_position(vehicle_pair[0].route, vehicle_pair[0].route_position)
        wp1 = route_position_to_world_position(vehicle_pair[1].route, vehicle_pair[1].route_position)
        distance = np.linalg.norm(wp1 - wp0)
        
        if distance <= CAR_COLLISION_DISTANCE:
            collision = True # just for screen pausing
            car_info = [vehicle_pair[0].name, vehicle_pair[1].name, cur_time]

            # changes the boolean to true (see vehicle.py -> vehicle.collided)
            vehicle_pair[0].collided = True
            vehicle_pair[1].collided = True

            print(f"Collision detected between {car_info[0]} and {car_info[1]} at time: {car_info[2]}")
            print(f"distance: {distance}")

    return collision