import numpy as np
from classes.vehicle import Vehicle, update_cmd
from classes.route import Route, route_position_to_world_position
from itertools import combinations
from scipy.optimize import minimize_scalar
from random import randint

CAR_COLLISION_DISTANCE = 3 # meters
MINIMUM_CRUISING_SPEED = 0
DESIRED_CRUISING_SPEED = 20
MAX_ACCELERATION = 3.5

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

def get_collision(manager: Manager, cur_time: float) -> Collision | None:
    """Return list of Collisions between Vehicles in manager's radius."""
    vehicle_pairs = combinations(manager.vehicles, 2)
    
    for vehicle_pair in vehicle_pairs:
        vehicle_out_of_bounds_time = int(min(time_until_end_of_route(vehicle_pair[0], cur_time), time_until_end_of_route(vehicle_pair[1], cur_time)))
        def distance_objective(t):
            wp0 = route_position_to_world_position(vehicle_pair[0].route, route_pos_at_delta_time(vehicle_pair[0], t, cur_time))
            wp1 = route_position_to_world_position(vehicle_pair[1].route, route_pos_at_delta_time(vehicle_pair[1], t, cur_time))
            if wp0 is None or wp1 is None:
                return np.inf
            return np.linalg.norm(wp1-wp0)
        
        result = minimize_scalar(distance_objective, bounds=(0, vehicle_out_of_bounds_time), method='bounded')
        if result.success:
            time_of_collision = result.x + cur_time
            if distance_objective(result.x) <= CAR_COLLISION_DISTANCE:
                return Collision(vehicle_pair[0], vehicle_pair[1], time_of_collision)
    return None

def get_collisions(manager: Manager, cur_time: float) -> list[Collision]:
    """Return list of Collisions between Vehicles in manager's radius."""
    collisions = []
    vehicle_pairs = combinations(manager.vehicles, 2)
    
    for vehicle_pair in vehicle_pairs:
        vehicle_out_of_bounds_time = int(min(time_until_end_of_route(vehicle_pair[0], cur_time), time_until_end_of_route(vehicle_pair[1], cur_time)))
        def distance_objective(t):
            wp0 = route_position_to_world_position(vehicle_pair[0].route, route_pos_at_delta_time(vehicle_pair[0], t, cur_time))
            wp1 = route_position_to_world_position(vehicle_pair[1].route, route_pos_at_delta_time(vehicle_pair[1], t, cur_time))
            if wp0 is None or wp1 is None:
                return np.inf
            return np.linalg.norm(wp1-wp0)
        
        result = minimize_scalar(distance_objective, bounds=(0, vehicle_out_of_bounds_time), method='bounded')
        if result.success:
            time_of_collision = result.x + cur_time
            if distance_objective(result.x) <= CAR_COLLISION_DISTANCE:
                collisions.append(Collision(vehicle_pair[0], vehicle_pair[1], time_of_collision))
    return collisions

def get_collisions_between_two_vehicles(vehicle0: Vehicle, vehicle1: Vehicle, cur_time: float) -> Collision | None:
    vehicle_out_of_bounds_time = int(min(time_until_end_of_route(vehicle0, cur_time), time_until_end_of_route(vehicle1, cur_time)))
    def distance_objective(t):
            wp0 = route_position_to_world_position(vehicle0.route, route_pos_at_delta_time(vehicle0, t, cur_time))
            wp1 = route_position_to_world_position(vehicle1.route, route_pos_at_delta_time(vehicle1, t, cur_time))
            if wp0 is None or wp1 is None:
                return np.inf
            return np.linalg.norm(wp1-wp0)
        
    result = minimize_scalar(distance_objective, bounds=(0, vehicle_out_of_bounds_time), method='bounded')
    if result.success:
        time_of_collision = result.x + cur_time
        if distance_objective(result.x) <= CAR_COLLISION_DISTANCE:
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
    manager.vehicles.sort(key=lambda v: v.route_position, reverse=True)
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
        print(f"collision between {collision.vehicle0.name} and {collision.vehicle1.name} at {collision.time}")

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

            if route_pos_before_cmd == route_pos_after_cmd:
                # our modifications to cmd is making no difference. Let's attempt to switch the priority of the vehicles
                # reset what we've done:
                cur_vehicle.command = cur_vehicle_original_cmd
                updated_vehicles.remove(cur_vehicle)

                # swap priority of vehicles
                cur_vehicle = collision.vehicle1 if cur_vehicle == collision.vehicle0 else collision.vehicle0

            attempts_to_deter_collision += 1
            collision = get_collisions_between_two_vehicles(collision.vehicle0, collision.vehicle1, elapsed_time)

        collision = get_collision(manager, elapsed_time)

def _compute_command(elapsed_time: float) -> tuple[np.array, np.array]:
    """Return np.array of acceleration and time values."""
    # t = [elapsed_time, elapsed_time + 2.5] # this will make cars crash for presets/collision_by_command.json
    # a = [0, 6]
    t = [elapsed_time, elapsed_time + randint(1, 3), elapsed_time + randint(3, 5)]
    a = [randint(1, 3), randint(-3, 3), 3]
    return np.array(t), np.array(a)