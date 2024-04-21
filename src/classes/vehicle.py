import numpy as np

class Vehicle:
    position: np.ndarray     = np.array([0,0]) # numpy array[2] describing world coordinates
    velocity: np.ndarray     = np.array([0,0]) # numpy array[2] describing euclidean velocity vector
    acceleration: np.ndarray = np.array([0,0]) # numpy array[2] describing euclidean acceleration vector
    direction: np.ndarray    = np.array([1,0]) # numpy array[2] describing unit directional vector
    width: float             = 1.95            # float representing width of car in meters. orthogonal to direction
    length: float            = 4.90            # float representing length of car in meters. parallel to direction

    def __init__(self, position: np.ndarray, velocity: np.ndarray, acceleration: np.ndarray, direction: np.ndarray, width: float, height: float):
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.dir = direction
        self.width = width
        self.height = height