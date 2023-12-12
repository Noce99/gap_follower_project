import math
import utils
import numpy as np
from typing import Callable

# MODIFY FROM HERE ...
"""
You can modify the following dict for enebling or disabling wanted tracks!
"""
utils.TRACKS = {1: True, 101: False, 2: False, 102: False}
"""
You can modify the following parameters for having a better or a worst Lidar!
Remember that you need to minimze the cost!
"""
utils.LIDAR_MAX_DISTANCE = 300 # A float between 0.0 and 500.0
utils.LIDAR_NUMBER_OF_RAY = 100 # An interge between 0 and 500
utils.LIDAR_ANGLE = 3*math.pi/4 # A float (angle in !RADIANTS!) between 0.0 and PI
utils.QUANTITY_OF_NOISED_RAY = 0 # Index of [0 %, 5 %, 10 %, 20 %] so an integer between 0 and 3
utils.ERROR_SIZE = 1 # Index of [0, 10, 20, 30] so an integer between 0 and 3
# TO HERE!

utils.check_lidar_parameters()


def gap_follower(distances: list[np.float64]) -> tuple[tuple[int, int], int, Callable]:
    """
    This function gets the lidar's information and return the 

    Parameters:
        distances (list[np.float64]): List of floats that are the distances from the car asked to the
                                      lidar len(distances) = utils.LIDAR_NUMBER_OF_RAY
        

    Returns:
        choosen_gap (tuple[int, int]): The starting index in the 'distances' list that
                                       identify the gap that you have choosen and its
                                       lenght.
                                       N.B. This is used just for visualizatio!
        choosen_direction (int): The index of the 'distances' list that identify the wanted direction!
        control_the_vehicle (function): The function called for control the acelleration and steering
                                        angle of the vehicle.
    """
    choosen_gap = None
    choosen_direction = 0
    
    # MODIFY FROM HERE ...
        
        
    # TO HERE!

    def control_the_vehicle(orientation: float, steering_angle: float, speed: float, wanted_direction: float) -> (bool, bool):
        """
        This function gets the vehicle's informations (orientation, steering_angle and speed) and the wanted
        direction calculated in 'gap_follower' function and returns the basic commands to control the
        car (acelleration/decelleration and steering).
distances
        Parameters:
            orientation (float): The orientation respect to Nord of the vehicle in radiants
            steering_angle (float): The actual steering angle of the vehicle in radiants
            speed (float): The actual speed of the vehicle in m/s
            wanted_direction (float): The wanted orietation respect to Nord in radiants

        Returns:
            acelleration (bool):    
                                    True -> Increase a little bit Speed
                                    False -> Decrease a little bit Speed
                                    None -> Do not modify Speed
            steering (bochoosen_gapol):
                                    True -> Turn steering a little bit to the Left
                                    False -> Turn steering a little bit to the Right
                                    None -> Do not modify Steering    
        """
        acceleration = None
        steering = None
        
        # MODIFY FROM HERE ...
        
        # remember that you have acess to 'distances' variable
        # also from there! Thanks Python! :-)
            
            
        # TO HERE!
        
        return acceleration, steering
    
    return choosen_gap, choosen_direction, control_the_vehicle

