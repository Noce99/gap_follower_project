import utils
import sys
import math
import random
from lidar import Lidar
import matplotlib.pyplot as plt
from matplotlib import patches
from visualizer import Visualizer
from vehicle import Vehicle
from gap_follower import gap_follower 
import utils

def load_game(track_name):
    track, waypoints = utils.load_track(track_name)
    if track is None:
        sys.exit()

    x, y = (waypoints[0][0], waypoints[0][1])
    orientation = math.atan2(waypoints[1][1] - waypoints[0][1], waypoints[1][0] - waypoints[0][0])

    a_simple_lidar = Lidar(max_distance=utils.LIDAR_MAX_DISTANCE, number_of_rays=utils.LIDAR_NUMBER_OF_RAY,
                             angle=utils.LIDAR_ANGLE, quantity_of_noised_ray=utils.QUANTITY_OF_NOISED_RAY,
                             error_size=utils.ERROR_SIZE, track=track)

    vehicle = Vehicle(starting_position=[x, y], starting_orientation=orientation)
    vis = Visualizer(vehicle, track, track_name, waypoints, a_simple_lidar.get_lidar, gap_follower)
    vis.start()

if __name__ == "__main__":
    for track_name in utils.TRACKS:
        if utils.TRACKS[track_name]:
            print(f"Starting Track {track_name}")
            load_game(track_name)
    print(f"Bye Bye!")
    

