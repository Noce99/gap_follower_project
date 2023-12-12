import os
import numpy as np
import math
import sys

TRACKS = None

LIDAR_MAX_DISTANCE = None
LIDAR_NUMBER_OF_RAY = None
LIDAR_ANGLE = None
QUANTITY_OF_NOISED_RAY = None
ERROR_SIZE = None

min_LIDAR_MAX_DISTANCE = 0.0
max_LIDAR_MAX_DISTANCE = 500.0
min_LIDAR_NUMBER_OF_RAY = 0
max_LIDAR_NUMBER_OF_RAY = 500
min_LIDAR_ANGLE = 0.0
max_LIDAR_ANGLE = math.pi
min_QUANTITY_OF_NOISED_RAY = 0
max_QUANTITY_OF_NOISED_RAY = 3
min_ERROR_SIZE = 0
max_ERROR_SIZE = 3

def load_track(track_seed):
    RADIUS_VORONOY = 30
    NUMBER_OF_WAYPOINTS = 20
    if not os.path.isdir("tracks"):
        print("No \"tracks\" folder found! :-(")
        return None
    tracks = [file_name[4:-4] for file_name in os.listdir("tracks") if file_name[-4:] == ".npy" and file_name[:4] == "mat_"]
    new_tracks = []
    for i in range(len(tracks)):
        try:
            new_tracks.append(int(tracks[i]))
        except (TypeError, ValueError):
            print(f"Bad name for \"mat_{tracks[i]}.npy\", ignoring it!")
    tracks = new_tracks
    
    if track_seed in tracks:
        track_seed_to_search = track_seed
        if track_seed >= 100:
            track_seed_to_search = track_seed - 100
        if os.path.isfile(f"tracks/track_{track_seed_to_search}.npy"):
            track_points = np.load(f"tracks/track_{track_seed_to_search}.npy")
            percentages = [1. / NUMBER_OF_WAYPOINTS * i for i in range(NUMBER_OF_WAYPOINTS)]
            points = np.zeros((len(percentages), 2))
            for i, percentage in enumerate(percentages):
                points[i, :] = track_points[int(percentage*track_points.shape[0]), :]
            matrix = np.load(f"tracks/mat_{track_seed}.npy")
            RESOLUTION_VORONOY = matrix.shape[0]
            points[:, 0] = 1.5*RADIUS_VORONOY + points[:, 0] * (RESOLUTION_VORONOY-3*RADIUS_VORONOY)
            points[:, 1] = 1.5*RADIUS_VORONOY + points[:, 1] * (RESOLUTION_VORONOY-3*RADIUS_VORONOY)         
            return matrix, points
        else:
            print(f"I was able to fine 'mat_{track_seed}.npy' but NOT 'track_{track_seed_to_search}.npy'")
            return np.load(f"tracks/mat_{track_seed}.npy"), None
    else:
        print(f"Can't find your seed! [{track_seed}]")
        print(f"The founded seeds are: {tracks}")
        return None

def calculate_lidar_cost():
    return 1/5 * (LIDAR_MAX_DISTANCE/500 + float(LIDAR_NUMBER_OF_RAY)/500 + LIDAR_ANGLE/math.pi + (3.-float(QUANTITY_OF_NOISED_RAY))/3. + (3.-float(ERROR_SIZE))/3.)

def check_lidar_parameters():
    if LIDAR_MAX_DISTANCE < min_LIDAR_MAX_DISTANCE or LIDAR_MAX_DISTANCE > max_LIDAR_MAX_DISTANCE:
        print(f"LIDAR_MAX_DISTANCE must be in [{min_LIDAR_MAX_DISTANCE}; {max_LIDAR_MAX_DISTANCE}]! ({LIDAR_MAX_DISTANCE})")
        sys.exit()
    if type(LIDAR_NUMBER_OF_RAY) is not int:
        print(f"LIDAR_NUMBER_OF_RAY must be ant int! ({type(LIDAR_NUMBER_OF_RAY)})")
        sys.exit()
    if LIDAR_NUMBER_OF_RAY < min_LIDAR_NUMBER_OF_RAY or LIDAR_NUMBER_OF_RAY > max_LIDAR_NUMBER_OF_RAY:
        print(f"LIDAR_NUMBER_OF_RAY must be in [{min_LIDAR_NUMBER_OF_RAY}; {max_LIDAR_NUMBER_OF_RAY}]! ({LIDAR_NUMBER_OF_RAY})")
        sys.exit()
    if LIDAR_ANGLE < min_LIDAR_ANGLE or LIDAR_ANGLE > max_LIDAR_ANGLE:
        print(f"LIDAR_ANGLE must be in [{min_LIDAR_ANGLE}; {max_LIDAR_ANGLE}]! ({LIDAR_ANGLE})")
        print("Are u using RADIANTS?")
        sys.exit()
    if type(QUANTITY_OF_NOISED_RAY) is not int:
        print(f"QUANTITY_OF_NOISED_RAY must be ant int! ({type(QUANTITY_OF_NOISED_RAY)})")
        sys.exit()
    if QUANTITY_OF_NOISED_RAY < min_QUANTITY_OF_NOISED_RAY or QUANTITY_OF_NOISED_RAY > max_QUANTITY_OF_NOISED_RAY:
        print(f"QUANTITY_OF_NOISED_RAY must be in [{min_QUANTITY_OF_NOISED_RAY}; {max_QUANTITY_OF_NOISED_RAY}]! ({QUANTITY_OF_NOISED_RAY})")
        sys.exit()
    if type(ERROR_SIZE) is not int:
        print(f"ERROR_SIZE must be ant int! ({type(ERROR_SIZE)})")
        sys.exit()
    if ERROR_SIZE < min_ERROR_SIZE or ERROR_SIZE > max_ERROR_SIZE:
        print(f"ERROR_SIZE must be in [{min_ERROR_SIZE}; {max_ERROR_SIZE}]! ({ERROR_SIZE})")
        sys.exit()
