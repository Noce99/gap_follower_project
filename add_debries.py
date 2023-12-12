import utils
import random
import matplotlib.pyplot as plt
import numpy as np

NUM_OF_DEBRIS = 10
RADIUS = 5

def add_debris(track_name):
    track, _ = utils.load_track(track_name)
    for i in range(NUM_OF_DEBRIS):
        while True:
            x = random.randrange(track.shape[0])
            y = random.randrange(track.shape[1])
            if track[x, y] == 1:
                track[x-RADIUS:x+RADIUS, y-RADIUS:y+RADIUS] = 0
                break
    return track

if __name__ == "__main__":
    tracks = [1, 2]
    for track_name in tracks:
        track = add_debris(track_name)
        np.save(f"tracks/mat_{100+track_name}.npy", track)

