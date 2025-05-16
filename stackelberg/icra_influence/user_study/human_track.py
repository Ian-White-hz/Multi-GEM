import numpy as np
import pickle
import TrackerState
import time

#Create vive tracker global coordinate frame
g0_r = np.array([[-1.9635], [-3.55037], [-1.5570]])
gxa_r = np.array([[-0.44308], [-1.6915], [-1.5448]])
tracker = TrackerState.TrackerState()
tracker.set_global_coords(g0_r, gxa_r)

counter = 0
start_time = time.time()

while(True):

    if time.time() - start_time >= 1.0:
        print("samples / second: ", counter)
        start_time = time.time()
        counter = 0

    # get vive tracker position
    vive_pos_raw, valid = tracker.get_tracker_pos()
    # update vive tracker pickle file
    if valid:
        vive_pos = np.array([vive_pos_raw[0][0], vive_pos_raw[1][0]])
        pickle.dump(vive_pos, open('vive_position.pkl', 'wb'))
        counter += 1
