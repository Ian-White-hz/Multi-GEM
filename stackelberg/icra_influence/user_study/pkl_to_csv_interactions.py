import numpy as np
from datetime import datetime
import csv
import pickle
# import matplotlib.pyplot as plt


# SAVE THE DATA
# [0] = timestamp; [1] = robot pos x; [2] = robot pos y; [3] = human pos x;
# [4] = human pos y; [5] = mpc action; [6] = switch; [7] = direction


# takes in name of data file
# returns (1) the number of total interactions (this should be 30)
# and (2) the total time the robot was yielding to the human at each interaction
def process_data(name):
    data = pickle.load( open( name, "rb" ) )
    data = np.array(data)
    interactions = 0
    amount_yielded = []
    yielded_this_interaction = 0.0
    for index in range(1, len(data)):
        item = data[index, :]
        prev_item = data[index-1, :]
        direction = item[7]
        prev_direction = prev_item[7]
        action = item[5]
        delta_time = item[0] - prev_item[0]
        if abs(direction - prev_direction) > 0.5 or index == len(data)-1:
            if interactions > 0:
                if yielded_this_interaction < 0.1:
                    yielded_this_interaction = 0.0
                amount_yielded.append(yielded_this_interaction)
            if index < len(data)-1:
                interactions += 1
                yielded_this_interaction = 0.0
        if interactions > 0:
            if action < 0 and direction > 0:
                yielded_this_interaction += delta_time
            if action > 0 and direction < 0:
                yielded_this_interaction += delta_time
    return interactions, amount_yielded

# this list should include all the users you want to analyze
users = [2, 3, 4, 6, 8, 9, 10, 11, 12, 13, 14]
# for file writing 
ctime = datetime.now()
fnameA = "algA " + str(ctime) + ".csv"
fnameB = "algB " + str(ctime) + ".csv"

# plots the time yielded vs. interactions averaged across users
amount_yielded_0 = np.array([0.0] * 40)
amount_yielded_1 = np.array([0.0] * 40)

with open(fnameA, 'a', newline='') as fileA, open(fnameB, 'a', newline='') as fileB:
    writerA = csv.writer(fileA, delimiter=',')
    writerB = csv.writer(fileB, delimiter=',')
    for user in users:
        interactions_0, yielded_0 = process_data("data/user" + str(user) + "/algo0")
        interactions_1, yielded_1 = process_data("data/user" + str(user) + "/algo1")
        yielded_0 = np.array(yielded_0)
        yielded_1 = np.array(yielded_1)

        print("writing user {} alg A".format(user))
        print("writing user {} alg B".format(user))
        writerA.writerow(yielded_0)
        writerB.writerow(yielded_1)

    fileA.close()
    fileB.close()
    # amount_yielded_0[0:interactions_0] += yielded_0
    # amount_yielded_1[0:interactions_1] += yielded_1
# amount_yielded_0 /= (1.0 * len(users))
# amount_yielded_1 /= (1.0 * len(users))
# plt.bar(range(25), amount_yielded_0[0:25])
# plt.bar(range(25), amount_yielded_1[0:25])
# plt.show()
