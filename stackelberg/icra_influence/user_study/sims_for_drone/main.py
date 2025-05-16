import numpy as np
import matplotlib.pyplot as plt
from mpc_quad import MPC
import copy
import pickle
import argparse



def belief_update(belief, human, robot):
    distance = np.linalg.norm(human - robot)
    P_crash = np.exp(-0.5*distance)
    P_safe = 1 - P_crash
    belief[0] *= P_crash
    belief[1] *= P_safe
    belief /= np.sum(belief)
    if belief[0] < 0.1:
        belief = np.array([0.1, 0.9])
    if belief[0] > 0.9:
        belief = np.array([0.9, 0.1])
    return belief



timesteps = 10
robot = np.array([-1.0, 0.0])
human = np.array([0.0, -1.0])
belief = np.array([0.5, 0.5])

mpc = MPC(horizon=5, max_iter=100)

xi_r = np.zeros((timesteps, 2))
xi_h = np.zeros((timesteps, 2))

for k in range(timesteps):

    xi_r[k, :] = np.copy(robot)
    xi_h[k, :] = np.copy(human)
    xi_r[k, 1] = k * 0.02
    xi_h[k, 0] = k * 0.02

    belief = belief_update(belief, human, robot)

    mpc.update_world(robot, human, belief, k)
    mpc.optimize_r()

    ur = mpc.robot_actions[0]
    uh = mpc.human_actions[0]

    # uh = 0.5

    print("robot: ", np.round(robot, 2), "human: ", np.round(human, 2), "belief: ", belief)

    robot[0] += ur
    human[1] += uh

plt.plot(xi_r[:, 0], xi_r[:, 1], 'rx-')
plt.plot(xi_h[:, 0], xi_h[:, 1], 'bo-')
plt.show()
