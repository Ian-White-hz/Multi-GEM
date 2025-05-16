import numpy as np
from scipy.optimize import minimize, LinearConstraint
import copy
import pickle


class MPC:

    def __init__(self, horizon, max_iter):
        self.horizon = horizon
        self.max_iter = max_iter
        self.robot = None
        self.human = None
        self.belief = None
        self.robot_actions = None
        self.human_actions = None
        self.control_inputs_r = np.array([0.0]*self.horizon)
        self.control_inputs_h = np.array([0.0]*self.horizon)
        self.robot_const = LinearConstraint(np.eye(self.horizon), -0.1, 0.1)
        self.human_const = LinearConstraint(np.eye(self.horizon), -0.1, 0.1)
        self.dist_to_human = None
        self.speed_of_robot = None
        self.direction = None
        self.switch = None

    def update_world(self, robot, human, belief, direction, switch):
        self.robot = np.copy(robot)
        self.human = np.copy(human)
        self.belief = np.copy(belief)
        self.direction = direction
        self.switch = switch

    def optimize_h(self):
        res = minimize(self.human_cost, self.control_inputs_h, method='SLSQP', constraints=self.human_const, options={'eps': 1e-3, 'maxiter': self.max_iter})
        self.human_actions = res.x.reshape(self.horizon, 1)

    def optimize_r(self):
        res = minimize(self.robot_cost, self.control_inputs_r, method='SLSQP', constraints=self.robot_const, options={'eps': 1e-3, 'maxiter': self.max_iter})
        self.robot_actions = res.x.reshape(self.horizon, 1)

    def robot_cost(self, control_inputs_r):
        self.robot_actions = control_inputs_r.reshape(self.horizon, 1)
        self.optimize_h()
        return self.dist_to_human * 10 - self.direction * self.speed_of_robot

    def human_cost(self, control_inputs_h):
        self.human_actions = control_inputs_h.reshape(self.horizon, 1)
        robot = np.copy(self.robot)
        human = np.copy(self.human)
        self.dist_to_human = 0
        self.speed_of_robot = 0
        self.entropy = 0
        dist_to_robot = 0
        speed_of_human = 0
        for idx in range(self.horizon):
            distance_human_robot = np.linalg.norm(robot - human)
            if switch:
                # self.dist_to_human += max([0, 0.7 - distance_human_robot])**2
                self.dist_to_human += max([0, 0.0 - distance_human_robot])**2
            else:
                self.dist_to_human += max([0, 1.25 - distance_human_robot])**2
            self.speed_of_robot += self.robot_actions[idx, 0]
            dist_to_robot += max([0, 1.5 - distance_human_robot])**2
            speed_of_human += self.human_actions[idx, 0]

            # P_crash = np.exp(-0.1*distance_human_robot)
            # P_safe = 1 - P_crash
            # self.belief[0] *= P_crash
            # self.belief[1] *= P_safe
            # self.belief /= np.sum(self.belief)
            # if self.belief[0] < 0.1:
            #     self.belief = np.array([0.1, 0.9])
            # if self.belief[0] > 0.9:
            #     self.belief = np.array([0.9, 0.1])
            #
            # self.entropy += -self.belief[0] * np.log2(self.belief[0]) -self.belief[1] * np.log2(self.belief[1])

            robot[1] += self.robot_actions[idx]
            human[0] += self.human_actions[idx]

        return dist_to_robot * 10 - self.direction * speed_of_human



mpc = MPC(horizon=5, max_iter=10)

while True:

    try:
        parameters = np.array(pickle.load(open('optimization_parameters.pkl', 'rb')))

        robot = parameters[0:2]
        human = parameters[2:4]
        belief = parameters[4:6]
        direction = parameters[6]
        switch = parameters[7]

        mpc.update_world(robot, human, belief, direction, switch)
        mpc.optimize_r()

        pickle.dump(mpc.robot_actions, open('actions.pkl', 'wb'))
        print("robot", mpc.robot_actions[0], "human", mpc.human_actions[0])
    except EOFError:
       True
