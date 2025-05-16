import numpy as np
from scipy.optimize import minimize, LinearConstraint
import copy


class MPC:

    def __init__(self, horizon, max_iter):
        self.horizon = horizon
        self.max_iter = max_iter
        self.world = None
        self.agents = None
        self.robot_actions = None
        self.human_actions = None
        self.control_inputs_r = np.array([0.0]*self.horizon)
        self.control_inputs_h = np.array([0.0]*self.horizon)
        self.robot_const = LinearConstraint(np.eye(self.horizon), -0.5, 0.5)
        self.human_const = LinearConstraint(np.eye(self.horizon), -0.5, 0.5)
        self.dist_to_human = None
        self.robot_to_target = None

    def update_world(self, world):
        self.world = world
        self.reset()

    def reset(self):
        self.agents = copy.deepcopy(self.world.dynamic_agents)

    def optimize_h(self):
        res = minimize(self.human_cost, self.control_inputs_h, method='SLSQP', constraints=self.human_const, options={'eps': 1e-3, 'maxiter': self.max_iter})

    def optimize_r(self):
        res = minimize(self.robot_cost, self.control_inputs_r, method='SLSQP', constraints=self.robot_const, options={'eps': 1e-3, 'maxiter': self.max_iter})

    def robot_cost(self, control_inputs_r):
        self.robot_actions = control_inputs_r
        self.optimize_h()
        return self.dist_to_human * 1000 + self.robot_to_target

    def human_cost(self, control_inputs_h):
        self.human_actions = control_inputs_h
        robot_car = self.agents[0]
        human_car = self.agents[1]
        self.dist_to_human = 0
        self.robot_to_target = 0
        dist_to_robot = 0
        human_to_target = 0
        for idx in range(self.horizon):
            self.dist_to_human += max([0, 10 - (robot_car.center - human_car.center).norm()])
            self.robot_to_target += (robot_car.x - 70)**2 * 2.5 + (robot_car.y - 120)**2
            dist_to_robot += max([0, 30 - (robot_car.center - human_car.center).norm()])
            human_to_target += (human_car.x - 70)**2 + (human_car.y - 120)**2
            robot_car.set_control(-0.02, self.robot_actions[idx])
            human_car.set_control(0.0, self.human_actions[idx])
            robot_car.tick(self.world.dt)
            human_car.tick(self.world.dt)
        self.reset()
        return dist_to_robot + human_to_target
