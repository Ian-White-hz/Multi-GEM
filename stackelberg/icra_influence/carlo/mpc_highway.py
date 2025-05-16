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
        self.control_inputs_r = np.array([0.0]*self.horizon*2)
        self.control_inputs_h = np.array([0.0]*self.horizon*2)
        self.robot_const = LinearConstraint(np.eye(self.horizon*2), -0.5, 0.5)
        self.human_const = LinearConstraint(np.eye(self.horizon*2), -0.5, 0.5)
        self.dist_to_human = None
        self.block_human = None
        self.heading = None

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
        self.robot_actions = control_inputs_r.reshape(self.horizon, 2)
        self.optimize_h()
        return self.dist_to_human * 100 + self.block_human + self.heading * 25

    def human_cost(self, control_inputs_h):
        self.human_actions = control_inputs_h.reshape(self.horizon, 2)
        robot_car = self.agents[0]
        human_car = self.agents[1]
        self.dist_to_human = 0
        self.block_human = 0
        self.heading = 0
        dist_to_robot = 0
        speed_of_human = 0
        for idx in range(self.horizon):
            self.dist_to_human += max([0, 10 - (robot_car.center - human_car.center).norm()])
            self.block_human += (human_car.center.x - robot_car.center.x)**2
            self.heading += (np.pi/2 - robot_car.heading)**2
            dist_to_robot += max([0, 30 - (robot_car.center - human_car.center).norm()])
            speed_of_human += human_car.velocity.y**2
            robot_car.set_control(self.robot_actions[idx, 0], self.robot_actions[idx, 1])
            human_car.set_control(self.human_actions[idx, 0], self.human_actions[idx, 1])
            robot_car.tick(self.world.dt)
            human_car.tick(self.world.dt)
        self.reset()
        return dist_to_robot - speed_of_human
