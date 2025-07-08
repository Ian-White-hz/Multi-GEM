from .world import World
from .agents import Car, RectangleBuilding, Painting
from .geometry import Point
from .mpc_highway import MPC
import numpy as np


class HighwayWorld:
    def __init__(self):
        self.dt = 0.1
        self.w = World(self.dt, width=120, height=120, ppm=5)
        self.mpc = MPC(horizon=2, max_iter=2)
        self.cars = []
        self.car_actions = []
        self.human_actions = []
        self.setup_world()

    def setup_world(self):
        # Add sidewalks, buildings, lane markings, etc.
        self.w.add(Painting(Point(25, 60), Point(50, 150), 'gray80'))
        self.w.add(Painting(Point(95, 60), Point(50, 150), 'gray80'))

        # Then we add some rectangle buildings on top
        self.w.add(RectangleBuilding(Point(20, 60), Point(48, 150)))
        self.w.add(RectangleBuilding(Point(100, 60), Point(48, 150)))
        # # Let's also add some lane markings
        for idx in range(0, 121, 10):
            self.w.add(Painting(Point(60, idx), Point(0.5, 3), 'white'))

    def init_car(self):
        # 1. Create ego car
        c1 = Car(Point(0, 0), 0, 'orange')
        c1.velocity = Point(0, 0)
        c1.max_speed = 2
        c1.min_speed = 0
   
        self.w.add(c1)
        self.cars.append(c1)
        # 2. Create other car(s)
        c2 = Car(Point(-4, 4), 0, 'blue')
        c2.velocity = Point(0, 0)
        c2.max_speed = 2
        c2.min_speed = 0
        self.w.add(c2)
        self.cars.append(c2)
               
    def mpc_highway_policy(self, current_state):

        # 2. Run MPC

        self.mpc.update_world(self.w)
        self.mpc.optimize_r()
        
        self.car_actions = self.mpc.robot_actions
        self.human_actions = self.mpc.human_actions
        # 3. Set control for each car
        for i, car in enumerate(self.cars):
            # first car is ego, second car is the other car
            if car.color == 'orange':

                car.set_control(self.car_actions[0, 0], self.car_actions[0, 1])
                # print("car actions", self.car_actions)
            else:
                car.set_control(self.human_actions[0,0], self.human_actions[0,1])
        
            
        self.w.update(current_state)

        return self.car_actions[0, 0], self.car_actions[0, 1]




