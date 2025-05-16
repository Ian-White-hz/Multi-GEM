from djitellopy import Tello
import pygame
import numpy as np
import time
import pickle
import argparse


# User ID
user_id = 1 # Manually change 2 to 3, etc. for each user

#options are 0 for baseline and 1 for ours
parser = argparse.ArgumentParser()
parser.add_argument('--algo', type=int, default=0)
args = parser.parse_args()


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


class JoystickControl(object):

    def __init__(self):
        pygame.init()
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.deadband = 0.1
        self.timeband = 0.5
        self.lastpress = time.time()

    def getInput(self):
        pygame.event.get()
        curr_time = time.time()
        A_pressed = self.gamepad.get_button(0) and (curr_time - self.lastpress > self.timeband)
        B_pressed = self.gamepad.get_button(1) and (curr_time - self.lastpress > self.timeband)
        X_pressed = self.gamepad.get_button(2) and (curr_time - self.lastpress > self.timeband)
        START_pressed = self.gamepad.get_button(7) and (curr_time - self.lastpress > self.timeband)
        if A_pressed or START_pressed or B_pressed or X_pressed:
            self.lastpress = curr_time
        return A_pressed, B_pressed, X_pressed, START_pressed


dataset = [] # storing the experiment data for each user
tello = Tello()
tello.connect()
joystick = JoystickControl()
tello.send_rc_control(0, 0, 0, 0)

takeoff = False
start_run = False
scale_actions = 60

robot = None
human = None
belief = np.array([0.5, 0.5])
actions = None
direction = 1.0
switch = False
on_left = False
on_right = True


while True:
    A_pressed, B_pressed, X_pressed, START_pressed = joystick.getInput()

    if A_pressed and not takeoff:
        takeoff = True
        tello.send_rc_control(0, 0, 0, 0)
        tello.takeoff()
    if B_pressed and takeoff:
        takeoff = False
        tello.send_rc_control(0, 0, 0, 0)
        tello.land()
    if X_pressed and not takeoff:
        tello.send_rc_control(0, 0, 0, 0)
        tello.end()
        break
    if START_pressed and takeoff:
        start_run = not start_run


    # get robot position
    try:
        robot = np.array(pickle.load(open('drone_position.pkl', 'rb')))
    except EOFError:
       True

    # get human position
    try:
        human = np.array(pickle.load(open('vive_position.pkl', 'rb')))
    except EOFError:
        True

    # get robot actions
    parameters = np.array([robot[0], robot[1], human[0], human[1], belief[0], belief[1], direction, switch])
    pickle.dump(parameters, open('optimization_parameters.pkl', 'wb'))
    try:
        actions = np.array(pickle.load(open('actions.pkl', 'rb')))
    except EOFError:
        True

    belief = belief_update(belief, human, robot)
    print("robot: ", np.round(robot, 2), "human: ", np.round(human, 2), "switch: ", switch)

    # human position at each timestep, robot position at each timestep, switch
    dataset = [human, robot, switch]

    # if start_run:
    if start_run:

        # assigns the robot's x movement speed
        if robot[0] > 0.95:
            dx = -10
        elif robot[0] < 0.75:
            dx = +10
        else:
            dx = 0

        # assigns the robot's y movement speed
        dy = actions[0]
        dy = int(dy * scale_actions * 7)
        # if robot[1] > 2.4 and dy > 0:
        #     dy = 0
        # if robot[1] > 2.7 and dy > 0:
        #     dy = -30
        # if robot[1] < 0.4 and dy < 0:
        #     dy = 0
        # if robot[1] < 0.1 and dy < 0:
        #     dy = +30
        if robot[1] > 2.1:
            direction = -1.0
            on_left = True
            if on_left and on_right and args.algo == 1:
                if np.random.rand() > 0.5:
                    switch = True
                else:
                    switch = False
            on_right = False
        elif robot[1] < 0.8:
            direction = 1.0
            on_right = True
            if on_left and on_right and args.algo == 1:
                if np.random.rand() > 0.5:
                    switch = True
                else:
                    switch = False
            on_left = False


    else:

        dx = 0
        dy = 0

    # send the robot move command
    tello.send_rc_control(dx, dy, 0, 0)

    if args.algo == 0:
        pickle.dump(dataset, open("runs/u" + str(user_id) + "_" + "alg0" + time.strftime("%a; %b %d %Y; %H.%M.%S") + ".pkl", "wb")) 
    else:
        pickle.dump(dataset, open("runs/u" + str(user_id) + "_" + "alg1" + time.strftime("%a; %b %d %Y; %H.%M.%S") + ".pkl", "wb")) 
    
