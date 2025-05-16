import numpy as np
from world import World
from agents import Car, RectangleBuilding, Pedestrian, Painting
from interactive_controllers import KeyboardController, SteeringWheelController
from geometry import Point
import time
import copy
from mpc_intersection import MPC
import pickle
from utils import I_score, Flag

# User ID
user_id = 12 # Manually change 2 to 3, etc. for each user

# initilize GUI
flag = Flag()
score = I_score()

total_score = 0 # Cumulative score across all scenes and iterations

dt = 1.0 # time steps in terms of seconds. In other words, 1/dt is the FPS.
w = World(dt, width = 120, height = 120, ppm = 5) # The world is 120 meters by 120 meters. ppm is the pixels per meter.

# Buildings
# Bottom left
w.add(Painting(Point(25, 25), Point(51, 51), 'gray80'))
w.add(RectangleBuilding(Point(24, 24), Point(49, 49)))

# Bottom Right
w.add(Painting(Point(95, 25), Point(51, 51), 'gray80'))
w.add(RectangleBuilding(Point(96, 24), Point(49, 49)))

# Top left
w.add(Painting(Point(25, 95), Point(51, 51), 'gray80'))
w.add(RectangleBuilding(Point(24, 96), Point(49, 49)))

# Top Right
w.add(Painting(Point(95, 95), Point(51, 51), 'gray80'))
w.add(RectangleBuilding(Point(96, 96), Point(49, 49)))

# Zebra Crossings
# Left
w.add(Painting(Point(45.5, 66), Point(8, 2.8), 'white'))
w.add(Painting(Point(45.5, 62), Point(8, 2.8), 'white'))
w.add(Painting(Point(45.5, 58), Point(8, 2.8), 'white'))
w.add(Painting(Point(45.5, 54), Point(8, 2.8), 'white'))

# Right
w.add(Painting(Point(74.5, 66), Point(8, 2.8), 'white'))
w.add(Painting(Point(74.5, 62), Point(8, 2.8), 'white'))
w.add(Painting(Point(74.5, 58), Point(8, 2.8), 'white'))
w.add(Painting(Point(74.5, 54), Point(8, 2.8), 'white'))

# Top
w.add(Painting(Point(54, 74.5), Point(2.8, 8), 'white'))
w.add(Painting(Point(58, 74.5), Point(2.8, 8), 'white'))
w.add(Painting(Point(62, 74.5), Point(2.8, 8), 'white'))
w.add(Painting(Point(66, 74.5), Point(2.8, 8), 'white'))

# Bottom
w.add(Painting(Point(54, 45.5), Point(2.8, 8), 'white'))
w.add(Painting(Point(58, 45.5), Point(2.8, 8), 'white'))
w.add(Painting(Point(62, 45.5), Point(2.8, 8), 'white'))
w.add(Painting(Point(66, 45.5), Point(2.8, 8), 'white'))


# A Car object is a dynamic object -- it can move.
c1 = Car(Point(37,55), 0.0, 'orange')
c1.velocity = Point(2.0, 0)
c1.max_speed = +0.75
c1.min_speed = -0.75
w.add(c1)
c2 = Car(Point(65,37), np.pi/2, 'blue')
c2.velocity = Point(0, 0)
c2.max_speed = +0.75
c2.min_speed = 0.0
w.add(c2)

w.render() # This visualizes the world we just constructed.
mpc = MPC(horizon=4, max_iter=3) # Create the controller for our robt

# Mechanics of the flag screen to display and close
flag.flag_text.config (bg = '#B81D13', text = flag.flag[0])
flag.root.update()
time.sleep(0.8)
flag.flag_text.config (bg = '#EFB700', text = flag.flag[1])
flag.root.update()
time.sleep(0.8)
flag.flag_text.config (bg = '#008450', text = flag.flag[2])
flag.root.update()
time.sleep(0.8)
flag.root.destroy()

# open the score file. First time is handled with try/except
try:
    total_score = pickle.load(open("runs/total_score_u" + str(user_id) + ".pkl", "rb"))
except:
    pass

dataset = []
controller = SteeringWheelController(w)
intersec_score = 0 # running score
for k in range(75):
    start_time = time.time()
    human_car_y0 = c2.y
    mpc.update_world(w)
    mpc.optimize_r()
    UR = mpc.robot_actions
    UH = mpc.human_actions
    c1.set_control(0, UR[0])
    c2.set_control(controller.steering, controller.throttle)
    state = [k, c1.x, c1.y, c1.heading, c2.x, c2.y, c2.heading, c1.collidesWith(c2), intersec_score, total_score]
    action = [0, UR[0], controller.steering, controller.throttle]
    dataset.append(state + action)
    w.tick()
    human_car_y1 = c2.y
    w.render()
    # print("step number: ", k, "step time: ", time.time() - start_time)
    delta_human_car_y = abs(human_car_y1 - human_car_y0)
    # Giving positive scores for staying on the road. Negative for off road
    if 52 <= c2.x <= 68:
        intersec_score += delta_human_car_y
        score.textbox3.delete(0, 'end')
        score.textbox3.insert(0, str(int(intersec_score)))
        score.root.update()
    else:
        intersec_score -= 10
        score.textbox3.delete(0, 'end')
        score.textbox3.insert(0, str(int(intersec_score)))
        score.root.update()
    if c1.collidesWith(c2):
        print('Collision!')
        intersec_score -= 100
        score.textbox3.delete(0, 'end')
        score.textbox3.insert(0, str(int(intersec_score)))
        score.root.update()
    try: # Handling the first time when a pickle does not exist
        score.textbox4.delete(0, 'end')
        score.textbox4.insert(0, str(int(total_score)))
        score.root.update()
    except:
        pass
        
w.close()
pickle.dump(dataset, open("runs/intersection_u" + str(user_id) + "_" + time.strftime("%a; %b %d %Y; %H.%M.%S") + ".pkl", "wb")) # Manually change u1 to u2, etc. for each user
try:
    pickle.dump(int(total_score + intersec_score), open("runs/total_score_u" + str(user_id) + ".pkl", "wb")) # Manually change u1 to u2, u3, etc.
except:
    pickle.dump(int(intersec_score), open("runs/intersection_score_u" + str(user_id) + ".pkl", "wb"))
