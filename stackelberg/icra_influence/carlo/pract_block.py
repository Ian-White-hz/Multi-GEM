import numpy as np
from world import World
from agents import Car, RectangleBuilding, Pedestrian, Painting
from interactive_controllers import KeyboardController, SteeringWheelController
from geometry import Point
import time


dt = 1.0 # time steps in terms of seconds. In other words, 1/dt is the FPS.
w = World(dt, width = 120, height = 120, ppm = 5) # The world is 120 meters by 120 meters. ppm is the pixels per meter.

# Road
w.add(Painting(Point(60, 60), Point(90, 90)))

# Block square
w.add(Painting(Point(60, 60), Point(40, 40), 'gray80'))

# Lane marking
for idx in range(30, 100, 10):
    w.add(Painting(Point(93, idx), Point(0.7, 3), 'white'))
for idx in range(30, 100, 10):
    w.add(Painting(Point(27, idx), Point(0.7, 3), 'white'))
for idx in range(30, 100, 10):
    w.add(Painting(Point(idx, 27), Point(3, 0.7), 'white'))
for idx in range(30, 100, 10):
    w.add(Painting(Point(idx, 93), Point(3, 0.7), 'white'))


# A Car object is a dynamic object -- it can move.
c2 = Car(Point(100,30), np.pi/2, 'blue')
c2.velocity = Point(0, 0)
c2.max_speed = +2.0
c2.min_speed = -2.0
w.add(c2)

w.render() # This visualizes the world we just constructed.
time.sleep(1)

controller = SteeringWheelController(w)
for k in range(2000):
    start_time = time.time()
    c2.set_control(controller.steering, controller.throttle)
    w.tick()
    w.render()
    time.sleep(0.15)
w.close()
