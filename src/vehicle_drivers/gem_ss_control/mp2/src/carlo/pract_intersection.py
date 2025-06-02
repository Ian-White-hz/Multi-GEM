import numpy as np
from world import World
from agents import Car, RectangleBuilding, Painting
from interactive_controllers import KeyboardController, SteeringWheelController
from geometry import Point
import time
from utils import Flag

# initilize GUI
flag = Flag()

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
c2 = Car(Point(65,37), np.pi/2, 'blue')
c2.velocity = Point(0, 0)
c2.max_speed = +0.75
c2.min_speed = 0.0
w.add(c2)

w.render() # This visualizes the world we just constructed.

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

controller = SteeringWheelController(w)
for k in range(75):
    start_time = time.time()
    c2.set_control(controller.steering, controller.throttle)
    w.tick()
    w.render()
    time.sleep(0.15)
w.close()
