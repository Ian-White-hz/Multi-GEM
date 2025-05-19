from entities import RectangleEntity, CircleEntity, RingEntity
from geometry import Point
from GNSS_msg import Gem_e4_gnss_msg
from GNSS_msg import Gem_e2_gnss_msg

# For colors, we use tkinter colors. See http://www.science.smith.edu/dftwiki/index.php/Color_Charts_for_TKinter
try:
    import rospy
except ImportError:
    print("ROS not found. Please install ROS to use this module.")
    pass
class Car(RectangleEntity):
    def __init__(self, center: Point, heading: float, color: str = 'red'):
        size = Point(8., 4.)
        movable = True
        friction = 0.06
        super(Car, self).__init__(center, heading, size, movable, friction)
        self.color = color
        self.collidable = True
        try:
            if color == 'orange':
                self.Gem_gnss_msg = Gem_e4_gnss_msg()
            else:
                self.Gem_gnss_msg = Gem_e2_gnss_msg()
        except rospy.exceptions.ROSInitException:
            print("ROS not initialized. Please initialize ROS before using this class.")
            self.Gem_gnss_msg = None

    def update(self, dt: float):    


        curr_x, curr_y, curr_yaw, curr_speed , new_acceleration, new_angular_velocity= self.Gem_gnss_msg.get_gem_state()

        self.center = Point(curr_x, curr_y)

        self.heading = self.Gem_gnss_msg.heading_to_yaw(curr_yaw)

        self.velocity = curr_speed
        self.acceleration = new_acceleration
        self.angular_velocity = new_angular_velocity
        self.buildGeometry()
class Pedestrian(CircleEntity):
    def __init__(self, center: Point, heading: float, color: str = 'LightSalmon3'): # after careful consideration, I decided my color is the same as a salmon, so here we go.
        radius = 0.5
        movable = True
        friction = 0.2
        super(Pedestrian, self).__init__(center, heading, radius, movable, friction)
        self.color = color
        self.collidable = True

class RectangleBuilding(RectangleEntity):
    def __init__(self, center: Point, size: Point, color: str = 'gray26'):
        heading = 0.
        movable = False
        friction = 0.
        super(RectangleBuilding, self).__init__(center, heading, size, movable, friction)
        self.color = color
        self.collidable = True

class CircleBuilding(CircleEntity):
    def __init__(self, center: Point, radius: float, color: str = 'gray26'):
        heading = 0.
        movable = False
        friction = 0.
        super(CircleBuilding, self).__init__(center, heading, radius, movable, friction)
        self.color = color
        self.collidable = True

class RingBuilding(RingEntity):
    def __init__(self, center: Point, inner_radius: float, outer_radius: float, color: str = 'gray26'):
        heading = 0.
        movable = False
        friction = 0.
        super(RingBuilding, self).__init__(center, heading, inner_radius, outer_radius, movable, friction)
        self.color = color
        self.collidable = True

class Painting(RectangleEntity):
    def __init__(self, center: Point, size: Point, color: str = 'gray26', heading: float = 0.):
        movable = False
        friction = 0.
        super(Painting, self).__init__(center, heading, size, movable, friction)
        self.color = color
        self.collidable = False
