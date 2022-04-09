from lib import constants as c
from picar import back_wheels
from picar import front_wheels
from time import sleep
from lib import constants as c
from math import atan, degrees, pi
# import readchar
import picar


class ControlUnit(object):
    def __init__(self):
        self.__turn = False
        self.__minimal_distance = c.lidar_distances["maximum distance"]
        self.__turning_distance = 0
        self.__free_space_right = False
        self.__free_space_left = False
        self.__speed = 0
        self.__max_speed = 0
        self.__delta = 0
        self.__turning_time = 0

    # Automotive Control Systems
    def __calculate_ackermann_steering(self):
        d = c.picar["tire-tire length"]
        lidar2bw = c.picar["lidar-back wheel"]
        rpm = c.picar["angular velocity"]
        radius = c.picar["tire radius"]
        tire_circumference = 2*pi*radius
        safety_gap = c.picar["safety gap"]
        half_width = (c.picar["tire-tire width"]+c.picar["offset"])/2

        length = self.__turning_distance + lidar2bw - half_width - safety_gap -70
        delta = int(round(degrees(atan(d/length))))
        path = 2*(90/360)*pi*length
        revs = path/tire_circumference
        turning_time = round((revs * 60)/rpm, 3)
        return delta, turning_time

    def __max_speed_ctrl(self, stop_distance, min_distance, max_distance):


        if min_distance <= self.__minimal_distance <= stop_distance:
            self.__max_speed = 0
        elif stop_distance < self.__minimal_distance < max_distance / 16:
            self.__max_speed = 30
        elif max_distance / 16 <= self.__minimal_distance < max_distance / 8:
            self.__max_speed = 40
        elif max_distance / 8 <= self.__minimal_distance < max_distance / 4:
            self.__max_speed = 60
        elif max_distance / 4 <= self.__minimal_distance < max_distance / 2:
            self.__max_speed = 80
        elif max_distance / 2 <= self.__minimal_distance <= max_distance:
            self.__max_speed = 100

    def calculate_max_manual_speed(self, mirror_mode):
        min_distance = c.lidar_distances["minimum distance"]
        max_distance = c.lidar_distances["maximum distance"]

        if mirror_mode == 0:
            stop_distance = max_distance * 31/800
        elif mirror_mode == 1:
            stop_distance = 180

        self.__max_speed_ctrl(stop_distance, min_distance, max_distance)

        """    
        if mirror_mode == 0:
            stop_distance = max_distance * 31/800
            if min_distance <= self.__minimal_distance < stop_distance:
                self.__max_speed = 0
            elif stop_distance <= self.__minimal_distance < max_distance / 16:
                self.__max_speed = 30

        elif mirror_mode == 1:
            stop_distance = 180
            if min_distance <= self.__minimal_distance < stop_distance:
                self.__max_speed = 0
            elif stop_distance <= self.__minimal_distance < max_distance / 16:
                self.__max_speed = 30

        if max_distance / 16 <= self.__minimal_distance < max_distance / 8:
            self.__max_speed = 40
        elif max_distance / 8 <= self.__minimal_distance < max_distance / 4:
            self.__max_speed = 60
        elif max_distance / 4 <= self.__minimal_distance < max_distance / 2:
            self.__max_speed = 80
        elif max_distance / 2 <= self.__minimal_distance <= max_distance:
            self.__max_speed = 100
        """
    def calculate_autonomous_speed(self):
        min_distance = c.lidar_distances["minimum distance"]
        max_distance = c.lidar_distances["maximum distance"]

        if min_distance <= self.__minimal_distance < c.lidar_distances["turning min"]:
            self.__turn = False
            self.__speed = -40
        elif c.lidar_distances["turning min"] <= self.__minimal_distance < c.lidar_distances["turning max"]:
            self.__turn = True
            self.__speed = 0
        elif c.lidar_distances["turning max"] <= self.__minimal_distance < c.lidar_distances["turning max"] + 150:
            self.__turn = False
            print("Slowing down")
            self.__speed = 30
        elif c.lidar_distances["turning max"] + 150 <= self.__minimal_distance < max_distance * 3/16:
            self.__turn = False
            self.__speed = 40
        elif max_distance * 3/16 <= self.__minimal_distance < max_distance / 4:
            self.__turn = False
            self.__speed = 50
        elif max_distance / 4 <= self.__minimal_distance <= max_distance / 2:
            self.__turn = False
            self.__speed = 70
        elif max_distance / 2 <= self.__minimal_distance <= max_distance * 3/4:
            self.__turn = False
            self.__speed = 80
        elif max_distance * 3/4 <= self.__minimal_distance <= max_distance:
            self.__turn = False
            self.__speed = 100

    def calculate_autonomous_turning(self):
        self.__turning_distance = self.__minimal_distance

        # Both sides of equal distance to each other(Turn Left in this case)
        if self.__free_space_right and self.__free_space_left:
            delta, self.__turning_time = self.__calculate_ackermann_steering()
            self.__delta = -delta
            self.__speed = 40
        # Left
        elif not self.__free_space_right and self.__free_space_left:
            delta, self.__turning_time = self.__calculate_ackermann_steering()
            self.__delta = -delta
            self.__speed = 40
        # Right
        elif self.__free_space_right and not self.__free_space_left:
            delta, self.__turning_time = self.__calculate_ackermann_steering()
            self.__delta = delta
            self.__speed = 40
        # None
        elif not self.__free_space_right and not self.__free_space_left:
            self.__delta = 0
            self.__turning_time = 0
            self.__speed = 0

        if self.__minimal_distance > c.lidar_distances["turning max"]:
            self.__turn = False

    def set_minimal_distance(self, minimal_distance):
        self.__minimal_distance = minimal_distance

    def set_free_space(self, free_space_right, free_space_left):
        self.__free_space_right = free_space_right
        self.__free_space_left = free_space_left



    @property
    def speed(self):
        print("Speed: " + str(self.__speed))
        return self.__speed

    @property
    def max_speed(self):
        return self.__max_speed

    @property
    def turn(self):
        return self.__turn

    @property
    def turning_angle(self):
        return self.__delta

    @property
    def turning_time(self):
        return self.__turning_time
