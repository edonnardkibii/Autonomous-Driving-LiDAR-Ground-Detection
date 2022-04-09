from picar import front_wheels, back_wheels
from time import sleep
from lib import constants as c
import picar


class Motor(object):
    def __init__(self):
        self.__bw = back_wheels.Back_Wheels(debug=False)
        self.__fw = front_wheels.Front_Wheels(debug=False)
        self.__turning_distance = 0
        self.__step = 20
        self.__turning_angle = 0
        self.__turning_time = 0
        self.__motor_speed = 0
        self.__max_motor_speed = 0

        self.__can_turn_right = False
        self.__can_turn_left = False

    def __drive(self, motor_speed):
        if motor_speed > 0:
            self.__bw.forward()
            self.__bw.speed = motor_speed
        elif motor_speed < 0:
            self.__bw.backward()
            self.__bw.speed = -motor_speed
        elif motor_speed == 0:
            self.__bw.stop()
            self.__bw.speed = motor_speed

    def __correct_servo_turn(self, angle):
        # Left Side: 0° - -90°    Right Side: 0° - 90°
        angle += 90
        self.__fw.turn(angle)

    # Car Setup
    def start(self):
        picar.setup()
        self.__bw.ready()
        self.__fw.ready()
        self.__fw.turn_straight()

    # Waiting Mode
    def slow_down(self):
        # Slow down current speed to 0 to protect DC-Motors
        print("Slowing Down")
        for i in range(abs(self.__motor_speed)):
            if self.__motor_speed > 0:
                self.__motor_speed -= 1
                self.__drive(self.__motor_speed)
            elif self.__motor_speed < 0:
                self.__motor_speed += 1
                self.__drive(self.__motor_speed)
        # self.__motor_speed = 0
        # self.__bw.stop()
        self.__fw.turn_straight()

    def control_max_speed(self):
        if self.__motor_speed > self.__max_motor_speed:
            self.__motor_speed = self.__max_motor_speed
            if self.__max_motor_speed == 0:
                print("For safety purposes, PiCar is not allowed to move forward")
            else:
                print("Maximum allowed speed: " + str(self.__max_motor_speed))

        self.__drive(self.__motor_speed)
        # print("Speed: " + str(self.__motor_speed))

    def turn_manual_right(self, mirror_mode):
        if mirror_mode == 0:
            if self.__turning_angle < 0:
                self.__turning_angle = 0
            if self.__turning_angle < 45:
                self.__turning_angle += 5
            else:
                self.__turning_angle = 45
        if mirror_mode == 1:
            if self.__can_turn_right:
                if self.__turning_angle < 0:
                    self.__turning_angle = 0
                if self.__turning_angle < 45:
                    self.__turning_angle += 5
                else:
                    self.__turning_angle = 45
            """
            else:
                self.__turning_angle = 0
            """
        self.__correct_servo_turn(angle=self.__turning_angle)

    def turn_manual_left(self, mirror_mode):
        if mirror_mode == 0:
            if self.__turning_angle > 0:
                self.__turning_angle = 0
            if self.__turning_angle > -45:
                self.__turning_angle -= 5
            else:
                self.__turning_angle = -45
        if mirror_mode == 1:
            if self.__can_turn_left:
                if self.__turning_angle > 0:
                    self.__turning_angle = 0
                if self.__turning_angle > -45:
                    self.__turning_angle -= 5
                else:
                    self.__turning_angle = -45
            """
            else:
                self.__turning_angle = 0
            """
        self.__correct_servo_turn(angle=self.__turning_angle)

    def drive_manual_forward(self):
        self.__motor_speed += self.__step
        if self.__motor_speed > 100:
            self.__motor_speed = 100
        self.__drive(self.__motor_speed)

    def drive_manual_reverse(self):
        self.__motor_speed -= self.__step
        if self.__motor_speed < -50:
            self.__motor_speed = -50
        self.__drive(self.__motor_speed)

    # Autonomous Control
    def stop(self):
        self.__bw.stop()
        self.__fw.turn_straight()

    # Switch off motors and end program
    def switch_off(self):
        self.stop()
        exit(0)

    def set_max_motor_speed(self, max_motor_speed):
        self.__max_motor_speed = max_motor_speed

    def set_motor_speed(self, motor_speed):
        self.__motor_speed = motor_speed
        print("Motor Speed: " + str(self.__motor_speed))

    def set_turning_angle(self, turning_angle):
        self.__turning_angle = turning_angle

    def set_turning_time(self, turning_time):
        self.__turning_time = turning_time

    def drive_autonomous_forward(self):
        self.__drive(self.__motor_speed)

    def set_can_turn_right(self, can_turn_right):
        self.__can_turn_right = can_turn_right

    def set_can_turn_left(self, can_turn_left):
        self.__can_turn_left = can_turn_left

    def turn_autonomous(self):
        self.__correct_servo_turn(angle=self.__turning_angle)
        self.__drive(motor_speed=self.__motor_speed)
        sleep(self.__turning_time)
        self.__fw.turn_straight()

