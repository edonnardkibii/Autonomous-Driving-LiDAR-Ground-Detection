from lidar import Lidar
from region_distribution import RegionDistribution
from obstacle_detection import ObstacleDetection
from ground_detection import GroundDetection
from control_unit import ControlUnit
from motor import Motor
import ground_plot
from time import sleep
import time
import os
from lib import constants as c
import readchar
import threading
import csv
import pandas as pd

lidar = Lidar(c.lidar["id Vendor"], c.lidar["id Product"])
lidar.connect()
regions = RegionDistribution()
obstacles = ObstacleDetection()
control_unit = ControlUnit()
ground_detection = GroundDetection()
# ground_detection.calculate_normal_vector()
motor = Motor()
motor.start()


def get_ch():
    ch = readchar.readchar()
    return ch

def keyboard_ctrl(key, mirror_mode):
    if key == 'w':
        motor.drive_manual_forward()
    elif key == 's':
        motor.drive_manual_reverse()
    elif key == 'a':
        motor.turn_manual_left(mirror_mode)
    elif key == 'd':
        motor.turn_manual_right(mirror_mode)
    else:
        print("Invalid input")

def control_mirror_mode(stop):
    while True:
        # start_time = time.time()
        time_delay = lidar.scan()
        regions.generate_regions(lidar.get_distances, lidar.get_angles, mirror_mode=1)
        start_time = time.time()
        ground_detection.view_front(regions.get_mirror_distances_front, regions.get_mirror_angles_front)
        ground_detection.run_ground_detection(regions.get_mirror_distances_right,
                                          regions.get_mirror_angles_right,
                                          regions.get_mirror_distances_left,
                                          regions.get_mirror_angles_left)
        stop_time = time.time()
        obstacles.check_ground(ground_detection.get_ground_coordinates_right, ground_detection.get_ground_coordinates_left)
        obstacles.check_forward(ground_detection.get_cart_axis_front)
        obstacles.compare_forward()
        control_unit.set_minimal_distance(obstacles.get_min_distance)
        control_unit.calculate_max_manual_speed(mirror_mode=1)
        motor.set_can_turn_right(obstacles.get_free_ground_right)
        motor.set_can_turn_left(obstacles.get_free_ground_left)
        motor.set_max_motor_speed(control_unit.max_speed)
        print("Max: " + str(control_unit.max_speed))
        motor.control_max_speed()
        # stop_time = time.time()
        print("Time Taken: ", stop_time-start_time)
        sleep(time_delay)
        if stop():
            break

def control_lidar_aut_mode():
    os.system('clear')
    time_delay = lidar.scan()
    regions.generate_regions(lidar.get_distances, lidar.get_angles, mirror_mode=0)

    obstacles.check_obstacles(regions.front_distances, regions.limits_right, regions.limits_left,
                              regions.blind_right, regions.blind_left)
    obstacles.check_turning_zone(regions.get_distances_right, regions.get_angles_right,
                                 regions.get_distances_left, regions.get_angles_left)

    control_unit.set_minimal_distance(obstacles.get_min_distance)

    return time_delay


def control_lidar_man_mode(stop):
    while True:
        time_delay = control_lidar_aut_mode()
        control_unit.calculate_max_manual_speed(mirror_mode=0)
        motor.set_max_motor_speed(control_unit.max_speed)
        print("Max: " + str(control_unit.max_speed))
        motor.control_max_speed()
        sleep(time_delay)
        if stop():
            print("stopping")
            break


def run_autonomous_mode():
    try:
        while True:
            time_delay = control_lidar_aut_mode()
            control_unit.set_free_space(obstacles.right_free, obstacles.left_free)
            control_unit.calculate_autonomous_speed()
            motor.set_motor_speed(control_unit.speed)

            if not control_unit.turn:
                motor.drive_autonomous_forward()
            elif control_unit.turn:
                motor.stop()
                sleep(0.5)
                control_unit.calculate_autonomous_turning()
                motor.set_motor_speed(control_unit.speed)
                motor.set_turning_angle(control_unit.turning_angle)
                motor.set_turning_time(control_unit.turning_time)
                motor.turn_autonomous()

            sleep(time_delay)

    except KeyboardInterrupt:
        motor.slow_down()


def run_manual_mode():
    kill_thread = False
    lidar_thread = threading.Thread(target=control_lidar_man_mode, args=(lambda: kill_thread,))
    lidar_thread.start()

    key = ""
    while True:
        print("Manual Mode")
        # time_delay = lidar_setup()
        key = ""
        key = get_ch()
        if key != 'x':
            keyboard_ctrl(key, mirror_mode=False)
        else:
            kill_thread = True
            lidar_thread.join()
            motor.slow_down()
            print("Motor Off")
            sleep(1)
            break
        # sleep(time_delay)

def run_ground_detection_mode():
    try:
        # os.system('clear')
        time_delay = lidar.scan()
        regions.generate_regions(lidar.get_distances, lidar.get_angles, mirror_mode=1)
        regions.obtain_mirror_RSSI(lidar.get_RSSI)
        ground_detection.load_config_ground_detection(regions.get_mirror_distances_right, regions.get_mirror_angles_right,
                                             regions.get_mirror_distances_left, regions.get_mirror_angles_left)
        sleep(time_delay)

        print("Welcome to ground detection mode")
        print("To collect data in a CSV file, press 1")
        print("For plot visualization, press 2")
        print("For driving mode, press 3")
        print("To exit, press x")
        key = input("Choose your mode and press ENTER: ")

        while True:
            if key != 'x':
                if key == '1':
                    num = input("Enter the number of scans you want: ")

                    csv_angles = []
                    csv_distances = []
                    csv_mirror_angles=[]
                    # csv_RSSI = []
                    csv_mirror_RSSI = []
                    mirror_x_axis = []
                    mirror_y_axis = []
                    mirror_z_axis = []
                    csv_std_dev_angles = []
                    csv_std_dev_distances = []

                    support_mirror_right = []
                    support_mirror_left = []

                    for i in range(int(num)):
                        time_delay = lidar.scan()


                        regions.generate_regions(lidar.get_distances, lidar.get_angles, mirror_mode=1)
                        regions.obtain_mirror_RSSI(lidar.get_RSSI)
                        regions.obtain_std_dev_values()
                        ground_detection.view_front(regions.get_mirror_distances_front, regions.get_mirror_angles_front)
                        ground_detection.run_ground_detection(regions.get_mirror_distances_right,
                                                               regions.get_mirror_angles_right,
                                                               regions.get_mirror_distances_left,
                                                               regions.get_mirror_angles_left)


                        csv_angles.extend(lidar.get_angles)
                        csv_distances.extend(lidar.get_distances)
                        # csv_RSSI.extend(lidar.get_RSSI)
                        csv_mirror_angles.extend(regions.get_mirror_angles_right)
                        csv_mirror_angles.extend(regions.get_mirror_angles_left)
                        csv_mirror_angles.extend(regions.get_mirror_angles_front)

                        csv_mirror_RSSI.extend(regions.get_right_mirror_RSSI)
                        csv_mirror_RSSI.extend(regions.get_left_mirror_RSSI)
                        csv_mirror_RSSI.extend(regions.get_front_view_RSSI)

                        mirror_x_axis.extend(ground_detection.get_x_axis)
                        mirror_y_axis.extend(ground_detection.get_y_axis)
                        mirror_z_axis.extend(ground_detection.get_z_axis)

                        # support_mirror_right.extend(lidar.get_mirror_support_right)
                        # support_mirror_left.extend(lidar.get_mirror_support_left)
                        # print("Support Right: " + str(support_mirror_right))

                        csv_std_dev_angles.extend(regions.get_std_dev_angles)
                        csv_std_dev_distances.extend(regions.get_std_dev_distances)
                        sleep(time_delay)

                    # Save CSV File
                    # dict = {'support right': support_mirror_right, 'support left': support_mirror_left}

                    # df = pd.DataFrame(dict)
                    # df.to_csv('lidar_mirror_support.csv', header=True, index=False)
                    # print(len(csv_mirror_angles), len(csv_mirror_RSSI), len(mirror_x_axis))
                    # print(csv_mirror_angles)
                    """
                    dict = {'mirror-angles': csv_mirror_angles, ' Mirror-RSSI': csv_mirror_RSSI, 'x-axis': mirror_x_axis,
                             'y-axis': mirror_y_axis, 'z-axis': mirror_z_axis}
            
                    df = pd.DataFrame(dict)
                    df.to_csv('lidar_data_diamond_reflector_0cm_45deg_20deg_10cm.csv', header=True, index=False)
                    """

                    with open('lidar_std_dev_0cm_30deg_0deg_10cm.csv', 'w') as file:
                        write = csv.writer(file)
                        # write.writerow(csv_angles)
                        # write.writerow(csv_distances)
                        # write.writerow(csv_mirror_RSSI)
                        write.writerow(csv_std_dev_angles)
                        write.writerow(csv_std_dev_distances)


                    print("Values saved in csv file")

                elif key == '2':
                    time_delay = lidar.scan()
                    regions.generate_regions(lidar.get_distances, lidar.get_angles, mirror_mode=1)
                    ground_detection.view_front(regions.get_mirror_distances_front, regions.get_mirror_angles_front)
                    ground_detection.run_ground_detection(regions.get_mirror_distances_right,
                                                          regions.get_mirror_angles_right,
                                                          regions.get_mirror_distances_left,
                                                          regions.get_mirror_angles_left)

                    ground_plot.plot_graph(ground_detection.get_x_axis, ground_detection.get_y_axis,
                                           ground_detection.get_z_axis)
                    # print("Getter Left: " +str(regions.get_mirror_distances_left))
                    sleep(time_delay)

                elif key == '3':
                    kill_thread = False
                    mirror_thread = threading.Thread(target=control_mirror_mode, args=(lambda: kill_thread,))
                    mirror_thread.start()

                    key = ""
                    while True:
                        # time_delay = lidar_setup()
                        key = ""
                        key = get_ch()
                        if key != 'x':
                            keyboard_ctrl(key, mirror_mode=1)
                        else:
                            kill_thread = True
                            mirror_thread.join()
                            motor.slow_down()
                            print("Motor Off")
                            sleep(1)
                            break
                        # sleep(time_delay)
                else:
                    print("Invalid Input")
                    # sleep(time_delay)

    except KeyboardInterrupt:
        motor.slow_down()






