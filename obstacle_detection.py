from lib import constants as c
from math import cos, radians
import numpy as np

class ObstacleDetection(object):
    def __init__(self):
        self.__minimal_distance = c.lidar_distances["maximum distance"]
        self.__free_space_right = False
        self.__free_space_left = False

        self.__free_ground_right = False
        self.__free_ground_left = False
        self.__free_forward_ground = 0
        self.__free_forward_front = 0
        # self.__free_forward = 0
        self.__no_space = True  # Either no ground detected, large obstacle detected or absolutely no light reflection back to LiDar

    def __check_blind_sides(self, bsd, limit_distances):
        cb_distances = []
        if bsd is not None:
            for i in range(len(limit_distances)):
                if bsd[i] <= limit_distances[i]:
                    cb_distances.append(bsd[i])
                else:
                    pass

        return cb_distances

    def check_obstacles(self, front_distances, limit_distances_right, limit_distances_left, bsd_right, bsd_left):
        cbd_left = self.__check_blind_sides(bsd_left, limit_distances_left)
        cbd_right = self.__check_blind_sides(bsd_right, limit_distances_right)

        # print("CBD Left: " + str(cbd_left))
        # print("BSD Left: " + str(bsd_left))
        # print("Limit Left: " + str(limit_distances_left))

        compare_values = []
        compare_values.extend(front_distances)
        compare_values.extend(cbd_left)
        compare_values.extend(cbd_right)

        self.__minimal_distance = min(filter(lambda x: x is not None, compare_values))


    # Turning Zone
    def __calculate_turning_zone_limits(self, angles, index_start, index_mid, index_stop):
        safety_gap = c.picar["safety gap"]
        half_width = (c.picar["tire-tire width"]+c.picar["offset"])/2
        turning_radius = self.__minimal_distance - safety_gap - half_width
        bw2front = 272
        turning_length = turning_radius + bw2front
        limit_distances_top = []
        limit_distances_bottom = []

        # Right Turning Side
        if angles[0] == 90:
            for i in range(index_start, index_mid):
                limit_distances_bottom.append(turning_length/cos(radians(90-angles[i])))
            for i in range(index_mid, index_stop):
                limit_distances_top.append(self.__minimal_distance/cos(radians(abs(angles[i]))))

            if limit_distances_top and limit_distances_bottom is not None:
                return limit_distances_top, limit_distances_bottom

        # Left Turning Side
        elif angles[0] == -0:
            for i in range(index_start, index_mid):
                limit_distances_top.append(self.__minimal_distance / cos(radians(abs(angles[i]))))

            for i in range(index_mid, index_stop):
                limit_distances_bottom.append(turning_length / cos(radians(90 - abs(angles[i]))))

            if limit_distances_top and limit_distances_bottom is not None:
                return limit_distances_top, limit_distances_bottom

    def __calculate_safety_distances(self, angles_right, angles_left):
        safety_distances_right = []
        safety_distances_left = []

        ldr_top, ldr_bottom = self.__calculate_turning_zone_limits(angles=angles_right, index_start=10, index_mid=30, index_stop=90)
        ldl_top, ldl_bottom = self.__calculate_turning_zone_limits(angles=angles_left, index_start=1, index_mid=60, index_stop=81)

        safety_distances_right.extend(ldr_bottom)
        safety_distances_right.extend(ldr_top)

        safety_distances_left.extend(ldl_top)
        safety_distances_left.extend(ldl_bottom)

        if safety_distances_right and safety_distances_left is not None:
            return safety_distances_right, safety_distances_left

    def __check_free_turning_space(self, distances, safety_distances):
        turning_zone_distances = []

        for i in range(len(safety_distances)):
            if distances[i] <= safety_distances[i]:
                turning_zone_distances.append(distances[i])
                # print("Turning: " + str(turning_zone_distances))

        if turning_zone_distances:
            free_space = False
            return free_space
        elif not turning_zone_distances:
            free_space = True
            return free_space

    def __check_closest_object(self, distances_right, distances_left):
        close_right = []
        close_left = []

        for i in range(10, 30):
            close_right.append(distances_right)
        closest_distance_right = min(x for x in close_right)

        for i in range(60, 80):
            close_left.append(distances_left)
        closest_distance_left = min(x for x in close_left)

        if closest_distance_right < closest_distance_left:
            free_space_right = False
            free_space_left = True
        elif closest_distance_right > closest_distance_left:
            free_space_right = True
            free_space_left = False
        else:
            free_space_right = True
            free_space_left = True

        return free_space_right, free_space_left

    def check_turning_zone(self, distances_right, angles_right, distances_left, angles_left):
        print("Minimal Distances: " + str(self.__minimal_distance))
        if c.lidar_distances["turning min"] <= self.__minimal_distance < c.lidar_distances["turning max"]:
            safety_distances_right, safety_distances_left = self.__calculate_safety_distances(angles_right, angles_left)
            free_space_right = self.__check_free_turning_space(distances_right, safety_distances_right)
            free_space_left = self.__check_free_turning_space(distances_left, safety_distances_left)

            print("Free Space Right 1: " + str(free_space_right))
            print("Free Space Left 1: " + str(free_space_left))

            if free_space_right and free_space_left is True:
                free_space_right, free_space_left = self.__check_closest_object(distances_right, distances_left)

            '''
            print("Safety Right: " + str(safety_distances_right))
            print("Safety Left: " + str(safety_distances_left))

            print("Distance Right: " + str(distances_right))
            print("Distance Left: " + str(distances_left))
            '''
            print("Free Space Right 2: " + str(free_space_right))
            print("Free Space Left 2: " + str(free_space_left))

            self.__free_space_right = free_space_right
            self.__free_space_left = free_space_left

    def check_forward(self, cartesian_axis_front):
        target_x_coord = []
        target_y_coord = []

        x_axis = np.array(cartesian_axis_front[:,0])
        y_axis = np.array(cartesian_axis_front[:,1])
        # print("X-axis: " +str(x_axis))

        for i in range(np.size(x_axis)):
            if -150 < x_axis[i] < 150:
                target_x_coord.append(x_axis[i])
        temp = set(target_x_coord)
        index_list = [i for i, val in enumerate(x_axis) if val in temp]

        for i in range(len(index_list)):
            target_y_coord.append(y_axis[index_list[i]])

        closest_distance = min(target_y_coord)
        self.__free_forward_front = closest_distance
        # print("Closest Object: " +str(closest_distance))


    def check_ground(self, ground_coordinates_right, ground_coordinates_left):
        x_axis_coord = np.append(ground_coordinates_right[:,0], ground_coordinates_left[:,0])
        y_axis_coord = np.append(ground_coordinates_right[:,1], ground_coordinates_left[:,1])
        z_axis_coord = np.append(ground_coordinates_right[:,2], ground_coordinates_left[:,2])

        target_x_coord = []
        target_y_coord = []
        target_z_coord = []

        self.__free_forward_ground = max(y_axis_coord)
        self.__no_space = False

        z_target = []

        for i in range(2):
            if -162 < z_axis_coord[i] < -148:
                z_target.append(z_axis_coord[i])

        z_array = np.array(z_target)
        if z_array.size == 0:
            self.__no_space = True
        else:
            self.__no_space = False

        if -162 < min(z_axis_coord) and max(z_axis_coord) < -148:
            self.__free_ground_right = True
            self.__free_ground_left = True

        else:
            for i in range(len(z_axis_coord)):
                if not (-162 < z_axis_coord[i] < -148):
                    target_z_coord.append(z_axis_coord[i])

            temp = set(target_z_coord)
            index_list = [i for i, val in enumerate(z_axis_coord) if val in temp]

            for i in range(len(index_list)):
                target_x_coord.append(x_axis_coord[index_list[i]])
                target_y_coord.append(y_axis_coord[index_list[i]])

            if not self.__no_space:
                self.__free_forward_ground = min(target_y_coord)
            else:
                self.__free_forward_ground = 180

            # if min(target_y_coord) >= 250:
            # self.__free_forward = True
            if self.__free_forward_ground <= 300:
                if min(target_x_coord) > 0:
                    self.__free_ground_right = False
                    self.__free_ground_left = True
                elif max(target_x_coord) < 0:
                    self.__free_ground_right = True
                    self.__free_ground_left = False
                else:
                    self.__free_ground_right = False
                    self.__free_ground_left = False
            else:
                self.__free_ground_right = True
                self.__free_ground_left = True

    def compare_forward(self):
        # if self.__free_ground_right and self.__free_ground_left:
            # free_forward = self.__free_forward_front
        # elif not self.__no_space:
        if self.__free_forward_ground <= self.__free_forward_front:
            free_forward = self.__free_forward_ground
        elif self.__free_forward_front <= self.__free_forward_ground:
            free_forward = self.__free_forward_front
        else:
            free_forward = self.__free_forward_ground
        # else:
        #     free_forward = 180

        self.__minimal_distance = free_forward
        # print("Ground: " +str(self.))
        # print("Forward: " +str(self.__free_forward))


    @property
    def get_min_distance(self):
        print("Minimal Distance:" +str(self.__minimal_distance))
        return self.__minimal_distance

    @property
    def get_free_ground_right(self):
        return self.__free_ground_right

    @property
    def get_free_ground_left(self):
        return self.__free_ground_left
    """
    @property
    def get_free_ground_forward(self):
        return self.__free_forward
    """
    @property
    def right_free(self):
        return self.__free_space_right

    @property
    def left_free(self):
        return self.__free_space_left

