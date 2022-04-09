from math import radians, degrees, cos, atan
import numpy as np
from lib import constants as c
from collections import deque
from statistics import mean

# Car Dimensions
beta = c.picar["beta"]
beta_rad = radians(beta)

width = c.picar["tire-tire width"] + c.picar["offset"]
lidar2picar = width/2


class RegionDistribution(object):
    def __init__(self):
        self.__distances = []
        self.__angles_deg = []

        self.__distances_right = []
        self.__angles_right = []
        self.__distances_left = []
        self.__angles_left = []

        self.__mirror_distances_right = []
        self.__mirror_distances_left = []
        self.__mirror_angles_right = []
        self.__mirror_angles_left = []
        self.__mirror_front_distances = []
        self.__mirror_front_angles = []

        self.__front_distances = []
        self.__front_angles = []
        self.__limit_distances_right = []
        self.__limit_distances_left = []
        self.__limit_angles_right = []
        self.__limit_angles_left = []
        self.__bsd_right = []
        self.__bsd_left = []

        self.__right_mirror_RSSI = []
        self.__left_mirror_RSSI = []
        self.__front_view_RSSI = []
        self.__std_dev_angles = []
        self.__std_dev_distances = []

    def __create_zone(self, region_multiplier, zone, distances, angles):
        """

        :param angles:
        :param distances:
        :param region_multiplier:
        :type zone: int
        """
        angle = []
        distance = []
        zonal_range = (45 * region_multiplier) * zone

        for i in range(zonal_range, zonal_range + (45 * region_multiplier)):
            distance.append(distances[i])
            angle.append(angles[i])

        if distance or angle is not None:
            return distance, angle

    def __create_zones(self):
        # distances_right = []
        # distances_left = []
        # angles_right = []
        # angles_left = []

        '''
        # 135° - 90°
        distances_0, angles_0 = self.__distribution(region_multiplier=1, zone=0, distances=self.distances,
                                                    angles=self.angles_deg)
        '''
        # 90° - 45°
        distances_1, angles_1 = self.__create_zone(region_multiplier=1, zone=1, distances=self.__distances,
                                                   angles=self.__angles_deg)
        # 45° - 0°
        distances_2, angles_2 = self.__create_zone(region_multiplier=1, zone=2, distances=self.__distances,
                                                   angles=self.__angles_deg)
        # 0° - -45°
        distances_3, angles_3 = self.__create_zone(region_multiplier=1, zone=3, distances=self.__distances,
                                                   angles=self.__angles_deg)
        # -45° - -90°
        distances_4, angles_4 = self.__create_zone(region_multiplier=1, zone=4, distances=self.__distances,
                                                   angles=self.__angles_deg)
        '''
        # -90° - -135°
        distances_5, angles_5 = self.__distribution(region_multiplier=1, zone=5, distances=self.distances,
                                                    angles=self.angles_deg)
        # print(distances_0, angles_0)
        '''

        # distances_right.extend(distances_1)
        # distances_right.extend(distances_2)
        # angles_right.extend(angles_1)
        # angles_right.extend(angles_2)

        # distances_left.extend(distances_3)
        # distances_left.extend(distances_4)
        # angles_left.extend(angles_3)
        # angles_left.extend(angles_4)
        self.__distances_right = np.append(distances_1, distances_2)
        self.__angles_right = np.append(angles_1, angles_2)
        self.__distances_left = np.append(distances_3, distances_4)
        self.__angles_left = np.append(angles_3, angles_4)

        mirror_front_distances = []
        mirror_front_angles = []
        for i in range(len(self.__angles_deg)):
            if -30 < self.__angles_deg[i] < 30:
                mirror_front_distances.append(self.__distances[i])
                mirror_front_angles.append(self.__angles_deg[i])

        self.__mirror_front_distances = mirror_front_distances
        self.__mirror_front_angles = mirror_front_angles

        # self.__mirror_front_distances = np.append(distances_2, distances_3)
        # self.__mirror_front_angles = np.append(angles_2, angles_3)
        # print("Front Distances: " + str(self.__mirror_front_distances))
        # print("Front Angles: " + str(self.__mirror_front_angles))

        # self.__distances_right = distances_right
        # self.__angles_right = angles_right
        # self.__distances_left = distances_left
        # self.__angles_left = angles_left


    @property
    def get_distances_right(self):
        return self.__distances_right

    @property
    def get_angles_right(self):
        return self.__angles_right

    @property
    def get_distances_left(self):
        return self.__distances_left

    @property
    def get_angles_left(self):
        return self.__angles_left

    # Front View
    def __calculate_alpha(self, start_index, end_index, theta):
        alpha = []
        bsd = []
        limit_angles = []
        limit_distances = []
        for i in range(start_index, end_index):
            alpha.append(abs(self.__angles_deg[i]) - theta)
            alpha.sort()
            bsd.append(self.__distances[i])
            limit_angles.append(self.__angles_deg[i])

        for i in range(len(alpha)):
            limit_distances.append(lidar2picar/cos(radians(alpha[i])))

        if alpha and bsd and limit_angles and limit_distances is not None:
            return alpha, bsd, limit_angles, limit_distances

    def __view_blind_zone(self, theta, index_right, index_left):
        # Object in front of the car & sensor
        if theta < 90:
            alpha_right, bsd_right, limit_angles_right, limit_distances_right = self.__calculate_alpha(46, index_right, theta)
            alpha_left, bsd_left, limit_angles_left, limit_distances_left = self.__calculate_alpha(index_left+1, 225, theta)

            if limit_distances_right and limit_distances_left and bsd_right and bsd_left and limit_angles_right and limit_angles_left is not None:
                return limit_distances_right, limit_distances_left, bsd_right, bsd_left, limit_angles_right, \
                       limit_angles_left

    def __check_surrounding(self, start_index, end_index, min_distance, max_distance):
        surrounding_objects_list = []
        surrounding_object = c.lidar_distances["minimum distance"]
        for i in range(start_index, end_index):
            surrounding_objects_list.append(self.__distances[i])
            surrounding_object = min(surrounding_objects_list)

        if min_distance < surrounding_object <= max_distance:
            surrounding_object_list = [surrounding_object]
            temp = set(surrounding_object_list)
            index_list = [i for i, val in enumerate(self.__distances) if val in temp]
            for x in range(len(index_list)):
                if start_index <= index_list[x] <= end_index:
                    index_list_range = [index_list[x]]
                    index = int(index_list_range[0])
                    forward_distances_deque = deque(maxlen=10)
                    for i in range(10):
                        forward_distances_deque.append(surrounding_object * abs(cos(radians(self.__angles_deg[index]))))
                        i += 1
                    forward_distances = list(forward_distances_deque)
                    forward_distance = mean(forward_distances)

                    # forward_distance = surrounding_object * abs(cos(radians(self.__angles_deg[index])))
                    # print(forward_distance)
                    return forward_distance
        else:
            pass

    def __scan_surrounding(self):
        minimum = c.lidar_distances["minimum distance"]
        maximum = c.lidar_distances["maximum distance"]

        # Forward distance is NOT dependent on LiDar Device
        forward_distance_100 = self.__check_surrounding(start_index=90, end_index=180, min_distance=minimum,
                                                        max_distance=100)
        forward_distance_200 = self.__check_surrounding(start_index=109, end_index=161, min_distance=100,
                                                        max_distance=200)
        forward_distance_300 = self.__check_surrounding(start_index=117, end_index=153, min_distance=200,
                                                        max_distance=300)
        forward_distance_500 = self.__check_surrounding(start_index=124, end_index=146, min_distance=300,
                                                        max_distance=500)
        forward_distance_1000 = self.__check_surrounding(start_index=129, end_index=141, min_distance=500,
                                                         max_distance=1000)
        forward_distance_2000 = self.__check_surrounding(start_index=132, end_index=138, min_distance=1000,
                                                         max_distance=2000)
        forward_distance_max = self.__check_surrounding(start_index=133, end_index=137, min_distance=2000,
                                                        max_distance=maximum)

        forward_distance_list = [forward_distance_100, forward_distance_200, forward_distance_300,
                                 forward_distance_500, forward_distance_1000, forward_distance_2000,
                                 forward_distance_max]

        return min(filter(lambda x: x is not None, forward_distance_list))

    def __view_front(self):
        length = self.__scan_surrounding()
        self.__front_distances = [length]
        theta_pos_list = []
        center_point = 135
        theta_list = [int(round(degrees(atan(lidar2picar / length))))]
        temp = set(theta_list)
        index_list = [i for i, val in enumerate(self.__angles_deg) if val in temp]
        index_right = int(index_list[0])
        index_range = center_point - index_right
        index_left = center_point + index_range

        theta_pos_list.append(self.__angles_deg[index_right])
        theta = int(theta_pos_list[0])

        if theta < 90 and theta and index_right and index_left is not None:
            self.__limit_distances_right, self.__limit_distances_left, self.__bsd_right, self.__bsd_left,\
            self.__limit_angles_right, self.__limit_angles_left = self.__view_blind_zone(theta, index_right, index_left)

    def __start_mirror_mode(self, start_index, end_index, distances, angles):
        mirror_distances = []
        mirror_angles = []

        for i in range(start_index, end_index):
            mirror_distances.append(distances[i])
            mirror_angles.append(angles[i])

        return mirror_distances, mirror_angles


    @property
    def front_distances(self):
        return self.__front_distances

    @property
    def limits_right(self):
        return self.__limit_distances_right

    @property
    def limits_left(self):
        return self.__limit_distances_left

    @property
    def blind_right(self):
        return self.__bsd_right

    @property
    def blind_left(self):
        return self.__bsd_left

    def generate_regions(self, distances, angles_deg, mirror_mode):
        self.__distances = distances
        self.__angles_deg = angles_deg

        # self.__view_front()
        self.__create_zones()
        if mirror_mode == 0:
            # self.__create_zones()
            self.__view_front()
        elif mirror_mode == 1:
            self.__mirror_distances_right, self.__mirror_angles_right = self.__start_mirror_mode(start_index=40,
                                                                                                 end_index = 70,
                                                                                                 distances= self.__distances,
                                                                                                 angles=self.__angles_deg)

            self.__mirror_distances_left, self.__mirror_angles_left = self.__start_mirror_mode(start_index=200,
                                                                                                 end_index = 230,
                                                                                                 distances= self.__distances,
                                                                                             angles=self.__angles_deg)
    def obtain_std_dev_values(self):
        std_dev_angles = []
        std_dev_distances = []
        for i in range(len(self.__angles_deg)):
            if self.__angles_deg[i] == 5 or self.__angles_deg[i] == 70 or self.__angles_deg[i] == -70:
                std_dev_angles.append(self.__angles_deg[i])
                std_dev_distances.append(self.__distances[i])
        self.__std_dev_angles = std_dev_angles
        self.__std_dev_distances = std_dev_distances
        # print(std_dev_angles, std_dev_distances)

    def __distribute_RSSI_values(self, start_index, end_index, RSSI):
        RSSI_values = []

        for i in range(start_index, end_index):
            RSSI_values.append(RSSI[i])

        return RSSI_values

    def obtain_mirror_RSSI(self, RSSI):
        right_mirror_RSSI = self.__distribute_RSSI_values(start_index=40, end_index=70, RSSI=RSSI)
        left_mirror_RSSI = self.__distribute_RSSI_values(start_index=200, end_index=230, RSSI=RSSI)
        # front_view_RSSI = self.__distribute_RSSI_values(start_index=90, end_index=179, RSSI=RSSI)
        front_view_RSSI = self.__distribute_RSSI_values(start_index=105, end_index=164, RSSI=RSSI)

        # print(right_mirror_RSSI)
        # print(left_mirror_RSSI)
        self.__right_mirror_RSSI = right_mirror_RSSI
        self.__left_mirror_RSSI = left_mirror_RSSI
        self.__front_view_RSSI = front_view_RSSI

    @property
    def get_right_mirror_RSSI(self):
        return self.__right_mirror_RSSI

    @property
    def get_left_mirror_RSSI(self):
        return self.__left_mirror_RSSI

    @property
    def get_front_view_RSSI(self):
        return self.__front_view_RSSI

    @property
    def get_mirror_distances_right(self):
        return self.__mirror_distances_right

    @property
    def get_mirror_distances_left(self):
        return self.__mirror_distances_left

    @property
    def get_mirror_angles_right(self):
        return self.__mirror_angles_right

    @property
    def get_mirror_angles_left(self):
        return self.__mirror_angles_left

    @property
    def get_mirror_distances_front(self):
        return self.__mirror_front_distances

    @property
    def get_mirror_angles_front(self):
        return self.__mirror_front_angles

    @property
    def get_std_dev_angles(self):
        return self.__std_dev_angles

    @property
    def get_std_dev_distances(self):
        return self.__std_dev_distances