"""
Bachelorarbeit
Author: James Edonnard Kibii, EIB
"""

import numpy as np
from lib import mirror_constants as mc
import math


class GroundDetection(object):
    def __init__(self):
        self.__angles_right = np.empty([1,0])
        self.__angles_left = np.empty([1,0])
        self.__distances_right = np.empty([1,0])
        self.__distances_left = np.empty([1,0])

        self.__intersection_points_right = np.empty([1,0])
        self.__mag_intersection_right = np.empty([1,0])
        self.__intersection_points_left = np.empty([1,0])
        self.__mag_intersection_left = np.empty([1,0])
        self.__unit_reflection_right = np.empty([1,0])
        self.__unit_reflection_left = np.empty([1,0])

        self.__ground_coordinates_right = np.empty([1,0])
        self.__ground_coordinates_left = np.empty([1,0])
        self.__cartesian_axis_front = np.empty([1,0])

        self.__x_axis_plot = np.empty([1,0])
        self.__y_axis_plot = np.empty([1,0])
        self.__z_axis_plot = np.empty([1,0])

        self.__mirror_x_axis = np.empty([1,0])
        self.__mirror_y_axis = np.empty([1, 0])
        self.__mirror_z_axis = np.empty([1, 0])


    def __convert_polar2cartesian(self, distances, angles):
        x1 = distances * np.sin(np.radians(angles))
        y1 = distances * np.cos(np.radians(angles))
        z1 = np.zeros(len(angles))

        cartesian_axis = []
        for i in range(len(angles)):
            cartesian_coord = [x1[i], y1[i], z1[i]]
            cartesian_axis.append(cartesian_coord)

        return np.array(cartesian_axis)

    def __calculate_normal_vector(self):
        """
        Newell's Method
        https://www.khronos.org/opengl/wiki/Calculating_a_Surface_Normal
        Nx = UyVz - UzVy
        Ny = UzVx - UxVz
        Nz = UxVy - UyVx

        """
        """
        # For Comparison
        Nx = (vector_U[1]*vector_V[2]) - (vectorU[2]*vector_V[1])
        Ny = (vector_U[2]*vector_V[0]) - (vectorU[0]*vector_V[2])
        Nz = (vector_U[0]*vector_V[1]) - (vectorU[1]*vector_V[0])
        N = [nx, Ny, Nz]
        """

        vector_u_right = np.subtract(mc.mirror["mirror vector p2 right"], mc.mirror["mirror vector p1 right"])
        vector_v_right = np.subtract(mc.mirror["mirror vector p3 right"], mc.mirror["mirror vector p1 right"])
        normal_vector_right = np.cross(vector_v_right, vector_u_right)
        # print("Normal Right:" +str(normal_vector_right))

        vector_u_left = np.subtract(mc.mirror["mirror vector p2 left"], mc.mirror["mirror vector p1 left"])
        vector_v_left = np.subtract(mc.mirror["mirror vector p3 left"], mc.mirror["mirror vector p1 left"])
        normal_vector_left = np.cross(vector_u_left, vector_v_left)
        # print("Normal Left:" + str(normal_vector_left))

        normal_vector_right = normal_vector_right
        unit_normal_right = (1/np.linalg.norm(normal_vector_right)) * normal_vector_right

        normal_vector_left = normal_vector_left
        unit_normal_left = (1 / np.linalg.norm(normal_vector_left)) * normal_vector_left
        # print("Unit Normal Left: " +str(self.__unit_normal_left))

        return normal_vector_right, unit_normal_right, normal_vector_left, unit_normal_left

    '''
    @property
    def get_normal_vector_right(self):
        return self.__normal_vector_right

    @property
    def get_normal_vector_left(self):
        return self.__normal_vector_left
    '''

    def __calculate_lambda(self, cartesian_points, normal_vector, support_vector):
        # distances length needs to be converted to vector
        lambda_scalar_list = []
        for i in range(len(cartesian_points)):
            lambda_scalar = np.dot(normal_vector, support_vector)/np.dot(normal_vector, cartesian_points[i])
            lambda_scalar_list.append(lambda_scalar)

        # self.__lambda_scalar_list = lambda_scalar_list
        return lambda_scalar_list

    def __calculate_intersection(self, lambda_scalar_list, cartesian_axis):
        intersection_points = []
        mag_intersection_points = []
        for i in range(len(lambda_scalar_list)):
            # intersection = [x * lambda_scalar_list[i] for x in cartesian_axis[i]]
            intersection = lambda_scalar_list[i] * np.array(cartesian_axis[i])
            intersection_points.append(intersection)
            mag_intersection = np.linalg.norm(intersection)
            # mag_intersection = np.sqrt(np.dot(intersection, intersection))
            # mag_intersection =
            mag_intersection_points.append(mag_intersection)

        # print("Magnitude Intersection: " + str(mag_intersection_points))
        return intersection_points, mag_intersection_points

    def __calculate_unit_reflection(self, mag_distance, distance_vector, unit_normal):
        unit_distance_points = []
        unit_reflection_points = []
        for i in range(len(distance_vector)):
            # unit_distance = [x * 1/mag_distance[i] for x in distance_vector[i]]
            unit_distance = 1 / mag_distance[i] * np.array(distance_vector[i])
            unit_distance_points.append(unit_distance)
        for i in range(len(unit_distance_points)):
            unit_reflection = unit_distance_points[i] - 2 * (
                        np.dot(unit_distance_points[i], unit_normal) * np.array(unit_normal))
            unit_reflection_points.append(unit_reflection)
        return unit_reflection_points

    def __calculate_gamma(self, distance, mag_intersection):
        # gamma = distance - mag_intersection
        # gamma = []
        gamma = np.subtract(np.abs(distance), np.abs(mag_intersection))
        # print("Gamma: " +str(gamma))
        return gamma

    def __calculate_reflected_vector(self, gamma, unit_reflection):
        reflected_vector_points = []
        for i in range(len(gamma)):
            reflected_vector = gamma[i] * np.array(unit_reflection[i])
            reflected_vector_points.append(reflected_vector)
        return reflected_vector_points

    def __calculate_ground_coordinates(self, intersection_vector, reflected_vector):
        # ground_coordinate_points = []
        ground_coordinate_points = np.add(intersection_vector, reflected_vector)
        # self.__x2 = ground_coordinate_points[:,0]
        # self.__y2 = ground_coordinate_points[:,1]
        # self.__z2 = ground_coordinate_points[:,2]

        return ground_coordinate_points

    def load_config_ground_detection(self, distances_right, angles_right, distances_left, angles_left):
        self.__distances_right = distances_right
        self.__distances_left = distances_left
        self.__angles_right = angles_right
        self.__angles_left = angles_left

        normal_vector_right,unit_normal_right,normal_vector_left,unit_normal_left = self.__calculate_normal_vector()

        # print("Left: " +str(self.__distances_left))
        # Many
        cartesian_axis_right = self.__convert_polar2cartesian(self.__distances_right, self.__angles_right)
        # print("Cartesian Right: " +str(self.__cartesian_axis_right))
        cartesian_axis_left = self.__convert_polar2cartesian(self.__distances_left, self.__angles_left)
        # print("Cartesian Left: " +str(self.__cartesian_axis_left))

        # Many
        lambda_scalar_list_right = self.__calculate_lambda(cartesian_axis_right,
                                                                  normal_vector_right,
                                                                  mc.mirror["support vector right"])
        # print("Lambda Right: " +str(self.__lambda_scalar_list_right))
        lambda_scalar_list_left = self.__calculate_lambda(cartesian_axis_left,
                                                                 normal_vector_left,
                                                                 mc.mirror["support vector left"])
        # print("Lambda Left: " +str(self.__lambda_scalar_list_left))

        # Once ?
        self.__intersection_points_right, self.__mag_intersection_right = self.__calculate_intersection(
            lambda_scalar_list_right,
            cartesian_axis_right)
        # print("Intersection Right: " +str(self.__intersection_points_right))
        # print("Mag Intersection Right: " +str(self.__mag_intersection_right))
        self.__intersection_points_left, self.__mag_intersection_left = self.__calculate_intersection(
            lambda_scalar_list_left, cartesian_axis_left)
        # print("Mag Intersection Left: " +str(self.__mag_intersection_left))
        # print("Intersection Left: " +str(self.__intersection_points_left))

        # Once ?
        self.__unit_reflection_right = self.__calculate_unit_reflection(self.__distances_right,
                                                                        cartesian_axis_right,
                                                                        unit_normal_right)
        # print("Unit Reflection Right: " +str(self.__unit_reflection_right))
        self.__unit_reflection_left = self.__calculate_unit_reflection(self.__distances_left,
                                                                       cartesian_axis_left,
                                                                       unit_normal_left)
        #  print("Unit Reflection Left: " +str(self.__unit_reflection_left))

    def run_ground_detection(self, distances_right, angles_right, distances_left, angles_left):
        self.__distances_right = distances_right
        self.__distances_left = distances_left
        self.__angles_right = angles_right
        self.__angles_left = angles_left

        # Many
        gamma_right = self.__calculate_gamma(self.__distances_right, self.__mag_intersection_right)
        # print("Gamma Right: " +str(self.__gamma_right))
        gamma_left = self.__calculate_gamma(self.__distances_left, self.__mag_intersection_left)

        # Many
        reflected_right = self.__calculate_reflected_vector(gamma_right, self.__unit_reflection_right)
        # print("Reflected Right: " +str(self.__reflected_right))
        reflected_left = self.__calculate_reflected_vector(gamma_left, self.__unit_reflection_left)
        self.__ground_coordinates_right = self.__calculate_ground_coordinates(self.__intersection_points_right,
                                                                              reflected_right)
        # print("Ground Coordinates Right: " + str(self.__ground_coordinates_right))
        self.__ground_coordinates_left = self.__calculate_ground_coordinates(self.__intersection_points_left,
                                                                             reflected_left)
        self.__mirror_x_axis = np.append(self.__ground_coordinates_right[:,0],self.__ground_coordinates_left[:,0])
        self.__x_axis_plot = np.append(self.__mirror_x_axis, self.__cartesian_axis_front[:,0])
        self.__mirror_y_axis = np.append(self.__ground_coordinates_right[:,1], self.__ground_coordinates_left[:,1])
        self.__y_axis_plot = np.append(self.__mirror_y_axis, self.__cartesian_axis_front[:,1])
        self.__mirror_z_axis = np.append(self.__ground_coordinates_right[:,2], self.__ground_coordinates_left[:,2])
        self.__z_axis_plot = np.append(self.__mirror_z_axis, self.__cartesian_axis_front[:,2])

        # print("Ground Coordinates Left: ", type(self.__ground_coordinates_left))
        # print("X axis: ",type(self.__x2))


    @property
    def get_mirror_x_axis(self):
        return self.__mirror_x_axis

    @property
    def get_mirror_y_axis(self):
        return self.__mirror_y_axis

    @property
    def get_mirror_z_axis(self):
        return self.__mirror_z_axis

    @property
    def get_x_axis(self):
        return self.__x_axis_plot

    @property
    def get_y_axis(self):
        return self.__y_axis_plot

    @property
    def get_z_axis(self):
        return self.__z_axis_plot

    def view_front(self, mirror_distances_front, mirror_angles_front):

        self.__cartesian_axis_front = self.__convert_polar2cartesian(mirror_distances_front, mirror_angles_front)
        # self.__cartesian_axis_front = np.array(cartesian_axis_front)
        # print("Front: ", type(self.__cartesian_axis_front[:,0]))

    @property
    def get_ground_coordinates_right(self):
        return self.__ground_coordinates_right

    @property
    def get_ground_coordinates_left(self):
        return self.__ground_coordinates_left

    @property
    def get_cart_axis_front(self):
        return self.__cartesian_axis_front