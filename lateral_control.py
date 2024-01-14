import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import time


class LateralController:
    '''
    Lateral control using the Stanley controller

    functions:
        stanley 

    init:
        gain_constant (default=5)
        damping_constant (default=0.5)
    '''

    ################################################################################
    def __init__(self, gain_constant=1.01, damping_constant=0.0021):
    ################################################################################
        self.gain_constant = gain_constant
        self.damping_constant = damping_constant
        self.previous_steering_angle = 0


    def stanley(self, waypoints, speed):
        '''
        ##### TODO #####
        one step of the stanley controller with damping
        args:
            waypoints (np.array) [2, num_waypoints]
            speed (float)
        '''

        # derive orientation error as the angle of the first path segment to the car orientation
        # derive stanley control law
        # derive cross track error as distance between desired waypoint at spline parameter equal zero ot the car position
        # prevent division by zero by adding as small epsilon 
        # derive damping
        # clip to the maximum stering angle (0.4) and rescale the steering action space
        ################################################################################
        center_of_map = 96 / 2 
        crosstrack_error = (waypoints[0, 0] - center_of_map)
        if crosstrack_error > 1:
            crosstrack_error *= 3
        orientation_error = np.arctan((waypoints[0, 1] - waypoints[0, 0]) / (waypoints[1, 1] - waypoints[1, 0])) * 0.9
        steer_angle = orientation_error + np.arctan((self.gain_constant * crosstrack_error) / ((speed + 10)+ 0.000001))
        damping = steer_angle - self.previous_steering_angle
        steer_angle -= self.damping_constant * damping
        steer_angle = np.clip(steer_angle, -0.4, 0.4)
        self.previous_steering_angle = steer_angle
        return steer_angle
        ################################################################################