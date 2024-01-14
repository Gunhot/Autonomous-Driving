import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import time
import sys


def normalize(v):
    norm = np.linalg.norm(v,axis=0) + 0.00001
    return v / norm.reshape(1, v.shape[1])

def curvature(waypoints_t):
    '''
    ##### TODO #####
    Curvature as  the sum of the normalized dot product between the way elements
    Implement second term of the smoothin objective.

    args: 
        waypoints [2, num_waypoints] !!!!!
    '''

    ################################################################################
    waypoints_t = np.transpose(waypoints_t)  
    curvature = 0

    for i in range(1, len(waypoints_t) - 1):
        diff_next = waypoints_t[i + 1] - waypoints_t[i]
        diff_prev = waypoints_t[i] - waypoints_t[i - 1]

        normalized_next = normalize(diff_next.reshape(2, 1))
        normalized_prev = normalize(diff_prev.reshape(2, 1))

        dot_product = np.dot(normalized_next[:, 0], normalized_prev[:, 0])
        curvature += dot_product
    ################################################################################

    return curvature


def smoothing_objective(waypoints, waypoints_center, weight_curvature=40):
    '''
    Objective for path smoothing

    args:
        waypoints [2 * num_waypoints] !!!!!
        waypoints_center [2 * num_waypoints] !!!!!
        weight_curvature (default=40)
    '''
    # mean least square error between waypoint and way point center
    ls_tocenter = np.mean((waypoints_center - waypoints)**2)

    # derive curvature
    curv = curvature(waypoints.reshape(2,-1))

    return -1 * weight_curvature * curv + ls_tocenter

def waypoint_prediction(roadside1_spline, roadside2_spline, num_waypoints=6, way_type = "smooth"):
    '''
    ##### TODO #####
    Predict waypoint via two different methods:
    - center
    - smooth 

    args:
        roadside1_spline
        roadside2_spline
        num_waypoints (default=6)
        parameter_bound_waypoints (default=1)
        waytype (default="smoothed")
    '''
    ls = np.linspace(0, 1, num_waypoints)
    # derive roadside points from spline
    roadside1_points = np.array(splev(ls, roadside1_spline))
    roadside2_points = np.array(splev(ls, roadside2_spline))

    # derive center between corresponding roadside points
    way_points_center = np.mean([roadside1_points, roadside2_points], axis=0)

    if way_type == "center":
        ##### TODO #####
        ################################################################################
        # print("center")
        way_points = way_points_center
        ################################################################################
    elif way_type == "smooth":
        ##### TODO #####
        ################################################################################
        # print("smooth")
        way_points_center = way_points_center.reshape(num_waypoints * 2)
        optimized_way_points = minimize(smoothing_objective, way_points_center, args=way_points_center)["x"]
        way_points = optimized_way_points.reshape(2, num_waypoints)
        ################################################################################
    return way_points


################################################################################
def target_speed_prediction(waypoints, num_waypoints_used=5,
                            max_speed=80, exp_constant=4.5, offset_speed=20):
################################################################################
    '''
    ##### TODO #####
    Predict target speed given waypoints
    Implement the function using curvature()

    args:
        waypoints [2,num_waypoints]   
        num_waypoints_used (default=5)
        max_speed (default=60)        
        exp_constant (default=4.5)    
        offset_speed (default=30)     
    
    output:
        target_speed (float)
    '''
    ################################################################################
    curv = curvature(waypoints[:, :num_waypoints_used])
    curv_cons = np.exp(-exp_constant * np.abs(num_waypoints_used - 2 - curv))
    target_speed = (max_speed - offset_speed) * curv_cons + offset_speed
    ################################################################################
    return target_speed