# This file contains all the required files to compute the utility of driver_d.
from random import betavariate
import math
import numpy as np
from coll_detect_g import simulate_collision_updated
import time


def saf_util(car_data_d, car_data_i_lst, action, beta):
    '''
    The function for safety utility computation, unweighted
    Args:
    car_data_d(tuple): (xd, yd, ud, vd, ld, wd)
    car_data_i_lst[list]: a list of tuples (xi, yi, ui, vi, exi, eyi, wi) for status of each car
    action(int): 0 for brake, 1 for no brake
    beta: constant factor'''
    [xd, yd, ud, vd, ld, wd] = [ _ for _ in car_data_d]
    if not car_data_i_lst:
        return 1
    if action == 0:
        ud, vd = 0, 0
    sum = 0
    speed_d = ud**2 + vd**2
    for item in car_data_i_lst:
        [xi, yi, ui, vi, exi, eyi, wi] = [ _ for _ in item]
        speed_i = ui**2 + vi**2
        ki = simulate_collision_updated(xd, yd, xi, yi, ud, vd, ui, vi, exi, eyi, ld, wd, wi)
        sum += math.exp(-(speed_i+speed_d)*ki*beta)
    return sum

def legal_util(action, traff_state, stop_line_state):
    '''Calculate the unweighted legal utility for driver_d
    Args:
    action(int): 0 brake, 1 not brake
    traff_state: 0 green, 1 red, 2 amber
    stop_line_state: 0 past stop line, 1 not past stop line
    '''
    if action == 0:
        return 1
    elif action == 1 and traff_state == 0:
        return 1
    elif action == 1 and traff_state == 1:
        return 0
    elif action == 1 and traff_state == 2:
        if stop_line_state == 0:
            return 1
        else:
            return betavariate(1.86, 1)

def time_util(speed_d, action, traff_state, light_time, intersection_length):
    '''Calculates time utility(unweighted) of driver_d.
    Args:
    vd: speed of driver_d
    action: 0 if no brake, 1 if brake
    traff_state: 0 if green, 1 if red, 2 if amber
    light_time: default 60 seconds(if the input is -1), or can be customized.
    intersection_length: legnth of the intersection.'''
    if light_time == -1:
        t = 60
    else:
        t = light_time
    if action == 0:
        return 1
    elif action == 1 and traff_state == 1 or 2:
        return (intersection_length/(t +  intersection_length/speed_d))/speed_d
    elif action == 1 and traff_state == 0:
        return 0

def comfort_util(ad, gamma, action):
    '''Calculate the comfort utility of driver_d
    Args:
    ad: acceleration of driver_d
    gamma: constant factor
    action: 0 if no brake, 1 if brake'''
    if action == 0:
        jerk = 0
    elif action == 1:
        jerk = ad/1.5
    return 1/(1+gamma*jerk**2)

weighting = np.array([1,1,1,1])
weighting_tuning = np.array([0.37, 0.37, 0.18, 0.08])


def total_util(action, weighting, car_data_d, car_data_i_lst, beta, traff_state, light_time, intersection_length, stop_line_state, ad, gamma):
    # This function uses safety_util, comfort_util, legal_util and time_util to compute total utility given the data provided. All args are defined before.
    [xd, yd, ud, vd, ld, wd] = [ _ for _ in car_data_d]
    speed_d = (ud**2+vd**2)**0.5
    if ud == vd == 0:
        return None
    raw_utility = np.array([saf_util(car_data_d, car_data_i_lst, action, beta), legal_util(action, traff_state, stop_line_state), time_util(speed_d, action, traff_state, light_time, intersection_length), comfort_util(ad, gamma, action)])
    print(raw_utility)
    return np.dot(weighting, raw_utility)
    pass

test_car_d = [0, 0, 10, 0, 4.5, 1.8]
test_car_i = []
test_traff_state = 0
test_stop_line_state = 0
test_intersection_length = 20
test_light_time = -1
test_ad = 40


print(total_util(1, weighting, test_car_d, test_car_i, 0.01, test_traff_state, test_light_time, test_intersection_length, test_stop_line_state, test_ad, 3))


def predictor(weighting, car_data_d, car_data_i_lst, beta, traff_state, light_time, intersection_length, stop_line_state, ad, gamma):
    '''return the probability of braking'''
    if ud == vd == 0: #we are not interested in the case where driver_d is already stationary
        return None
    util_0 = total_util(0, weighting, test_car_d, test_car_i, beta, test_traff_state, test_light_time, test_intersection_length, test_stop_line_state, test_ad, gamma)
    util_1 = total_util(1, weighting, test_car_d, test_car_i, beta, test_traff_state, test_light_time, test_intersection_length, test_stop_line_state, test_ad, gamma)
    prob = util_1/(util_0+util_1)
    return prob


