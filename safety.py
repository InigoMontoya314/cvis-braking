from coll_detect_g import simulate_collision_updated
import math

all_tests = [
    # Existing test cases
    (0, 0, 10, 0, 10, 0, -10, 0, 0, 20, 4.5, 1.8, 1.8),
    # Additional test cases
    (0, 0, 2, 0, 0, 1, -2, 0, 0, -2, 1, 1, 1),  # Car_d stationary, car_i moving towards car_d
    (0, 0, 2, 0, 1, 0, 2, 0, 0, -2, 1, 1, 1),  # Car_d and car_i moving in the same direction, car_i is faster
    (0, 0, -2, 0, -1, 0, -2, 0, -2, -2, 1, 1, 1),  # Car_d and car_i moving in the same direction, car_d is faster
    (0, 0, 2, 2, 1, 0, 2, 0, 0, -2, 1, 1, 1),  # Car_d and car_i moving in parallel lanes in the same direction
    (0, 0, 100, 100, 50, 50, -50, -50, 0, -2, 1, 1, 1),  # Car_d and car_i stationary
]

# Apply the simulate_collision_updated function to the test cases
all_test_results_simulation_updated = [simulate_collision_updated(*test) for test in all_tests]



def saf_util(car_data_d, car_data_i_lst, action, beta):
    '''
    The function for safety utility computation, unweighted
    Args:
    car_data_d(tuple): (xd, yd, ud, vd, ld, wd)
    car_data_i_lst[list]: a list of tuples (xi, yi, ui, vi, exi, eyi, wi) for status of each car interacting with driver_d.
    action(int): 0 for brake, 1 for no brake
    beta: constant factor'''
    [xd, yd, ud, vd, ld, wd] = [ _ for _ in car_data_d]
    if action == 0:
        ud, vd = 0, 0
    sum = 0
    speed_d = ud**2 + vd**2
    for item in car_data_i_lst:
        [xi, yi, ui, vi, exi, eyi, wi] = [ _ for _ in item]
        speed_i = ui**2 + vi**2
        ki = simulate_collision_updated(xd, yd, xi, yi, ud, vd, ui, vi, exi, eyi, ld, wd, wi)
        print(ki)
        sum += math.exp(-(speed_i+speed_d)*ki*beta)
    return sum

test_d = [0,0,5,6,11,12]
test_i = [(3,4,7,8,9,10,13)]

print(saf_util(test_d,test_i, 1, 0.01))

print(saf_util(test_d,test_i, 0, 0.01))



