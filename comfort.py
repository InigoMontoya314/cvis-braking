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