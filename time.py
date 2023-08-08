def time_util(vd, action, traff_state, light_time, intersection_length):
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
        return vd
    elif action == 1 and light_state == 1 or 2:
        return length/(t +  length/vd)
    elif action == 1 and light_state == 0:
        return 0








