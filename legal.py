from random import betavariate

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


print(legal_util(1,0,0))

