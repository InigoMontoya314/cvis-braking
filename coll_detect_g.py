import numpy as np

def update_position(pos, vel, dt):
    """
    Update the car's position based on its velocity.

    Args:
    pos (tuple): Current position of the car (x, y).
    vel (tuple): Velocity of the car (u, v).
    dt (float): Time step.

    Returns:
    tuple: Updated position of the car.
    """
    return (pos[0] + vel[0]*dt, pos[1] + vel[1]*dt)

def get_corners(pos, l, w, vel):
    """
    Get the corners of the rectangle given its center, length, width, and velocity vector.

    Args:
    pos (tuple): Position of the rectangle's center (x, y).
    l (float): Length of the rectangle.
    w (float): Width of the rectangle.
    vel (tuple): Velocity vector of the rectangle (u, v).

    Returns:
    list: Coordinates of the rectangle's corners.
    """
    # The velocity vector's direction gives the orientation of the rectangle
    angle = np.arctan2(vel[1], vel[0])  # Corrected to use arctan2

    # Calculate the coordinates of the corners
    corners = [
        (pos[0] - l/2*np.cos(angle) - w/2*np.sin(angle), pos[1] - l/2*np.sin(angle) + w/2*np.cos(angle)),
        (pos[0] + l/2*np.cos(angle) - w/2*np.sin(angle), pos[1] + l/2*np.sin(angle) + w/2*np.cos(angle)),
        (pos[0] + l/2*np.cos(angle) + w/2*np.sin(angle), pos[1] + l/2*np.sin(angle) - w/2*np.cos(angle)),
        (pos[0] - l/2*np.cos(angle) + w/2*np.sin(angle), pos[1] - l/2*np.sin(angle) - w/2*np.cos(angle)),
    ]

    return corners

def projections_overlap(corners1, corners2, axis):
    """
    Check whether the projections of two rectangles on a given axis overlap.

    Args:
    corners1 (list): Coordinates of the corners of the first rectangle.
    corners2 (list): Coordinates of the corners of the second rectangle.
    axis (tuple): Coordinates of the axis (x, y).

    Returns:
    bool: True if the projections overlap, False otherwise.
    """
    # Project the corners onto the axis
    projections1 = [np.dot(corner, axis) for corner in corners1]
    projections2 = [np.dot(corner, axis) for corner in corners2]

    # Check if the projections overlap
    if max(projections1) < min(projections2) or max(projections2) < min(projections1):
        return False
    else:
        return True

def check_collision_updated(pos_d, pos_i, ld, wd, li, wi, vel_d, vel_i):
    """
    Check whether two cars (represented as rectangles) are overlapping.

    Args:
    pos_d (tuple): Position of car_d (x, y).
    pos_i (tuple): Position of car_i (x, y).
    ld (float): Length of car_d.
    wd (float): Width of car_d.
    li (float): Length of car_i.
    wi (float): Width of car_i.
    vel_d (tuple): Velocity of car_d (u, v).
    vel_i (tuple): Velocity of car_i (u, v).

    Returns:
    bool: True if the cars are overlapping (indicating a collision), False otherwise.
    """
    # Get the corners of the rectangles
    corners_d = get_corners(pos_d, ld, wd, vel_d)
    corners_i = get_corners(pos_i, li, wi, vel_i)

    # Check for overlaps along the axes defined by the rectangles' edges
    for i in range(4):
        axis_d = (corners_d[i][0] - corners_d[i-1][0], corners_d[i][1] - corners_d[i-1][1])
        axis_i = (corners_i[i][0] - corners_i[i-1][0], corners_i[i][1] - corners_i[i-1][1])

        if not (projections_overlap(corners_d, corners_i, axis_d) and projections_overlap(corners_d, corners_i, axis_i)):
            return False

    return True

def simulate_collision_updated(xd, yd, xi, yi, ud, vd, ui, vi, exi, eyi, ld, wd, wi):
    """
    Simulate the cars' movements over time and check for a collision at each time step.

    Args:
    xd (float): x-coordinate of the front middle of car_d.
    yd (float): y-coordinate of the front middle of car_d.
    xi (float): x-coordinate of the front middle of car_i.
    yi (float): y-coordinate of the front middle of car_i.
    ud (float): x-component of velocity for car_d.
    vd (float): y-component of velocity for car_d.
    ui (float): x-component of velocity for car_i.
    vi (float): y-component of velocity for car_i.
    exi (float): x-coordinate of end middle of car_i.
    eyi (float): y-coordinate of end middle of car_i.
    ld (float): Length of car_d.
    wd (float): Width of car_d.
    wi (float): Width of car_i.

    Returns:
    int: 1 if a collision is detected within 1.5 seconds, 0 otherwise.
    """
    pos_d = (xd, yd)
    pos_i = (xi, yi)
    vel_d = (ud, vd)
    vel_i = (ui, vi)

    li = np.linalg.norm(np.array([xi-exi, yi-eyi]))  # Length of car_i

    dt = 0.01  # Time step
    max_time = 1.5  # Maximum time for which to simulate

    for _ in np.arange(0, max_time, dt):
        pos_d = update_position(pos_d, vel_d, dt)
        pos_i = update_position(pos_i, vel_i, dt)

        if check_collision_updated(pos_d, pos_i, ld, wd, li, wi, vel_d, vel_i):
            return 1

    return 0

# Test cases
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



