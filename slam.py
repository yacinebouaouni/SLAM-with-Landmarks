import numpy as np


def initialize_constraints(N, num_landmarks, world_size):

    """
    :param N: number of poses (timestamps)
    :param num_landmarks: number of landmarks
    :param world_size: the size of the 2D world
    :return: omega,xi initialized constraint matrix and vector
    """
    matrix_size = 2 *(N + num_landmarks)

    # for the initial x, y location of our robot
    omega =np.zeros((matrix_size, matrix_size))
    omega[0][0] = 1  # first x position
    omega[1][1] = 1  # first y position

    # assume that the robot starts out in the middle of the world with 100% confidence
    xi = np.zeros((matrix_size, 1))
    xi[0] = world_size / 2
    xi[1] = world_size / 2

    return omega, xi

# ----------------------------------------------------------------------------------------------------------------------


def update(omega,xi,old_x,new_x,strength,value_x,value_y):

    """
    :param omega: Constraint matrix
    :param xi: constraint vector
    :param old_x: the index of the first point x coordinate
    :param new_x: the index of the new point x coordinate
    :param strength: the strength of the value (1/variance) or (1/noise)
    :param value_x: represents either the motion_x or the measurement_x (distance)
    :param value_y: represents either the motion_y or the measurement_y (distance)
    :return:updated omega and xi
    """

    # y_index come right after x_index
    old_y = old_x+1
    new_y = new_x+1

#  <<UPDATE THE X MOTION CONSTRAINTS>>
    omega[old_x][old_x] += strength
    omega[new_x][new_x] += strength
    omega[old_x][new_x] += -strength
    omega[new_x][old_x] += -strength

#  <<UPDATE THE Y MOTION CONSTRAINTS>>
    omega[old_y][old_y] += strength
    omega[new_y][new_y] += strength
    omega[old_y][new_y] += -strength
    omega[new_y][old_y] += -strength

#  <<UPDATE THE Xi Vector with motion_x and motion_y>>
    xi[old_x][0] += -value_x
    xi[old_y][0] += -value_y
    xi[new_x][0] += value_x
    xi[new_y][0] += value_y

    return omega,xi

# ----------------------------------------------------------------------------------------------------------------------


def slam(data, N, num_landmarks, world_size, motion_noise, measurement_noise):

    # Initialize constraint matrix and vector:
    omega, xi = initialize_constraints(N, num_landmarks, world_size)

    # Iterate over timestamps and get motion and measurements then update the constraint matrix and vector
    for index in range(len(data)):

        measurement_data = data[index][0]
        motion = data[index][1]

        # Compute motion data according to their variance(noise)
        motion_strength = 1.0 / motion_noise
        motion_x = motion_strength * motion[0]
        motion_y = motion_strength * motion[1]

        # Index of old point and new one (x coordinate the y is in update)
        old_x_idx = 2 * index
        new_x_idx = old_x_idx + 2

        # Update constraint matrix and vector
        omega, xi = update(omega, xi, old_x_idx, new_x_idx, motion_strength, motion_x, motion_y)

        # Compute the measurement strength
        measure_strength = 1.0 / measurement_noise

        # In the present position iterate over the different measurements
        for measure in measurement_data:

            L = measure[0]
            measure_x = measure_strength * measure[1]
            measure_y = measure_strength * measure[2]

            # Compute the new index of landmark in the constraint matrix
            # The matrix has N poses so 2*N corresponds to (x,y) of each pose.
            # starting from 2*N+2 until 2*(N+num_landmarks) we have the landmarks.

            index_lx = 2 * (len(data) + 1) + 2 * L
            omega, xi = update(omega, xi, old_x_idx, index_lx, measure_strength, measure_x, measure_y)

    omega_inv = np.linalg.inv(np.matrix(omega))

    # Compute mu the vector that contains absolute coordinate for poses and landmarks
    mu = omega_inv * xi

    return mu

# -----------------------------------------------------------------------------------------------------------------------


def get_poses_landmarks(mu, N,num_landmarks):

    """
    This function return two lists of poses and landmarks
    :param mu: The result vector of absolute values
    :param N: Number of timestamps
    :param num_landmarks:number of landmarks
    :return: poses,landmarks
    """

    # create a list of poses
    poses = []
    for i in range(N):
        poses.append((mu[2*i].item(), mu[2*i+1].item()))

    # create a list of landmarks
    landmarks = []
    for i in range(num_landmarks):
        landmarks.append((mu[2*(N+i)].item(), mu[2*(N+i)+1].item()))

    # return completed lists
    return poses, landmarks

# -----------------------------------------------------------------------------------------------------------------------


def print_all(poses, landmarks):
    """"
    This function prints the poses and landmarks
    """

    print('\n')
    print('Estimated Poses:')
    for i in range(len(poses)):
        print('['+', '.join('%.3f'%p for p in poses[i])+']')
    print('\n')
    print('Estimated Landmarks:')
    for i in range(len(landmarks)):
        print('['+', '.join('%.3f'%l for l in landmarks[i])+']')


