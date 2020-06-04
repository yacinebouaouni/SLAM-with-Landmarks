from robot_class import robot
from math import *
import random
import numpy as np
import matplotlib.pyplot as plt


def display_world(world_size=100, position=[50,50], landmarks=[[20,20],[80,80]]):

    """
    :param world_size:The height and width of the 2D world
    :param position:Position of the robot
    :param landmarks:list of landmarks
    :return:Shows 2D world with position of the robot and landmarks
    """

    # Set dark background
    plt.style.use("dark_background")

    # Plot grid of values
    world_grid = np.zeros((world_size + 1, world_size + 1))

    # Set minor axes in between the labels
    ax = plt.gca()
    cols = world_size + 1
    rows = world_size + 1

    ax.set_xticks([x for x in range(1, cols)], minor=True)
    ax.set_yticks([y for y in range(1, rows)], minor=True)

    # Plot grid on minor axes in gray (width = 1)
    plt.grid(which='minor', ls='-', lw=1, color='red')

    # Plot grid on major axes in larger width
    plt.grid(which='major', ls='-', lw=2, color='red')

    # Create an 'o' character that represents the robot
    # ha = horizontal alignment, va = vertical
    ax.text(position[0], position[1], 'o', ha='center', va='center', color='w', fontsize=30)

    # Draw landmarks if they exists
    if (landmarks is not None):
        # loop through all path indices and draw a dot (unless it's at the car's location)
        for pos in landmarks:
            if (pos != position):
                ax.text(pos[0], pos[1], 'x', ha='center', va='center', color='white', fontsize=20)

    # Display final result
    plt.show()


# --------
# this routine makes the robot data
# the data is a list of measurements and movements: [measurements, [dx, dy]]
# collected over a specified number of time steps, N
#
def make_data(N, num_landmarks, world_size, measurement_range, motion_noise,
              measurement_noise, distance):
    """

    :param N: number of poses
    :param num_landmarks:number of landmarks
    :param world_size:size of 2D world
    :param measurement_range:(float) Minimum distance to detect landmark
    :param motion_noise:(float) noise (variance) of motion .
    :param measurement_noise :noise(variance) of measurement
    :param distance:(float) distance of one step.
    :return :Nested list axis 0 --> time stamp each time stamp contains [measurements ,[dx,dy]]

    """

    complete = False

    r = robot(world_size, measurement_range, motion_noise, measurement_noise)
    r.make_landmarks(num_landmarks)

    while not complete:

        data = []

        seen = [False for row in range(num_landmarks)]

        # guess an initial motion
        orientation = random.random() * 2.0 * pi
        dx = cos(orientation) * distance
        dy = sin(orientation) * distance

        for k in range(N - 1):

            # collect sensor measurements in a list, Z
            Z = r.sense()

            # check off all landmarks that were observed
            for i in range(len(Z)):
                seen[Z[i][0]] = True

            # move
            while not r.move(dx, dy):
                # if we'd be leaving the robot world, pick instead a new direction
                orientation = random.random() * 2.0 * pi
                dx = cos(orientation) * distance
                dy = sin(orientation) * distance

            # collect/memorize all sensor and motion data
            data.append([Z, [dx, dy]])

        # we are done when all landmarks were observed; otherwise re-run
        complete = (sum(seen) == num_landmarks)

    print(' ')
    print('Landmarks: ', r.landmarks)
    print(r)

    return data



