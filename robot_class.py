from math import *
import random


"""
robot class:
 - Creates a 2D world (x,y) and a robot object ,
 - Initialize the position of the robot in the middle of the 2D world.
 - The world has some landmarks generated randomly using make landmarks
 
 Attributes:
    - measurement_noise: noise related to the measurement since it's a gaussian 
    - motion noise: noise related to the motion ,the robot can overshoot or undershoot.
    - measurement range:the sensor can only sense landmarks within the range.
    - landmarks :coordinates (x,y) of the landmarks.
    - x and y:the absolute coordinates of the robot initially in the middle of the world.
    
  Methods:

    - rand() :returns a random float between 0 and 1 usefull for giving a random value for the measured values within the 
            measurement noise.
    - sense() : iterates over landmarks in the current position returns a list of [landmark_index,dx,dy] dx and dy are the distance
            between current position and the landmark.
    - move(dx,dy) :moves the robot with distance dx,dy if that falls outside the world the robot won't move and the function 
            returns false.
    - make_landmarks(nb):creates a number of landmarks in random positions in the 2D world.
"""

class robot:

    # --------
    # init:
    #   creates a robot with the specified parameters and initializes
    #   the location (self.x, self.y) to the center of the world
    #
    def __init__(self, world_size=100.0, measurement_range=30.0,
                 motion_noise=1.0, measurement_noise=1.0):

        self.world_size = world_size
        self.measurement_range = measurement_range
        self.x = world_size / 2.0
        self.y = world_size / 2.0
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        self.landmarks = []
        self.num_landmarks = 0

    # returns a positive, random float
    def rand(self):
        return random.random() * 2.0 - 1.0

    
    def move(self, dx, dy):

        x = self.x + dx + self.rand() * self.motion_noise
        y = self.y + dy + self.rand() * self.motion_noise

        if x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
            return False
        else:
            self.x = x
            self.y = y
            return True


    def sense(self):
        ''' This function does not take in any parameters, instead it references internal variables
            (such as self.landamrks) to measure the distance between the robot and any landmarks
            that the robot can see (that are within its measurement range).
            This function returns a list of landmark indices, and the measured distances (dx, dy)
            between the robot's position and said landmarks.
            This function should account for measurement_noise and measurement_range.
            One item in the returned list should be in the form: [landmark_index, dx, dy].
            '''

        measurements = []

        for index, landmark in enumerate(self.landmarks):

            dx = self.x - landmark[0]
            dy = self.y - landmark[1]

            dx += self.rand() * self.measurement_noise
            dy += self.rand() * self.measurement_noise

            if abs(dx) < self.measurement_range and abs(dy) < self.measurement_range:
                measurements.append([index, dx, dy])

        return measurements


    def make_landmarks(self, num_landmarks):

        self.landmarks = []
        for i in range(num_landmarks):
            self.landmarks.append([round(random.random() * self.world_size),
                                   round(random.random() * self.world_size)])
        self.num_landmarks = num_landmarks

    # called when print(robot) is called; prints the robot's location
    def __repr__(self):
        return 'Robot: [x=%.5f y=%.5f]' % (self.x, self.y)

####### END robot class #######
