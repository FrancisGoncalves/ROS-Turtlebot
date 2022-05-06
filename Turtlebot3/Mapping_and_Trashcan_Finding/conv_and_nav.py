#!/usr/bin/env python

##----------------------------------- ME 485 - Midterm Project -------------------
##---------------------------------------- Navigation Part ---------------------
##------------------------------------- Francisco Gonçalves -------------------
## 

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import matplotlib.pyplot as plt
import math
import tf
import time
import scipy.signal



class Turtlebot3Controller:

    def __init__(self) -> None:
        # Initialize variables
        self.idx = 0
        # Subscribe to laser scan messages and odom messages
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        # Create node
        rospy.init_node('turtlebot3_controller', anonymous=True)
        # Define tranform listener
        self.listener = tf.TransformListener()
        # Define update rate - 10 Hz
        self.rate = rospy.Rate(10)

    def map_callback(self, data):
        ## Get occupancy grid; vector with values -1, 0, or 100, depending if the robot has no information, knows that nothing is there 
        ## or knows that there is an obstacle, respectively
        # Define x and y vectors containing values from 0 to 19.2 spaced by 0.05
        xv = np.arange(0, data.info.width, 1)*data.info.resolution
        yv = np.arange(0, data.info.height, 1)*data.info.resolution
        # Reshape occupancy grid from a vector to a matrix
        self.map = np.reshape(data.data, (data.info.width,data.info.height))
        # Rearrange x and y values information into a meshgrid
        self.X, self.Y = np.meshgrid(xv, yv)
        # Offsetting x and y values to coincide with world frame
        self.X += data.info.origin.position.x
        self.Y += data.info.origin.position.y
        # Get boundary points between known empty and unknown
        self.map_diff = np.abs(np.diff(self.map, axis=1))
        self.y, self.x = np.where(self.map_diff == 1)

    # Create kernel for convolution
    def create_kernel(self):
        self.kernel = np.zeros((16, 16))
        radius = 6
        xc = 8
        yc = 8
        for angle in range(0, 360):
            x = xc + radius*np.cos(angle*np.pi/180)
            y = yc + radius*np.sin(angle*np.pi/180)
            x1 = xc + (radius + 1)*np.cos(angle*np.pi/180)
            y1 = yc + (radius + 1)*np.sin(angle*np.pi/180)
            self.kernel[int(x), int(y)] = 100
            self.kernel[int(x1), int(y1)] = 100

        while radius > 0:
            radius = radius - 1
            for angle in range(0, 360):                            
                x = xc + radius*np.cos(angle*np.pi/180)
                y = yc + radius*np.sin(angle*np.pi/180)
                self.kernel[int(x), int(y)] = -1

    def conv2d(self):
        # Do a convolution with the map and the created kernel in order to find the trash cans
        convolution = scipy.signal.convolve2d(self.map, self.kernel)
        # Find points with convolution value greater than 90% of the maximum value
        ind = np.argwhere(convolution > 0.9 * max(map(max, convolution)))
        # Pre assign list that stores points too close to others
        close_ind = []
        for i in range(len(ind) - 1):
            # print(f'i: {i}')
            for j in range(i + 1, len(ind)):
                # Calculate norm between points
                if np.linalg.norm(np.array([self.X[0, ind[i][1]], self.Y[ind[i][0], 0]]) - np.array([self.X[0, ind[j][1]], self.Y[ind[j][0], 0]])) < 0.5 and j != i:
                    # Store index of the point that is too close to another point
                    close_ind.append(j)
        # Remove repeted indeces
        close_ind_ = []
        [close_ind_.append(i) for i in close_ind if i not in close_ind_]
        # Remove points with respective indices
        for k in range(len(close_ind_)):
            ind = np.delete(ind, close_ind_[k] - k, 0)
        # Convert coordinates to meters
        self.waypoints = np.array([np.array([self.X[0, ind[0][1]], self.Y[ind[0][0], 0]])])
        for p in range(1, len(ind)):
            self.waypoints = np.append(self.waypoints, [np.array([self.X[0, ind[p][1]], self.Y[ind[p][0], 0]])], axis=0)

        # Print map with signalized trash cans locations
        plt.figure()
        plt.imshow(convolution, origin='lower')
        plt.plot(ind[:, 1],ind[:, 0], '*', color='r')
        plt.show()


    def run_node_loop(self):
        # Get robot position and orientation relatice to the map frame
        try:
            self.listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
            (self.trans,self.rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('tf listener exception')

    def distance_to_waypoint(self, point):
        # Calculate distance between 2 points
        return math.sqrt((self.trans[0] - point[0])**2 + (self.trans[1] - point[1])**2)

    def movebase_client(self, idx):
        # Offset trash cans locations for navigation purposes
        self.waypoints[idx][0] -= 0.75
        self.waypoints[idx][1] -= 1
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.waypoints[idx][0] 
        goal.target_pose.pose.position.y = self.waypoints[idx][1] 
        goal.target_pose.pose.orientation.w = 1.0
        client.send_goal(goal)
        print(f'goal sent: {self.waypoints[idx][0], self.waypoints[idx][1]}')

    
    def sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Turtlebot3Controller() 
        # initial_time = time.time()
        while not rospy.is_shutdown():  
            node.create_kernel()
            node.conv2d()
            # While there are still waypoints in the list
            while node.idx < len(node.waypoints):
                node.run_node_loop()
                # Get distance to waypoint
                distance_to_waypoint = node.distance_to_waypoint(node.waypoints[node.idx])
                # Save initial position
                init_position = node.trans
                # Drive to waypoint
                node.movebase_client(node.idx)
                # Store initial time
                init_time = time.time()
                while distance_to_waypoint > 0.1: 
                    node.run_node_loop()
                    distance_to_waypoint = node.distance_to_waypoint(node.waypoints[node.idx])
                    distance_to_initial_position = node.distance_to_waypoint(init_position)
                    # If the robot does not move for more than 4 seconds move to next waypoint                                       
                    if  distance_to_initial_position < 0.1 and abs(time.time() - init_time) > 4:
                        distance_to_waypoint = -1
                    node.sleep()
                node.idx += 1
                node.sleep()            
            node.sleep()
    except rospy.ROSInterruptException:
        pass