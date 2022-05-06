#!/usr/bin/env python

##----------------------------------- ME 485 - Midterm Project -------------------
##------------------------------------------ Mapping part -------------------------
##-------------------------------------- Francisco Gonçalves ------------------------
## 

# 
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from sklearn.cluster import KMeans
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import os
import tf
import math


class Turtlebot3Controller:

    def __init__(self) -> None:
        # Initialize variables
        self.angle_increment = 0
        # Subscribe to laser scan messages and odom messages
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        # Create node
        rospy.init_node('turtlebot3_controller', anonymous=True)
        # Define tranform listener
        self.listener = tf.TransformListener()
        # # Define update rate - 30 Hz
        self.rate = rospy.Rate(1)

    def map_callback(self, data):
        ## Get occupancy grid; vector with values -1, 0, or 100, depending if the robot has no information, knows that nothing is there 
        ## or knows that there is an obstacle, respectively
        # Define x and y vectors containing values from 0 to 19.2 spaced by 0.05
        xv = np.arange(0, data.info.width, 1)*data.info.resolution
        yv = np.arange(0, data.info.height, 1)*data.info.resolution
        # Reshape occupancy grid from a vector to a matrix
        map = np.reshape(data.data, (data.info.width,data.info.height))
        # Rearrange x and y values information into a meshgrid
        self.X, self.Y = np.meshgrid(xv, yv)
        # Offsetting x and y values to coincide with world frame
        self.X += data.info.origin.position.x
        self.Y += data.info.origin.position.y
        # Get boundary points between known empty and unknown
        self.map_diff = np.abs(np.diff(map, axis=1))
        self.y, self.x = np.where(self.map_diff == 1)
    
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


    def kmeans_algorithm(self):
        # Get boundary points in meters
        boundaries = np.transpose(np.append([self.X[self.y, self.x]], [self.Y[self.y, self.x]], axis=0))
        # Applying kmean algorithm
        kmeans = KMeans(n_clusters=8)
        kmeans.fit(boundaries)
        # Get points belonging to each cluster
        y_kmeans = kmeans.predict(boundaries)
        # Get clusters centers location
        self.centers = kmeans.cluster_centers_
        # Get number of points belonging to each cluster
        number_points = np.bincount(y_kmeans)
        # Initiating minimum distance with large value
        minimum_distance = 1000000
        # Defining next search point based on distance to robot, and cluster dimension
        for k in range(len(self.centers)):
            # Get distance to cluster center
            distance_to_center = math.sqrt((self.trans[0] - self.centers[k][0])**2 + (self.trans[1] - self.centers[k][1])**2)
            if number_points[k] > 8 and distance_to_center > 0.5 and distance_to_center < minimum_distance:
                minimum_distance = distance_to_center
                self.next_search_waypoint = self.centers[k]
                self.cluster = k

    def movebase_client(self):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.next_search_waypoint[0]
        goal.target_pose.pose.position.y = self.next_search_waypoint[1]
        goal.target_pose.pose.orientation.w = 1.0
        client.send_goal(goal)
        print('Goal sent')
        # wait = client.wait_for_result()
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     return client.get_result()
    
    def sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Turtlebot3Controller()
        # Saving initial run time to keep track of time for saving the map 
        initial_time = time.time()
        while not rospy.is_shutdown(): 
            print('1')
            # Get robot location in the map
            node.run_node_loop()
            # Get next search point
            node.kmeans_algorithm()
            # Move robot to next search point
            node.movebase_client()
            # Save current time to keep track if the robot is moving or not
            init_time = time.time()
            init_time2 = time.time()
            # Save current position to keep track if the robot is moving or not
            init_position = node.trans
            # Calculate distance to next search point
            distance_to_waypoint = node.distance_to_waypoint(node.next_search_waypoint)
            # While the robot has not reached the next search point
            while distance_to_waypoint > 0.5: 
                print('2')
                node.run_node_loop()
                # Get distance to the initial position
                distance_to_initial_position = node.distance_to_waypoint(init_position) 
                if abs(time.time() - init_time2) > 3:
                    init_position = node.trans
                    init_time2 = time.time()                        
                # Get current position and calculate distance to the next search point
                distance_to_waypoint = node.distance_to_waypoint(node.next_search_waypoint)
                 
                print(f'Cluster: {node.cluster}')
                # If the robot did not move for more than 5 seconds choose another cluster center to go to 
                print(f'Distance to initial position: {distance_to_initial_position}')                                    
                if  distance_to_initial_position < 0.1 and abs(time.time() - init_time) > 3:
                    print('5')
                    if node.cluster < (len(node.centers) - 1):
                        print('3')
                        init_time = time.time()
                        node.cluster += 1 
                        node.run_node_loop()
                        # init_position = node.trans
                        node.next_search_waypoint = node.centers[node.cluster]
                        node.movebase_client()
                        # Get current position and calculate distance to the next search point
                        distance_to_waypoint = node.distance_to_waypoint(node.next_search_waypoint)
                    elif node.cluster == (len(node.centers) - 1):
                        print('4')
                        node.cluster = 0
                        init_time = time.time()
                        node.run_node_loop()
                        # init_position = node.trans
                        node.next_search_waypoint = node.centers[node.cluster]
                        node.movebase_client()
                        # Get current position and calculate distance to the next search point
                        distance_to_waypoint = node.distance_to_waypoint(node.next_search_waypoint)
                node.sleep()
            # Save the map every 10 seconds
            if time.time() >= initial_time + 10:
                os.system('rosrun map_server map_saver -f ~/map_francisco')
                initial_time = time.time()
            node.sleep()
    except rospy.ROSInterruptException:
        pass