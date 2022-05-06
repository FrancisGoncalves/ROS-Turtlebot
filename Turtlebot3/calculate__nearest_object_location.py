#!/usr/bin/env python

##----------------------------------- ME 485 - HW5 -------------------
##-------------------------------- Francisco Gonçalves -------------------
## 

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import transform3d

class Turtlebot3Controller:

    def __init__(self) -> None:
        # Initialize variables
        self.ranges = np.zeros(2)
        self.angle_increment = 0
        # Subscribe to laser scan messages and odom messages
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # Create node
        rospy.init_node('turtlebot3_controller', anonymous=True)

    def scan_callback(self, data):
        # Get the ranges vector
        self.ranges = np.array(data.ranges)
        # Get the angle increment for each measurement
        self.angle_increment = data.angle_increment

    def odom_callback(self, data):
        # Get position of the robot in the world frame
        Position = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        # Get orientation of the robot in the world frame
        Quaternion = np.array([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        # Get homogoeneous transformation from the world to robot frame
        self.tf = transform3d.Transform(p=Position, quat=Quaternion)


    def locate_closest_object(self):
        # Get closest object distance
        minimum_distance = min(self.ranges)
        # Get closest object angle 
        angle = np.argmin(self.ranges) * self.angle_increment
        # Get X and Y coordinates in robot frame
        self.x_robot_frame = minimum_distance * np.cos(angle)
        self.y_robot_frame = minimum_distance * np.sin(angle)
        # print('The XY location of the nearest object is:\n')
        # print(f'X:{self.x_robot_frame}, Y:{self.y_robot_frame}\n')
        # print(f'Angle:{angle}, Distance:{minimum_distance}')

    def convert_to_world_frame(self):
        # Defining vector position of the object in the robot frame
        location_r = np.array([self.x_robot_frame, self.y_robot_frame, 0])
        # Getting position of the object in the world frame
        location_w = self.tf @ location_r
        print(f'World frame location: {location_w}')

    def sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Turtlebot3Controller()
        while not rospy.is_shutdown():
            node.locate_closest_object()
            node.convert_to_world_frame()
            # node.sleep()
    except rospy.ROSInterruptException:
        pass