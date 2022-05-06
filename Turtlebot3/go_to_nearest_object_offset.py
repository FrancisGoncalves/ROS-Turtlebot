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
import copy as cp

DISTANCE_TOLERANCE = 0.1
ORIENTATION_TOLERANCE = 0.1

class Turtlebot3Controller:

    def __init__(self) -> None:
        # Initialize variables
        self.ranges = np.zeros(2)
        self.angle_increment = 0
        # Subscribe to laser scan messages and odom messages
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # Create publisher to send velocity commands
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Create node
        rospy.init_node('turtlebot3_controller', anonymous=True)
        # # Define update rate - 30 Hz
        self.rate = rospy.Rate(30)

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

    def convertions_between_frames(self):
        # Defining vector position of the object in the robot frame
        location_r = np.array([self.x_robot_frame, self.y_robot_frame, 0])
        # Getting position of the object in the world frame
        location_w = self.tf @ location_r
        # print(f'World frame location: {location_w}')
        target_location_w = cp.deepcopy(location_w)
        # Offsetting the target location
        target_location_w[1] -= 0.5
        print(f'Target location in world frame: {target_location_w}')
        # Getting the target location in robot frame coordinates
        self.target_location_r = self.tf.inv @ target_location_w
        print(f'Target location in robot frame: {self.target_location_r}')

    def calculate_errors(self):
        # Get orientation error
        self.orientation_error = np.arctan2(self.target_location_r[1], self.target_location_r[0])
        # Get position error, or distance to target location
        self.distance_error = np.sqrt((self.target_location_r[0]) ** 2 + (self.target_location_r[1]) ** 2)
        # print(f'\n Distance to Target: {self.distance_error}\n')
        # print(f'\n Orientation error: {self.orientation_error}\n')

    def control_turtlebot3(self, Kp_linear, Kp_orientation):
        # Create a twist message
        self.cmd_vel = Twist()
        # Controlling linear and angular velocities based on errors
        self.cmd_vel.linear.x = Kp_linear * self.distance_error
        self.cmd_vel.angular.z = Kp_orientation * self.orientation_error + self.orientation_error * self.distance_error
    
    def pub_vel_turtlebot3(self):
        # Sending the velocity commands
        self.vel_pub.publish(self.cmd_vel)

    # Function that stops the turtlebot
    def stop_turtlebot3(self):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
    
    def sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Turtlebot3Controller() 
        while not rospy.is_shutdown():
            node.locate_closest_object() 
            node.convertions_between_frames()   
            node.calculate_errors()
            node.control_turtlebot3(0.5, 2)
            if node.distance_error < DISTANCE_TOLERANCE:   
                node.stop_turtlebot3()
            node.pub_vel_turtlebot3()
            node.sleep()
    except rospy.ROSInterruptException:
        pass