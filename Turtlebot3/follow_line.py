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
import transforms3d
import copy as cp


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
        # Define update rate - 30 Hz
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
        # Convert quaternion into euler angles
        euler = transforms3d.euler.quat2euler(Quaternion)
        # Get the yaw angle
        self.yaw = euler[0]

    def define_line(self, p0, p1):
        # Considering the line equation Ax + By + C = 0, and calculating the constants of this equation for a line defined by 
        # two points in the world coordinate frame: (1,1) and (2,2)
        A = p1[1] - p0[1]
        B = p0[0] - p1[0]
        C = p1[0]*p0[1] - p0[0]*p1[1]
        return A, B, C

    def convertions_between_frames(self):
        # Choose points that define the line to follow
        self.p0_w = np.array([1, 0, 0])
        self.p1_w = np.array([2, 1, 0])
        # Conver them into world frame
        p0_r = self.tf.inv @ self.p0_w
        p1_r = self.tf.inv @ self.p1_w
        # Defining the line in the robot frame
        self.A_r, self.B_r, self.C_r = self.define_line(p0_r, p1_r)
    
    def get_distance_to_line(self):
        # Get distance to the line
        self.distance_to_line = self.C_r/(np.sqrt(self.A_r**2 + self.B_r **2))

    def angle_of_line(self):
        # Get angle of the line
        return np.arctan2(self.p1_w[1] - self.p0_w[1], self.p1_w[0] - self.p0_w[0]) 

    def control_turtlebot3(self, Kp_distance, Kp_orientation):
        # Create a twist message
        self.cmd_vel = Twist()
        # Assigning constant linear velocity
        self.cmd_vel.linear.x = 0.3
        # Controlling angular velocity based on angle error and distance to the line
        distance_with_gain = Kp_distance * self.distance_to_line
        if distance_with_gain > np.pi/2:
            distance_with_gain = np.pi/2
        elif distance_with_gain < -np.pi/2:
            distance_with_gain = -np.pi/2
        # Define desired angle based on the distance to the line as well
        angle_desired = self.angle_of_line() + distance_with_gain 
        # Calculate angle error
        angle_error = self.yaw - angle_desired
        # Adjusting the sign of the error depending on the shortest turning angle
        if angle_error > 0:
            if angle_error < np.pi:
                angle_error = -angle_error
            else:
                angle_error = 2*np.pi - angle_error
        else:
            if angle_error > -np.pi:
                angle_error = -angle_error
            else:
                angle_error = -(2*np.pi + angle_error)
        # print(f'Desired angle: {angle_desired}')
        # print(f'LIne angle: {self.angle_of_line()}')
        # print(f'Angle error: {angle_error}')
        # print(f'Distance to line: {self.distance_to_line}')
        # print(f'YAW: {self.yaw}')
        # Assigning angular veocity
        self.cmd_vel.angular.z = Kp_orientation * angle_error

    def pub_vel_turtlebot3(self):
        # Send velocity command to the turtlebot
        self.vel_pub.publish(self.cmd_vel)

    # Stop the turtlebot
    def stop_turtlebot3(self):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
    
    def sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Turtlebot3Controller() 
        while not rospy.is_shutdown():  
            node.convertions_between_frames()   
            node.get_distance_to_line()
            node.control_turtlebot3(1, 4)
            node.pub_vel_turtlebot3()
            node.sleep()
    except rospy.ROSInterruptException:
        pass