#!/usr/bin/env python

##----------------------------------- ME 485 - HW3 -------------------
##-------------------------------- Francisco Gonçalves -------------------
## This is the script that publishes the desire velocity for Problem 2, exercise d)

import rospy
from geometry_msgs.msg import Twist, Pose2D
import matplotlib.pyplot as plt
import time


class DesiredVelocity:

    def __init__(self):
        # Initialize x, y position, and orientation
        self.x_position = 0
        self.y_position = 0
        self.orientation = 0
        self.velocity = Twist()
        # Define the publisher
        self.pub_vel = rospy.Publisher('desired_velocity', Twist, queue_size=10)
        # Subscribe to topic 'robot_pose'
        rospy.Subscriber("robot_pose", Pose2D, self.callback_pose)
        # Create node
        rospy.init_node('node2', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

    # Define the callback function that gives current position and orientation
    def callback_pose(self, data):
        self.x_position = data.x
        self.y_position = data.y
        self.orientation = data.theta

    # Define input velocity function
    def input_velocity(self):
        self.velocity.linear.x = 1
        self.velocity.linear.y = 1
        self.velocity.angular.z = 1
        return self.velocity

    # Function that ublishes input velocity
    def run(self):
        input_velocity = self.input_velocity()
        self.pub_vel.publish(input_velocity)

    # Function that updates plot according to velocity and updated position
    def plot_pose(self):
        axs[0].plot(self.x_position, self.y_position, 'rx')
        axs[1].plot(self.x_position, self.orientation, 'bx')
        # plt.plot(self.x_position, self.y_position, 'rx')
        plt.draw()
        plt.pause(0.001)

    def sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
    # plt.axis([0,11,0,11])
    fig, axs = plt.subplots(2)
    axs[0].axis([0,11,0,11])
    axs[1].axis([0,11,0,11])
    axs[0].set(xlabel='X', ylabel='Y')
    axs[1].set(xlabel='X', ylabel='Theta')
    fig.suptitle('X, Y, and Theta')
    
    plt.ion()
    plt.show()
    try:
        # Define node
        node = DesiredVelocity()
        while not rospy.is_shutdown():
            # Publish velocity
            node.run()
            # Update plot according to velocity and updated position
            node.plot_pose()
            node.sleep()
    except rospy.ROSInterruptException:
        pass