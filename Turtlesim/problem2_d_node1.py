#!/usr/bin/env python

##----------------------------------- ME 485 - HW3 -------------------
##-------------------------------- Francisco Gonçalves -------------------
## This is the script that updates pose for Problem 2, exercise d)

# license removed for brevity
import rospy
from geometry_msgs.msg import Twist, Pose2D
import time
import math

class UpdatePose:

    def __init__(self):
        self.pose = Pose2D()
        # Initialize pose variables
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = math.pi/4
        # Initialize time
        self.previous_time = time.time()
        # Define publisher
        self.pub_pose = rospy.Publisher('robot_pose', Pose2D, queue_size=10)
        # Subscribe to topic 'desired_velocity'
        rospy.Subscriber("desired_velocity", Twist, self.callback_velocity)
        # Create node
        rospy.init_node('node1', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

    # Function that updates the read input velocity
    def callback_velocity(self, data):
        self.x_linear_vel = data.linear.x
        self.y_linear_vel = data.linear.y
        self.z_angular_vel = data.angular.z
    
    # Update pose according to read velocity
    def update_pose(self):
        delta_t = time.time() - self.previous_time
        self.pose.x = self.pose.x + self.x_linear_vel * delta_t
        self.pose.y = self.pose.y + self.y_linear_vel * delta_t
        self.pose.theta = self.pose.theta + self.z_angular_vel * delta_t
        self.previous_time = time.time()
        return self.pose
    
    # Function that publishes updated pose
    def run(self):
        updated_pose = self.update_pose()
        self.pub_pose.publish(updated_pose)
    
    def sleep(self):
        self.rate.sleep()


if __name__ == '__main__':
    try:
        # Define node
        node = UpdatePose()
        while not rospy.is_shutdown():
            # Publish updated pose
            node.run()
            node.sleep()
    except rospy.ROSInterruptException:
        pass