#!/usr/bin/env python3

##----------------------------------- ME 485 - HW3 -------------------
##-------------------------------- Francisco Gonçalves -------------------
## This is the script for one the turtle nodes for Problem 3

# Import the necessary packages
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Spawn
from std_srvs.srv import Empty
import math
import time

## Define initial waypoints of parts of the face drawing
LEFT_EYE_INITIAL_WAYPOINT = [4, 7]
RIGHT_EYE_INITIAL_WAYPOINT = [7, 7]
MOUTH_INITIAL_WAYPOINT = [4.5, 4]
NOSE_INITIAL_WAYPOINT = [6, 6]
FACE_INITIAL_WAYPOINT = [6, 8.5]

# Gather the waypoints in a single list
# Added one waypoint in front of each of them to serve as orientation reference once they get there
INITIAL_WAYPOINTS_LIST = [RIGHT_EYE_INITIAL_WAYPOINT, [7, 10], NOSE_INITIAL_WAYPOINT, [6, 4], FACE_INITIAL_WAYPOINT, [7, 8.5], [1, 2]]

# Create a class for the turtle controller
class TurtleController:

    # Define an __init__ function to define initial parameters
    def __init__(self):
        
        self.clear = rospy.ServiceProxy('/clear', Empty)
        self.next_waypoint = 0
        # Initilize list of waypoints index
        self.index = -1
        # Define publisher to cmd_vel
        self.pub_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # Subscribe to pose topic
        rospy.Subscriber('/turtle1/pose', Pose, self.callback_pose)
        self.set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)
        # Create node
        rospy.init_node('turtle_controller', anonymous=True)
        # Define update rate - 10 Hz
        self.rate = rospy.Rate(10)

    # Pose callback function - grabs data from pose topic and saves it in different variables
    def callback_pose(self, data):
        self.x_position = data.x
        self.y_position = data.y
        self.orientation = data.theta

    # This function calculates errors to be used for the proportional controller (distance error and orietation error)
    def calculate_errors(self):
        # Distance to waypoint
        self.distance = math.sqrt((self.x_position - self.next_waypoint[0])**2 + (self.y_position - self.next_waypoint[1])**2)
        # Waypoint angle relative to current turtle orientation
        course = math.atan2((self.next_waypoint[1]-self.y_position), (self.next_waypoint[0]-self.x_position))
        # Orientation error: difference between current orientation and the orientation aligned with the waypoint
        self.orientation_error = course - self.orientation
        # Correcting angle so the turtle makes the smallest turn
        if self.orientation_error > math.pi:
            self.orientation_error = math.pi - self.orientation_error
    
    # Function that returns distance to waypoint
    def get_distance_to_waypoint(self):
        return self.distance

    # Function that implements a proportional controller to get linear and angular velocities commands
    def cmd_velocity(self, Kp_position, Kp_orientation):
        cmd_vel = Twist()
        cmd_vel.linear.x = Kp_position * self.distance
        cmd_vel.angular.z = Kp_orientation * self.orientation_error + self.orientation_error * self.distance
        return cmd_vel

    # Select next waypoint in the list
    def set_next_waypoint(self, initial_waypoints_list):
        self.index += 1
        self.next_waypoint = initial_waypoints_list[self.index]
        # print(self.next_waypoint)

    # Turn off the pen
    def set_pen_off(self,off,r=255,g=0,b=0,width=4):
        self.set_pen(r,g,b,width,off)

    # Clear turtlesim screen 
    def clear_screen(self):
        self.clear()

    # Publish the velocities commands 
    def run_turtle_controller(self, Kp_position, Kp_orientation):
        cmd_vel = self.cmd_velocity(Kp_position, Kp_orientation)
        self.pub_vel.publish(cmd_vel)
    
    # Path for the eye drawing motion
    def run_eye_drawing(self):
        cmd_vel = Twist()
        # While the turtle is not oriented with the reference waypoint apply only angular velocity corrections
        while abs(self.orientation_error) > 0.01:
            self.calculate_errors()           
            self.run_turtle_controller(0, 4) # Only angular gain is nonzero
        # Define a timout to stop the eye drawing
        timout = time.time() + math.pi  
        # Draw a circle          
        while time.time() < timout:      
            cmd_vel.linear.x = 1
            cmd_vel.angular.z = -2
            self.pub_vel.publish(cmd_vel)

    # Path for the mouth drawing motion
    def run_mouth_drawing(self):
        cmd_vel = Twist()
        # While the turtle is not oriented with the reference waypoint apply only angular velocity corrections
        while abs(self.orientation_error) > 0.01:
            self.calculate_errors()           
            self.run_turtle_controller(0, 4) # Only angular gain is nonzero
        # Define a timout to stop the eye drawing
        timout = time.time() + math.pi/1.7
        # Draw a semi circle
        while time.time() < timout:   
            cmd_vel.linear.x = 3
            cmd_vel.angular.z = 1.7
            self.pub_vel.publish(cmd_vel)

    # Path for the nose drawing motion
    def run_nose_drawing(self):
        cmd_vel = Twist()
        # While the turtle is not oriented with the reference waypoint apply only angular velocity corrections
        while abs(self.orientation_error) > 0.01:
            self.calculate_errors()           
            self.run_turtle_controller(0, 4) # Only angular gain is nonzero
        # Define a timout to stop the eye drawing
        timout = time.time() + 1
        # Draw a vertical straight line
        while time.time() < timout:    
            cmd_vel.linear.x = 1
            cmd_vel.angular.z = 0
            self.pub_vel.publish(cmd_vel)

    # Path for the face drawing motion
    def run_face_drawing(self):
        cmd_vel = Twist()
        # While the turtle is not oriented with the reference waypoint apply only angular velocity corrections
        while abs(self.orientation_error) > 0.01:
            self.calculate_errors()           
            self.run_turtle_controller(0, 4) # Only angular gain is nonzero
        # Define a timout to stop the eye drawing
        timout = time.time() + 2 * math.pi/0.9
        # Draw a bigger circle
        while time.time() < timout:   
            cmd_vel.linear.x = 4
            cmd_vel.angular.z = -1
            self.pub_vel.publish(cmd_vel)

    def sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
    try:
        # Define the node object       
        node_controller = TurtleController()
        # Clear the turtlesim screen
        node_controller.clear_screen()
        # While the list of waypoints is not finished
        while node_controller.index < (len(INITIAL_WAYPOINTS_LIST) - 1):
            # Turn off the pen
            node_controller.set_pen_off(True)
            # Set the next waypoint
            node_controller.set_next_waypoint(INITIAL_WAYPOINTS_LIST)
            # Calculate errors
            node_controller.calculate_errors()
            # While ROS is on and the turtle has not reached the waypoint
            while not rospy.is_shutdown() and node_controller.get_distance_to_waypoint() > 0.1:
                node_controller.calculate_errors()
                # Run the controller commands
                node_controller.run_turtle_controller(1, 1)
                node_controller.sleep()               
            # Turn on the pen
            node_controller.set_pen_off(False,r = 0, g = 255, b = 50)
            try:
                # Go to orientation reference waypoint 
                node_controller.set_next_waypoint(INITIAL_WAYPOINTS_LIST)
            except IndexError:
                pass

            node_controller.calculate_errors()
            # Run the draw paths depending on the current waypoint
            if node_controller.index == 1:
                node_controller.calculate_errors()
                node_controller.run_eye_drawing()
                node_controller.sleep()
            if node_controller.index == 3:
                node_controller.calculate_errors()
                node_controller.run_nose_drawing()
                node_controller.sleep()
            if node_controller.index == 5:
                node_controller.calculate_errors()
                node_controller.run_face_drawing()
                node_controller.sleep()

    except rospy.ROSInterruptException:
        pass

