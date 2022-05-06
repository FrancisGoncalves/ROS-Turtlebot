#!/usr/bin/env python

##----------------------------------- ME 485 - HW3 -------------------
##-------------------------------- Francisco Gonçalves -------------------
## This is the script for the listener example of Problem 1, exercise a)

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    # Create node
    rospy.init_node('listener', anonymous=True)
    # Subscribe to topic chatter
    rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()