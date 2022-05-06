#!/usr/bin/env python

##----------------------------------- ME 485 - HW3 -------------------
##-------------------------------- Francisco Gonçalves -------------------
## This is the script for the scrambler for Problem 1, exercise c)
# Subscribes to the topic 'chatter2' and prints the reversed word

import rospy
from std_msgs.msg import String

def callback(data):
    print(data.data)
    
def listener():

    # Create node
    rospy.init_node('listener_scrambler', anonymous=True)

    # Subscribe to topic
    rospy.Subscriber("chatter2", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()