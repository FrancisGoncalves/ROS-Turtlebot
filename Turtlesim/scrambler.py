#!/usr/bin/env python

##----------------------------------- ME 485 - HW3 -------------------
##-------------------------------- Francisco Gonçalves -------------------
## This is the script for the scrambler for Problem 1, exercise c)
# Subscribes to the topic 'chatter', reverses the word, and publishes it to the topic 'chatter2'

import rospy
from std_msgs.msg import String

def callback_scrambler(data):
    pub = rospy.Publisher('chatter2', String, queue_size=10)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # Calculate length of the word
        string_length=len(data.data) 
        # Reverse letters order
        reversed_string=data.data[string_length::-1] 
        pub.publish(reversed_string)
        rate.sleep()

def scrambler():
    rospy.init_node('scrambler', anonymous=True)

    rospy.Subscriber("chatter", String, callback_scrambler)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        scrambler()
    except rospy.ROSInterruptException:
        pass
