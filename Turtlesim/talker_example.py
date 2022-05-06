#!/usr/bin/env python

##----------------------------------- ME 485 - HW3 -------------------
##-------------------------------- Francisco Gonçalves -------------------
## This is the script for the talker example of Problem 1, exercise a)

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        talker_string = "First talker %s" % rospy.get_time()
        rospy.loginfo(talker_string)
        pub.publish(talker_string)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass