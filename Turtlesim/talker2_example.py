#!/usr/bin/env python

##----------------------------------- ME 485 - HW3 -------------------
##-------------------------------- Francisco Gonçalves -------------------
## This is the script for the second talker for Problem 1, exercise b)

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker2', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        talker_string = "Second talker %s" % rospy.get_time()
        rospy.loginfo(talker_string)
        pub.publish(talker_string)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass