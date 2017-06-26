##!/usr/bin/env python

# TODO
# Connect pprzlink callback so pprz_from_uav_cb is called

#CATKIN_BASE = get_env()

import sys
import rospy
import rospkg
rospack = rospkg.RosPack()

from pprzros_msgs.msg import PprzrosMsg
import pprzros

def from_uav_test(ros_msg):
    print ros_msg.
    

def pprzros():
    test_sub = rospy.Subscriber('from_uav', PprzrosMsg, from_uav_test)
    
    rospy.init_node('pprzros_test', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz

if __name__ == '__main__':
    try:
        pprzros()
    except rospy.ROSInterruptException:
        pass

