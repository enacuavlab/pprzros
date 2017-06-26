##!/usr/bin/env python

# TODO
# Connect pprzlink callback so pprz_from_uav_cb is called

#CATKIN_BASE = get_env()

import sys
import rospy
import rospkg
rospack = rospkg.RosPack()
PPRZROS_BASE = rospack.get_path('pprzros')
#print(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python/pprzlink')
sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python')

from pprzros_msgs.msg import PprzrosMsg
from pprzlink.message import PprzMessage

#from pprzros.ros import RosMessagesInterface
from pprzlink.serial import SerialMessagesInterface
#from pprzlink.ivy import IvyMessagesInterface

# Only one supported for now
#interface_type = serial
interface = SerialMessagesInterface

def ros2pprz(ros_msg):
    pprz_msg. = ros_msg

def pprz2ros(pprz_msg):


def pprz_to_uav_cb(ros_msg):
    pprz_msg = interface.ros2pprz(ros_msg)
    interface.send(pprzmsg, 0)

def pprz_from_uav_cb(pprz_msg):
    ros_msg = interface.pprz2ros(pprz_msg)
    pprzros_pub.publish(ros_msg)

def pprzros():
    pprzros_pub = rospy.Publisher('from_uav', PprzrosMsg, queue_size=10)
    pprzros_sub = rospy.Subscriber('to_uav', PprzrosMsg, pprz_to_uav_cb)
    
    rospy.init_node('pprzros_node', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz
    
    #if (interface_type = serial)
    #    interface = SerialMessagesInterface
    #else if (interface_type = ivy)
    #    interface = IvyMessagesInterface
    #else
    #    interface = SerialMessagesInterface
    #    print("Interface type not set. Now set to serial.")
        
    
    while not rospy.is_shutdown():
        rospy.spin()
        #rate.sleep()

if __name__ == '__main__':
    try:
        pprzros()
    except rospy.ROSInterruptException:
        pass

