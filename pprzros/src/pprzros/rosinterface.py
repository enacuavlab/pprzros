#!/usr/bin/env python

import sys
import threading

import rospy
import rospkg
rospack = rospkg.RosPack()
PPRZROS_BASE = rospack.get_path('pprzros')
sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python')

from pprzros.rospprzconverter import PprzRosConverter
from pprzros_msgs.msg import PprzrosMsg
from pprzlink.message import PprzMessage

# Abstract class, not usable as is
class RosMessagesInterface():
    def __init__(self):
        self.sub = rospy.Subscriber('from_ros', PprzrosMsg, self.from_ros)
        self.pub = rospy.Publisher('to_ros', PprzrosMsg, queue_size=10)
        self.converter = PprzRosConverter()

        rospy.init_node('rosudp_node', anonymous=True)
        self.rate = rospy.Rate(10) # 10 Hz

    def stop(self):
        print("End thread and close interface")
        self.interface.stop()

    def __del__(self):
        try:
            self.stop()
        except:
            pass

    def shutdown(self):
        self.interface.shutdown()
        
    def run(self):
        self.interface.start()
        while (not rospy.is_shutdown()) and self.interface.isAlive():      
            self.rate.sleep()
        self.interface.stop()

def test():
    a=1

if __name__ == '__main__':
    test()

