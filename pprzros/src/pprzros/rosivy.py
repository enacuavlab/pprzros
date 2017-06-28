#!/usr/bin/env python

import sys
import threading

import rospy
import rospkg
rospack = rospkg.RosPack()
PPRZROS_BASE = rospack.get_path('pprzros')
sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python')

from pprzros.rosinterface import RosMessagesInterface
from pprzros_msgs.msg import PprzrosMsg
from pprzlink.serial import IvyMessagesInterface


class RosSerialMessagesInterface(pprzros.RosMessagesInterface):
    def __init__(self, in_ivy_bus='/dev/ttyUSB0'):
        self.interface = IvyMessagesInterface(ivy_bus=in_ivy_bus)
        self.interface.subscribe(callback=self.to_ros)

    def from_ros(self, ros_msg):
        pprz_msg = self.ros2pprz(ros_msg)
        self.interface.send(pprz_msg)
    
    def to_ros(self, sender_id, pprz_msg):
        ros_msg = self.pprz2ros(sender_id, pprz_msg)
        self.pub.publish(ros_msg)

def test():
    a=1

if __name__ == '__main__':
    test()

