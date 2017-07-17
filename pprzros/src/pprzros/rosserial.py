#!/usr/bin/env python

import sys
import rospy
import rospkg
rospack = rospkg.RosPack()
PPRZROS_BASE = rospack.get_path('pprzros')
sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python')

from pprzros.rosinterface import RosMessagesInterface
from pprzros_msgs.msg import PprzrosMsg
from pprzlink.serial import SerialMessagesInterface


class RosSerialMessagesInterface(RosMessagesInterface):
    def __init__(self, verbose=False, device='/dev/ttyUSB0', baudrate=115200, msg_class='telemetry'):
        self.interface = SerialMessagesInterface(self.to_ros, verbose, device, baudrate, msg_class)
        RosMessagesInterface.__init__(self)

    def from_ros(self, ros_msg):
        pprz_msg = self.converter.ros2pprz(ros_msg)
        self.interface.send(pprz_msg, ros_msg.sender_id)
    
    def to_ros(self, sender_id, pprz_msg):
        ros_msg = self.converter.pprz2ros(sender_id, pprz_msg)
        self.pub.publish(ros_msg)

def test():
    a=1

if __name__ == '__main__':
    test()

