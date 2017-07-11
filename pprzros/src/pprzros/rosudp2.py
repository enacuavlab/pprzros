#!/usr/bin/env python

# TODO
# Maintain table with sender_id/address correspondances
# Fill the whole pprz_msg (not just the binary data) ?

#CATKIN_BASE = get_env()

import sys
import threading

import rospy
import rospkg
rospack = rospkg.RosPack()
PPRZROS_BASE = rospack.get_path('pprzros')
sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python')

from pprzros.rosinterface import RosMessagesInterface
from pprzros_msgs.msg import PprzrosMsg
from pprzlink.message import PprzMessage
from pprzlink.udp import *

class RosUdpMessagesInterface(RosMessagesInterface):
    def __init__(self, address='127.0.0.1'):
        self.interface = UdpMessagesInterface(callback=self.to_ros, uplink_port=UPLINK_PORT, downlink_port=DOWNLINK_PORT, msg_class='telemetry', verbose=False)
        self.address = address
        RosMessagesInterface.__init__(self)

    def from_ros(self, ros_msg):
        pprz_msg = self.converter.ros2pprz(ros_msg)
        self.interface.send(pprz_msg, ros_msg.sender_id, self.address)
    
    def to_ros(self, sender_id, address, pprz_msg, length):
        ros_msg = self.converter.pprz2ros(sender_id, pprz_msg)
        if pprz_msg.name == 'PONG':
            print('KIKOU')
        self.pub.publish(ros_msg)

def test():
    a=1

if __name__ == '__main__':
    test()
