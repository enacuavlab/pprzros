#!/usr/bin/env python

# TODO
# Maintain table with sender_id/address correspondances

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
    def __init__(self, destination_adress='127.0.0.1', verbose=False, uplink_port=UPLINK_PORT, downlink_port=DOWNLINK_PORT, msg_class='telemetry'):
        self.interface = UdpMessagesInterface(self.to_ros, verbose, uplink_port, downlink_port, msg_class)
        self.destination_adress = destination_adress
        RosMessagesInterface.__init__(self)

    def from_ros(self, ros_msg):
        pprz_msg = self.converter.ros2pprz(ros_msg)
        self.interface.send(pprz_msg, ros_msg.sender_id, self.destination_adress)
    
    def to_ros(self, sender_id, destination_adress, pprz_msg, length):
        ros_msg = self.converter.pprz2ros(sender_id, pprz_msg)
        self.pub.publish(ros_msg)

def test():
    a=1

if __name__ == '__main__':
    test()
