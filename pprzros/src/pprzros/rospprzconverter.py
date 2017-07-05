#!/usr/bin/env python

import sys
import threading

# Putting pprzlink in known path
import rospkg
rospack = rospkg.RosPack()
PPRZROS_BASE = rospack.get_path('pprzros')
sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python')

from pprzros_msgs.msg import PprzrosMsg
from pprzlink.message import PprzMessage

# Abstract class, not usable as is
class PprzRosConverter():
#    def __init__(self):
        
    def ros2pprz(self, ros_msg):      
        pprz_msg = PprzMessage(ros_msg.class_name,ros_msg.msg_name)
        pprz_msg.binary_to_payload(ros_msg.data)
        return pprz_msg

    def pprz2ros(self, sender_id, pprz_msg):
        ros_msg = PprzrosMsg()
        ros_msg.version = PprzrosMsg.PPRZLINK_V10
        ros_msg.data = pprz_msg.payload_to_binary()
        ros_msg.len = len(ros_msg.data)
        ros_msg.class_name = pprz_msg.msg_class
        ros_msg.comp_id = 0
        ros_msg.msg_name = pprz_msg.name
        ros_msg.sender_id = sender_id
        ros_msg.receiver_id = 0
        return ros_msg

def test():
    a=1

if __name__ == '__main__':
    test()

