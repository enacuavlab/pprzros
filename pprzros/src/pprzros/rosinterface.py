#!/usr/bin/env python

import sys
import threading

import rospy
import rospkg
rospack = rospkg.RosPack()
PPRZROS_BASE = rospack.get_path('pprzros')
sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python')

from pprzros_msgs.msg import PprzrosMsg
from pprzlink.message import PprzMessage

# Abstract class, not usable as is
class RosMessagesInterface():
    def __init__(self):
        self.sub = rospy.Subscriber('from_ros', PprzrosMsg, self.from_ros)
        self.pub = rospy.Publisher('to_ros', PprzrosMsg, queue_size=10)

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
        
    def ros2pprz(self, ros_msg):      
        pprz_msg = PprzMessage('datalink',ros_msg.msg_id)
        pprz_msg.binary_to_payload(ros_msg.data)
        return pprz_msg

    def pprz2ros(self, sender_id, pprz_msg):
        ros_msg = PprzrosMsg()
        ros_msg.version = PprzrosMsg.PPRZLINK_V10
        ros_msg.data = pprz_msg.payload_to_binary()
        ros_msg.len = len(ros_msg.data)
        ros_msg.class_id = 2
        ros_msg.comp_id = 0
        ros_msg.msg_id = pprz_msg.msg_id
        ros_msg.sender_id = sender_id
        ros_msg.receiver_id = 0
        return ros_msg

def test():
    a=1

if __name__ == '__main__':
    test()

