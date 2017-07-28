#!/usr/bin/env python

import sys
import rospy
import rospkg
rospack = rospkg.RosPack()
PPRZROS_BASE = rospack.get_path('pprzros')
sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python')

from pprzros.rosinterface import RosMessagesInterface
from pprzros_msgs.msg import PprzrosMsg
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.ivy import IVY_BUS


class RosIvyMessagesInterface(RosMessagesInterface):
    def __init__(self, agent_name=None, start_ivy=True, verbose=False, ivy_bus=IVY_BUS):
        self.interface = IvyMessagesInterface(agent_name, start_ivy, verbose, ivy_bus)
        RosMessagesInterface.__init__(self)
        # only subscribe to telemetry messages eg. starting with AC_ID
        self.interface.subscribe(callback=self.to_ros, regex_or_msg='(^[0-9]+ .*)')

    def from_ros(self, ros_msg):
        pprz_msg = self.converter.ros2pprz(ros_msg)
        self.interface.send(pprz_msg)    
    
    def to_ros(self, sender_id, pprz_msg):
        ros_msg = self.converter.pprz2ros(sender_id, pprz_msg)
        self.pub.publish(ros_msg)

def test():
    a=1

if __name__ == '__main__':
    test()

