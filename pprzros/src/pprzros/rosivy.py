#!/usr/bin/env python
#
# Copyright (C) 2017 Guido Manfredi
#                    Gautier Hattenberger <gautier.hattenberger@enac.fr>
#
# This file is part of paparazzi.
#
# paparazzi is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi.  If not, see <http://www.gnu.org/licenses/>.
#


import sys
from os import getenv

import rospy
import rospkg
rospack = rospkg.RosPack()

PPRZ_HOME = getenv("PAPARAZZI_HOME")
if PPRZ_HOME is not None:
    sys.path.append(PPRZ_HOME + '/var/lib/python')
else:
    PPRZROS_BASE = rospack.get_path('pprzros')
    sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v2.0/python')

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

