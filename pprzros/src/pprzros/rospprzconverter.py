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
import threading
from os import getenv

# Putting pprzlink in known path
import rospkg
rospack = rospkg.RosPack()

PPRZ_HOME = getenv("PAPARAZZI_HOME")
if PPRZ_HOME is not None:
    sys.path.append(PPRZ_HOME + '/var/lib/python')
else:
    PPRZROS_BASE = rospack.get_path('pprzros')
    sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v2.0/python')

from pprzros_msgs.msg import PprzrosMsg
from pprzlink.message import PprzMessage

class PprzRosConverter():
#    def __init__(self):
        
    def ros2pprz(self, ros_msg):      
        pprz_msg = PprzMessage(ros_msg.class_name,ros_msg.msg_name)
        pprz_msg.binary_to_payload(ros_msg.data)
        return pprz_msg

    def pprz2ros(self, sender_id, pprz_msg):
        ros_msg = PprzrosMsg()
        ros_msg.version = PprzrosMsg.PPRZLINK_V20
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

