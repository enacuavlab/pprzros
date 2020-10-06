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

import rospy
import rospkg
rospack = rospkg.RosPack()

PPRZ_HOME = getenv("PAPARAZZI_HOME")
if PPRZ_HOME is not None:
    sys.path.append(PPRZ_HOME + '/var/lib/python')
else:
    PPRZROS_BASE = rospack.get_path('pprzros')
    sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v2.0/python')

from pprzros.rospprzconverter import PprzRosConverter
from pprzros_msgs.msg import PprzrosMsg
from pprzlink.message import PprzMessage

# Abstract class, not usable as is
class RosMessagesInterface():
    def __init__(self):
        self.sub = rospy.Subscriber('pprzros/from_ros', PprzrosMsg, self.from_ros)
        self.pub = rospy.Publisher('pprzros/to_ros', PprzrosMsg, queue_size=10)
        self.converter = PprzRosConverter()

        rospy.init_node('pprzros_interface_node', anonymous=True)
        self.rate = rospy.Rate(10) # 10 Hz

    def stop(self):
        print("End thread and close interface")
        self.interface.stop()

    def __del__(self):
        try:
            self.stop()
        except:
            pass
    
    # Overlay, because IVY interface doesn't have a isAlive function (will return True)
    def isAlive(self):
        try:
            return self.interface.isAlive()
        except:
            return True

    def shutdown(self):
        self.interface.shutdown()
        
    def run(self):
        self.interface.start()
        while (not rospy.is_shutdown()) and self.isAlive():      
            self.rate.sleep()
        self.interface.stop()

def test():
    a=1

if __name__ == '__main__':
    test()

