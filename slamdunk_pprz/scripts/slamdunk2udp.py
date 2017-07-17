#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy, rospkg
import sys
from geometry_msgs.msg import PoseStamped
import math
import time
import numpy as np
from datetime import datetime
from subprocess import call
import traceback

from scapy.all import srp1,Ether,ARP,conf

rospack = rospkg.RosPack()
PPRZROS_BASE = rospack.get_path('pprzros')
sys.path.append(PPRZROS_BASE + '/../pprzlink/lib/v1.0/python')
sys.path.append(PPRZROS_BASE + '/src/pprzros')

import rosudp2
from pprzlink.message import PprzMessage

min_velocity_samples = 4
drone_id = 10

class Light(object):
    def __init__(self):
        self.hasBlinked = False

    def blink(self, n, timer, leds):
        timer /= float(1000)
        for i in range(n):
            for led in leds:
                with open("/sys/class/leds/"+led+"/brightness", "w") as bright:
                   bright.write("1")
            time.sleep(timer)
            for led in leds:
                with open("/sys/class/leds/"+led+"/brightness", "w") as bright:
                   bright.write("0")
            time.sleep(timer)
        return 0

class Drone(object):
    address = ''
    id = 0
    x = 0
    y = 0
    z = 0
    qx = 0
    qy = 0
    qz = 0
    qw = 1
    vel_x = 0
    vel_y = 0
    vel_z = 0
    speed_x = 0
    speed_y = 0
    speed_z = 0
    vel_samples = 0
    vel_transmit = 0

    def __init__(self, id):
        self.id = id

    def hasMoved(self, x, y, z, qx, qy, qz, qw):
        return (self.x != x or self.y != y or self.z != z or self.qx != qx
                or self.qy != qy or self.qz != qz or self.qw != qw)



def quaternionToEulerianAngle(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

def callback(pose):
    # print rospy.loginfo(mag)
    # print rospy.loginfo(pose)

    new_x = pose.pose.position.x * 100
    new_y = pose.pose.position.y * 100
    new_z = pose.pose.position.z * 100
    new_qx = pose.pose.orientation.x
    new_qy = pose.pose.orientation.y
    new_qz = pose.pose.orientation.z
    new_qw = pose.pose.orientation.w

    # Differentiate the position to get the speed
    if drone.hasMoved(new_x, new_y, new_z, new_qx, new_qy, new_qz, new_qw):
        drone.vel_x += new_x - drone.x
        drone.vel_y += new_y - drone.y
        drone.vel_z += new_z - drone.z
        drone.vel_samples += 1

    drone.x = new_x
    drone.y = new_y
    drone.z = new_z
    drone.qx = new_qx
    drone.qy = new_qy
    drone.qz = new_qz
    drone.qw = new_qw

    # Calculate the derivative of the sum to get the correct velocity
    drone.vel_transmit += 1
    if drone.vel_samples >= min_velocity_samples:
        sample_time = drone.vel_transmit/freq_transmit
        drone.speed_x = drone.vel_x / sample_time
        drone.speed_y = drone.vel_y / sample_time
        drone.speed_z = drone.vel_z / sample_time
        drone.vel_x = 0
        drone.vel_y = 0
        drone.vel_z = 0
        drone.vel_samples = 0
        drone.vel_transmit = 0

    # Normalized heading in rad
    heading = quaternionToEulerianAngle(drone.qx, drone.qy, drone.qz, drone.qw)[2]

    # Generating tow

    tow = np.uint32((int(time.strftime("%w"))-1) * (24 * 60 * 60 * 1000)
                    + int(time.strftime("%H")) * (60 * 60 * 1000)
                    + int(time.strftime("%M")) * (60 * 1000)
                    + int(time.strftime("%S")) * 1000
                    + int(datetime.now().microsecond / 1000))

    message = PprzMessage("datalink", "REMOTE_GPS_LOCAL")
    message.set_values([np.uint8(drone.id),
                        np.int32(drone.x),
                        np.int32(drone.y),
                        np.int32(drone.z),
                        np.int32(drone.speed_x),
                        np.int32(drone.speed_y),
                        np.int32(drone.speed_z),
                        np.uint32(tow),
                        np.int32(heading)])
    udp.interface.send(message, drone.id, drone.address)

    if not lights.hasBlinked:
        lights.blink(3, 400, ["front:left:blue", "front:right:blue", "rear:left:blue", "rear:right:blue"])
        lights.hasBlinked = True

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/pose', PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
	freq_transmit = 30.
	udp = rosudp2.RosUdpMessagesInterface()
	drone = Drone(drone_id)
	lights = Light()

	# Search drone's IP address (7.5s runtime)
	conf.verb = 0
	ans = srp1(Ether(dst="ff:ff:ff:ff:ff:ff")/ARP(pdst = '192.168.43.0/27'), timeout = 0.5, iface='usb1',inter=0.1)
	try:
		drone.address = ans[0][1].sprintf(r"%ARP.psrc%")
		lights.blink(1, 1500, ["front:left:green", "front:right:green", "rear:left:green", "rear:right:green"])
	except Exception as e:
		lights.blink(2, 1000, ["front:left:red", "front:right:red", "rear:left:red", "rear:right:red"])
		traceback.print_exc()
		exit(1)

	listener()

    

