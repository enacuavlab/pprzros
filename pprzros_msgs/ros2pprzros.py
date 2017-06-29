#!/usr/bin/env python

from lxml import etree
import rostopic
import rospy
import os, sys
import re
from subprocess import call
import fileinput

from pprzros_msgs.msg import PprzrosMsg


message_file = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                          "msg/messages.xml"))

def xml2camelcase(string):
    return "".join(list(map((lambda elt: elt.capitalize()), string.split("_"))))

def camelcase2xml(string):
    l = re.findall('[A-Z][^A-Z]+', string)
    return "_".join(list(map((lambda elt: elt.upper()), l)))

def generate_msgs(topic):
    type = rostopic.get_topic_type(topic, True)
    tree = etree.parse(message_file)
    root = tree.getroot()
    os.chdir(os.path.dirname(os.path.realpath(__file__)) + "/msg")
    def rec_parser(xml_message_name):
        message_name = xml2camelcase(xml_message_name)
        msg_file = open(message_name + ".msg", 'w')
        for the_class in root:
            for the_message in the_class:
                att = the_message.attrib["name"]
                if xml_message_name == att:
                    for the_attribute in the_message:
                        att_att = the_attribute.attrib
                        if att_att["type"][:2] == "m_":
                            msg_file.write(xml2camelcase(att_att["type"][2:]) + " " + att_att["name"].lower() + "\n")
                            rec_parser(att_att["type"][2:])
                        else:
                            msg_file.write(xml2camelcase(att_att["type"]).lower() + " " + att_att["name"].lower() + "\n")
        msg_file.close()
        return 0
    message_name = type[0].split("/")[1]
    xml_message_name = camelcase2xml(message_name)
    rec_parser(xml_message_name)

def add_msgs(files):
    good_line = False

    for line in fileinput.input('../CMakeLists.txt', inplace=1):
        if "PprzrosMsg.msg" in line:
            good_line = True
        else:
            if good_line:
                for file in files:
                    print "  " + file + ".msg"
            good_line = False
        print line,

    call(["catkin_make", "-C" ,"../.."])

def delete_msgs():
    to_delete = []
    for file in os.listdir(os.path.dirname(os.path.realpath(__file__)) + "/msg"):
        if file.endswith(".msg") and file != "PprzrosMsg.msg":
            os.remove(file)
            to_delete.append("  " + file + "\n")
    f = open("../CMakeLists.txt", "r+")
    lines = f.readlines()
    f.seek(0)
    for line in lines:
        if line not in to_delete:
            f.write(line)
    f.truncate()
    f.close()

def subscribe(topic):
    to_import_list = []
    for file in os.listdir(os.path.dirname(os.path.realpath(__file__)) + "/msg"):
        if file.endswith(".msg") and file != "PprzrosMsg.msg":
            to_import_list.append(file[:-4])
    add_msgs(to_import_list)
    msgs = __import__('pprzros_msgs.msg', globals(), locals(), to_import_list, -1)
    rospy.Subscriber('chatter', msgs.PoseWithCovarianceStamped, to_PprzrosMsg)
    return 0

def to_PprzrosMsg():
    #TODO
    return 0

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    #parser("POSE_STAMPED")
    generate_msgs("/chatter")
    subscribe("/chatter")
    delete_msgs()