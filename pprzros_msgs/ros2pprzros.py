#!/usr/bin/env python

from lxml import etree
import rostopic
import rospy
import os, sys
import re

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

def subscribe(topic):
    # for file in os.listdir(os.path.dirname(os.path.realpath(__file__)) + "/msg"):
    #     if file.endswith(".msg"):
    #         __import__(file[:4])
    # print sys.modules.keys()
    # rospy.Subscriber('chatter', PoseWithCovarianceStamped, callback)
    return 0

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    #parser("POSE_STAMPED")
    generate_msgs("/chatter")
    subscribe("/chatter")