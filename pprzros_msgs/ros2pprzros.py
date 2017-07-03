#!/usr/bin/env python

from lxml import etree
import rostopic
import rospy
import os, sys
import re
from subprocess import call
import fileinput

message_file = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                          "msg/messages.xml"))
acronyms = ["ID", "TF", "DOF"]

def xml2camelcase(string):
    return "".join(list(map((lambda elt: elt.capitalize() if elt not in acronyms else elt), string.split("_"))))

def camelcase2xml(string):
    l = re.findall('([0-9]+|[A-Z]([^A-Z]+|[A-Z]+((?=[A-Z][a-z])|(?=$)|(?=[0-9]))))', string)
    return "_".join(list(map((lambda elt: elt[0].upper()), l)))

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
                        if att_att["type"][-2:] == "[]":
                            att_att["type"] = att_att["type"][:-2]
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
                    if file != "PprzrosMsg":
                        print "  " + file + ".msg"
            good_line = False
        print line,

    call(["catkin_make", "-j8", "--pkg", "pprzros", "pprzros_msgs", "-C", "../.."])

def delete_msgs(with_files = True):
    to_delete = []
    for file in os.listdir(os.path.dirname(os.path.realpath(__file__)) + "/msg"):
        if file.endswith(".msg") and file != "PprzrosMsg.msg":
            if with_files:
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
    type = rostopic.get_topic_type(topic)
    generate_msgs(topic)
    to_import_list = []
    for file in os.listdir(os.path.dirname(os.path.realpath(__file__)) + "/msg"):
        if file.endswith(".msg"):
            to_import_list.append(file[:-4])
    delete_msgs(False)
    add_msgs(to_import_list)
    msgs = __import__('pprzros_msgs.msg', globals(), locals(), to_import_list, -1)
    if type[0].split("/")[1] != "PprzrosMsg":
        rospy.Subscriber(type[1][1:], getattr(msgs, type[0].split("/")[1]), to_PprzrosMsg)
    else:
        rospy.Subscriber(type[1], getattr(msgs, type[0].split("/")[1]), to_Ros)
    rospy.spin()
    return 0

def to_PprzrosMsg(data):
    tree = etree.parse(message_file)
    root = tree.getroot()
    os.chdir(os.path.dirname(os.path.realpath(__file__)) + "/msg")
    def rec_parser(data, parsed):
        message_name = type(data).__name__
        xml_message_name = camelcase2xml(message_name)
        for the_class in root:
            for the_message in the_class:
                att = the_message.attrib["name"]
                if xml_message_name == att:
                    for the_attribute in the_message:
                        att_att = the_attribute.attrib
                        if att_att["type"][:2] == "m_":
                            rec_parser(getattr(data, att_att["name"].lower()), parsed)
                        else:
                            if att_att["type"] == "time":
                                parsed.append(("int32", data.to_nsec()))
                            else:
                                parsed.append((att_att["type"], getattr(data, att_att["name"].lower())))
        return 0
    parsed = []
    rec_parser(data, parsed)
    print parsed
    #TODO change the "parse" list of tuples into uint8[] for data field of PprzrosMsg
    return 0

def to_Ros(data):
    print data
    #TODO ?
    return 0

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    if sys.argv[1] == "list":
        call(["rostopic", "list"])
        exit(0)
    else:
        call(["catkin", "clean", "-y"])
        subscribe(sys.argv[1])
        delete_msgs()