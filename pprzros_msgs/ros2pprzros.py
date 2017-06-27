from lxml import etree
import os

message_file = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                          "../msg/messages.xml"))

def creator(message_name):
    tree = etree.parse(message_file)
