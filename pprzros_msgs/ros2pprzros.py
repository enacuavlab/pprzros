from lxml import etree
import os

message_file = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                          "msg/messages.xml"))


def parser(message_name):
    tree = etree.parse(message_file)
    root = tree.getroot()
    spacing = ""
    def rec_parser(root, message_name, spacing):
        for the_class in root:
            for the_message in the_class:
                att = the_message.attrib["name"]
                if message_name == att:
                    for the_attribute in the_message:
                        att_att = the_attribute.attrib
                        print spacing + att_att["name"]
                        if att_att["type"][:2] == "m_":
                            rec_parser(root, att_att["type"][2:], spacing + "-")
        return 0
    rec_parser(root, message_name, spacing)

if __name__ == '__main__':
    parser("POSE_STAMPED")

