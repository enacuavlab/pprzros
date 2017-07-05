# PPRZLINK generic message

# PPRZLINK version
uint8 PPRZLINK_V10 = 1
uint8 PPRZLINK_V20 = 2

std_msgs/Header header

uint8 version
uint8 len
string class_name
uint8 comp_id
string msg_name
uint8 sender_id
uint8 receiver_id
uint16 checksum
uint8[] data

