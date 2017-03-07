# TODO Modify this MAVROS msg to fit PPRZ message content
# Mettre le nom du message ou au moins sa classe
# mavlink_framing_t enum
uint8 FRAMING_OK = 1
uint8 FRAMING_BAD_CRC = 2
uint8 FRAMING_BAD_SIGNATURE = 3

# stx values
uint8 MAVLINK_V10 = 254
uint8 MAVLINK_V20 = 253

std_msgs/Header header
uint8 framing_status

uint8 magic		# STX byte
uint8 len
uint8 incompat_flags
uint8 compat_flags
uint8 seq
uint8 sysid
uint8 compid
uint32 msgid		# 24-bit message id
uint16 checksum
uint64[] payload64
uint8[] signature	# optional signature
