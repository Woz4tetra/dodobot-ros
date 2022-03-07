import struct


NULL_ERROR = -1
NO_ERROR = 0
PACKET_0_ERROR = 1
PACKET_1_ERROR = 2
PACKET_TOO_SHORT_ERROR = 3
CHECKSUMS_DONT_MATCH_ERROR = 4
PACKET_COUNT_NOT_FOUND_ERROR = 5
PACKET_COUNT_NOT_SYNCED_ERROR = 6
PACKET_CATEGORY_ERROR = 7
INVALID_FORMAT_ERROR = 8
PACKET_STOP_ERROR = 9
SEGMENT_TOO_LONG_ERROR = 10
PACKET_TIMEOUT_ERROR = 11


def to_uint16_bytes(integer):
    return integer.to_bytes(2, 'big')

def to_int32_bytes(integer):
    return integer.to_bytes(4, 'big', signed=True)

def to_float_bytes(floating_point):
    return struct.pack('d', floating_point)
    
def to_int(raw_bytes):
    return int.from_bytes(raw_bytes, 'big')

def to_float(raw_bytes):
    return struct.unpack('d', raw_bytes)[0]
