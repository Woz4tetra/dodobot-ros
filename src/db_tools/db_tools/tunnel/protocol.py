import os
import time
import rospy
import struct
from .result import PacketResult
from .util import *


class TunnelProtocol:
    def __init__(self):
        self.PACKET_START_0 = b'\x12'
        self.PACKET_START_1 = b'\x13'
        self.PACKET_STOP = b'\n'
        self.PACKET_SEP = b'\t'
        self.PACKET_SEP_STR = '\t'

        self.max_packet_len = 128
        self.max_segment_len = 64

        self.read_packet_num = -1
        self.recv_packet_num = 0
        self.write_packet_num = 0

        self.buffer_index = 0
        self.current_segment = b''

        self.packet_error_codes = {
            NO_ERROR: "no error",
            PACKET_0_ERROR: "c1 != %s" % str(self.PACKET_START_0),
            PACKET_1_ERROR: "c2 != %s" % str(self.PACKET_START_1),
            PACKET_TOO_SHORT_ERROR: "packet is too short",
            CHECKSUMS_DONT_MATCH_ERROR: "checksums don't match",
            PACKET_COUNT_NOT_FOUND_ERROR: "packet count segment not found",
            PACKET_COUNT_NOT_SYNCED_ERROR: "packet counts not synchronized",
            PACKET_CATEGORY_ERROR: "failed to find category segment",
            INVALID_FORMAT_ERROR: "invalid format",
            PACKET_STOP_ERROR: "packet didn't end with stop character",
            SEGMENT_TOO_LONG_ERROR: "packet segment is too long",
            PACKET_TIMEOUT_ERROR: "packet receive timed out",
        }
        self.packet_warning_codes = [
            NO_ERROR,
            PACKET_COUNT_NOT_SYNCED_ERROR,
        ]

        self.minimum_packet = self.make_packet("x")
        self.write_packet_num = 0
        self.min_packet_len = len(self.minimum_packet)

    def make_packet(self, category, *args):
        packet = self.packet_header(category)
        for arg in args:
            if type(arg) == int:
                packet += to_int32_bytes(arg)
            elif type(arg) == float:
                packet += to_float_bytes(arg)
            elif type(arg) == str or type(arg) == bytes:
                assert len(arg) <= self.max_segment_len, arg
                len_bytes = to_uint16_bytes(len(arg))
                if type(arg) == str:
                    arg = arg.encode()
                packet += len_bytes + arg
            else:
                rospy.logwarn("Invalid argument type: %s, %s" % (type(arg), arg))

        packet = self.packet_footer(packet)

        self.write_packet_num += 1
        
        return packet

    def packet_header(self, category):
        packet = to_int32_bytes(self.write_packet_num)
        packet += str(category).encode() + self.PACKET_SEP
        return packet

    def packet_footer(self, packet):
        calc_checksum = self.calculate_checksum(packet)

        packet += b"%02x" % calc_checksum

        packet_len = len(packet)
        packet_len_bytes = to_uint16_bytes(packet_len)

        packet = self.PACKET_START_0 + self.PACKET_START_1 + packet_len_bytes + packet
        packet += self.PACKET_STOP

        return packet
    
    def calculate_checksum(self, packet):
        calc_checksum = 0
        for val in packet:
            calc_checksum += val
        calc_checksum &= 0xff

        return calc_checksum
    
    def extract_checksum(self, packet):
        try:
            return int(packet[-2:], 16)
        except ValueError as e:
            rospy.logwarn("Failed to parse checksum as int: %s" % str(e))
            return -1

    def parse_buffer(self, buffer: bytes):
        rospy.logdebug("Parse buffer: %s" % str(buffer))
        results = []
        def get_char(n=1):
            nonlocal buffer
            if n > len(buffer):
                buffer = b''
                return b''
            c = buffer[0:n]
            buffer = buffer[n:]
            return c

        while True:
            if len(buffer) == 0:
                rospy.logdebug("No characters left in the buffer")
                break
            c = get_char()
            if c != self.PACKET_START_0:
                continue
            packet = c
            c = get_char()
            if c != self.PACKET_START_1:
                continue
            packet += c
            raw_length = get_char(2)
            if len(raw_length) == 0:
                rospy.logdebug("Buffer does not encode a length. Skipping")
                break
            packet += raw_length
            length = to_int(raw_length)
            rospy.logdebug("Found packet length: %s" % length)
            packet += get_char(length + 1)
            if len(packet) == 0:
                rospy.logdebug("Packet length exceeds buffer size. Skipping")
                break
            rospy.logdebug("Found packet: %s" % packet)
            if packet[-1:] != self.PACKET_STOP:
                rospy.logdebug("Packet doesn't end with PACKET_STOP. Skipping")
                break

            result = self.parse_packet(packet)
            results.append(result)
        
        return buffer, results

    def parse_packet(self, packet: bytes):
        recv_time = time.time()
        if len(packet) < self.min_packet_len:
            rospy.logwarn("Packet is not the minimum length (%s): %s" % (self.min_packet_len, repr(packet)))
            return PacketResult(PACKET_TOO_SHORT_ERROR, recv_time)
        
        if packet[0:1] != self.PACKET_START_0:
            rospy.logwarn("Packet does not start with PACKET_START_0: %s" % repr(packet))
            self.read_packet_num += 1
            return PacketResult(PACKET_0_ERROR, recv_time)
        if packet[1:2] != self.PACKET_START_1:
            rospy.logwarn("Packet does not start with PACKET_START_1: %s" % repr(packet))
            self.read_packet_num += 1
            return PacketResult(PACKET_1_ERROR, recv_time)
        if packet[-1:] != self.PACKET_STOP:
            rospy.logwarn("Packet does not stop with PACKET_STOP: %s" % repr(packet))
            self.read_packet_num += 1
            return PacketResult(PACKET_STOP_ERROR, recv_time)
        
        full_packet = packet
        packet = packet[4:-1]  # remove start, length, and stop characters
        calc_checksum = self.calculate_checksum(packet[:-2])
        recv_checksum = self.extract_checksum(packet)
        if recv_checksum != calc_checksum:
            rospy.logwarn("Checksum failed! recv %02x != calc %02x. %s" % (recv_checksum, calc_checksum, repr(full_packet)))
            self.read_packet_num += 1
            return PacketResult(CHECKSUMS_DONT_MATCH_ERROR, recv_time)
        
        packet = packet[:-2]  # remove checksum

        self.buffer_index = 0

        # get packet num segment
        if not self.get_next_segment(packet, 4):
            rospy.logwarn("Failed to find packet number segment! %s" % (repr(full_packet)))
            self.read_packet_num += 1
            return PacketResult(PACKET_COUNT_NOT_FOUND_ERROR, recv_time)
        self.recv_packet_num = to_int(self.current_segment)
        
        packet_result = PacketResult(NULL_ERROR, recv_time)

        if self.read_packet_num == -1:
            self.read_packet_num = self.recv_packet_num
        if self.recv_packet_num != self.read_packet_num:
            rospy.logwarn("Received packet num doesn't match local count. "
                           "recv %s != local %s", self.recv_packet_num, self.read_packet_num)
            rospy.logdebug("Buffer: %s" % packet)
            self.read_packet_num = self.recv_packet_num
            packet_result.set_error_code(PACKET_COUNT_NOT_SYNCED_ERROR)
        rospy.logdebug("Packet num %s passes for %s" % (self.recv_packet_num, str(packet)))

        # find category segment
        if not self.get_next_segment(packet, tab_separated=True):
            rospy.logwarn(
                "Failed to find category segment %s! %s" % (repr(self.current_segment), repr(full_packet)))
            self.read_packet_num += 1
            return PacketResult(PACKET_CATEGORY_ERROR, recv_time)
        try:
            category = self.current_segment.decode()
        except UnicodeDecodeError:
            rospy.logwarn("Category segment contains invalid characters: %s, %s" % (
                repr(self.current_segment), repr(full_packet)))
            self.read_packet_num += 1
            return PacketResult(PACKET_CATEGORY_ERROR, recv_time)
        
        if len(category) == 0:
            rospy.logwarn("Category segment is empty: %s, %s" % (
                repr(self.current_segment), repr(full_packet)))
            self.read_packet_num += 1
            return PacketResult(PACKET_CATEGORY_ERROR, recv_time)
        
        rospy.logdebug("Category '%s' found in %s" % (category, str(packet)))

        packet_result.set_start_index(self.buffer_index)
        packet_result.set_stop_index(len(packet) + 1)
        packet_result.set_buffer(packet)
        packet_result.set_error_code(NO_ERROR)
        packet_result.set_category(category)

        self.read_packet_num += 1

        return packet_result

    def get_next_segment(self, buffer, length=None, tab_separated=False):
        if self.buffer_index >= len(buffer):
            return False
        if tab_separated:
            sep_index = buffer.find(self.PACKET_SEP, self.buffer_index)
            if sep_index == -1:
                self.current_segment = buffer[self.buffer_index:]
                self.buffer_index = len(buffer)
            else:
                self.current_segment = buffer[self.buffer_index: sep_index]
                self.buffer_index = sep_index + 1
            return True
        else:
            if length is None:
                # assume first 2 bytes contain the length
                len_bytes = buffer[self.buffer_index: self.buffer_index + 2]
                length = to_int(len_bytes)
                self.buffer_index += 2
                if length >= len(buffer):
                    rospy.logerr("Parsed length %s exceeds buffer length! %s" % (length, len(buffer)))
            self.current_segment = buffer[self.buffer_index: self.buffer_index + length]
            self.buffer_index += length
            return True

    def is_code_error(self, error_code):
        return error_code in self.packet_warning_codes

    def log_packet_error_code(self, error_code, packet_num=None):
        if packet_num is None:
            packet_num = self.read_packet_num
        if error_code == NO_ERROR:
            rospy.logdebug("Packet %s has no error" % packet_num)
            return
        
        if not self.is_code_error(error_code):
            rospy.logwarn("Packet %s returned a warning:" % packet_num)
        else:
            rospy.logwarn("Packet %s returned an error:" % packet_num)
        if error_code in self.packet_error_codes:
            rospy.logwarn("\t%s" % self.packet_error_codes[error_code])
        else:
            rospy.logwarn("\tUnknown error code: %s" % error_code)

if __name__ == '__main__':
    def main():
        protocol = TunnelProtocol()

        # print(protocol.minimum_packet)
        # code, recv_time, result = protocol.parse_packet(protocol.minimum_packet, {'x': ''})
        # protocol.log_packet_error_code(code)
        # print(result)
                
        # data = "something", b"else", 2.0, 4
        # test_packet = protocol.make_packet(*data)
        # code, recv_time, result = protocol.parse_packet(test_packet, {"something": 'sfd'})
        # assert data == result, "%s != %s" % (data, result)
        # protocol.log_packet_error_code(code)
        # print(result)

        data = "ping", 4.0
        test_packet = protocol.make_packet(*data)
        print(test_packet)
        code, recv_time, result = protocol.parse_packet(test_packet, {"ping": 'f'})
        assert data == result, "%s != %s" % (data, result)
        protocol.log_packet_error_code(code)
        print(result)

        # data = "something", 50.0, 10, b"else"
        data = "cmd", 5.3, 2.1, -6.6
        test_packet = protocol.make_packet(*data)
        print(test_packet)
        code, recv_time, result = protocol.parse_packet(test_packet, {"cmd": 'fff'})
        assert data == result, "%s != %s" % (data, result)
        protocol.log_packet_error_code(code)
        print(result)


    main()
