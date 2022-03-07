import os
import time
import serial
import struct
import logging
import datetime
import threading


class MyFormatter(logging.Formatter):
    converter = datetime.datetime.fromtimestamp

    def formatTime(self, record, datefmt=None):
        ct = self.converter(record.created)
        if datefmt:
            s = ct.strftime(datefmt)
        else:
            s = ct.strftime("%Y-%m-%dT%H:%M:%S,%f")
            # s = "%s,%03d" % (t, record.msecs)
        return s


def make_logger(level):
    logger = logging.getLogger("dodobot_power_box")
    logger.setLevel(level)

    log_format = "%(levelname)s\t%(asctime)s\t[%(name)s, %(filename)s:%(lineno)d]\t%(message)s"
    formatter = MyFormatter(log_format)
    print_handle = logging.StreamHandler()
    print_handle.setLevel(level)
    print_handle.setFormatter(formatter)
    logger.addHandler(print_handle)

    return logger


logger = make_logger(logging.INFO)


class SerialInterface:
    def __init__(self, address, baud):
        self.address = address
        self.baud = baud
        self.device = None

        self.should_stop = False
        self.serial_device_paused = False

        self.read_task = threading.Thread(target=self.read_task_fn)
        self.read_thread_exception = None

        self.read_update_rate_hz = 60.0
        self.update_delay = 1.0 / self.read_update_rate_hz

        self.prev_packet_time = 0.0

        self.write_packet_num = 0
        self.read_packet_num = -1
        self.recv_packet_num = 0

        self.prev_packet_num_report_time = 0.0

        self.write_lock = threading.Lock()
        self.read_lock = threading.Lock()

        self.PACKET_START_0 = b'\x12'
        self.PACKET_START_1 = b'\x13'
        self.PACKET_STOP = b'\n'
        self.PACKET_SEP = b'\t'
        self.PACKET_SEP_STR = '\t'

        self.max_packet_len = 128
        self.max_segment_len = 64

        self.ready_state = {
            "name": "",
            "is_ready": False,
            "time_ms": 0
        }

        self.read_buffer = b''
        self.buffer_index = 0
        self.current_segment = ""
        self.current_segment_num = 0
        self.parsed_data = []
        self.recv_time = 0.0

        self.initial_connect_delay = 2.0
        self.check_ready_timeout = 10.0
        self.write_timeout = 1.0
        self.packet_read_timeout = 10.0
        self.packet_ok_timeout = 10.0

        self.ready_keyword = "dodobot"

        self.packet_error_codes = {
            0: "no error",
            1: "c1 != %s" % str(self.PACKET_START_0),
            2: "c2 != %s" % str(self.PACKET_START_1),
            3: "packet is too short",
            4: "checksums don't match",
            5: "packet count segment not found",
            6: "packet counts not synchronized",
            7: "failed to find category segment",
            8: "invalid format",
            9: "packet didn't end with stop character",
            10: "packet segment is too long",
            11: "packet receive timed out",
        }

        self.device_start_time = 0.0
        self.offset_time_ms = 0

        self.is_active = False

        self.wait_for_ok_reqs = {}

        self._sd_card_directory = {}
        self._sd_card_directory_root = ""
        self.large_packets = {}

    def start(self):
        logger.info("Starting serial interface")

        self.device = serial.Serial(
            self.address,
            baudrate=self.baud,
            timeout=5.0,
            write_timeout=5.0
        )
        time.sleep(self.initial_connect_delay)

        self.read_task.start()
        logger.info("Read thread started")
        time.sleep(1.0)

        self.check_ready()

    def process_packet(self, category):
        if category == "txrx" and self.parse_segments("du"):
            packet_num = self.parsed_data[0]
            error_code = self.parsed_data[1]

            if packet_num in self.wait_for_ok_reqs:
                self.wait_for_ok_reqs[packet_num] = error_code
                logger.debug("txrx ok_req %s: %s" % (packet_num, error_code))

            if error_code != 0:
                self.log_packet_error_code(error_code, packet_num)
            else:
                logger.debug("No error in transmitted packet #%s" % packet_num)

            # if error_code == 4:  # mismatched checksum
            #     if self.parse_next_segment("s"):
            #         logger.warn("mismatched checksum packet: %s" % self.parsed_data[2])

        elif category == "ready" and self.parse_segments("ds"):
            self.ready_state["time_ms"] = self.parsed_data[0]
            self.ready_state["name"] = self.parsed_data[1]
            self.ready_state["is_ready"] = True

            logger.info("Ready signal received! %s" % self.ready_state)
        
        else:
            self.packet_callback(category)
    
    def packet_callback(self, category):
        pass

    def check_ready(self):
        logger.info("Checking if device is ready")
        self.write("?", self.ready_keyword)

        begin_time = time.time()
        write_time = time.time()

        while not self.ready_state["is_ready"]:
            if time.time() - begin_time > self.check_ready_timeout:
                break
            if time.time() - write_time > self.write_timeout:
                logger.info("Writing ready signal again")
                self.write("?", self.ready_keyword)
                write_time = time.time()

            if self.device.inWaiting() > 2:
                self.read()

        if self.ready_state["is_ready"]:
            self.set_start_time(self.ready_state["time_ms"])
            logger.info("Serial device is ready. Device name is %s" % self.ready_state["name"])
        else:
            raise Exception("Failed to receive ready signal within %ss" % self.check_ready_timeout)

    def write_stop(self):
        self.write("!", self.ready_keyword)

    def update(self):
        if self.read_thread_exception is not None:
            logger.error("Error detected in read task. Raising exception")
            raise self.read_thread_exception

        current_time = time.time()
        if current_time - self.prev_packet_num_report_time > 60.0:
            logger.info("Packet numbers: Read: %s. Write: %s" % (
                self.read_packet_num, self.write_packet_num
            ))
            self.prev_packet_num_report_time = current_time

    def stop(self):
        if self.should_stop:
            logger.info("Stop flag already set")
            return

        logger.info("Stopping serial interface")

        self.should_stop = True
        logger.info("Set stop flag")

        self.pre_serial_stop_callback()

        time.sleep(0.1)

        self.write_stop()

        with self.read_lock:
            self.device.close()
        logger.info("Device connection closed")

    def pre_serial_stop_callback(self):
        pass

    @staticmethod
    def to_uint16_bytes(integer):
        return integer.to_bytes(2, 'big')

    @staticmethod
    def to_int32_bytes(integer):
        return integer.to_bytes(4, 'big', signed=True)

    @staticmethod
    def to_float_bytes(floating_point):
        return struct.pack('f', floating_point)

    def wait_for_ok(self, packet_num=None):
        if packet_num is None:
            packet_num = self.write_packet_num - 1
        self.wait_for_ok_reqs[packet_num] = None
        start_timer = time.time()
        while True:
            if time.time() - start_timer > self.packet_ok_timeout:
                logger.warn("Timed out while waiting for response from packet #%s" % packet_num)
                return False
            if self.wait_for_ok_reqs[packet_num] is not None:
                error_code = self.wait_for_ok_reqs.pop(packet_num)
                logger.debug("Received response for packet #%s: %s" % (packet_num, error_code))
                return error_code == 0 or error_code == 6
            time.sleep(0.01)

    def write_txrx(self, code):
        pass
        # self.write("txrx", self.read_packet_num, code)

    def write_large(self, name, destination, arg):
        assert type(arg) == str or type(arg) == bytes
        if type(arg) == str:
            arg = arg.encode()

        segments = []
        for index in range(0, len(arg), self.max_segment_len):
            offset = min(self.max_segment_len, len(arg) - index)
            segments.append(arg[index: index + offset])
        num_segments = len(segments)
        for index, segment in enumerate(segments):
            self.write(name, destination, index, num_segments, segment)
            if not self.wait_for_ok():
                logger.warn("Failed to receive ok signal on segment %s of %s. %s" % (index + 1, num_segments, name))
                break
            else:
                logger.info("Uploaded segment #%s of %s" % (index + 1, num_segments))

    def write(self, name, *args):
        self.write_lock.acquire()

        if self.serial_device_paused:
            logger.debug("Serial device is paused. Skipping write: %s, %s" % (str(name), str(args)))
            return

        packet = self.packet_header(name)
        for arg in args:
            if type(arg) == int:
                packet += self.to_int32_bytes(arg)
            elif type(arg) == float:
                packet += self.to_float_bytes(arg)
            elif type(arg) == str or type(arg) == bytes:
                assert len(arg) <= self.max_segment_len, arg
                len_bytes = self.to_uint16_bytes(len(arg))
                if type(arg) == str:
                    arg = arg.encode()
                packet += len_bytes + arg
            else:
                logger.warn("Invalid argument type: %s, %s" % (type(arg), arg))

        packet = self.packet_footer(packet)
        self._write_packet(packet)

        self.write_lock.release()

    def _write_packet(self, packet):
        logger.debug("Writing %s" % str(packet))
        try:
            self.device.write(packet)
        except BaseException as e:
            logger.error("Exception while writing packet %s: %s" % (packet, str(e)), exc_info=True)

        self.write_packet_num += 1
        time.sleep(0.0005)  # give the microcontroller a chance to not drop the next packet

    def packet_header(self, name):
        packet = self.to_int32_bytes(self.write_packet_num)
        packet += str(name).encode() + self.PACKET_SEP
        return packet

    def packet_footer(self, packet):
        calc_checksum = 0
        for val in packet:
            calc_checksum += val
        calc_checksum &= 0xff

        packet += b"%02x" % calc_checksum

        packet_len = len(packet)
        packet_len_bytes = self.to_uint16_bytes(packet_len)

        packet = self.PACKET_START_0 + self.PACKET_START_1 + packet_len_bytes + packet
        packet += self.PACKET_STOP

        return packet

    def wait_for_packet_start(self):
        begin_time = time.time()
        msg_buffer = b""
        c1 = ""
        c2 = ""

        while True:
            if time.time() - begin_time > self.packet_read_timeout:
                return False

            if self.device.inWaiting() < 2:
                continue

            c1 = self.device.read(1)
            # logger.debug("buffer: %s" % msg_buffer)
            if c1 == self.PACKET_START_0:
                c2 = self.device.read(1)
                if c2 == self.PACKET_START_1:
                    return True
            elif c1 == self.PACKET_STOP:
                logger.info("Device message: %s" % (msg_buffer))
                msg_buffer = b""
            else:
                msg_buffer += c1

    def get_next_segment(self, length=None, tab_separated=False):
        if self.buffer_index >= len(self.read_buffer):
            return False
        if tab_separated:
            sep_index = self.read_buffer.find(self.PACKET_SEP, self.buffer_index)
            if sep_index == -1:
                self.current_segment = self.read_buffer[self.buffer_index:]
                self.buffer_index = len(self.read_buffer)
            else:
                self.current_segment = self.read_buffer[self.buffer_index: sep_index]
                self.buffer_index = sep_index + 1
            return True
        else:
            if length is None:
                # assume first 2 bytes contain the length
                len_bytes = self.read_buffer[self.buffer_index: self.buffer_index + 2]
                length = int.from_bytes(len_bytes, 'big')
                self.buffer_index += 2
                if length >= len(self.read_buffer):
                    logger.error("Parsed length %s exceeds buffer length! %s" % (length, len(self.read_buffer)))
            self.current_segment = self.read_buffer[self.buffer_index: self.buffer_index + length]
            self.buffer_index += length
            return True

    def read(self):
        with self.read_lock:
            return self._read()

    def readline(self):
        begin_time = time.time()
        buffer = b""
        len_bytes = b''
        packet_len = 0
        counter = 0
        while True:
            if time.time() - begin_time > self.packet_read_timeout:
                break
            if not self.device.inWaiting():
                continue
            c = self.device.read(1)
            if len(len_bytes) < 2:
                len_bytes += c
                if len(len_bytes) == 2:
                    packet_len = int.from_bytes(len_bytes, "big")
                continue

            counter += 1
            if counter > packet_len:
                if c != self.PACKET_STOP:
                    logger.error("Packet didn't end with stop character: %s. buffer: %s" % (c, buffer))
                    return None
                break
            buffer += c
        return buffer

    def _read(self):
        # if self.serial_device_paused:
        #     logger.debug("Serial device is paused. Skipping read")
        #     return

        if not self.wait_for_packet_start():
            return False

        buffer = self.readline()
        if buffer is None:
            self.write_txrx(9)  # packet didn't end properly
            logger.error("Error encountered while reading packet. Skipping packet")
            return False

        # logger.debug("buffer: %s" % buffer)

        if len(buffer) < 5:
            logger.error("Received packet has an invalid number of characters! %s" % repr(buffer))
            self.write_txrx(3)  # packet is too short
            self.read_packet_num += 1
            return False

        calc_checksum = 0
        for val in buffer[:-2]:
            calc_checksum += val
        calc_checksum &= 0xff

        self.read_buffer = buffer
        # try:
        #     self.read_buffer = buffer.decode()
        # except UnicodeDecodeError as e:
        #     logger.error(e)
        #     logger.error("buffer: %s" % buffer)
        #     return False
        try:
            recv_checksum = int(self.read_buffer[-2:], 16)
        except ValueError as e:
            logger.error("Failed to parsed checksum as hex int: %s. %s" % (repr(self.read_buffer[-2:]), str(e)))
            self.write_txrx(4)  # checksums don't match
            self.read_packet_num += 1
            return False
        if calc_checksum != recv_checksum:
            logger.error(
                "Checksum failed! recv %02x != calc %02x. %s" % (recv_checksum, calc_checksum, repr(self.read_buffer)))
            self.write_txrx(4)  # checksums don't match
            self.read_packet_num += 1
            return False
        logger.debug("Checksum passes for %s" % str(self.read_buffer))
        self.read_buffer = self.read_buffer[:-2]

        self.buffer_index = 0

        # get packet num segment
        if not self.get_next_segment(4):
            logger.error(
                "Failed to find packet number segment %s! %s" % (repr(self.current_segment), repr(self.read_buffer)))
            self.write_txrx(5)  # packet count segment not found
            self.read_packet_num += 1
            return False
        self.recv_packet_num = int.from_bytes(self.current_segment, 'big')
        if self.read_packet_num == -1:
            self.read_packet_num = self.recv_packet_num
        if self.recv_packet_num != self.read_packet_num:
            logger.warning("Received packet num doesn't match local count. "
                           "recv %s != local %s", self.recv_packet_num, self.read_packet_num)
            logger.debug("Buffer: %s" % self.read_buffer)
            self.write_txrx(6)  # packet counts not synchronized
            self.read_packet_num = self.recv_packet_num
        logger.debug("Packet num %s passes for %s" % (self.recv_packet_num, str(self.read_buffer)))

        # find category segment
        if not self.get_next_segment(tab_separated=True):
            logger.error(
                "Failed to find category segment %s! %s" % (repr(self.current_segment), repr(self.read_buffer)))
            self.write_txrx(6)  # failed to find category segment
            self.read_packet_num += 1
            return False
        try:
            category = self.current_segment.decode()
        except UnicodeDecodeError:
            logger.error("Category segment contains invalid characters: %s, %s" % (
                repr(self.current_segment), repr(self.read_buffer)))
            self.write_txrx(6)  # failed to find category segment
            self.read_packet_num += 1
            return False
        
        if len(category) == 0:
            logger.error("Category segment is empty: %s, %s" % (
                repr(self.current_segment), repr(self.read_buffer)))
            self.write_txrx(6)  # failed to find category segment
            self.read_packet_num += 1
            return False
        
        logger.debug("Category '%s' found in %s" % (category, str(self.read_buffer)))

        try:
            self.process_packet(category)
        except BaseException as e:
            logger.error("Exception while processing packet %s" % (str(e)), exc_info=True)
            logger.error("Error packet: %s" % self.read_buffer)
            return False
        finally:
            self.write_txrx(0)  # no error
            self.read_packet_num += 1
        return True

    def parse_next_segment(self, f):
        if f == 'd' or f == 'u' or f == 'f':
            length = 4
        # elif f == 'f':
        #     length = 8
        else:
            length = None

        if not self.get_next_segment(length):
            return False

        if f == 'd' or f == 'u':
            self.parsed_data.append(int.from_bytes(self.current_segment, 'big'))
        elif f == 's' or f == 'x':
            self.parsed_data.append(self.current_segment)
        elif f == 'f':
            parsed_float = struct.unpack('f', self.current_segment)
            self.parsed_data.append(parsed_float[0])

        return True

    def parse_segments(self, formats):
        self.parsed_data = []
        self.recv_time = time.time()
        for index, f in enumerate(formats):
            if not self.parse_next_segment(f):
                logger.error("Failed to parse segment #%s. Buffer: %s" % (index, self.read_buffer))
                return False
        return True

    def read_task_fn(self):
        self.prev_packet_time = time.time()

        while True:
            time.sleep(self.update_delay)
            if self.should_stop:
                logger.info("Exiting read thread")
                return

            while self.device.inWaiting() > 0:
                self.read()

    def log_packet_error_code(self, error_code, packet_num):
        logger.warning("Packet %s returned an error:" % packet_num)
        if error_code in self.packet_error_codes:
            logger.warning("\t%s" % self.packet_error_codes[error_code])
        else:
            logger.warning("\tUnknown error code: %s" % error_code)

    def set_start_time(self, time_ms):
        self.device_start_time = time.time()
        self.offset_time_ms = time_ms

    def get_device_time(self, time_ms):
        return self.device_start_time + (time_ms - self.offset_time_ms) / 1000.0

    def pause_serial_device(self):
        logger.info("Pausing serial device")
        self.serial_device_paused = True

    def resume_serial_device(self):
        logger.info("Resuming serial device")
        self.serial_device_paused = False


class PowerBoxInterface(SerialInterface):
    def __init__(self, address, baud):
        super().__init__(address, baud)
    
    def packet_callback(self, category):
        if category == "power" and self.parse_segments("fffff"):
            # print(self.parsed_data)
            shunt_voltage = self.parsed_data[0]
            bus_voltage = self.parsed_data[1]
            current_mA = self.parsed_data[2]
            power_mW = self.parsed_data[3]
            load_voltage = self.parsed_data[4]

            logger.info(
                "\nshunt_voltage: %0.4f V\n" \
                "bus_voltage: %0.4f V\n" \
                "current_mA: %0.4f mA\n" \
                "power_mW: %0.4f mW\n" \
                "load_voltage: %0.4f V\n" % (
                    shunt_voltage,
                    bus_voltage,
                    current_mA,
                    power_mW,
                    load_voltage
                )
            )
    
    def write_large_test(self):
        data = b""
        total_checksum = 0
        for counter in range(1024):
            value = counter % 0x95
            char = int.to_bytes(value, 1, 'big', signed=False)
            data += char
            total_checksum += value
        total_checksum &= 0xff
        data += b"%02x" % total_checksum

        logger.info("Total checksum is %s" % total_checksum)

        self.write_large("large-test", "something", data)


def main():
    interface = PowerBoxInterface("/dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_018D432A-if00-port0", 38400)

    timer = time.time()
    pattern_indexer = 0
    try:
        interface.start()
        # interface.write("pix", 1)
        # interface.write_large_test()
        while True:
            interface.update()
            time.sleep(0.05)

            # if time.time() - timer > 1.0:
            #     timer = time.time()
            #     interface.write("pix", pattern_indexer)
            #     pattern_indexer += 1
            #     if pattern_indexer > 2:
            #         pattern_indexer = 0
    finally:
        interface.stop()


if __name__ == '__main__':
    main()
