import time
import struct
import serial
import datetime
import threading

from .device_port import DevicePort
from .battery_state import BatteryState
from .task import Task
from ..node import Node
from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from lib.exceptions import *

device_port_config = ConfigManager.get_device_port_config()
robot_config = ConfigManager.get_robot_config()
logger = LoggerManager.get_logger()


class Robot(Node):
    def __init__(self, session):
        super(Robot, self).__init__(session)

        self.device = DevicePort(
            device_port_config.address,
            device_port_config.baud_rate,
            device_port_config.timeout,
            device_port_config.write_timeout
        )

        self.should_stop = False
        self.serial_device_paused = False

        self.read_task = Task(self.read_task_fn)
        self.write_date_task = Task(self.write_date_task_fn)
        self.write_date_delay = robot_config.write_date_delay

        self.read_update_rate_hz = device_port_config.update_rate_hz
        self.update_delay = 1.0 / self.read_update_rate_hz

        self.prev_packet_time = 0.0

        self.write_packet_num = 0
        self.read_packet_num = -1
        self.recv_packet_num = 0

        self.prev_packet_num_report_time = 0.0

        self.write_lock = threading.Lock()
        self.read_lock = threading.Lock()

        self.PACKET_START_0 = b'\x12'
        self.PACKET_START_1 = b'\x34'
        self.PACKET_STOP = b'\n'
        self.PACKET_SEP = b'\t'
        self.PACKET_SEP_STR = b'\t'

        self.large_packet_len = 0x1000

        self.ready_state = {
            "name"    : "",
            "is_ready": False,
            "time_ms" : 0
        }

        self.power_state = {
            "recv_time"     : 0.0,
            "current_mA"    : 0.0,
            "power_mW"      : 0.0,
            "load_voltage_V": 0.0
        }

        self.robot_state = {
            "recv_time"    : 0.0,
            "is_active"    : False,
            "battery_ok"   : True,
            "motors_active": False,
            "loop_rate"    : 0.0,
            "free_mem"     : 0,
        }

        self.battery_state = BatteryState()

        self.read_buffer = ""
        self.buffer_index = 0
        self.current_segment = ""
        self.current_segment_num = 0
        self.parsed_data = []
        self.recv_time = 0.0

        self.check_ready_timeout = robot_config.check_ready_timeout
        self.write_timeout = robot_config.write_timeout
        self.packet_read_timeout = robot_config.packet_read_timeout

        self.packet_error_codes = {
            0: "no error",
            1: "c1 != \\x12",
            2: "c2 != \\x34",
            3: "packet is too short",
            4: "checksums don't match",
            5: "packet count segment not found",
            6: "packet counts not synchronized",
            7: "failed to find category segment",
            8: "invalid format",
            9: "packet didn't end with stop character",
        }

        self.device_start_time = 0.0
        self.offset_time_ms = 0

        self.is_active = False

        self.shutdown_timer = 0.0
        self.shutdown_starting = False
        self.prev_display_countdown = None
        self.shutdown_time_limit = robot_config.shutdown_time_limit

        self.wait_for_ok_reqs = {}
        self.packet_ok_timeout = 1.0

    def start(self):
        logger.info("Starting rover client")

        self.device.configure()
        logger.info("Device configured")

        self.read_task.start()
        logger.info("Read thread started")
        time.sleep(1.0)

        self.check_ready()
        self.prev_command_time = time.time()

        self.set_reporting(True)

        self.write_date_task.start()

    def process_packet(self, category):
        if category == "txrx" and self.parse_segments("dd"):
            packet_num = self.parsed_data[0]
            error_code = self.parsed_data[1]

            if packet_num in self.wait_for_ok_reqs:
                self.wait_for_ok_reqs[packet_num] = error_code
                logger.info("txrx ok_req %s: %s" % (packet_num, error_code))

            if error_code != 0:
                self.log_packet_error_code(error_code, packet_num)
            else:
                logger.debug("No error in transmitted packet #%s" % packet_num)

            if error_code == 4:  # mismatched checksum
                if self.parse_next_segment("s"):
                    logger.warn("mismatched checksum packet: %s" % self.parsed_data[2])

        elif category == "ready" and self.parse_segments("ds"):
            self.ready_state["time_ms"] = self.parsed_data[0]
            self.ready_state["name"] = self.parsed_data[1]
            self.ready_state["is_ready"] = True

            logger.info("Ready signal received! %s" % self.ready_state)

        elif category == "batt" and self.parse_segments("ufff"):
            self.power_state["recv_time"] = self.get_device_time(self.parsed_data[0])
            self.power_state["current_mA"] = self.parsed_data[1]
            self.power_state["power_mW"] = self.parsed_data[2]
            self.power_state["load_voltage_V"] = self.parsed_data[3]
            logger.debug("power_state: %s" % str(self.power_state))
            state_changed = self.battery_state.set(self.power_state)
            if state_changed:
                self.battery_state.log_state()
            if self.battery_state.should_shutdown():
                raise LowBatteryException(
                    "Battery is critically low: {load_voltage_V:0.2f}!! Shutting down.".format(**self.power_state))

        elif category == "state" and self.parse_segments("uddfu"):
            self.robot_state["recv_time"] = self.get_device_time(self.parsed_data[0])
            self.robot_state["battery_ok"] = self.parsed_data[1]
            self.robot_state["motors_active"] = self.parsed_data[2]
            self.robot_state["loop_rate"] = self.parsed_data[3]
            self.robot_state["free_mem"] = self.parsed_data[4]

        elif category == "latch_btn" and self.parse_segments("ud"):
            button_state = self.parsed_data[1]
            if button_state == 1:
                self.start_shutdown()
            else:
                self.cancel_shutdown()

        elif category == "unlatch" and self.parse_segments("u"):
            logger.warning("Unlatch signal received! System unlatching soon.")

        elif category == "control" and self.parse_segments("sd"):
            name = self.parsed_data[0]
            control_type = self.parsed_data[1]
            if name == b"dodobot":
                if control_type == 0:
                    logger.info("'Shutdown now' signal received")
                    raise ShutdownException
                elif control_type == 1:
                    logger.info("'Reboot' signal received")
                    raise RebootException
                elif control_type == 2:
                    logger.info("'Restart ROS' signal received")
                elif control_type == 3:
                    logger.info("'Restart client' signal received")
                    raise RelaunchException
                elif control_type == 4:
                    logger.info("'Restart microcontroller' signal received")
                    raise DeviceRestartException
            else:
                logger.warning("Received name doesn't match expected: %s" % name)

    def start_shutdown(self):
        logger.info("Starting shutdown timer")
        self.shutdown_timer = time.time()
        self.shutdown_starting = True

    def cancel_shutdown(self):
        logger.info("Canceling shutdown")
        self.shutdown_starting = False
        self.prev_display_countdown = None

    def check_shutdown_timer(self):
        if not self.shutdown_starting:
            return

        current_time = time.time()
        countdown_time = self.shutdown_time_limit - (current_time - self.shutdown_timer)
        countdown_time_int = int(countdown_time) + 1
        if countdown_time_int != self.prev_display_countdown:
            logger.info("%s..." % countdown_time_int)
            self.prev_display_countdown = countdown_time_int
        if countdown_time <= 0.0:
            logger.info("Shutting down")
            self.write_shutdown_signal()
            time.sleep(0.15)
            raise ShutdownException

    def write_shutdown_signal(self):
        self.write("shutdown", "dodobot")

    def set_reporting(self, state):
        self.write("[]", 1 if state else 0)

    def set_active(self, state):
        self.is_active = state
        self.write("<>", 1 if state else 0)

    def write_date(self):
        date_str = datetime.datetime.now().strftime("%I:%M:%S%p")
        self.write("date", date_str)

    def write_date_task_fn(self, should_stop):
        failed_write_attempts = 0
        while True:
            if should_stop():
                return
            time.sleep(self.write_date_delay)
            if self.serial_device_paused:
                continue
            try:
                self.write_date()
                failed_write_attempts = 0
            except serial.SerialTimeoutException as e:
                failed_write_attempts += 1
                if failed_write_attempts >= 10:
                    return e

    def check_ready(self):
        self.write("?", "dodobot")

        begin_time = time.time()
        write_time = time.time()

        while not self.ready_state["is_ready"]:
            if time.time() - begin_time > self.check_ready_timeout:
                break
            if time.time() - write_time > self.write_timeout:
                logger.info("Writing ready signal again")
                self.write("?", "dodobot")
                write_time = time.time()

            if self.device.in_waiting() > 2:
                self.read()

        if self.ready_state["is_ready"]:
            self.set_start_time(self.ready_state["time_ms"])
            logger.info("Serial device is ready. Robot name is %s" % self.ready_state["name"])
        else:
            raise DeviceNotReadyException("Failed to receive ready signal within %ss" % self.check_ready_timeout)

    def update(self):
        if not self.read_task.is_errored():
            logger.error("Error detected in read task. Raising exception")
            raise self.read_task.thread_exception

        if not self.write_date_task.is_errored():
            logger.error("Error detected in write date task. Raising exception")
            raise self.write_date_task.thread_exception

        self.check_shutdown_timer()

        current_time = time.time()
        if current_time - self.prev_packet_num_report_time > 60.0:
            logger.info("Packet numbers: Read: %s. Write: %s | Free memory: %s" % (
                self.read_packet_num, self.write_packet_num, self.robot_state["free_mem"]
            ))
            self.prev_packet_num_report_time = current_time

    def stop(self):
        if self.should_stop:
            logger.info("Stop flag already set")
            return

        logger.info("Stopping robot client")

        self.should_stop = True
        logger.info("Set stop flag")

        self.read_task.stop()
        self.write_date_task.stop()

        self.pre_serial_stop_callback()

        self.set_reporting(False)
        self.set_active(False)

        time.sleep(0.1)

        with self.read_lock:
            self.device.stop()
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
                logger.info("Received response for packet #%s: %s" % (packet_num, error_code))
                return error_code == 0 or error_code == 6
            time.sleep(0.01)

    def write_large(self, name, arg):
        assert type(arg) == str or type(arg) == bytes
        if type(arg) == str:
            arg = arg.encode()

        segments = []
        for index in range(0, len(arg), self.large_packet_len):
            offset = min(self.large_packet_len, len(arg) - index)
            segments.append(arg[index: index + offset])
        num_segments = len(segments)
        for index, segment in enumerate(segments):
            self.write(name, index, num_segments, segment)
            if not self.wait_for_ok():
                logger.warn("Failed to receive ok signal on segment %s of %s. %s" % (index + 1, num_segments, name))
                break

    def write(self, name, *args):
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
                assert len(arg) <= self.large_packet_len, arg
                len_bytes = self.to_uint16_bytes(len(arg))
                if type(arg) == str:
                    arg = arg.encode()
                packet += len_bytes + arg
            else:
                logger.warn("Invalid argument type: %s, %s" % (type(arg), arg))

        packet = self.packet_footer(packet)
        self._write_packet(packet)

    def _write_packet(self, packet):
        with self.write_lock:
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

            if self.device.in_waiting() < 2:
                continue

            c1 = self.device.read(1)
            # logger.info("buffer: %s" % msg_buffer)
            if c1 == self.PACKET_START_0:
                c2 = self.device.read(1)
                if c2 == self.PACKET_START_1:
                    return True
            elif c1 == self.PACKET_STOP:
                # time.sleep(self.update_delay)
                logger.info("Device message: %s" % (msg_buffer))
                msg_buffer = b""
            else:
                msg_buffer += c1

    def get_next_segment(self, length=None, tab_separated=False):
        if self.buffer_index >= len(self.read_buffer):
            return False
        if tab_separated:
            sep_index = self.read_buffer.find(self.PACKET_SEP_STR, self.buffer_index)
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
                    logger.error("Parsed length %s exceeds buffer length! %s" % (length, self.read_buffer))
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
            if not self.device.in_waiting():
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
            logger.error("Error encountered while reading packet. Skipping packet")
            return False

        logger.debug("buffer: %s" % buffer)

        if len(buffer) < 5:
            logger.error("Received packet has an invalid number of characters! %s" % repr(buffer))
            self.read_packet_num += 1
            return False

        calc_checksum = 0
        for val in buffer[:-2]:
            calc_checksum += val
        calc_checksum &= 255

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
            logger.error("Failed to parsed checksum as hex int: %s" % repr(self.read_buffer))
            self.read_packet_num += 1
            return False
        if calc_checksum != recv_checksum:
            logger.error(
                "Checksum failed! recv %02x != calc %02x. %s" % (recv_checksum, calc_checksum, repr(self.read_buffer)))
            self.read_packet_num += 1
            return False
        self.read_buffer = self.read_buffer[:-2]

        self.buffer_index = 0

        # get packet num segment
        if not self.get_next_segment(4):
            logger.error(
                "Failed to find packet number segment %s! %s" % (repr(self.current_segment), repr(self.read_buffer)))
            self.read_packet_num += 1
            return False
        self.recv_packet_num = int.from_bytes(self.current_segment, 'big')
        if self.read_packet_num == -1:
            self.read_packet_num = self.recv_packet_num
        if self.recv_packet_num != self.read_packet_num:
            logger.warning("Received packet num doesn't match local count. "
                           "recv %s != local %s", self.recv_packet_num, self.read_packet_num)
            logger.debug("Buffer: %s" % self.read_buffer)
            self.read_packet_num = self.recv_packet_num

        # find category segment
        if not self.get_next_segment(tab_separated=True):
            logger.error(
                "Failed to find category segment %s! %s" % (repr(self.current_segment), repr(self.read_buffer)))
            self.read_packet_num += 1
            return False
        try:
            category = self.current_segment.decode()
        except UnicodeDecodeError:
            logger.error("Category segment contains invalid characters: %s, %s" % (repr(self.current_segment), repr(self.read_buffer)))
            self.read_packet_num += 1
            return False
        logger.debug("category: %s" % category)

        try:
            self.process_packet(category)
        except (ShutdownException, RebootException, RelaunchException, DeviceRestartException, LowBatteryException):
            raise
        except BaseException as e:
            logger.error("Exception while processing packet %s" % (str(e)), exc_info=True)
            logger.error("Error packet: %s" % self.read_buffer)
            return False
        finally:
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
        elif f == 's':
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

    def read_task_fn(self, should_stop):
        self.prev_packet_time = time.time()

        while True:
            time.sleep(self.update_delay)
            if should_stop():
                logger.info("Exiting read thread")
                return

            while self.device.in_waiting() > 2:
                self.read()

    def log_packet_error_code(self, error_code, packet_num):
        logger.warning("Packet %s returned an error:" % packet_num)
        logger.warning("\t%s" % self.packet_error_codes[error_code])

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
