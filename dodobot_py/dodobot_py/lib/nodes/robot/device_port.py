import time
import serial

from lib.logger_manager import LoggerManager

logger = LoggerManager.get_logger()


class DevicePortWriteException(Exception):
    pass


class DevicePortReadException(Exception):
    pass


class DevicePort:
    def __init__(self, address, baud, timeout=0.1, write_timeout=0.1):
        self.device = None
        self.address = address
        self.baud = baud
        self.timeout = timeout
        self.write_timeout = write_timeout

        # from https://www.raspberrypi.org/forums/viewtopic.php?t=61955
        self.out_buf_max_bytes = 16 * 12
        self.in_buf_max_bytes = 16 * 8

    def configure(self):
        logger.debug("Attempting to open address '%s'" % self.address)
        self.device = serial.Serial(
            self.address,
            self.baud,
            timeout=self.timeout,
            write_timeout=self.write_timeout
        )

        # wait for the device to send data
        check_time = time.time()
        while self.in_waiting() < 0:
            time.sleep(0.001)

            if time.time() - check_time > self.write_timeout:
                logger.info(
                    "Waited for '%s' for %ss with no response..." % (
                        self.address, self.timeout)
                )
                return
        logger.info("%s is ready" % self.address)

    def write(self, packet: bytes):
        if not self.device:
            raise DevicePortWriteException("Device '%s' was never opened for writing" % self.address)
        if not self.is_open():
            raise DevicePortWriteException("Device '%s' is not open for writing" % self.address)
        self.device.write(packet)

    def out_waiting(self):
        """
        Safely check the serial buffer.
        :return: None if an OSError occurred, otherwise an integer value indicating the buffer size
        """
        try:
            return self.device.out_waiting
        except OSError:
            logger.error("Failed to check serial. Is there a loose connection?")
            raise

    def in_waiting(self):
        """
        Safely check the serial buffer.
        :return: None if an OSError occurred, otherwise an integer value indicating the buffer size
        """
        try:
            return self.device.inWaiting()
        except OSError:
            logger.error("Failed to check serial. Is there a loose connection?")
            raise

    def is_open(self):
        """Wrapper for isOpen"""
        return self.device is not None and self.device.isOpen()

    def read(self, size=None):
        if self.device.isOpen():
            if size is None:
                size = self.in_waiting()
            return self.device.read(size)
        else:
            raise DevicePortReadException("Serial port wasn't open for reading...")

    def readline(self):
        return self.device.readline()

    def stop(self):
        if self.is_open():
            logger.info("Closing device '{}'".format(self.address))
            self.device.close()

    # def __del__(self):
    #     self.stop()
