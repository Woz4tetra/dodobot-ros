import time

from lib.config import ConfigManager
from lib.logger_manager import LoggerManager

battery_config = ConfigManager.get_battery_config()
logger = LoggerManager.get_logger()


class BatteryState:
    UNKNOWN = 0
    FULL = 1
    OK = 2
    LOW = 3
    CRITICAL = 4

    def __init__(self):
        self.state = 0
        self.recv_time = 0.0
        self.current_mA = 0.0
        self.power_mW = 0.0
        self.voltage_V = 0.0
        self.prev_critical_time = None
        self.prev_V = 0.0

    def update_state(self, power_state):
        self.recv_time = power_state["recv_time"]
        self.current_mA = power_state["current_mA"]
        self.power_mW = power_state["power_mW"]
        self.voltage_V = power_state["load_voltage_V"]

    def set(self, power_state):
        self.update_state(power_state)
        if self.voltage_V >= battery_config.full_voltage:
            state = self.FULL
            self.prev_critical_time = None
        # elif self.voltage_V >= battery_config.ok_voltage:
        elif self.voltage_V > battery_config.low_voltage:
            state = self.OK
            self.prev_critical_time = None
        elif self.voltage_V <= battery_config.critical_voltage:
            state = self.CRITICAL
            if self.prev_critical_time is None:
                self.prev_critical_time = time.time()
        elif self.voltage_V <= battery_config.low_voltage:
            state = self.LOW
            self.prev_critical_time = None
        else:
            state = self.state

        if state != self.state:
            self.state = state
            if abs(self.voltage_V - self.prev_V) > 0.25:
                self.prev_V = self.voltage_V
                return True
        return False

    def should_shutdown(self):
        return (
                self.state == self.CRITICAL and
                self.prev_critical_time is not None and
                time.time() - self.prev_critical_time > battery_config.critical_voltage_timeout_s
        )

    def log_state(self):
        if self.state == self.FULL:
            logger.info("Fully charged: %0.2f" % self.voltage_V)
        elif self.state == self.OK:
            logger.info("Battery ok: %0.2f" % self.voltage_V)
        elif self.state == self.LOW:
            logger.warn("Battery is low: %0.2f" % self.voltage_V)
        elif self.state == self.CRITICAL:
            logger.error("Battery is critically low: %0.2f!!" % self.voltage_V)
        else:
            logger.error("Battery is in an unknown state")
