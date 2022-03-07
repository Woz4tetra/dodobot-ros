import time
import pprint
import threading
from jtop import jtop

from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from .node import Node

logger = LoggerManager.get_logger()


class JetsonStats(Node):
    def __init__(self, session):
        super(JetsonStats, self).__init__(session)

        self.log_delay = 60.0
        self.log_timer = time.time()

        self.should_stop = False
        self.should_stop_fn = (lambda: self.should_stop,)
        self.thread = threading.Thread(target=self.update_task, args=self.should_stop_fn)
        self.thread.daemon = True
        self.thread_exception = None
    
    def update_task(self, should_stop):
        with jtop() as jetson:
            self.log_one_time_stats(jetson)

            while jetson.ok():
                if should_stop():
                    break
                
                self.check_power_usage()
                current_time = time.time()
                if current_time - self.log_timer < self.log_delay:
                    continue
                self.log_timer = current_time
                self.log_continuous_stats(jetson)
    
    def check_power_usage(self, jetson):
        power_usage_mW = jetson.power[0]["cur"]
        if power_usage_mW > self.high_usage_threshold:
            logger.warn("Power usage is very high: %s" % power_usage_mW)

    def log_continuous_stats(self, jetson):
        self.log_stat("Fan", jetson.fan)
        self.log_stat("Uptime", jetson.uptime)
        self.log_stat("Temperature", jetson.temperature)
        self.log_stat("Power", jetson.power)

    def log_one_time_stats(self, jetson):
        if jetson.ok():
            self.log_stat("CPUs", jetson.cpu)
            self.log_stat("GPU", jetson.gpu)
            self.log_stat("Engine", jetson.engine)
            self.log_stat("NV Power Model", jetson.nvpmodel)
            self.log_stat("jetson_clocks", jetson.jetson_clocks)
            self.log_stat("Disk", jetson.disk)
            self.log_stat("Local Interfaces", jetson.local_interfaces)
            self.log_stat("EMC", jetson.emc)
            self.log_stat("RAM", jetson.ram)
            self.log_stat("iram", jetson.iram)
            self.log_stat("MTS", jetson.mts)
            self.log_continuous_stats(jetson)
        else:
            logger.warn("Failed to log initial stats!")

    def log_stat(self, name, stat):
        logger.info("*** %s ***" % name)
        logger.info(pprint.pformat(stat))

    def task_running(self):
        return self.thread_exception is None

    def start(self):
        self.thread.start()
    
    def update(self):
        if not self.task_running():
            logger.error("Error detected in read task. Raising exception")
            raise self.thread_exception

    def stop(self):
        self.should_stop = True
        logger.info("Set jetson stats thread stop flag")
