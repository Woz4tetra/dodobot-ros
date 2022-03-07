from lib.nodes.robot import Dodobot
from lib.nodes.joystick import Joystick
from lib.nodes.data_logger import DataLogger
from lib.nodes.sounds import Sounds
from lib.nodes.jetson_stats import JetsonStats


class Session:
    def __init__(self):
        self.robot = Dodobot(self)
        self.joystick = Joystick(self)
        self.data_logger = DataLogger(self)
        self.sounds = Sounds(self)
        self.jetson_stats = JetsonStats(self)

    def start(self):
        self.sounds.start()
        self.robot.start()
        self.joystick.start()
        self.data_logger.start()
        self.jetson_stats.start()

    def update(self):
        self.robot.update()
        self.joystick.update()
        self.data_logger.update()
        self.sounds.update()
        self.jetson_stats.update()

    def stop(self):
        self.robot.stop()
        self.joystick.stop()
        self.data_logger.stop()
        self.sounds.stop()
        self.jetson_stats.stop()
