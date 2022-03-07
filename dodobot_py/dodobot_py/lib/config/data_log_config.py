import os
import logging
from .config import Config


class DataLogConfig(Config):
    def __init__(self, base_dir):
        self.name = "dodobot_data"
        self.level = logging.DEBUG
        self.file_name = "data"
        self.suffix = "%Y-%m-%d.log"
        self.format = "%(asctime)s:\t%(message)s"
        self.log_freq_hz = 1.0
        self.enabled = False

        super(DataLogConfig, self).__init__("data_logging.yaml", base_dir)

        self.dir = os.path.join(self.base_dir, "data")
        if not os.path.isdir(self.dir):
            os.makedirs(self.dir)
        self.path = os.path.join(self.dir, self.file_name)

    def to_dict(self):
        return {
            "enabled": self.enabled,
            "name": self.name,
            "level": self.level,
            "file_name": self.file_name,
            "suffix": self.suffix,
            "format": self.format,
            "path": self.path,
            "dir": self.dir,
            "log_freq_hz": self.log_freq_hz,
        }
