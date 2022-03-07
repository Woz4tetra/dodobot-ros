from .config import Config


class DevicePortConfig(Config):
    def __init__(self, base_dir):
        self.baud_rate = 115200
        self.address = "/dev/serial0"
        self.timeout = 5.0
        self.write_timeout = 5.0
        self.update_rate_hz = 30
        super(DevicePortConfig, self).__init__("device_port.yaml", base_dir)

    def to_dict(self):
        return {
            "baud_rate": self.baud_rate,
            "address": self.address,
            "timeout": self.timeout,
            "write_timeout": self.write_timeout,
            "update_rate_hz": self.update_rate_hz,
        }
