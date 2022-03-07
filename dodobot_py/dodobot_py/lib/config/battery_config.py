from .config import Config


class BatteryConfig(Config):
    def __init__(self, base_dir):
        self.full_voltage = 8.5
        self.ok_voltage = 7
        self.low_voltage = 6.3
        self.critical_voltage = 5.5
        self.critical_voltage_timeout_s = 3.0
        self.battery_log_report_time = 30.0

        super(BatteryConfig, self).__init__("battery.yaml", base_dir)

    def to_dict(self):
        return {
            "full_voltage": self.full_voltage,
            "ok_voltage": self.ok_voltage,
            "low_voltage": self.low_voltage,
            "critical_voltage": self.critical_voltage,
            "critical_voltage_timeout_s": self.critical_voltage_timeout_s,
            "battery_log_report_time": self.battery_log_report_time,
        }
