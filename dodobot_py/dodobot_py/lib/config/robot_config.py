import os
from .config import Config


class RobotConfig(Config):
    def __init__(self, base_dir):
        self.pid_ks = {}
        self.check_ready_timeout = 5.0
        self.write_timeout = 1.0
        self.packet_read_timeout = 0.15

        self.drive_command_timeout = 30.0
        self.drive_command_repeat_timeout = 0.75
        self.drive_command_update_rate = 30.0

        self.joystick_deadzone = 0.1
        self.max_joy_val = 1.0
        self.max_tilt_speed = 5.0
        self.max_grab_speed = 1.0
        self.force_threshold = 700

        self.shutdown_time_limit = 3.0

        self.write_date_delay = 0.5

        self.stepper_max_speed = 31250000 * 8
        self.stepper_max_accel = 1250000 * 8
        self.drive_max_speed = 6800.0
        self.drive_min_speed = 2500.0

        self.gripper_open = 0
        self.gripper_closed = 180

        self.breakout_levels = "breakout"

        self.startup_image_name = ""
        self.startup_image_path = ""
        self.startup_image_size = (160, 108)
        self.startup_image_quality = 15

        self.wifi_name = "wlan0"
        self.hotspot_name = "dodobot-host"

        super(RobotConfig, self).__init__("robot.yaml", base_dir)

    def load(self):
        super(RobotConfig, self).load()
        self.startup_image_path = os.path.join(self.config_dir, self.startup_image_path)
        self.breakout_levels = os.path.join(self.config_dir, self.breakout_levels)

    def to_dict(self):
        return {
            "pid_ks": self.pid_ks,
            "check_ready_timeout": self.check_ready_timeout,
            "write_timeout": self.write_timeout,
            "packet_read_timeout": self.packet_read_timeout,
            "drive_command_timeout": self.drive_command_timeout,
            "drive_command_update_delay": self.drive_command_update_delay,
            "joystick_deadzone": self.joystick_deadzone,
            "max_joy_val": self.max_joy_val,
            "shutdown_time_limit": self.shutdown_time_limit,
            "write_date_delay": self.write_date_delay,
            "stepper_max_speed": self.stepper_max_speed,
            "drive_max_speed": self.drive_max_speed,
            "drive_min_speed": self.drive_min_speed,
            "gripper_open": self.gripper_open,
            "gripper_closed": self.gripper_closed,
            "startup_image_path": self.startup_image_path,
            "startup_image_size": self.startup_image_size,
            "startup_image_quality": self.startup_image_quality,
            "wifi_name": self.wifi_name,

        }
