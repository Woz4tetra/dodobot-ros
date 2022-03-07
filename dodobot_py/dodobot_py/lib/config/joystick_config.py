from .config import Config


class JoystickConfig(Config):
    def __init__(self, base_dir):
        self.path = "/dev/input/js0"
        self.enabled = True

        super(JoystickConfig, self).__init__("joystick.yaml", base_dir)

    def to_dict(self):
        return {
            "path": self.path,
            "enabled": self.enabled,
        }
