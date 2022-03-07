from .config import Config


class GeneralConfig(Config):
    def __init__(self, base_dir):
        self.update_rate_hz = 15

        super(GeneralConfig, self).__init__("general.yaml", base_dir)
