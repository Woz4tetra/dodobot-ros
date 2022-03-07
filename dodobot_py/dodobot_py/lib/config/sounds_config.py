from .config import Config


class SoundsConfig(Config):
    def __init__(self, base_dir):
        self.audio_sink = "0"
        self.volume = 0.5
        self.volume_raw_max = 0x10000
        self.volume_raw_min = 0
        self.sounds = {}
        super(SoundsConfig, self).__init__("sounds.yaml", base_dir)

    def to_dict(self):
        return {
            "audio_sink": self.audio_sink,
            "volume": self.volume,
            "sounds": self.sounds,
        }
