from pydub import generators

from lib.config import ConfigManager
from lib.logger_manager import LoggerManager
from .pulseaudio import Audio, Pacmd
from ..node import Node

sound_config = ConfigManager.get_sounds_config()
logger = LoggerManager.get_logger()


class Sounds(Node):
    def __init__(self, session):
        super(Sounds, self).__init__(session)

        self.controller = Pacmd(
            sound_config.audio_sink,
            sound_config.volume_raw_min,
            sound_config.volume_raw_max,
        )

        self.sounds = {}
        self.load_audio(sound_config.sounds)

    def start(self):
        self.controller.set_volume(sound_config.volume)

    def stop(self):
        for audio in self.sounds.values():
            audio.unload()

    def __getitem__(self, name):
        return self.sounds[name]

    def load_audio(self, config: dict):
        for name, value in config.items():
            if len(value) == 0:
                logger.info("Skipping '%s' audio file" % str(name))
                self.sounds[name] = Audio()
            elif value.startswith(":generate:"):
                logger.info("Loading '%s' as generated sound: %s" % (name, value))
                audio = self.generate_sound(value)
                self.sounds[name] = Audio.load(audio)
            else:
                try:
                    logger.info("Loading '%s' as audio file: %s" % (name, value))
                    self.sounds[name] = Audio.load_from_path(value)
                except BaseException as e:
                    logger.error(str(e), exc_info=True)


    def generate_sound(self, generate_params):
        class_name = generate_params.pop("class_name")
        duration = generate_params.pop("duration")
        volume = generate_params.pop("volume", 0.0)

        generator_class = getattr(generators, class_name)

        generator_class(**generate_params)
