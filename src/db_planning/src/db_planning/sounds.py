import rospy

from .pulseaudio import Audio, Pacmd


class Sounds:
    def __init__(self, audio_sink, paths):
        self.controller = Pacmd(audio_sink)
        self.sounds = {}
        self.load_audio(paths)

    def __del__(self):
        for audio in self.sounds.values():
            audio.unload()

    def __getitem__(self, name):
        return self.sounds[name]

    def load_audio(self, config):
        for name, value in config.items():
            self.sounds[name] = Audio.load_from_path(value)
