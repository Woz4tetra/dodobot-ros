#!/usr/bin/env python
import rospy
import std_msgs.msg

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

sounds = None

def sounds_callback(msg):
    sounds[msg.data].play()


def main():
    global sounds

    node_name = "db_audio"
    rospy.init_node(
        node_name
        # disable_signals=True
        # log_level=rospy.DEBUG
    )

    audio_sink = rospy.get_param("~audio_sink", "alsa_output.usb-Generic_USB2.0_Device_20130100ph0-00.analog-stereo")
    audio_paths = rospy.get_param("~audio_paths")
    assert type(audio_paths) == dict

    try:
        sounds = Sounds(audio_sink, audio_paths)
    except BaseException as e:
        rospy.logerr("Failed to load sounds: %s" % str(e))
        return
    
    rospy.Subscriber("sounds", std_msgs.msg.String, sounds_callback)
    rospy.spin()


if __name__ == "__main__":
    main()