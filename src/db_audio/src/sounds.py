#!/usr/bin/env python3
import os
import rospy

import std_msgs.msg

from pulseaudio import Audio

from db_audio.srv import PlayAudio, PlayAudioResponse
from db_audio.srv import StopAudio, StopAudioResponse


class Sounds:
    def __init__(self, audio_sink, paths):
        # self.controller = Pacmd(audio_sink, "/usr/bin/pacmd", 5.0)
        Audio.set_sink_by_name(audio_sink)
        self.sounds = {name: Audio() for name in paths.keys()}
        self.paths = paths

        self.max_loaded_num = 5
        self.prev_sound = None

    def __del__(self):
        self.unload_all()

    def unload_all(self):
        for name in self.sounds.keys():
            self.sounds[name].unload()
    
    def num_loaded_sounds(self):
        count = 0
        for sound in self.sounds.values():
            if sound.is_loaded():
                count += 1
        return count
        
    def manage_loaded(self):
        for name, sound in self.sounds.items():
            if self.num_loaded_sounds() <= self.max_loaded_num:
                rospy.loginfo("num loaded sounds: %s" % self.num_loaded_sounds())
                return
            if not sound.is_playing():
                if sound.unload():
                    rospy.loginfo("Unloading sound: %s" % name)
    
    def check_loaded(self, name):
        if not self.sounds[name].is_loaded():
            rospy.loginfo("%s is not loaded. Loading" % name)
            self.sounds[name].reload_from_path(self.paths[name])
    
    def check_available(self, name):
        return name in self.sounds
    
    def check_playing(self, name):
        if self.prev_sound is None or self.prev_sound == name:
            return
        self.sounds[self.prev_sound].stop()

    def play(self, name):
        rospy.loginfo("Playing %s" % name)
        if not self.check_available(name):
            return False
        
        self.check_loaded(name)
        self.check_playing(name)

        self.sounds[name].play()
        self.prev_sound = name
        
        self.manage_loaded()
        
        return True
    
    def stop(self, name):
        rospy.loginfo("Stopping %s" % name)
        if not self.check_available(name):
            return False
        self.sounds[name].stop()
        return True

SOUNDS = None
SERVICES = {}


def play_sound_callback(req):
    global SOUNDS
    SOUNDS.play(req.name)
    return PlayAudioResponse(True)

def stop_sound_callback(req):
    global SOUNDS
    SOUNDS.stop(req.name)
    return StopAudioResponse(True)


def load_paths():
    audio_dir = rospy.get_param("~audio_dir")
    audio_paths = {}
    supported_filetypes = "mp3", "mp4", "wav", "ogg", "m4a", "flv", "raw"
    rospy.loginfo("Searching for audio files in %s" % audio_dir)
    for dirpath, dirnames, filenames in os.walk(audio_dir):
        for filename in filenames:
            name, extension = os.path.splitext(filename)

            extension = extension[1:].lower()
            if extension in supported_filetypes:
                path = os.path.join(dirpath, filename)
                audio_paths[name] = path
                rospy.loginfo("\t%s: %s" % (name, path))
    return audio_paths


def create_service(name, srv_type, callback):
    global SERVICES
    rospy.loginfo("Setting up service %s" % name)
    srv_obj = rospy.Service(name, srv_type, callback)
    rospy.loginfo("%s service is ready" % name)
    SERVICES[name] = srv_obj
    return srv_obj


def main():
    global SOUNDS

    node_name = "db_audio"
    rospy.init_node(
        node_name
        # disable_signals=True
        # log_level=rospy.DEBUG
    )
    rospy.loginfo("%s is initializing" % node_name)

    audio_sink = rospy.get_param("~audio_sink", "USB2.0 Device: Audio (hw:2,0)")
    audio_paths = load_paths()
    try:
        SOUNDS = Sounds(audio_sink, audio_paths)
    except BaseException as e:
        rospy.logerr("Failed to load sounds: %s" % str(e))
        return
    
    create_service("play_audio", PlayAudio, play_sound_callback)
    create_service("stop_audio", StopAudio, stop_sound_callback)
    rospy.loginfo("%s is ready" % node_name)
    rospy.spin()
    rospy.loginfo("%s is exiting" % node_name)


if __name__ == "__main__":
    main()