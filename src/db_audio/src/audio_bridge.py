import os
import re
import time
import math
import rospy
from subprocess import Popen, PIPE
from threading import Thread, Lock

import pydub.utils
from pydub import AudioSegment

import pyaudio

from ctypes import *
from contextlib import contextmanager

# helpful forum posts:
# how to get audio to play:
# https://askubuntu.com/questions/14077/how-can-i-change-the-default-audio-device-from-command-line
#   pacmd list-sinks
#   pacmd set-default-sink 0

# fix volume issue:
# https://chrisjean.com/fix-for-usb-audio-is-too-loud-and-mutes-at-low-volume-in-ubuntu/

# playing audio on threads:
# https://stackoverflow.com/questions/60695093/play-audio-file-in-the-background

# playing audio in chunks with pyaudio:
# https://github.com/jiaaro/pydub/blob/master/pydub/playback.py

# suppress warnings from alsa:
# https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time


# From alsa-lib Git 3fd4ab9be0db7c7430ebd258f2717a976381715d
# $ grep -rn snd_lib_error_handler_t
# include/error.h:59:typedef void (*snd_lib_error_handler_t)(const char *file, int line, const char *function, int err, const char *fmt, ...) /* __attribute__ ((format (printf, 5, 6))) */;
# Define our error handler type
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def suppress_alsa_error():
    asound = cdll.LoadLibrary("libasound.so")
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)

def enumerate_devices():
    with suppress_alsa_error():
        pyaudio_obj = pyaudio.PyAudio()
    info = pyaudio_obj.get_host_api_info_by_index(0)
    num_devices = info.get("deviceCount")
    for index in range(num_devices):
        yield index, pyaudio_obj.get_device_info_by_host_api_device_index(0, index)
        

class Audio:
    device_info = {}
    play_lock = Lock()

    def __init__(self):
        self.audio = None
        self.thread = None

        self._is_playing = False
        self._jump_stream = None
    
    @classmethod
    def set_sink_by_name(cls, name):
        names = []
        for index, info in enumerate_devices():
            device_name = info.get("name")
            names.append(device_name)
            if name in device_name:
                cls.set_sink(index)
        if len(cls.device_info) == 0:
            raise Exception("Failed to find device name '%s'. Available: %s" % (name, "\n\t".join(names)))

    @classmethod
    def set_sink(cls, index):
        with suppress_alsa_error():
            pyaudio_obj = pyaudio.PyAudio()
        cls.device_info = pyaudio_obj.get_device_info_by_host_api_device_index(0, index)
        assert type(cls.device_info) == dict

    @classmethod
    def load_from_path(cls, path):
        self = cls()
        self.reload_from_path(path)
        return self

    @classmethod
    def load(cls, audio_segment):
        self = cls()
        self.reload(audio_segment)
        return self
    
    def reload(self, audio_segment):
        self.unload()
        self.audio = audio_segment

        # match sample rate with device
        self.audio = self.audio.set_frame_rate(int(self.device_info.get("defaultSampleRate")))
        self.audio = self.audio.set_channels(int(self.device_info.get("maxOutputChannels")))

    def reload_from_path(self, path):
        filename = os.path.basename(path)
        extension = os.path.splitext(filename)[1]
        extension = extension.lower()
        if extension == "mp3":
            segment = AudioSegment.from_mp3(path)
        elif extension == "wav":
            segment = AudioSegment.from_wav(path)
        elif extension == "ogg":
            segment = AudioSegment.from_ogg(path)
        elif extension == "flv":
            segment = AudioSegment.from_flv(path)
        else:
            segment = AudioSegment.from_file(path)
        self.reload(segment)

    def _play_thread(self):
        self.thread = Thread(target=self._play_task)
        self.thread.daemon = True
        self.thread.start()
        self._is_playing = True
    
    def _play_task(self):
        self.__class__.play_lock.acquire()
        with suppress_alsa_error():
            pyaudio_obj = pyaudio.PyAudio()
        audio_format = pyaudio_obj.get_format_from_width(self.audio.sample_width)
        # audio_format = pyaudio.paInt16
        
        rospy.loginfo("Audio sample rate: %s, channels: %s, format: %s, width: %s. Using device %s" % (
            self.audio.frame_rate, self.audio.channels, audio_format, self.audio.sample_width, self.device_info
        ))
        
        stream = pyaudio_obj.open(
            format=audio_format,
            channels=self.audio.channels,
            rate=self.audio.frame_rate,
            output=True,
            output_device_index=self.device_info.get("index")
        )

        try:
            if not self._is_playing:
                return
                
            chunk_size = 250  # ms
            # break audio into chunks (to allow keyboard interrupts)
            chunks = pydub.utils.make_chunks(self.audio, chunk_size)
            index = 0
            while index < len(chunks):
                chunk = chunks[index]
                stream.write(chunk._data)
                index += 1
                
                if not self._is_playing:
                    break
                if self._jump_stream is not None:
                    index = int(self._jump_stream / chunk_size)
                    self._jump_stream = None

        finally:
            stream.stop_stream()
            stream.close()

            pyaudio_obj.terminate()
            self._is_playing = False
            self.__class__.play_lock.release()
    
    def _jump_to(self, time_ms):
        self._jump_stream = time_ms

    def unload(self):
        self.stop()
        if self.is_loaded():
            del self.audio
            self.audio = None
            return True
        else:
            return False

    def play(self):
        if self.is_loaded():
            if self.is_playing():
                rospy.loginfo("Audio playing. Jumping back to start")
                self._jump_to(0)
            else:
                self._play_thread()
        else:
            raise Exception("Audio is not loaded. Can't play")

    def is_playing(self):
        return self.is_loaded() and self.thread is not None and self._is_playing
    
    def is_loaded(self):
        return self.audio is not None

    def wait(self):
        if self.is_playing():
            self.thread.join()

    def stop(self):
        if self.is_playing():
            self._is_playing = False
    
    def __del__(self):
        self.stop()

class Pacmd:
    def __init__(self, sink, pacmd_exec="/usr/bin/pacmd", timeout=5.0):
        self.pacmd_exec = pacmd_exec
        self.sink = str(sink)
        self.max_volume = 0x10000
        self.volume = self.max_volume
        self.sink_timeout_s = timeout

        if self.check_exec():
            raise Exception("Failed to get audio controller. Check executable path: %s" % str(pacmd_exec))

        if self.wait_for_sinks() == 0:
            raise Exception("Failed to find audio sinks!")

        self.set_sink()

    def list_sinks(self):
        return self._run_cmd(["list-sinks"])
    
    def check_exec(self):
        output = self._run_cmd(["--help"])
        return len(output) > 0

    def set_volume(self, percent):
        # percent is 0...1
        self.volume = max(min(percent, 1.0), 0.0)
        rospy.loginfo("Setting volume to %0.3f" % self.volume)
        volume = str(int(self.max_volume * percent))
        self._run_cmd(["set-sink-volume", self.sink, volume])

    def wait_for_sinks(self):
        start_time = time.time()
        regex = r"(\d*) sink\(s\) available"
        while time.time() - start_time < self.sink_timeout_s:
            output = self.list_sinks()
            match = re.search(regex, output)
            if match:
                sinks = int(match.group(1))
                if sinks > 0:
                    rospy.loginfo("Found %s available sinks" % sinks)
                    return sinks

        rospy.logerr("Failed to find sinks!")
        return 0

    def set_sink(self):
        rospy.loginfo("Setting audio sink to %s" % self.sink)
        self._run_cmd(["set-default-sink", self.sink])

    def _run_cmd(self, command, log_output=False):
        p = Popen([self.pacmd_exec] + command, stdout=PIPE, shell=True)
        output = p.communicate()[0]
        output = output.decode()
        if log_output:
            rospy.loginfo("pacmd output: '%s'" % output)
        return output
