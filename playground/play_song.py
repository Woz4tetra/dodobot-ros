#!/usr/bin/env python3

import sys
from pydub import AudioSegment
from pydub.playback import _play_with_simpleaudio

path = sys.argv[1]

audio = AudioSegment.from_file(path)
playback = _play_with_simpleaudio(audio)
try:
    playback.wait_done()
finally:
    playback.stop()
