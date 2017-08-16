# Copyright 2016 Mycroft AI, Inc.
#
# This file is part of Mycroft Core.
#
# Mycroft Core is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Mycroft Core is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Mycroft Core.  If not, see <http://www.gnu.org/licenses/>.


import subprocess
from os.path import join

from mycroft import MYCROFT_ROOT_PATH
from mycroft.tts import TTS, TTSValidator

__author__ = 'seanfitz', 'chrisl8'

BIN = join(MYCROFT_ROOT_PATH, '..', 'scripts', 'tts.sh')


class ArlobotTTS(TTS):
    def __init__(self, lang, voice):
        super(ArlobotTTS, self).__init__(lang, voice, ArlobotTTSValidator(self))

    def execute(self, sentence):
        subprocess.call(
            [BIN, sentence])


class ArlobotTTSValidator(TTSValidator):
    def __init__(self, tts):
        super(ArlobotTTSValidator, self).__init__(tts)

    def validate_lang(self):
        # TODO
        pass

    def validate_connection(self):
        try:
            print BIN
            subprocess.call([BIN, ''])
        except:
            raise Exception(
                'Arlobot source is not installed. See https://github.com/chrisl8/arlobot for installation details.')

    def get_tts_class(self):
        return ArlobotTTS
