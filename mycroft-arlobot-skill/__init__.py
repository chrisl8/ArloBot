#
# The 'arlobot' skills are intended to be 100% triggered by events,
# not speach.
# Therefore each has a single keyword that is hopefully something
# STT will never come up with.
#
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

from os.path import dirname

from adapt.intent import IntentBuilder
from mycroft.skills.core import MycroftSkill
from mycroft.util.log import getLogger

__author__ = 'chrisl8'
# Based on helloworld by eward

LOGGER = getLogger(__name__)


class ArlobotSkill(MycroftSkill):

    def __init__(self):
        super(ArlobotSkill, self).__init__(name="ArlobotSkill")

    def initialize(self):
        self.load_data_files(dirname(__file__))

        unplugged_intent = IntentBuilder("UnpluggedIntent").\
            require("RobotHasBeenUnpluggedKeyword").build()
        self.register_intent(unplugged_intent,
                             self.handle_unplugged_intent)

        plugged_in_intent = IntentBuilder("RobotHasBeenPluggedInIntent"). \
            require("RobotHasBeenPluggedInKeyword").build()
        self.register_intent(plugged_in_intent,
                             self.handle_plugged_in_intent)

        # For saying own name on startup, code stolen from the speak_skill
        prefixes = [
            'arlobotstartupskill']
        self.__register_prefixed_regex(prefixes, "(?P<Words>.*)")

        intent = IntentBuilder("StartupIntent").require(
            "RobotStartupKeyword").require("Words").build()
        self.register_intent(intent, self.handle_startup_intent)

    def __register_prefixed_regex(self, prefixes, suffix_regex):
        for prefix in prefixes:
            self.register_regex(prefix + ' ' + suffix_regex)

    def handle_startup_intent(self, message):
        words = message.data.get("Words")
        words = 'Hello my name is ' + words
        self.speak(words)

    # def handle_thank_you_intent(self, message):
    #     self.speak_dialog("welcome")
    #
    # def handle_how_are_you_intent(self, message):
    #     self.speak_dialog("how.are.you")

    def handle_unplugged_intent(self, message):
        self.speak_dialog("unplugged")

    def handle_plugged_in_intent(self, message):
        self.speak_dialog("plugged.in")

    def stop(self):
        pass


def create_skill():
    return ArlobotSkill()
