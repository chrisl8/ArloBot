# These 'skills' are 100% tied to the ArloBot platform
# and have no use outside of it.
#
# Some 'arlobot' skills are triggered by events, not speech.
# Essentially the robot "speaks" into the message bus.
# Therefore some have a single keyword that is hopefully something
# STT will never come up with.
# Other things are speech inputs sent as commands into the
# Arlobot node service.
#
# TL;DR: This is robot speech triggered by robot events,
#        AND robot functions triggered by human speech.

from os.path import dirname

from adapt.intent import IntentBuilder
from mycroft.skills.core import MycroftSkill
from mycroft.util.log import getLogger
from mycroft.messagebus import Message

__author__ = "Christen Lofland"
# Based on helloworld by eward

LOGGER = getLogger(__name__)


class ArlobotSkill(MycroftSkill):
    def __init__(self):
        super(ArlobotSkill, self).__init__(name="ArlobotSkill")

    def initialize(self):
        self.load_data_files(dirname(__file__))

        start_ros_intent = (
            IntentBuilder("StartROSIntent").require("StartROSKeyword").build()
        )
        self.register_intent(start_ros_intent, self.handle_start_ros_intent)

        make_map_intent = (
            IntentBuilder("MakeMapIntent").require("MakeMapKeyword").build()
        )
        self.register_intent(make_map_intent, self.handle_make_map_intent)

        unplug_yourself_intent = (
            IntentBuilder("UnplugYourselfIntent")
            .require("UnplugYourselfKeyword")
            .build()
        )
        self.register_intent(unplug_yourself_intent, self.handle_unplug_yourself_intent)

    def handle_start_ros_intent(self, message):
        self.bus.emit(Message("arlobot", data={"action": "startROS"}))

    def handle_make_map_intent(self, message):
        self.bus.emit(Message("arlobot", data={"action": "makeMap"}))

    def handle_unplug_yourself_intent(self, message):
        self.bus.emit(Message("arlobot", data={"action": "unplugYourself"}))

    def stop(self):
        pass


def create_skill():
    return ArlobotSkill()
