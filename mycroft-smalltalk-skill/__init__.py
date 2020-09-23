# These are just MY PERSONAL banter that I want my robot to
# carry on.

from os.path import dirname

from adapt.intent import IntentBuilder
from mycroft.skills.core import MycroftSkill
from mycroft.util.log import getLogger

__author__ = "Christen Lofland"
# Based on helloworld by eward

LOG = getLogger(__name__)


class SmallTalkSkill(MycroftSkill):
    def __init__(self):
        super(SmallTalkSkill, self).__init__(name="SmallTalkSkill")

    def initialize(self):
        self.load_data_files(dirname(__file__))

        hi_intent = IntentBuilder("HiIntent").require("HiKeyword").build()
        self.register_intent(hi_intent, self.handle_hi_intent)

        good_morning_intent = (
            IntentBuilder("GoodMorningIntent").require("GoodMorningKeyword").build()
        )
        self.register_intent(good_morning_intent, self.handle_good_morning_intent)

        do_you_have_my_glasses = (
            IntentBuilder("GlassesIntent").require("GlassesKeyword").build()
        )
        self.register_intent(do_you_have_my_glasses, self.handle_do_you_have_my_glasses)

        # TODO: Add a fallback catcher to inject My response to unknown requests.
        # See: https://www.freecodecamp.org/news/how-to-delete-a-git-branch-both-locally-and-remotely/

        # This does not work in Mycroft now.
        # I'm not sure what the replacement is yet, but commenting out
        # until I have time to look into it
        # self.emitter.on('fallback_failure', self.handle_fallback)

    def handle_hi_intent(self, message):
        self.speak_dialog("hi")

    def handle_good_morning_intent(self, message):
        self.speak_dialog("good.morning")

    def handle_do_you_have_my_glasses(self, message):
        self.speak_dialog("glasses")

    def handle_fallback(self, message):
        utt = message.data.get("utterance")
        LOG.debug("SmallTalk fallback attempt: " + utt)
        self.speak_dialog("fallback", data={"phrase": utt})

    def stop(self):
        pass


def create_skill():
    return SmallTalkSkill()
