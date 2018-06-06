#!/usr/bin/env python

import qi
import sys
import numpy
import time
import random

from naoqi import ALProxy
from naoqi import ALModule
from naoqi import ALBroker

from const import INTENSITY_THRESHOLD

SURPRISE   = 0
HAPPY   = 1
NEUTRAL = 2
SAD     = 3


conjunctions = ["Ah", "Oh", "Nice", "Hmm", "Er"]

surprise_words_low = ["You look surprised.",
                      "What's up?",
                      "Is there anything on my face?"]

# upward tone
surprise_words_high = ["Wow, you look so surprised",
                       "I know that my cleverness makes people surprised. You will get used to it.",
                       "What on earth is going on that makes you so surprised."]

happy_words_low = ["You look happy.",
                   "Is there any good news?",
                   "You are attractive when you smile"]

# upward tone
happy_words_high = ["Wow, you look so hapy.",
                    "You must have heard some good news, right?",
                    "You make me feel happy too! Your happiness is contagious!"]

sad_words_low = ["Oh, you look sad.",
                 "What happened?",
                 "Is there anything I could possibly help"]

# downward tone
sad_words_high = ["Are you all right? You look so sad",
                  "Is there anything you want to share? I am here to listen.",
                  "Everything gonna be all right. Just let me know if you need any help, okay?"]

class ReactToTouch(ALModule):
    ready = False

    def __init__(self, name):
        ALModule.__init__(self, name)
        global memory
        memory = ALProxy("ALMemory")
        self.tts = ALProxy("ALTextToSpeech")
        memory.subscribeToEvent("TouchChanged", "ReactToTouch", "onTouched")

    def onTouched(self, strVarName, value):
        if not value[0][1]:
            return
        memory.unsubscribeToEvent("TouchChanged", "ReactToTouch")

        if self.ready:
            self.tts.say("Experiment over, thank you ...")
            self.ready = False
        else:
            self.tts.say("Experiment start now ...")
            self.ready = True

        memory.subscribeToEvent("TouchChanged", "ReactToTouch", "onTouched")


class RobotStrategy:
    time_interval = 7

    def __init__(self):
        self.selection = 0
        self.last_time = time.time()

        try:
            self.animation_player = ALProxy("ALAnimationPlayer")
            self.awareness = ALProxy("ALBasicAwareness")
            self.tts = ALProxy("ALTextToSpeech")
            self.tts.say("Ready to start ...")

        except RuntimeError:
            print "Cann't connect to naoqi"
            sys.exit(1)

    # def intro(self):
    #     self.awareness.setTrackingMode("WholeBody")
    #     self.awareness.setStimulusDetectionEnabled("Sound", False)
    #     self.awareness.setStimulusDetectionEnabled("Touch", False)
    #     self.awareness.setStimulusDetectionEnabled("Movement", False)
    #     self.awareness.setStimulusDetectionEnabled("People", True)

    #     self.tts.say("Hello, my name is Pepper. Welcome to join this user study with me. "
    #                  "This experiment has three sessions. "
    #                  "In each session, you are supposed to act some emotions."
    #                  "I will give you some demos on my tablet."
    #                  "You can touch the tablet to start watching the demos.")

    def execute(self, emotion, intensity):
        print intensity

        gestureTag = ""
        wordToSay = ""

        choice = self.selection

        if len(emotion) < 4 or len(intensity[0]) < 2:
            return
        else:
            print emotion
            if choice > 0:
                cat = numpy.argmax(emotion)
                level = intensity[0][1] > INTENSITY_THRESHOLD

                if level:
                    gestureTag = "body language"

                print level
                if cat == SURPRISE:
                    print "surprise"
                    if level:
                        gestureTag = "enthusiastic"
                        index = random.randint(0, len(surprise_words_high)-1)
                        wordToSay = surprise_words_high[index]
                    else:
                        index = random.randint(0, len(surprise_words_low)-1)
                        wordToSay = surprise_words_low[index]

                if cat == HAPPY:
                    print "happy"
                    if level:
                        index = random.randint(0, len(happy_words_high)-1)
                        wordToSay = happy_words_high[index]
                        gestureTag = "excited"
                    else:
                        index = random.randint(0, len(happy_words_low)-1)
                        wordToSay = happy_words_low[index]

                if cat == SAD:
                    print "sad"
                    if level:
                        index = random.randint(0, len(surprise_words_high)-1)
                        wordToSay = sad_words_high[index]
                        gestureTag = "frustrated"
                    else:
                        index = random.randint(0, len(surprise_words_high)-1)
                        wordToSay = sad_words_low[index]

                if level:
                    self.time_interval -= 1
                    if self.time_interval < 5:
                        self.time_interval = 5
                else:
                    self.time_interval += 1
                    if self.time_interval > 9:
                        self.time_interval = 9

            else:
                index = random.randint(0, len(conjunctions)-1)
                wordToSay = conjunctions[index]

        if time.time() - self.last_time > self.time_interval:
            self.tts.post.say(wordToSay)

            if choice == 2 and len(gestureTag) > 1:
                self.animation_player.post.runTag(gestureTag)

            self.last_time = time.time()

    def pickStrategy(self, strat):
        self.selection = strat

    def test(self):
        pass
        # gestureTag = "wow"
        # self.tts.say(gestureTag)

class Robot:
    def __init__(self):
        myBroker = ALBroker("myBroker", "0.0.0.0", 0, "192.168.1.101", 9559)
        global ReactToTouch
        ReactToTouch = ReactToTouch("ReactToTouch")
        self.action = RobotStrategy()

    def run(self, emotion, intensity):
        if ReactToTouch.ready:
            self.action.execute(emotion, intensity)

    def pickStrategy(self, number):
        self.action.pickStrategy(number)

if __name__ == "__main__":
    test = Robot()
    while 1:
        test.run(None, None)
