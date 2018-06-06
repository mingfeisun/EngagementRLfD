#!/usr/bin/env python

import rospy
import json
import urllib2

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from emotion_intensity.msg import Skeleton
from emotion_intensity.msg import Emotion

KINECT_JOINTS ={
14 	:  "AnkleLeft",
18 	:  "AnkleRight",
5 	:  "ElbowLeft",
9 	:  "ElbowRight",
15 	:  "FootLeft",
19 	:  "FootRight",
7 	:  "HandLeft",
11 	:  "HandRight",
21 	:  "HandTipLeft",
23 	:  "HandTipRight",
3 	:  "Head",
12 	:  "HipLeft",
16 	:  "HipRight",
13 	:  "KneeLeft",
17 	:  "KneeRight",
2 	:  "Neck",
4 	:  "ShoulderLeft",
8 	:  "ShoulderRight",
0 	:  "SpineBase",
1 	:  "SpineMid",
20 	:  "SpineShoulder",
22 	:  "ThumbLeft",
24 	:  "ThumbRight",
6 	:  "WristLeft",
10 	:  "WristRight"
}

class node_kinect_interpretor:
    def __init__(self):
        rospy.init_node("kinect_interpretator")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initializing kinect skeleton interpretator...")

        self.skeleton = Skeleton()
        self.emotion = Emotion()

        pub_sk = rospy.Publisher('node_skeleton/skeleton', Skeleton, queue_size=10)
        pub_fa = rospy.Publisher('node_face/emotion_category', Emotion, queue_size=10)

        while not rospy.is_shutdown():
            msg = urllib2.urlopen("http://192.168.1.104:1337").read()

            self.skeleton.header.frame_id = "/skeleton"
            self.skeleton.header.stamp = rospy.Time.now()

            self.skeleton.user_id = 0
            self.skeleton.name = list()
            self.skeleton.confidence = list()
            self.skeleton.position = list()
            self.skeleton.orientation = list()

            try:
                data = json.loads(msg)
                # faceExpr = data[u'face']
                skeleton = data[u'body']

                # if len(faceExpr) >= 10:
                #     self.parseFace(faceExpr)

                for each in skeleton[u'joints']:
                    index = skeleton[u'joints'].index(each)
                    self.skeleton.name.append(KINECT_JOINTS[index])
                    self.skeleton.confidence.append(1)

                    temp_point = Vector3()
                    temp_point.x = each[u'cameraX']
                    temp_point.y = each[u'cameraY']
                    temp_point.z = each[u'cameraZ']
                    self.skeleton.position.append(temp_point)

                    temp_quater = Quaternion()
                    temp_quater.x = each[u'orientationX']
                    temp_quater.y = each[u'orientationY']
                    temp_quater.z = each[u'orientationZ']
                    temp_quater.w = each[u'orientationW']
                    self.skeleton.orientation.append(temp_quater)
            except KeyError:
                rospy.logwarn("No person detected")

            pub_fa.publish(self.emotion)
            pub_sk.publish(self.skeleton)

    def shutdown(self):
        rospy.loginfo("Shutdown kinect skeleton interpretating.")

    def parseFace(self, json_data):
        data = json.loads(json_data)[0] # choose the first data
        self.emotion.anger     = data[u'scores'][u'anger']
        self.emotion.contempt  = data[u'scores'][u'contempt']
        self.emotion.disgust   = data[u'scores'][u'disgust']
        self.emotion.fear      = data[u'scores'][u'fear']
        self.emotion.happiness = data[u'scores'][u'happiness']
        self.emotion.neutral   = data[u'scores'][u'neutral']
        self.emotion.sadness   = data[u'scores'][u'sadness']
        self.emotion.surprise  = data[u'scores'][u'surprise']

if __name__ == "__main__":
    node_kinect_interpretor()
