#!/usr/bin/env python

import numpy
import rospy
import math

from naoqi import ALBroker
from const import NAO_IP
from const import NAO_PORT

from scipy.signal import wiener

from const import MAX_SEQUENCE_LENGTH
from const import KINECT_JOINT_NUMBER

from emotion_intensity.msg import Emotion
from emotion_intensity.msg import Skeleton

from JointMapping import BVH_JOINTS
from JointMapping import MAP_KINECT_STR
from JointMapping import MAP_BVH_KINECT

from keras.models import load_model

from node_strategy import RobotStrategy
from node_strategy import ReactToTouch

class FacialPoseSubscriber:

    def __init__(self):
        self.emotion = ["anger", "happiness", "sadness", "neutral"]
        self.facial_expr = numpy.array([])
        self.pose_sequence = list()

        rospy.init_node("facial_pose_subscriber")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initializing facial/pose subscriber...")
        rospy.Subscriber("node_face/emotion_category", Emotion, self.emotionCallback)
        rospy.Subscriber("node_skeleton/skeleton", Skeleton, self.skeletonCallback)

    def emotionCallback(self, msg):
        self.facial_expr = numpy.array([])
        self.facial_expr = numpy.append(self.facial_expr, msg.surprise)
        self.facial_expr = numpy.append(self.facial_expr, msg.happiness)
        self.facial_expr = numpy.append(self.facial_expr, msg.neutral)
        self.facial_expr = numpy.append(self.facial_expr, msg.sadness)
        self.facial_expr /= sum(self.facial_expr)

    def skeletonCallback(self, msg):
        kinect_skeleton = dict()

        names = msg.name
        orientation = msg.orientation
        # print "Names len: %d, orientation len: %d"%(len(names), len(orientation))
        for each in names:
            i = names.index(each)
            try:
                kinect_skeleton[each] = self.quaternion2euler(orientation[i])
            except IndexError:
                print "Name: %s, index: %d"%(each, i)
                print "Names len: %d, orientation len: %d"%(len(names), len(orientation))

        if len(kinect_skeleton) == KINECT_JOINT_NUMBER:
            skeleton = self.formatKinectData(kinect=kinect_skeleton)
            if len(self.pose_sequence) > MAX_SEQUENCE_LENGTH:
                self.pose_sequence.pop(0)
            if len(skeleton) == 17*3:
                skeleton = numpy.array(skeleton)
                self.pose_sequence.append(skeleton)

    def formatKinectData(self, kinect):
        # kinect is dict type. YXZ
        joint_list = []
        for i in range(BVH_JOINTS.NUMBER):
            index = MAP_BVH_KINECT[i]
            try:
                joint_list.append(kinect[MAP_KINECT_STR[index]][1])
                joint_list.append(kinect[MAP_KINECT_STR[index]][0])
                joint_list.append(kinect[MAP_KINECT_STR[index]][2])
            except Exception, e:
                rospy.logwarn("Format kinect data: %s"%(e))
        return joint_list

    def quaternion2euler(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        pitchY = math.atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / math.pi * 180.0
        yawZ = math.asin(2 * ((w * y) - (x * z))) / math.pi * 180.0
        rollX = math.atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / math.pi * 180.0

        return (pitchY, yawZ, rollX)

        # temp_tuple = (quat.x, quat.y, quat.z, quat.w)
        # return tf.transformations.euler_from_quaternion(temp_tuple)

    def shutdown(self):
        rospy.loginfo("Shutdown facial/pose subscriber node")

class Model:
    def __init__(self):
        self.model = self.loadTrainedModel()

    def loadTrainedModel(self):
        model_name = "models/mpi_lstm.h5"
        return load_model(model_name)

    def run(self, facial, sequence):

        if len(sequence) >= 3 and len(facial) != 0:
            # print self.sequences
            est = self.model.predict([numpy.array([numpy.array(sequence)]),
                                      numpy.array([facial])],
                                     batch_size=1)
            return facial, est

        return [], [] # otherwise return none



if __name__ == "__main__":
    myBroker = ALBroker("myBroker", "0.0.0.0", 0, NAO_IP, NAO_PORT)
    global ReactToTouch
    ReactToTouch = ReactToTouch("ReactToTouch")
    action = RobotStrategy()

    action.pickStrategy(1)

    sub = FacialPoseSubscriber()
    model = Model()

    while not rospy.is_shutdown():
        emotion, intensity = model.run(sub.facial_expr, sub.pose_sequence)
        if ReactToTouch.ready:
            action.execute(emotion, intensity)
        rospy.sleep(0.1)
