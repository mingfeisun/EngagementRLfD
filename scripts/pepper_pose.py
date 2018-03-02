#!/usr/bin/env python

import qi
import motion
import numpy
import pandas

import os.path

from pandas import HDFStore
from mpi_proc import SkeletonFromMPI

from JointMapping import PEPPER_JOINTS
from JointMapping import MAP_PEPPER_STR
from JointMapping import MAP_PEPPER_BVH


class PepperMotion:
    def __init__(self):
        self.poses = dict()
        for i in range(PEPPER_JOINTS.NUMBER):
            self.poses[MAP_PEPPER_STR[i]] = list()

    def generate(self, data):
        file = "data/pepper_pose.h5"
        if os.path.exists(file):
            store = HDFStore(file)
            self.pose_data = store['data']
            return

        print "Generating pepper pose..."
        motion_data = data["motion_data"].values
        self.emotion_intended = data["intended_emotion"].values
        self.emotion_perceived = data["accurate_category"].values

        for seq in motion_data:
            for i in range(PEPPER_JOINTS.NUMBER):
                seq_array = numpy.array(seq)
                to_pose = seq_array[:, MAP_PEPPER_BVH[i]] * motion.TO_RAD
                self.poses[MAP_PEPPER_STR[i]].append(to_pose)

        self.pose_data = pandas.DataFrame(self.poses)

        store = HDFStore(file)
        store['data'] = self.pose_data

    def showEmotion(self, emo):
        pass

    def test(self):
        print "testing"
        session = qi.Session()

        joints = list()
        for i in range(PEPPER_JOINTS.NUMBER):
            joints.append(MAP_PEPPER_STR[i])

        try:
            session.connect("tcp://192.168.1.101:9559")
            motion_service = session.service("ALMotion")
            posture_service = session.service("ALRobotPosture")

            motion_service.wakeUp()
            posture_service.goToPosture("StandInit", 0.6)
            motion_service.setMoveArmsEnabled(True, True)

            selector = 9
            for index in range(self.pose_data["HeadYaw"][selector].shape[0]):
                poses = list()
                for each in joints:
                    poses.append(float(self.pose_data[each][selector][index]))

                motion_service.setAngles(joints, poses, 0.5)



        except RuntimeError:
            print "Connection error"

        # motion_service.wakeUp()
        # posture_service.goToPosture("StandInit", 0.6)
        # motion_service.setStiffnesses("Body", 0.0)


if __name__ == "__main__":
    ske = SkeletonFromMPI()
    data = ske.loadSpace()
    # data = None

    pepper = PepperMotion()
    pepper.generate(data)

    pepper.test()


