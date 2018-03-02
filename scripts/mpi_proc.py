#!/usr/bin/env python

import rospy
import pandas
import numpy
import keras
import os.path

from pandas import HDFStore
from const import MAX_SEQUENCE_LENGTH
from const import TRAINING_PERCENTAGE

from emotion_intensity.msg import Skeleton
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from keras import  losses
from keras.models import Sequential, Model
from keras.layers import Dense, LSTM, Input
from keras.optimizers import RMSprop
from keras.preprocessing import sequence
from keras.utils.np_utils import to_categorical
from keras.models import load_model

from math import radians, cos, sin
from cgkit.bvh import BVHReader
from numpy import array, dot

intensity_levels = 4

# intensity_levels = 2

class BvhPublisher:
    def __init__(self):
        rospy.init_node('body_motion_publisher')
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initializing body motion publisher...")

        self.rate = rospy.Rate(120) # mpi data captured at 120 Hz

        self.skeleton = Skeleton()

        self.pub_skeleton = rospy.Publisher("node_skeleton/skeleton", Skeleton, queue_size=1)


    def publish(self, data_list):
        for each in data_list:
            self.skeleton.header.frame_id = "/skeleton"
            self.skeleton.header.stamp = rospy.Time.now()

            self.skeleton.name = []
            self.skeleton.position = []
            self.skeleton.orientation = []

            for name, value in each.iteritems():
                self.skeleton.name.append(name)

                temp_point = Point()
                temp_point.x = value[0]
                temp_point.y = value[1]
                temp_point.z = value[2]
                self.skeleton.position.append(temp_point)

                temp_orien = Quaternion()
                temp_orien.w = 1
                self.skeleton.orientation.append(temp_orien)

            self.skeleton.confidence = numpy.ones(len(self.skeleton.name))

            self.rate.sleep()
            self.pub_skeleton.publish(self.skeleton)

    def populate_skeleton(self, node):
        if len(node.children) != 0:
            self.skeleton.name.append(node.name)
            self.skeleton.confidence.append(1)

            temp_point = Point()
            temp_point.x = node.worldpos[1][0]
            temp_point.y = node.worldpos[1][1]
            temp_point.z = node.worldpos[1][2]

            temp_orien = Quaternion()
            temp_orien.w = 1

            self.skeleton.position.append(temp_point)
            self.skeleton.orientation.append(temp_orien)
            for child in node.children:
                self.populate_skeleton(child)

    def shutdown(self):
        rospy.loginfo("Shutting down body motion publisher...")


class JointClass:
    def __init__(self, name):
        self.name = name
        self.children = []
        self.channels = []  # Set later.  Ordered list of channels: each
        self.hasparent = 0  # flag
        self.parent = 0  # joint.addchild() sets this
        self.strans = array([0.,0.,0.])

        self.stransmat = array([ [0.,0.,0.,0.],
                                 [0.,0.,0.,0.],
                                 [0.,0.,0.,0.],
                                 [0.,0.,0.,0.] ])

        self.trtr = {}
        self.worldpos = {}

    def addchild(self, childjoint):
        self.children.append(childjoint)
        childjoint.hasparent = 1
        childjoint.parent = self


class BvhLoader(BVHReader):
    def onHierarchy(self, root):
        self.root = root
        self.keyframes = []

    def onMotion(self, frames, dt):
        self.frames = frames
        self.dt = dt

    def onFrame(self, values):
        self.keyframes.append(values)

    def ProcessBvhNode(self, node, parentname='hips'):
        name = node.name
        if (name == "End Site") or (name == "end site"):
            name = parentname + "End"
        b1 = JointClass(name)
        b1.channels = node.channels

        b1.strans[0] = node.offset[0]
        b1.strans[1] = node.offset[1]
        b1.strans[2] = node.offset[2]

        b1.stransmat = array([ [1.,0.,0.,0.],
                               [0.,1.,0.,0.],
                               [0.,0.,1.,0.],
                               [0.,0.,0.,1.] ])

        b1.stransmat[0,3] = b1.strans[0]
        b1.stransmat[1,3] = b1.strans[1]
        b1.stransmat[2,3] = b1.strans[2]

        for child in node.children:
            b2 = self.ProcessBvhNode(child, name)
            b1.addchild(b2)
        return b1

    def ProcessBvhKeyframe(self, keyframe, joint, t, DEBUG=0):

        counter = 0
        dotrans = 0
        dorot = 0

        drotmat = array([ [1.,0.,0.,0.],
                          [0.,1.,0.,0.],
                          [0.,0.,1.,0.],
                          [0.,0.,0.,1.] ])

        for channel in joint.channels:
            keyval = keyframe[counter]
            if(channel == "Xposition"):
                dotrans = 1
                xpos = keyval
            elif(channel == "Yposition"):
                dotrans = 1
                ypos = keyval
            elif(channel == "Zposition"):
                dotrans = 1
                zpos = keyval
            elif(channel == "Xrotation"):
                dorot = 1
                xrot = keyval
                theta = radians(xrot)
                mycos = cos(theta)
                mysin = sin(theta)
                drotmat2 = array([ [1.,0.,0.,0.],
                                   [0.,1.,0.,0.],
                                   [0.,0.,1.,0.],
                                   [0.,0.,0.,1.] ])
                drotmat2[1,1] = mycos
                drotmat2[1,2] = -mysin
                drotmat2[2,1] = mysin
                drotmat2[2,2] = mycos
                drotmat = dot(drotmat, drotmat2)

            elif(channel == "Yrotation"):
                dorot = 1
                yrot = keyval
                theta = radians(yrot)
                mycos = cos(theta)
                mysin = sin(theta)
                drotmat2 = array([ [1.,0.,0.,0.],
                                   [0.,1.,0.,0.],
                                   [0.,0.,1.,0.],
                                   [0.,0.,0.,1.] ])
                drotmat2[0,0] = mycos
                drotmat2[0,2] = mysin
                drotmat2[2,0] = -mysin
                drotmat2[2,2] = mycos
                drotmat = dot(drotmat, drotmat2)

            elif(channel == "Zrotation"):
                dorot = 1
                zrot = keyval
                theta = radians(zrot)
                mycos = cos(theta)
                mysin = sin(theta)
                drotmat2 = array([ [1.,0.,0.,0.],
                                   [0.,1.,0.,0.],
                                   [0.,0.,1.,0.],
                                   [0.,0.,0.,1.] ])
                drotmat2[0,0] = mycos
                drotmat2[0,1] = -mysin
                drotmat2[1,0] = mysin
                drotmat2[1,1] = mycos
                drotmat = dot(drotmat, drotmat2)
            else:
                return
            counter += 1

        if dotrans:
            dtransmat = array([ [1.,0.,0.,0.],
                                [0.,1.,0.,0.],
                                [0.,0.,1.,0.],
                                [0.,0.,0.,1.] ])
            dtransmat[0,3] = xpos
            dtransmat[1,3] = ypos
            dtransmat[2,3] = zpos

        if joint.hasparent:  # Not hips
            parent_trtr = joint.parent.trtr[t]  # Dictionary-based rewrite
            localtoworld = dot(parent_trtr,joint.stransmat)

        else:  # Hips
            localtoworld = dot(joint.stransmat,dtransmat)

        trtr = dot(localtoworld,drotmat)
        joint.trtr[t] = trtr

        worldpos = array([ localtoworld[0,3],
                           localtoworld[1,3],
                           localtoworld[2,3],
                           localtoworld[3,3] ])

        joint.worldpos[t] = worldpos
        newkeyframe = keyframe[counter:]  # Slices from counter+1 to end

        for child in joint.children:
            newkeyframe = self.ProcessBvhKeyframe(newkeyframe, child, t, DEBUG=DEBUG)
            if(newkeyframe == 0):  # If retval = 0
                return
        return newkeyframe

    def Populate(self, root_joint, node_dict):
        if len(root_joint.children) != 0:
            node_dict[root_joint.name] = (root_joint.worldpos[1][0],
                                          root_joint.worldpos[1][1],
                                          root_joint.worldpos[1][2])

            for child in root_joint.children:
                self.Populate(child, node_dict)

    def ParseDataframe(self):
        data_dict = list()
        root_node = self.ProcessBvhNode(self.root)
        for frame in self.keyframes:
            temp_dict = dict()
            self.ProcessBvhKeyframe(frame, root_node, 1)
            self.Populate(root_node, temp_dict)
            data_dict.append(temp_dict)
        return data_dict


class SkeletonFromMPI:
    '''
    CSV DataFrame Column Names
    .----------------------------------------------------------------------------.
    |motion_id,     | intended_emotion, | intended_polarity,| modal_category,    |
    |---------------|-------------------|-------------------|--------------------|
    |modal_polarity,| accurate_category,| accurate_polarity,| duration,          |
    |---------------|-------------------|-------------------|--------------------|
    |peaks,         | speed,            | acting_subtask,   | actor,             |
    |---------------|-------------------|-------------------|--------------------|
    |gender,        | age,              | handedness,       | native_tongue,     |
    |---------------|-------------------|-------------------|--------------------|
    |responses,     | consistency,      | text,             | emotion_intensity  |
    .----------------------------------------------------------------------------.
    '''
    def __init__(self):
        pass

    def loadData(self):
        print "Loading data..."
        mpi_folder = "/home/mingfei/Documents/InteractDetect/dataset/MPI/"
        sub_folder = "bvh/"

        csv_file = "emotional_body_motion_database_metadata.csv"
        self.csv_data = pandas.read_csv( mpi_folder + csv_file, index_col=0)

        self.csvParser()

        raw_data = []
        index = 1
        total = len(self.csv_data)
        failed_files = []
        for each in self.csv_data.motion_id:
            filename = "%s%s%s.bvh"%(mpi_folder, sub_folder, each)
            try:
                bvh = BvhLoader(filename)
                bvh.read()
                # temp_data = bvh.ParseDataframe()
                temp_data = bvh.keyframes
            except ValueError:
                temp_data = numpy.nan
                failed_files.append(each)

            raw_data.append(temp_data)
            index += 1
        self.csv_data["motion_data"] = raw_data
        self.csv_data = self.csv_data.dropna(subset=["motion_data"])

        # print failed_files

    def csvParser(self):
        print "Load csv meta data ..."
        emotion_intensity = list()

        # consider only four basic emotions
        basic_emotions = ["surprise", "joy", "sadness", "neutral"]
        selector = [each in basic_emotions for each in self.csv_data["intended_emotion"]]
        self.csv_data = self.csv_data[selector]

        for index, row in self.csv_data.iterrows():
            occu = row["responses"].count(row["intended_emotion"])
            total_occu = 11     # always 11 observations
            intensity_level = occu * 1.0/ total_occu
            emotion_intensity.append(intensity_level)

        self.csv_data["emotion_intensity"] = emotion_intensity

    def loadSpace(self):
        file = "data/mpi_data.h5"
        if os.path.exists(file):
            store = HDFStore(file)
            return store['data']

        self.loadData()

        # remove redundant columns
        delete_cols = [ "motion_id", "accurate_polarity",
                        "duration", "peaks", "speed", "acting_subtask", "actor",
                        "gender", "age", "handedness", "native_tongue", "text",
                        "span", "acting_task"]
        for col in delete_cols:
            del self.csv_data[col]

        # save data
        store = HDFStore(file)
        store['data'] = self.csv_data

        return self.csv_data


class MotionProcessing:

    def __init__(self):
        self.train_data = []
        self.emotion_label = []
        self.intensity_label = []
        self.model_name = "models/mpi_lstm.h5"
        self.emotion = ["surprise", "joy", "sadness", "neutral"]

        # self.emotion = ["surprise"]
        # self.emotion = ["joy"]
        # self.emotion = ["sadness"]
        # self.emotion = ["neutral"]

    def preprocess(self, data_df):
        print "Preprocessing ..."

        # remove row #542: anger
        # data_df = data_df.drop(data_df.index[542])

        # remove row #474: surprise
        data_df = data_df.drop(data_df.index[595])

        self.train_data = data_df["motion_data"].values


        ################################################################################
        # calculate deviations
        # pre_pose = []
        # for i in range(len(self.train_data)) :
        #     for j in range(len(self.train_data[i])):
        #         if len(pre_pose) == 0:
        #             pre_pose = numpy.array(self.train_data[i][j])
        #         else:
        #             pre_pose = numpy.array(self.train_data[i][j])
        #             self.train_data[i][j] = numpy.array(self.train_data[i][j]) - pre_pose
        ################################################################################

        self.emotion_label = [self.emotion.index(each) for each in data_df["intended_emotion"].values]
        self.emotion_label = numpy.array(self.emotion_label)

        if intensity_levels == 4:
            labels = ["{0}".format(int(4*i)) for i in numpy.arange(0, 1, 0.25)]
            self.intensity_label = pandas.cut(data_df.emotion_intensity,
                                              numpy.arange(-0.01, 1.2, 0.25),
                                              labels=labels).values
        if intensity_levels == 2:
            labels = ["{0}".format(int(2*i)) for i in numpy.arange(0, 1, 0.51)]
            self.intensity_label = pandas.cut(data_df.emotion_intensity,
                                              [-0.01, 0.2, 1.1],
                                              labels=labels).values


    def dataManipulation(self):
        data = self.train_data
        sk_label = self.intensity_label
        fa_label = self.emotion_label

        # delete dimensions in data
        delete_cols = [11, 7, 4, 3, 2, 1] # must be reverse order
        for i in range(len(data)):
            each = numpy.array(data[i])

            time_to_go = 3
            while time_to_go: # remove first 3 offset cols
                each = numpy.delete(each, 0, axis=1)
                time_to_go -= 1

            for col in delete_cols: # remove none-corresponding cols
                time_to_go = 3 # 3 coordinates
                while time_to_go:
                    each = numpy.delete(each, col*3, axis=1)
                    time_to_go -= 1

            data[i] = each
        dim_sk = 17*3

        dim_fa = 4

        # random splitting
        select_index = numpy.arange(1, len(data)+1)
        select_index = numpy.random.permutation(select_index)

        percent = TRAINING_PERCENTAGE

        x_train = data[select_index <= percent*len(data)]
        y_train_sk = sk_label[select_index <= percent*len(data)]
        y_train_fa = fa_label[select_index <= percent*len(data)]
        y_category_train_sk = to_categorical(y_train_sk, num_classes=None)
        y_category_train_fa = to_categorical(y_train_fa, num_classes=None)

        x_test = data[select_index > percent*len(data)]
        y_test_sk = sk_label[select_index > percent*len(data)]
        y_test_fa = fa_label[select_index > percent*len(data)]
        y_category_test_sk = to_categorical(y_test_sk, num_classes=None)
        y_category_test_fa = to_categorical(y_test_fa, num_classes=None)

        # padding
        max_length = MAX_SEQUENCE_LENGTH
        x_train = sequence.pad_sequences(x_train, maxlen=max_length)
        x_test = sequence.pad_sequences(x_test, maxlen=max_length)

        return x_train, y_category_train_sk, y_category_train_fa, \
               x_test, y_category_test_sk, y_category_test_fa, dim_sk, dim_fa

    def styleLearningMod(self):
        print "Training using models..."
        x, y, x_fa, x_test, y_test, x_test_fa, dim_sk, dim_fa = self.dataManipulation()

        model = self.constructModel(dim_sk, dim_fa)

        model.fit([x, x_fa], y, epochs=15, batch_size=32)

        score, acc = model.evaluate([x_test, x_test_fa], y_test, batch_size=32)
        print 'Test score: %f, accuracy: %f'%(score, acc)

        model.save(self.model_name)
        print "Model saved as: %s"%(self.model_name)

    def constructModel(self, skeleton_dim, face_dim):
        main_input = Input(shape=(None, skeleton_dim), name="main_input")
        lstm_128 = LSTM(128, return_sequences=True)(main_input)
        lstm_out = LSTM(32)(lstm_128)

        aux_input = Input(shape=(face_dim,), name="aux_input")
        joint_x = keras.layers.concatenate([lstm_out, aux_input])

        # main_output = Dense(4, activation="softmax")(joint_x)
        main_output = Dense(intensity_levels, activation="softmax")(joint_x)

        model = Model(inputs=[main_input, aux_input], outputs=main_output)

        optimizer = RMSprop(lr=0.02)
        model.compile(loss=losses.categorical_crossentropy,
                      optimizer=optimizer,
                      metrics=['accuracy'])
        return model


    def loadModel(self):
        self.model = load_model(self.model_name)
        return self.model


if __name__ == "__main__":

    test = SkeletonFromMPI()
    data = test.loadSpace()

    proc = MotionProcessing()
    # model = proc.loadModel()
    proc.preprocess(data)
    proc.styleLearningMod()

    # pub = BvhPublisher()
    # pub.publish(data["motion_data"][0])

    # mpi_folder = "/home/mingfei/Documents/InteractDetect/dataset/MPI/"
    # sub_folder = "bvh/"
    # filename = "AnBh_0208_ftf_wd_t3-001_8_91017_94192_3175_Surprise.bvh"
    # test = BvhLoader(mpi_folder + sub_folder + filename)
    # test.read()

    # node = test.ProcessBvhNode(test.root)

    # frames = test.ProcessBvhKeyframe(test.keyframes, node, test.dt)

    # pub = BvhPublisher()
    # b.publish(frames)
