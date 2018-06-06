
class KINECT_JOINTS:
    SpineBase      =  0
    SpineMid       =  1
    Neck           =  2
    Head           =  3
    ShoulderLeft   =  4
    ElbowLeft      =  5
    WristLeft      =  6
    HandLeft       =  7
    ShoulderRight  =  8
    ElbowRight     =  9
    WristRight     =  10
    HandRight      =  11
    HipLeft        =  12
    KneeLeft       =  13
    AnkleLeft      =  14
    FootLeft       =  15
    HipRight       =  16
    KneeRight      =  17
    AnkleRight     =  18
    FootRight      =  19
    SpineShoulder  =  20
    HandTipLeft    =  21
    ThumbLeft      =  22
    HandTipRight   =  23
    ThumbRight     =  24

    NUMBER         =  25



MAP_KINECT_STR = {
    KINECT_JOINTS.SpineBase     : "SpineBase",
    KINECT_JOINTS.SpineMid      : "SpineMid",
    KINECT_JOINTS.Neck          : "Neck",
    KINECT_JOINTS.Head          : "Head",
    KINECT_JOINTS.ShoulderLeft  : "ShoulderLeft",
    KINECT_JOINTS.ElbowLeft     : "ElbowLeft",
    KINECT_JOINTS.WristLeft     : "WristLeft",
    KINECT_JOINTS.HandLeft      : "HandLeft ",
    KINECT_JOINTS.ShoulderRight : "ShoulderRight",
    KINECT_JOINTS.ElbowRight    : "ElbowRight",
    KINECT_JOINTS.WristRight    : "WristRight",
    KINECT_JOINTS.HandRight     : "HandRight ",
    KINECT_JOINTS.HipLeft       : "HipLeft",
    KINECT_JOINTS.KneeLeft      : "KneeLeft",
    KINECT_JOINTS.AnkleLeft     : "AnkleLeft",
    KINECT_JOINTS.FootLeft      : "FootLeft",
    KINECT_JOINTS.HipRight      : "HipRight",
    KINECT_JOINTS.KneeRight     : "KneeRight",
    KINECT_JOINTS.AnkleRight    : "AnkleRight",
    KINECT_JOINTS.FootRight     : "FootRight",
    KINECT_JOINTS.SpineShoulder : "SpineShoulder",
    KINECT_JOINTS.HandTipLeft   : "HandTipLeft",
    KINECT_JOINTS.ThumbLeft     : "ThumbLeft",
    KINECT_JOINTS.HandTipRight  : "HandTipRight",
    KINECT_JOINTS.ThumbRight    : "ThumbRight"
}



'''
class BVH_JOINTS:
    Hips           =   0     # px, py, pz, ry, rx, rz
    Chest          =   1     # ry, rx, rz to be removed
    Chest2         =   2     # ry, rx, rz to be removed
    Chest3         =   3     # ry, rx, rz to be removed
    Chest4         =   4     # ry, rx, rz to be removed
    Neck           =   5     # ry, rx, rz
    Head           =   6     # ry, rx, rz
    RightCollar    =   7     # ry, rx, rz to be removed
    RightShoulder  =   8     # ry, rx, rz
    RightElbow     =   9     # ry, rx, rz
    RightWrist     =  10     # ry, rx, rz
    LeftCollar     =  11     # ry, rx, rz to be removed
    LeftShoulder   =  12     # ry, rx, rz
    LeftElbow      =  13     # ry, rx, rz
    LeftWrist      =  14     # ry, rx, rz
    RightHip       =  15     # ry, rx, rz
    RightKnee      =  16     # ry, rx, rz
    RightAnkle     =  17     # ry, rx, rz
    RightToe       =  18     # ry, rx, rz
    LeftHip        =  19     # ry, rx, rz
    LeftKnee       =  20     # ry, rx, rz
    LeftAnkle      =  21     # ry, rx, rz
    LeftToe        =  22     # ry, rx, rz

    NUMBER 		   =  23
'''

class PEPPER_JOINTS:
    HeadYaw         =  0
    HeadPitch       =  1
    LShoulderPitch  =  2
    LShoulderRoll   =  3
    LElbowYaw       =  4
    LElbowRoll      =  5
    LWristYaw       =  6
    RShoulderPitch  =  7
    RShoulderRoll   =  8
    RElbowYaw       =  9
    RElbowRoll      = 10
    RWristYaw       = 11
    # HipRoll         = 12
    # HipPitch        = 13
    # KneePitch       = 14

    NUMBER          = 12

MAP_PEPPER_STR = {
    PEPPER_JOINTS.HeadYaw         : "HeadYaw",
    PEPPER_JOINTS.HeadPitch       : "HeadPitch",
    PEPPER_JOINTS.LShoulderPitch  : "LShoulderPitch",
    PEPPER_JOINTS.LShoulderRoll   : "LShoulderRoll",
    PEPPER_JOINTS.LElbowYaw       : "LElbowYaw",
    PEPPER_JOINTS.LElbowRoll      : "LElbowRoll",
    PEPPER_JOINTS.LWristYaw       : "LWristYaw",
    # PEPPER_JOINTS.LHand           : "LHand",
    PEPPER_JOINTS.RShoulderPitch  : "RShoulderPitch",
    PEPPER_JOINTS.RShoulderRoll   : "RShoulderRoll",
    PEPPER_JOINTS.RElbowYaw       : "RElbowYaw",
    PEPPER_JOINTS.RElbowRoll      : "RElbowRoll",
    PEPPER_JOINTS.RWristYaw       : "RWristYaw"
    # PEPPER_JOINTS.RHand           : "RHand",
    # PEPPER_JOINTS.HipRoll         : "HipRoll",
    # PEPPER_JOINTS.HipPitch        : "HipPitch",
    # PEPPER_JOINTS.KneePitch       : "KneePitch"
}

class BVH_JOINTS:
    Hips           =   0     # px, py, pz, ry, rx, rz
    Neck           =   1     # ry, rx, rz
    Head           =   2     # ry, rx, rz
    RightShoulder  =   3     # ry, rx, rz
    RightElbow     =   4     # ry, rx, rz
    RightWrist     =   5     # ry, rx, rz
    LeftShoulder   =   6     # ry, rx, rz
    LeftElbow      =   7     # ry, rx, rz
    LeftWrist      =   8     # ry, rx, rz
    RightHip       =   9     # ry, rx, rz
    RightKnee      =  10     # ry, rx, rz
    RightAnkle     =  11     # ry, rx, rz
    RightToe       =  12     # ry, rx, rz
    LeftHip        =  13     # ry, rx, rz
    LeftKnee       =  14     # ry, rx, rz
    LeftAnkle      =  15     # ry, rx, rz
    LeftToe        =  16     # ry, rx, rz

    NUMBER 		   =  17



MAP_BVH_STR = {
    BVH_JOINTS.Hips          :  "Hips",
    BVH_JOINTS.Neck          :  "Neck",
    BVH_JOINTS.Head          :  "Head",
    BVH_JOINTS.RightShoulder :  "RightShoulder",
    BVH_JOINTS.RightElbow    :  "RightElbow",
    BVH_JOINTS.RightWrist    :  "RightWrist",
    BVH_JOINTS.LeftShoulder  :  "LeftShoulder",
    BVH_JOINTS.LeftElbow     :  "LeftElbow",
    BVH_JOINTS.LeftWrist     :  "LeftWrist",
    BVH_JOINTS.RightHip      :  "RightHip",
    BVH_JOINTS.RightKnee     :  "RightKnee",
    BVH_JOINTS.RightAnkle    :  "RightAnkle",
    BVH_JOINTS.RightToe      :  "RightToe",
    BVH_JOINTS.LeftHip       :  "LeftHip",
    BVH_JOINTS.LeftKnee      :  "LeftKnee",
    BVH_JOINTS.LeftAnkle     :  "LeftAnkle",
    BVH_JOINTS.LeftToe       :  "LeftToe"
}


MAP_KINECT_BVH = {
    KINECT_JOINTS.SpineBase      :   BVH_JOINTS.Hips ,
    KINECT_JOINTS.Neck           :   BVH_JOINTS.Neck ,
    KINECT_JOINTS.Head           :   BVH_JOINTS.Head ,
    KINECT_JOINTS.ShoulderLeft   :   BVH_JOINTS.LeftShoulder ,
    KINECT_JOINTS.ElbowLeft      :   BVH_JOINTS.LeftElbow ,
    KINECT_JOINTS.WristLeft      :   BVH_JOINTS.LeftWrist ,
    KINECT_JOINTS.ShoulderRight  :   BVH_JOINTS.RightShoulder ,
    KINECT_JOINTS.ElbowRight     :   BVH_JOINTS.RightElbow ,
    KINECT_JOINTS.WristRight     :   BVH_JOINTS.RightWrist ,
    KINECT_JOINTS.HipLeft        :   BVH_JOINTS.LeftHip ,
    KINECT_JOINTS.KneeLeft       :   BVH_JOINTS.LeftKnee ,
    KINECT_JOINTS.AnkleLeft      :   BVH_JOINTS.LeftAnkle ,
    KINECT_JOINTS.FootLeft       :   BVH_JOINTS.LeftToe ,
    KINECT_JOINTS.HipRight       :   BVH_JOINTS.RightHip ,
    KINECT_JOINTS.KneeRight      :   BVH_JOINTS.RightKnee ,
    KINECT_JOINTS.AnkleRight     :   BVH_JOINTS.RightAnkle ,
    KINECT_JOINTS.FootRight      :   BVH_JOINTS.RightToe ,
    KINECT_JOINTS.HandRight      :   None,
    KINECT_JOINTS.HandLeft       :   None ,
    KINECT_JOINTS.HandTipLeft    :   None,
    KINECT_JOINTS.ThumbLeft      :   None,
    KINECT_JOINTS.HandTipRight   :   None,
    KINECT_JOINTS.ThumbRight     :   None
}



MAP_BVH_KINECT = {
    BVH_JOINTS.Hips          :    KINECT_JOINTS.SpineBase     ,
    BVH_JOINTS.Neck          :    KINECT_JOINTS.Neck          ,
    BVH_JOINTS.Head          :    KINECT_JOINTS.Head          ,
    BVH_JOINTS.LeftShoulder  :    KINECT_JOINTS.ShoulderLeft  ,
    BVH_JOINTS.LeftElbow     :    KINECT_JOINTS.ElbowLeft     ,
    BVH_JOINTS.LeftWrist     :    KINECT_JOINTS.WristLeft     ,
    BVH_JOINTS.RightShoulder :    KINECT_JOINTS.ShoulderRight ,
    BVH_JOINTS.RightElbow    :    KINECT_JOINTS.ElbowRight    ,
    BVH_JOINTS.RightWrist    :    KINECT_JOINTS.WristRight    ,
    BVH_JOINTS.LeftHip       :    KINECT_JOINTS.HipLeft       ,
    BVH_JOINTS.LeftKnee      :    KINECT_JOINTS.KneeLeft      ,
    BVH_JOINTS.LeftAnkle     :    KINECT_JOINTS.AnkleLeft     ,
    BVH_JOINTS.LeftToe       :    KINECT_JOINTS.FootLeft      ,
    BVH_JOINTS.RightHip      :    KINECT_JOINTS.HipRight      ,
    BVH_JOINTS.RightKnee     :    KINECT_JOINTS.KneeRight     ,
    BVH_JOINTS.RightAnkle    :    KINECT_JOINTS.AnkleRight    ,
    BVH_JOINTS.RightToe      :    KINECT_JOINTS.FootRight
}

MAP_PEPPER_BVH = {
    PEPPER_JOINTS.HeadYaw         : BVH_JOINTS.Head * 3 + 2,
    PEPPER_JOINTS.HeadPitch       : BVH_JOINTS.Head * 3,
    PEPPER_JOINTS.LShoulderPitch  : BVH_JOINTS.LeftShoulder * 3,
    PEPPER_JOINTS.LShoulderRoll   : BVH_JOINTS.LeftShoulder * 3 + 1,
    PEPPER_JOINTS.LElbowYaw       : BVH_JOINTS.LeftElbow * 3 + 2,
    PEPPER_JOINTS.LElbowRoll      : BVH_JOINTS.LeftElbow * 3 + 1,
    PEPPER_JOINTS.LWristYaw       : BVH_JOINTS.LeftWrist * 3 + 2,
    PEPPER_JOINTS.RShoulderPitch  : BVH_JOINTS.RightShoulder * 3,
    PEPPER_JOINTS.RShoulderRoll   : BVH_JOINTS.RightShoulder * 3 + 1,
    PEPPER_JOINTS.RElbowYaw       : BVH_JOINTS.RightElbow * 3 + 2,
    PEPPER_JOINTS.RElbowRoll      : BVH_JOINTS.RightElbow * 3 + 1,
    PEPPER_JOINTS.RWristYaw       : BVH_JOINTS.RightWrist * 3 + 2
    # PEPPER_JOINTS.HipRoll         : BVH_JOINTS.Hips * 3 + 1,
    # PEPPER_JOINTS.HipPitch        : BVH_JOINTS.Hips * 3,
    # PEPPER_JOINTS.KneePitch       : BVH_JOINTS.RightKnee * 3
}
