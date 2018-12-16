import rospy
import std_msgs
import numpy as np
from xml.dom.minidom import parseString

class PepperRobot(object):
    def __init__(self):
        self.joint_names = []
        self.joint_limits = dict()

        self.sub_joint = rospy.Subscriber('/joint_states', self.cb_jointUpdate)
        self.joint_states = dict()

    @staticmethod
    def get_param(name, value=None):
        # https://github.com/ros/joint_state_publisher/blob/kinetic-devel/joint_state_publisher/joint_state_publisher/joint_state_publisher#L33
        private = '~%s'%name
        if rospy.has_param(private):
            return rospy.get_param(private)
        elif rospy.has_param(name):
            return rospy.get_param(name)
        else: 
            return value

    def init_joint(self):
        robot = self.robot_urdf.getElementsByTagName('robot')[0]

        # get all joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype in ['fixed', 'floating', 'planar']:
                    continue
                name = child.getAttribute('name')
                self.joint_names.append(name)
                self.joint_limits[name] = [0.0, 0.0]
                if jtype == 'continuous':
                    self.joint_limits[name][0] = -np.pi
                    self.joint_limits[name][1] = np.pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    self.joint_limits[name][0] = float(limit.getAttribute('lower'))
                    self.joint_limits[name][1] = float(limit.getAttribute('upper'))

        # set all joints to 0.0
        for joint in self.joint_names:
            self.sendCommand(joint, 0.0)

    def init(self):
        self.robot_urdf = parseString(self.get_param('robot_description'))
        self.init_joint()

    def sendCommand(self, _joint_name, _val):
        assert _val >= self.joint_limits[_joint_name][0] and _val <= self.joint_limits[_joint_name][1]
        command_str = self.getCommandString(_joint_name)
        tmp_pub = rospy.Publisher(command_str, std_msgs.msg.Float64)
        tmp_msg = std_msgs.msg.Float64()
        tmp_msg.data = 0.0
        tmp_pub.publish(tmp_msg)

    def getCommandString(self, _joint_name):
        return "/pepper_dcm/" + _joint_name + "_position_controller/command"

    def getJointsConfigs(self):
        if len(self.joint_states) == 0:
            return None
        return self.joint_states

    def getJointConfigs(self, _joint_name):
        assert _joint_name in self.joint_names
        return self.joint_states[_joint_name]

    def cb_jointUpdate(self, msg_data):
        t_name = msg_data.name
        t_position = msg_data.position
        t_velocity = msg_data.velocity
        t_effort = msg_data.effort

        for idx in range(len(t_name)):
            name = t_name[idx]
            assert name in self.joint_names
            position = t_position[idx]
            velocity = t_velocity[idx]
            effort = t_effort[idx]
            tmp_dict = self.joint_states.get(name, {})
            tmp_dict['position'] = position
            tmp_dict['velocity'] = velocity
            tmp_dict['effort'] = effort

class BodyPose(object):
    def __init__(self):
        pass

class ApproxMimic(object):
    def __init__(self):
        pass

    def __call__(self):
        pass 

    def robotInitPose(self):
        pass

    def humanInitPose(self):
        pass

    def findCorrespondence(self):
        pass

    def getTF(self, _parent, _child):
        pass

    def mimicTF(self):
        pass

if __name__ == "__main__":
    pass