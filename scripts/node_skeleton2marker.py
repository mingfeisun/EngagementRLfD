#!/usr/bin/env python

import rospy

from emotion_intensity.msg import Skeleton

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

NUMBER_BM = 15
NUMBER_MPI = 23
NUMBER_KINECT = 25

JOINT_NUMBER = NUMBER_KINECT
# JOINT_NUMBER = NUMBER_BM
# JOINT_NUMBER = NUMBER_MPI

class PublishMarkers():
    def __init__(self):
        rospy.init_node('skeleton_markers')

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Initializing Skeleton Markers Node...")

        self.rate = rospy.get_param('~rate', 10)
        self.scale = rospy.get_param('~scale', 0.1)
        self.lifetime = rospy.get_param('~lifetime', 0) # 0 is forever
        self.color = rospy.get_param('~color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})

        rate = rospy.Rate(self.rate)

        # Subscribe to the skeleton topic.
        rospy.Subscriber('node_skeleton/skeleton', Skeleton, self.skeleton_handler)

        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('node_skeleton/skeleton_markers', MarkerArray)

        self.marker_array = MarkerArray()
        self.count = 0

        while not rospy.is_shutdown():
            self.marker_pub.publish(self.marker_array)
            rate.sleep()

    def skeleton_handler(self, msg):
        # Initialize the marker points list.

        for joint in msg.name:
            i = msg.name.index(joint)

            marker = Marker()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.MODIFY
            marker.scale.x = self.scale
            marker.scale.y = self.scale
            marker.scale.z = self.scale
            marker.color.r = self.color['r']
            marker.color.g = self.color['g']
            marker.color.b = self.color['b']
            marker.color.a = self.color['a']

            marker.header.frame_id = "skeleton_marker"
            marker.header.stamp = rospy.Time.now()


            try:
                marker.pose.orientation.x = msg.orientation[i].x
                marker.pose.orientation.y = msg.orientation[i].y
                marker.pose.orientation.z = msg.orientation[i].z
                marker.pose.orientation.w = msg.orientation[i].w
                marker.pose.position.x = msg.position[i].x
                marker.pose.position.y = msg.position[i].y
                marker.pose.position.z = msg.position[i].z
            except IndexError:
                rospy.loginfo('Position or orientation index error')

            if self.count > JOINT_NUMBER:
                self.marker_array.markers.pop(0)
                self.count -= 1

            self.marker_array.markers.append(marker)
            self.count += 1

    def shutdown(self):
        rospy.loginfo('Shutting down Skeleton Marker Node.')

if __name__ == "__main__":
    PublishMarkers()
