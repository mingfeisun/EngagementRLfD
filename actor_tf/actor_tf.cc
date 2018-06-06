#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>

std::map<std::string, std::string> parentMap;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void callbackMsg(ConstPoseAnimationPtr &_msg)
{
  static tf::TransformBroadcaster br;

  tf::Transform transform;
  ros::Time timestamp = ros::Time::now();
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0));

  gazebo::msgs::Pose actorPose;

  for(int i = 0; i < _msg->pose_size(); i++){
    const gazebo::msgs::Pose currPose = _msg->pose(i);

    std::size_t _pActor =currPose.name().find("actor::");

    if( _pActor != std::string::npos || currPose.name() == "Hips"){
      continue;
    }

    std::string thisFrame = "actor_" + currPose.name();
    std::string parentFrame = "actor_" + parentMap[currPose.name()];

    if(currPose.name() == "actor" ){
      thisFrame = "actor_Hips";
      parentFrame = "odom";
    }

    const gazebo::msgs::Quaternion quat = currPose.orientation();
    const gazebo::msgs::Vector3d pos = currPose.position();

    tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w());
    transform.setRotation(q);

    tf::Vector3 p(pos.x(), pos.y(), pos.z());
    transform.setOrigin(p);

    br.sendTransform( tf::StampedTransform(transform, timestamp, parentFrame, thisFrame));
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  ros::init(_argc, _argv, "actor_publisher");

  // add tree structure
  parentMap["LeftToeBase"]  = "LeftFoot";
  parentMap["LeftFoot"]     = "LeftLeg";
  parentMap["LeftLeg"]      = "LeftUpLeg";
  parentMap["LeftUpLeg"]    = "LHipJoint";
  parentMap["LHipJoint"]    = "Hips";

  parentMap["RightToeBase"] = "RightFoot";
  parentMap["RightFoot"]    = "RightLeg";
  parentMap["RightLeg"]     = "RightUpLeg";
  parentMap["RightUpLeg"]   = "RHipJoint";
  parentMap["RHipJoint"]    = "Hips";

  parentMap["Head"]    = "Neck1";
  parentMap["Neck1"]   = "Neck";
  parentMap["Neck"]    = "Spine1";
  parentMap["Spine1"]  = "Spine";
  parentMap["Spine"]   = "LowerBack";
  parentMap["LowerBack"]   = "Hips";

  parentMap["LeftHandIndex1"]   = "LeftFingerBase";
  parentMap["LeftFingerBase"]   = "LeftHand";
  parentMap["LeftHand"]         = "LeftForeArm";
  parentMap["LeftForeArm"]      = "LeftArm";
  parentMap["LeftArm"]          = "LeftShoulder";
  parentMap["LeftShoulder"]     = "Spine1";
  parentMap["LThumb"]           = "LeftHand";

  parentMap["RightHandIndex1"]  = "RightFingerBase";
  parentMap["RightFingerBase"]  = "RightHand";
  parentMap["RightHand"]        = "RightForeArm";
  parentMap["RightForeArm"]     = "RightArm";
  parentMap["RightArm"]         = "RightShoulder";
  parentMap["RightShoulder"]    = "Spine1";
  parentMap["RThumb"]           = "RightHand";

  parentMap["Hips"] = "odom";

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/skeleton_pose/info", callbackMsg);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
