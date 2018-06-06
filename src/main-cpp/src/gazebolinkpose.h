#ifndef GAZEBOLINKPOSE_H
#define GAZEBOLINKPOSE_H

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class GazeboLinkPose
{
public:
  GazeboLinkPose();
  void Init();

  Vector3d* actorPose;
  Vector3d* pepperPose;

private:
  ros::Subscriber* mLinkStateSub;
  ros::NodeHandle mNodeHandle;
  std::map<std::string, int> mActorJointIndex;
  std::map<std::string, int> mPepperJointIndex;

  void OnUpdateCallback();
  void ParseMsg();
};

#endif // GAZEBOLINKPOSE_H
