#include "gazebolinkpose.h"

GazeboLinkPose::GazeboLinkPose()
  :mLinkStateSub(nullptr)
{
  this->pepperPose = new Vector3f[4];
  this->actorPose = new Vector3f[20];
  const std::string actorJoints[] = {
    "actor::Hips"             ,
    "actor::LHipJoint"        ,
    "actor::LeftUpLeg"        ,
    "actor::LeftLeg"          ,
    "actor::LeftFoot"         ,
    "actor::LeftToeBase"      ,
    "actor::RHipJoint"        ,
    "actor::RightUpLeg"       ,
    "actor::RightLeg"         ,
    "actor::RightFoot"        ,
    "actor::RightToeBase"     ,
    "actor::LowerBack"        ,
    "actor::Spine"            ,
    "actor::Spine1"           ,
    "actor::Neck"             ,
    "actor::Neck1"            ,
    "actor::Head"             ,
    "actor::LeftShoulder"     ,
    "actor::LeftArm"          ,
    "actor::LeftForeArm"      ,
    "actor::LeftHand"         ,
    "actor::LeftFingerBase"   ,
    //"actor::LeftHandFinger1",
    "actor::LeftHandIndex1"   ,
    "actor::LThumb"           ,
    "actor::RightShoulder"    ,
    "actor::RightArm"         ,
    "actor::RightForeArm"     ,
    "actor::RightHand"        ,
    "actor::RightFingerBase"  ,
    //"actor::RightHandFinger1",
    "actor::RightHandIndex1"  ,
    "actor::RThumb"           ,
    "actor::actor_pose"};
}

void GazeboLinkPose::Init()
{
  this->mNodeHandle = new ros::NodeHandle('~GazeboLinkPose');
  this->mLinkStateSub = this->mNodeHandle->subscribe('/gazebo/link_states', 100, this->OnUpdateCallback);
}

void GazeboLinkPose::OnUpdateCallback()
{

}

void GazeboLinkPose::ParseMsg()
{

}
