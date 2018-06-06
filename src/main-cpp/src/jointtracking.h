#ifndef JOINTTRACKING_H
#define JOINTTRACKING_H

#include <eigen3/Eigen/Dense>

#include "particlefilter.h"

using namespace Eigen;

class JointTracking
{
public:
  JointTracking();

private:
  int* mTrackedJoints;

  ParticleFilter* mPFilter;

  Vector3f* mLastMeasurement2;
  Vector3f* mLastMeasurement1;

};

#endif // JOINTTRACKING_H
