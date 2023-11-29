#include "MotionObject.h"

namespace MotionEngine
{

// adding Pose as a MotionObject property
MotionObject::MotionObject() :
    ObjectInitialPose(Eigen::Matrix4d::Identity())
{
    ObjectName.Reset(20);
    ObjectType = MotionObjectType::UNKNOWN;
}

MotionObject::~MotionObject()
{

}

}