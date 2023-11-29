#include "Joint.h"

namespace MotionEngine
{

JointLimits::JointLimits()
{
    Clear();
}

JointLimits::~JointLimits()
{

}

JointLimits::JointLimits(const JointLimits* JointLimitSpecs)
{
    if(JointLimitSpecs)
    {
        UpperLimit = JointLimitSpecs->UpperLimit;
        LowerLimit = JointLimitSpecs->LowerLimit;
        VelocityLimit = JointLimitSpecs->VelocityLimit;
        EffortLimit = JointLimitSpecs->EffortLimit;
    }
    else
    {
        Clear();
    }
}

void JointLimits::Clear()
{
    UpperLimit = 0;
    LowerLimit = 0;
    VelocityLimit = 0;
    EffortLimit = 0;
}

Joint::Joint()
{
    Type = JointType::UNKNOWN;

    JointAxis = Eigen::Vector3d::Zero();

    OriginXYZ = Eigen::Vector3d::Zero();
    OriginRPY = Eigen::Vector3d::Zero();
    OriginG = Eigen::Matrix4d::Identity();

    Limits.Clear();

    ParentLinkName.Reset(20);
    ChildLinkName.Reset(20);

    ElementType = ArticulatedElement::JOINT;
}

Joint::~Joint()
{

}

Joint::Joint(const Joint* JointSpecs) :
    ArticulatedElement(JointSpecs),
    Limits(JointSpecs->Limits)
{
    if(JointSpecs)
    {
        Type = JointSpecs->Type;

        JointAxis = JointSpecs->JointAxis;

        OriginXYZ = JointSpecs->OriginXYZ;
        OriginRPY = JointSpecs->OriginRPY;
        OriginG = JointSpecs->OriginG;

        ParentLinkName = JointSpecs->ParentLinkName;
        ChildLinkName = JointSpecs->ChildLinkName;
    }
    else
    {
        Type = JointType::UNKNOWN;

        JointAxis = Eigen::Vector3d::Zero();

        OriginXYZ = Eigen::Vector3d::Zero();
        OriginRPY = Eigen::Vector3d::Zero();
        OriginG = Eigen::Matrix4d::Identity();

        ParentLinkName.Reset(20);
        ChildLinkName.Reset(20);

        ElementType = ArticulatedElement::JOINT;
    }
}

JointState::JointState()
{
    JointPosition = MakeShareable(new double(0));
    JointVelocity = MakeShareable(new double(0));
    JointAcceleration = MakeShareable(new double(0));
    JointEffort = MakeShareable(new double(0));

    JointG = MakeShareable(new Eigen::Matrix4d());
    *JointG = Eigen::Matrix4d::Identity();
}

JointState::~JointState()
{

}

JointState::JointState(const Joint* JointSpecs) :
    Joint(JointSpecs)
{
    JointPosition = MakeShareable(new double(0));
    JointVelocity = MakeShareable(new double(0));
    JointAcceleration = MakeShareable(new double(0));
    JointEffort = MakeShareable(new double(0));

    JointG = MakeShareable(new Eigen::Matrix4d());
    *JointG = Eigen::Matrix4d::Identity();
}

}