#pragma once

#include "CoreMinimal.h"
#include "Eigen/Dense"
#include "ArticulatedElement.h"

namespace MotionEngine
{

class Link;

class JointLimits
{
    public:
        JointLimits();
        ~JointLimits();

        JointLimits(const JointLimits* JointLimitSpecs);

        double UpperLimit;
        double LowerLimit;
        double VelocityLimit;
        double EffortLimit;

        void Clear(void);
};

class Joint : public ArticulatedElement
{
    public:
        Joint();
        ~Joint();

        Joint(const Joint* JointSpecs);

        enum JointType
        {
            UNKNOWN,
            REVOLUTE,
            PRISMATIC,
            CONTINUOUS,
            FIXED
        };

        JointType Type;

        Eigen::Vector3d JointAxis;

        JointLimits Limits;

        FString ParentLinkName;
        FString ChildLinkName;
};

class JointState : public Joint
{
    public:
        JointState();
        ~JointState();

        JointState(const Joint* JointSpecs);

        TSharedPtr< double > JointPosition;
        TSharedPtr< double > JointVelocity;
        TSharedPtr< double > JointAcceleration;
        TSharedPtr< double > JointEffort;

        TSharedPtr< Eigen::Matrix4d > JointG;
        // 1-3rd row of 4th column - point where the joint is located at that time instant
        // 1-3rd row of 3rd column -  unit vector representing axis
};

}