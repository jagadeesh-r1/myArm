#pragma once

#include "CoreMinimal.h"
#include "Eigen/Dense"

namespace MotionEngine
{

class MotionObject
{
    public:
        MotionObject();
        ~MotionObject();

        enum MotionObjectType
        {
            UNKNOWN,
            SIMPLE_MOTION_OBJECT,
            ARTICULATED_MOTION_OBJECT
        };

        MotionObjectType ObjectType;

        FString ObjectName;

        // added Pose as a MotionObject property
        Eigen::Matrix4d ObjectInitialPose;
};

}