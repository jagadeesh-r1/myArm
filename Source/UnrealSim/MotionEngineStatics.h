#pragma once

#include "CoreMinimal.h"
#include "Eigen/Dense"

namespace MotionEngine
{

class MotionEngineStatics
{
    public:
        MotionEngineStatics();
        ~MotionEngineStatics();

        static Eigen::Vector3d UnrealToROS(const FVector &Vec);
        static Eigen::Quaterniond UnrealToROS(const FQuat &Quat);

        static FVector ROSToUnreal(const Eigen::Vector3d &Vec);
        static FQuat ROSToUnreal(const Eigen::Quaterniond &Quat);
};

}