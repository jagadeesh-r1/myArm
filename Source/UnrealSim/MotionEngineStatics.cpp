#include "MotionEngineStatics.h"

namespace MotionEngine
{

MotionEngineStatics::MotionEngineStatics()
{

}

MotionEngineStatics::~MotionEngineStatics()
{

}

Eigen::Vector3d MotionEngineStatics::UnrealToROS(const FVector &Vec)
{
    Eigen::Vector3d Res(Vec.X/100,-Vec.Y/100,Vec.Z);
    


    return Res;
}

Eigen::Quaterniond MotionEngineStatics::UnrealToROS(const FQuat &Quat)
{
    Eigen::Quaterniond Res;


    return Res;
}

FVector MotionEngineStatics::ROSToUnreal(const Eigen::Vector3d &Vec)
{
    FVector Res;

    Res.X = Vec(0) * 100;
    Res.Y = Vec(1) * -100;
    Res.Z = Vec(2) * 100;

    return Res;
}

FQuat MotionEngineStatics::ROSToUnreal(const Eigen::Quaterniond &Quat)
{
    FQuat Res;

    Res.W = Quat.w();
    Res.X = -Quat.x();
    Res.Y = Quat.y();
    Res.Z = -Quat.z();

    return Res;
}

}