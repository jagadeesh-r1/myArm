#pragma once

#include "CoreMinimal.h"
#include "Eigen/Dense"

namespace MotionEngine
{

class ArticulatedElement
{
    public:
        ArticulatedElement();
        ~ArticulatedElement();

        ArticulatedElement(const ArticulatedElement* ElementSpecs);

        enum ArticulatedElementType
        {
            UNKNOWN,
            JOINT,
            LINK
        };

        ArticulatedElementType ElementType;

        FString Name;

        TSharedPtr< ArticulatedElement > ParentElement;
        TArray< TSharedPtr< ArticulatedElement > > ChildElements;

        Eigen::Vector3d OriginXYZ;
        Eigen::Vector3d OriginRPY;
        Eigen::Matrix4d OriginG;

        Eigen::Matrix4d ElementG;
};

}