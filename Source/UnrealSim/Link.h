#pragma once

#include "CoreMinimal.h"
#include "MotionObjectProperties.h"
#include "ArticulatedElement.h"

namespace MotionEngine
{

class Link : public ArticulatedElement
{
    public:
        Link();
        ~Link();

        Link(const Link* LinkSpecs);

        TSharedPtr< Visual > LinkVisual;
        TSharedPtr< Collision > LinkCollision;
        TSharedPtr< Inertial > LinkInertial;
};

class LinkState : public Link
{
    public:
        LinkState();
        ~LinkState();

        LinkState(const Link* LinkSpecs);

        TSharedPtr< Eigen::Matrix4d > LinkG;
};

}