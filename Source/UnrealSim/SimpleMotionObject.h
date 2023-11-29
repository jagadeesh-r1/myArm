#pragma once

#include "MotionObject.h"
#include "MotionObjectProperties.h"
#include "Eigen/Dense"

namespace MotionEngine
{

class Link;
class LinkState;

class SimpleMotionObject : public MotionObject
{
    public:
        SimpleMotionObject();
        ~SimpleMotionObject();

        TSharedPtr<Link> ObjectLink;
        TSharedPtr<LinkState> ObjectLinkState;

        void InitializeLink(const Link* NewLink);
};

}