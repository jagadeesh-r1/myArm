#include "SimpleMotionObject.h"
#include "Link.h"

namespace MotionEngine
{

SimpleMotionObject::SimpleMotionObject()
{
    ObjectType = MotionObjectType::SIMPLE_MOTION_OBJECT;

    ObjectLink.Reset();
    ObjectLinkState.Reset();
}

SimpleMotionObject::~SimpleMotionObject()
{

}

void SimpleMotionObject::InitializeLink(const Link* NewLink)
{
    ObjectLink.Reset();
    ObjectLinkState.Reset();

    ObjectLinkState = MakeShareable(new LinkState(NewLink));
    ObjectLink = ObjectLinkState;
}

}