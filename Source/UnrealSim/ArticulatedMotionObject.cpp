#include "ArticulatedMotionObject.h"
#include "Link.h"
#include "Joint.h"
#include "Tree.h"

namespace MotionEngine
{

ArticulatedMotionObject::ArticulatedMotionObject()
{
    ObjectType = MotionObjectType::ARTICULATED_MOTION_OBJECT;

    ObjectTree = MakeShareable(new Tree());
}

ArticulatedMotionObject::~ArticulatedMotionObject()
{

}

void ArticulatedMotionObject::AddLink(const Link* NewLink)
{
    TSharedPtr<Link> NewLinkPtr;
    TSharedPtr<LinkState> NewLinkState(new LinkState(NewLink));
    NewLinkPtr = NewLinkState;

    Links.Push(NewLinkPtr);
    LinkMap.Add(NewLinkPtr->Name, NewLinkPtr);

    ObjectTree->TreeLinkStates.Push(NewLinkState);
    ObjectTree->TreeLinkStateMap.Add(NewLinkState->Name, NewLinkState);
}

void ArticulatedMotionObject::AddJoint(const Joint* NewJoint)
{
    TSharedPtr<Joint> NewJointPtr;
    TSharedPtr<JointState> NewJointState(new JointState(NewJoint));
    NewJointPtr = NewJointState;

    Joints.Push(NewJointPtr);
    JointMap.Add(NewJointPtr->Name, NewJointPtr);

    ObjectTree->TreeJointStates.Push(NewJointState);
    ObjectTree->TreeJointStateMap.Add(NewJointState->Name, NewJointState);
}

}