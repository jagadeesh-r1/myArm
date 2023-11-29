#pragma once

#include "MotionObject.h"

namespace MotionEngine
{

class Link;
class Joint;
class Tree;

class ArticulatedMotionObject : public MotionObject
{
    public:
        ArticulatedMotionObject();
        ~ArticulatedMotionObject();

        TArray< TSharedPtr< Link > > Links;
        TArray< TSharedPtr< Joint > > Joints;

        TMap< FString, TSharedPtr< Link > > LinkMap;
        TMap< FString, TSharedPtr< Joint > > JointMap;

        TSharedPtr< Tree > ObjectTree;

        void AddLink(const Link* NewLink);
        void AddJoint(const Joint* NewJoint);
};

}